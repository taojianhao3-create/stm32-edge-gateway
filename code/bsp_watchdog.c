/**
 * @file bsp_watchdog.c
 * @brief 独立看门狗（IWDG）板级实现：任务签到制喂狗 + LSI 实测校准
 *
 * 详见 bsp_watchdog.h。两块内容：
 *   A. AppDogCheckIn() / AppDogTryRefresh()  —— 签到制喂狗
 *   B. AppWatchdog_Calibrate()               —— TIM5_CH4 数 LSI 边沿，反算重装载值
 *
 * 本文件不依赖调度器（校准部分在 main() 里、调度器启动前就被调用）。
 */
#include "bsp_watchdog.h"

#include "stm32f4xx_hal.h"
#include "iwdg.h"        /* hiwdg */
#include "rtc.h"         /* hrtc（tickless 的 RTC 唤醒源） */
#include "FreeRTOS.h"
#include "task.h"        /* taskENTER_CRITICAL / taskEXIT_CRITICAL */
#include "flash.h"       /* Log_Write（只在签到部分用） */
#include "app_data.h"    /* APP_IWDG_TIMEOUT_S / DOG_ALL_BITS */

/* ==========================================================================
 * A. 签到制喂狗
 * ========================================================================== */

/** 任务签到位集合。Dog_task 读并清，各任务置自己那一位 */
static volatile uint8_t dog_bits = 0U;

/** Dog_task 连续未喂狗的次数 */
static uint32_t dog_miss_count = 0U;

/**
 * 记录"哪些任务没签到"的**累积位图**，用于现场排查。
 *
 * 为什么需要它：只输出一句 "task stalled" 时，你无法判断是哪个任务卡住了。
 * 这里把每次检查中"缺失的位"累积起来（OR），等任务恢复时把这一个数写进日志，
 * 再配合 bit 表就能反推出"是谁、在什么时候卡的"：
 *
 *   bit0 = UI    bit1 = Modbus    bit2 = MQTT    bit3 = AT
 *
 * 例：日志出现 "WDG stall bits=0x0C" → 0x0C = bit2|bit3 → MQTT 和 AT 都没签到。
 *
 * 注意是"累积 OR"而不是"某一瞬间"：因为任务可能只卡了十几秒，
 * 单次采样不一定撞上，累积值才能保证这一点被记下来。
 */
static uint8_t dog_stall_bits = 0U;

void AppDogCheckIn(uint8_t bit)
{
    /* 单一位的 |= 是"读-改-写"。不加临界区的原因：各任务写的是互不相同的位，
     * 唯一会整体清位的是 AppDogTryRefresh()，它已经用临界区保护。 */
    dog_bits |= bit;
}

/** 把缺失位图转成可读的任务名缩写（顺序固定，便于对照日志） */
const char *AppDogStallName(uint8_t bits)
{
    /* 静态缓冲：本函数只在 Dog_task 里调用，无并发问题 */
    static char buf[24];
    int n = 0;

    buf[0] = '\0';
    if (bits & DOG_BIT_UI)     { buf[n++]='U'; buf[n++]='I'; buf[n++]=','; }
    if (bits & DOG_BIT_MODBUS) { buf[n++]='M'; buf[n++]='B'; buf[n++]=','; }
    if (bits & DOG_BIT_MQTT)   { buf[n++]='M'; buf[n++]='Q'; buf[n++]=','; }
    if (bits & DOG_BIT_AT)     { buf[n++]='A'; buf[n++]='T'; buf[n++]=','; }
    if (n > 0) { buf[n-1] = '\0'; }   /* 去掉末尾逗号 */
    return buf;
}

int AppDogTryRefresh(void)
{
    uint8_t snapshot;
    uint8_t missing;

    /* 读-比较-清零必须原子：否则可能在"比较通过"与"清零"之间某个任务刚好置位，
     * 那一位就被这次清零吞掉，表现为"明明活着却总不喂狗"。 */
    taskENTER_CRITICAL();
    snapshot = dog_bits;
    if (snapshot == DOG_ALL_BITS) {
        dog_bits = 0U;              /* 清位，等下一轮全部重新签到 */
    }
    taskEXIT_CRITICAL();

    if (snapshot == DOG_ALL_BITS) {
        /* 全部签到：喂狗。若之前有过缺失，把累积的位图记下来再恢复。 */
        if (dog_stall_bits != 0U) {
            char line[64];
            snprintf(line, sizeof(line), "WDG stall bits=0x%02X (%s)",
                     (unsigned)dog_stall_bits, AppDogStallName(dog_stall_bits));
            Log_Write(LOG_WARN, line);
            dog_stall_bits = 0U;
            dog_miss_count = 0U;
            Log_Write(LOG_INFO, "WDG: all tasks alive");
        }
        HAL_IWDG_Refresh(&hiwdg);
        return 1;
    }

    /* 有任务没签到：记录缺失位、不喂狗，让 IWDG 到期复位。
     *
     * 之所以能在复位后看到诊断信息：复位发生在"最后一次喂狗 + 20 秒"，
     * 而这条日志在最坏情况下（首次缺失）距离复位还有约 20 秒 ——
     * 写一条 128 字节日志到 W25Q64 约 2~3 ms，来得及。 */
    missing = (uint8_t)(DOG_ALL_BITS & (uint8_t)~snapshot);
    dog_stall_bits |= missing;

    dog_miss_count++;
    if (dog_miss_count == 1U) {
        /* 首次发现缺失时立刻记一条，缩短"从卡住到留下证据"的延迟 */
        char line[64];
        snprintf(line, sizeof(line), "WDG miss bits=0x%02X (%s)",
                 (unsigned)missing, AppDogStallName(missing));
        Log_Write(LOG_ERROR, line);
    }
    return 0;
}

/* ==========================================================================
 * B. LSI 实测 + 动态重装载值
 * ========================================================================== */

/** LSI 合理范围（Hz）。
 *  F4 数据手册标称 32kHz、实际最坏约 17kHz；这里取 10~45kHz 作为"可信"区间。
 *  上限刻意收窄：若测到 50kHz 以上，说明测量有误（或芯片异常），
 *  此时按目标超时算出的 RLR 会超过 12 位上限，配置会失效——宁可不改。 */
#define LSI_MIN_VALID_HZ     10000U
#define LSI_MAX_VALID_HZ     45000U

/** ---- LSI 测量（捕获周期法）----
 *
 * 原理：TIM5 是 32 位定时器，PSC=0 时以**定时器时钟**自由计数。
 * 把 TIM5_CH4 配成输入捕获（RM0383：LSI 内部连到 TIM5_CH4），
 * 每来一个 LSI 上升沿，硬件把当前 CNT 锁存进 CCR4。
 *
 *   相邻两次捕获值之差 = 一个 LSI 周期的定时器计数
 *   实测 LSI 频率 f_lsi = 定时器时钟频率 / 周期计数
 *
 * ⚠️ 为什么不用"固定窗口内数边沿"（原实现）：那需要 CNT 被边沿清零，
 *    而"从模式复位"的触发源 TIM_TS_TI4FP 在当前 HAL 里**没有定义**
 *    （HAL 只提供 TIM_TS_TI1FP1），无法可靠配置；且 PSC≠0 时 CNT 与
 *    边沿数不再一一对应。捕获周期法只用 CCR4 差值，不需要从模式，
 *    也不受 PSC 影响。
 *
 * ⚠️ 为什么 32 位差值天然安全：一个 LSI 周期约 31us，而 100MHz 的
 *    32 位计数器要 42.9 秒才绕回一圈。所以两次捕获之间绝不会发生
 *    计数器回绕，`now - prev` 的无符号减法就是准确周期，
 *    不需要统计溢出次数（原实现正是在这里算错了）。
 */
#define LSI_TIM_PSC          (0U)             /* PSC=0：CNT 直接以定时器时钟计数 */
#define LSI_TIM_ARR          (0xFFFFFFFFU)    /* 32 位满量程，测量期间不重载 */

/**
 * 取 TIM5 的定时器时钟（Hz）—— 测量 LSI 的"精确尺子"。
 *
 * 规则（RM0383 / 参考手册）：定时器挂在 APB1 上，
 *   APB1 预分频 = 1  →  定时器时钟 = PCLK1
 *   APB1 预分频 ≠ 1  →  定时器时钟 = 2 × PCLK1
 *
 * 本项目是后者：HSE 25MHz /M12 ×N96 /P2 = 100MHz SYSCLK，HCLK=100MHz，
 * APB1=HCLK/2=50MHz → 定时器时钟 = 2×50MHz = 100MHz。
 *
 * ⚠️ 用运行时查询而不是写死 100000000：
 *    写死的话，以后任何人改 PLL 或 APB1 分频，LSI 测量会**等比出错**
 *    而且症状隐蔽（超时偏差被误当成 LSI 离散性）。这里查一次，代价为零。
 */
static uint32_t lsi_timer_clock_hz(void)
{
    uint32_t pclk1   = HAL_RCC_GetPCLK1Freq();
    uint32_t hclk    = HAL_RCC_GetHCLKFreq();
    uint32_t apb1div = (hclk != 0U && pclk1 != 0U) ? (hclk / pclk1) : 1U;

    return (apb1div == 1U) ? pclk1 : (2U * pclk1);
}

/** 采样超时（ms）。正常只需约 2ms（63 个 32kHz 周期），留足余量防卡死 */
#define LSI_CAPTURE_TIMEOUT_MS  (20U)

/** 采样点数：取奇数便于取中值。63 个周期约 2ms@32kHz，开销可忽略 */
#define LSI_SAMPLES          (63U)

/** 中值取在"首个有效周期 ±25%"之外的样本视为毛刺，剔除 */
#define LSI_PERIOD_TOL_NUM   (3U)             /* 偏差 > 3/4 或 < 1/4 视为异常 */
#define LSI_PERIOD_TOL_DEN   (4U)

/** 写出中值。必须在总中断关闭后调用，或由调用方保证无并发 */
static uint32_t lsi_period_median(const uint32_t *a, uint32_t n)
{
    uint32_t tmp[LSI_SAMPLES];
    uint32_t i, j, key;

    if (n == 0U || n > LSI_SAMPLES) {
        return 0U;
    }
    for (i = 0U; i < n; i++) {
        tmp[i] = a[i];
    }
    /* 简单的插入排序：n<=64，且只在校准时跑一次 */
    for (i = 1U; i < n; i++) {
        key = tmp[i];
        j   = i;
        while (j > 0U && tmp[j - 1U] > key) {
            tmp[j] = tmp[j - 1U];
            j--;
        }
        tmp[j] = key;
    }
    return tmp[n / 2U];
}

/**
 * TIM5 句柄 —— 必须是**文件级**的。
 *
 * 原来它是 AppWatchdog_Calibrate() 的局部变量，那样 TIM5_IRQHandler /
 * HAL_TIM_IC_CaptureCallback 就够不到它（HAL 回调只传 htim 指针，
 * 而中断入口必须有一个可达的句柄才能调 HAL_TIM_IRQHandler）。
 * stm32f4xx_hal_timebase_tim.c 里 htim4 也是同样的文件级做法。
 *
 * 并发安全性：句柄只在 AppWatchdog_Calibrate() 执行期间被使用，
 * 且测量结束时会关掉定时器、关掉 CC4 中断、关掉 NVIC 通道，
 * 之后不会再有任何中断碰它。函数返回后 TIM5 即被释放。
 */
static TIM_HandleTypeDef s_htim5;

/* ---- 捕获中断共享变量 ----------------------------------------------------
 * 只在 AppWatchdog_Calibrate() 期间有效；该函数在调度器启动前调用，
 * 调用期间只有本模块的 ISR 会碰它们，因此不需要额外加锁。 */
static volatile uint32_t s_cap_period[LSI_SAMPLES];
static volatile uint32_t s_cap_count   = 0U;      /* 已收集的周期数（<= LSI_SAMPLES） */
static volatile uint32_t s_cap_prev    = 0U;      /* 上一次捕获的 CNT */
static volatile uint8_t  s_cap_has_prev = 0U;     /* 是否已有上一次捕获 */
static volatile uint8_t  s_cap_done     = 0U;     /* 收满 LSI_SAMPLES 个周期 */

/**
 * @brief TIM5 输入捕获回调：每次 LSI 上升沿硬件锁存 CNT，这里算差值
 * @note  由 HAL_TIM_IRQHandler() 调用，运行在 TIM5 中断上下文。
 *        采样满后立即关掉捕获中断，后续边沿不再产生中断。
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint32_t now;

    if (htim->Instance != TIM5 || htim->Channel != HAL_TIM_ACTIVE_CHANNEL_4) {
        return;
    }

    now = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);

    if (s_cap_has_prev == 0U) {
        s_cap_prev     = now;
        s_cap_has_prev = 1U;
        return;
    }

    if (s_cap_count < LSI_SAMPLES) {
        /* 32 位无符号差值：测量期间不可能回绕，见上面的说明 */
        s_cap_period[s_cap_count] = now - s_cap_prev;
        s_cap_count++;
    }
    s_cap_prev = now;

    if (s_cap_count >= LSI_SAMPLES) {
        s_cap_done = 1U;
        /* 收满即停：关通道捕获中断，避免继续进 ISR 占 CPU */
        __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC4);
    }
}

/**
 * TIM5 中断服务程序。
 *
 * ⚠️ 必须自己定义：startup_stm32f411xe.s 里 TIM5_IRQHandler 只是 [WEAK] 的
 *    默认死循环，而 TIM5 又不在 CubeMX 工程里（.ioc 没启用它），
 *    所以没有 CubeMX 生成的版本。漏掉它的后果很隐蔽：
 *    捕获中断进不来 HAL_TIM_IRQHandler → HAL_TIM_IC_CaptureCallback 永不执行
 *    → s_cap_count 恒为 0 → 测量安静地失败，只看到校准返回 -1。
 *
 * 放在这里而不是 stm32f4xx_it.c：后者是 CubeMX 生成的文件，重新生成会被覆盖；
 * 而 TIM5 的使用完全属于本模块（LSI 测量），放一起更好维护。
 */
void TIM5_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&s_htim5);
}

/** 改写 IWDG 寄存器时等待 PVU/RVU 的超时（ms） */
#define IWDG_UPDATE_TIMEOUT_MS  100U

/** IWDG 可用分频系数（编码值取自 stm32f4xx_hal_iwdg.h）。
 *  顺序按"分频从大到小"，选型时会跳过装不下目标超时的那些。 */
static const uint16_t lsi_psc_table[6][2] = {
    { 256U, IWDG_PRESCALER_256 },
    { 128U, IWDG_PRESCALER_128 },
    { 64U,  IWDG_PRESCALER_64  },
    { 32U,  IWDG_PRESCALER_32  },
    { 16U,  IWDG_PRESCALER_16  },
    { 8U,   IWDG_PRESCALER_8   },
};

/** 等待 IWDG 状态寄存器的某个更新位清零。返回 0 成功，-1 超时 */
static int iwdg_wait_clear(uint32_t flag)
{
    uint32_t t0 = HAL_GetTick();
    while ((IWDG->SR & flag) != 0U) {
        if ((HAL_GetTick() - t0) > IWDG_UPDATE_TIMEOUT_MS) {
            return -1;
        }
    }
    return 0;
}

int AppWatchdog_Calibrate(wdg_cal_result_t *res)
{
    TIM_IC_InitTypeDef ic;
    uint32_t t0, f_lsi, median;
    uint32_t i, n_valid, rlr = 0U, chosen_psc = 0U, psc_value = 0U;
    uint8_t  tim_on = 0U;      /* TIM5 是否已启用（失败路径按此决定要不要关） */
    int      rc = -1;

    if (res != NULL) {
        res->measured_hz = 0U;
        res->reload      = 0U;
        res->prescaler   = 0U;
        res->status      = -1;
    }

    /* ---------------- 0. 复位采样状态 ---------------- */
    s_cap_count    = 0U;
    s_cap_prev     = 0U;
    s_cap_has_prev = 0U;
    s_cap_done     = 0U;

    /* ---------------- 1. TIM5 时基 ----------------
     * 时钟来自 HSE→PLL→APB1 定时器时钟（本项目为 100MHz：
     * HSE 25MHz /M12 ×N96 /P2 = 100MHz SYSCLK；APB1 = HCLK/2 = 50MHz，
     * 因 APB1 预分频 ≠ 1，定时器时钟 = 2 × PCLK1 = 100MHz）。
     * 这是"精确参考"，用它去量不确定的 LSI。 */
    __HAL_RCC_TIM5_CLK_ENABLE();

    s_htim5.Instance               = TIM5;
    s_htim5.Init.Prescaler         = LSI_TIM_PSC;      /* 0 → 以 100MHz 直接计数 */
    s_htim5.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim5.Init.Period            = LSI_TIM_ARR;      /* 32 位满量程 */
    s_htim5.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim5.Init.RepetitionCounter = 0U;
    s_htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_IC_Init(&s_htim5) != HAL_OK) {
        if (res != NULL) { res->status = -2; }
        goto cleanup;
    }

    /* ---------------- 2. CH4 输入捕获（上升沿，开捕获中断）----------------
     * RM0383：LSI 内部连接到 TIM5_CH4，不需要配 GPIO。
     * 每次上升沿硬件把 CNT 锁存进 CCR4，我们在中断里取相邻差值。 */
    ic.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
    ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
    ic.ICPrescaler = TIM_ICPSC_DIV1;
    ic.ICFilter    = 0U;

    if (HAL_TIM_IC_ConfigChannel(&s_htim5, &ic, TIM_CHANNEL_4) != HAL_OK) {
        if (res != NULL) { res->status = -2; }
        goto cleanup;
    }

    /* TIM5 不在 CubeMX 工程里，NVIC 向量优先级也就没配过 —— 这里补上。
     * 取 (5,0) 与工程里其它外设中断一致。 */
    HAL_NVIC_SetPriority(TIM5_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);

    __HAL_TIM_SET_COUNTER(&s_htim5, 0U);
    __HAL_TIM_CLEAR_FLAG(&s_htim5, TIM_FLAG_CC4);
    __HAL_TIM_ENABLE_IT(&s_htim5, TIM_IT_CC4);

    if (HAL_TIM_IC_Start_IT(&s_htim5, TIM_CHANNEL_4) != HAL_OK) {
        if (res != NULL) { res->status = -2; }
        goto cleanup;
    }
    tim_on = 1U;

    /* ---------------- 3. 等采样完成（带超时兜底）----------------
     * 正常耗时：63 个 LSI 周期 ≈ 2ms @32kHz。给 20ms 上限，
     * 若 LSI 完全没信号（或 CH4 没连上）也不会卡死在这里。 */
    t0 = HAL_GetTick();
    while (s_cap_done == 0U) {
        if ((HAL_GetTick() - t0) > LSI_CAPTURE_TIMEOUT_MS) {
            break;
        }
    }

    __HAL_TIM_DISABLE_IT(&s_htim5, TIM_IT_CC4);
    (void)HAL_TIM_IC_Stop_IT(&s_htim5, TIM_CHANNEL_4);
    __HAL_TIM_DISABLE(&s_htim5);
    tim_on = 0U;

    __HAL_RCC_TIM5_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM5_IRQn);

    if (s_cap_done == 0U || s_cap_count < (LSI_SAMPLES / 2U)) {
        /* 采到的样本太少：LSI 无输出或测量通路异常 */
        if (res != NULL) { res->status = -1; }
        goto cleanup;
    }

    /* ---------------- 4. 去毛刺 + 取中值 ----------------
     * ⚠️ 基准必须用**中值**，不能用**最小值**。
     *    这是实测验证出来的坑：若有一个"偏小的毛刺"（例如捕获到接近 0 的
     *    CNT 差值），最小值会被它拉到极低，±25% 的容差窗口随之落到错误区间，
     *    结果是 63 个样本里只剩 1 个"有效"，算出完全错误的频率。
     *    中值本身抗离群，用它做基准再筛选，毛刺无论偏大偏小都筛得掉。
     *
     * 两步：先对全部样本取中值当基准 → 只保留 [中值×3/4, 中值×4/3] → 再取中值。 */
    {
        uint32_t base;
        uint32_t lo, hi;
        uint32_t keep[LSI_SAMPLES];

        /* 样本里可能有 0（两个边沿落在同一 tick），先剔除再取基准 */
        {
            uint32_t clean[LSI_SAMPLES];
            uint32_t n_clean = 0U;

            for (i = 0U; i < s_cap_count; i++) {
                if (s_cap_period[i] != 0U) {
                    clean[n_clean] = s_cap_period[i];
                    n_clean++;
                }
            }
            if (n_clean == 0U) {
                if (res != NULL) { res->status = -1; }
                goto cleanup;
            }
            base = lsi_period_median(clean, n_clean);
        }

        if (base == 0U) {
            if (res != NULL) { res->status = -1; }
            goto cleanup;
        }

        lo = (base * LSI_PERIOD_TOL_NUM) / LSI_PERIOD_TOL_DEN;
        hi = (base * LSI_PERIOD_TOL_DEN) / LSI_PERIOD_TOL_NUM;

        n_valid = 0U;
        for (i = 0U; i < s_cap_count; i++) {
            if (s_cap_period[i] >= lo && s_cap_period[i] <= hi) {
                keep[n_valid] = s_cap_period[i];
                n_valid++;
            }
        }
        if (n_valid == 0U) {
            if (res != NULL) { res->status = -1; }
            goto cleanup;
        }
        median = lsi_period_median(keep, n_valid);
    }

    if (median == 0U) {
        if (res != NULL) { res->status = -1; }
        goto cleanup;
    }

    /* ---------------- 5. 换算频率 ----------------
     * f_lsi = 定时器时钟 / 一个 LSI 周期的计数
     * 加半个除数做四舍五入，减少整数除法截断带来的偏差。 */
    {
        uint32_t tclk = lsi_timer_clock_hz();

        if (tclk == 0U) {
            if (res != NULL) { res->status = -1; }
            goto cleanup;
        }
        f_lsi = (tclk + (median / 2U)) / median;
    }

    if (f_lsi < LSI_MIN_VALID_HZ || f_lsi > LSI_MAX_VALID_HZ) {
        /* 测量不可信：保持原 IWDG 配置。宁可超时不准，也不要乱改。 */
        if (res != NULL) { res->status = -1; res->measured_hz = f_lsi; }
        goto cleanup;
    }

    /* ---------------- 4. 反算分频系数与重装载值 ----------------
     * 目标 (RLR+1) × PSC / f_lsi = APP_IWDG_TIMEOUT_S
     *   →  RLR = APP_IWDG_TIMEOUT_S × f_lsi / PSC − 1
     * 约束 RLR ≤ 4095（IWDG_RLR 只有 12 位）。
     *
     * 选型规则：从"分辨最细"的最小分频开始，选第一个能装下的。
     *   ⚠️ 不能反过来"大分频优先"：分频越大，同样的超时需要的 RLR 越大，
     *   目标超时较长时（例如 45 秒）÷256 会需要 RLR>4095，只好硬凑成 4095，
     *   结果实际超时远小于目标（25kHz 下达 -6.8%，45kHz 下达 -48%）。
     *   从小分频开始则总能落进合法区间，误差只由整数除法截断产生（<0.1%）。
     *
     * 例（目标 45 秒、f_lsi=30kHz）：÷8 需 RLR=16874>4095 跳过，
     *     ÷16 需 8436 跳过，÷32 需 4217 跳过，÷64 需 2108 ✓ 采用。 */
    for (i = (sizeof(lsi_psc_table) / sizeof(lsi_psc_table[0])) - 1U; ; i--) {
        uint32_t psc = lsi_psc_table[i][0];
        uint32_t r   = (APP_IWDG_TIMEOUT_S * f_lsi) / psc;

        if (r > 0U && (r - 1U) <= 4095U) {
            rlr        = r - 1U;
            chosen_psc = lsi_psc_table[i][1];
            psc_value  = psc;
            break;
        }
        if (i == 0U) {
            break;      /* 全部装不下（理论到不了：÷256 能覆盖到 61 秒） */
        }
    }

    if (chosen_psc == 0U) {
        rlr        = 4095U;
        chosen_psc = IWDG_PRESCALER_256;
        psc_value  = 256U;
    }

    /* ---------------- 5b. 校验配置是否真的落在目标超时附近 ----------------
     * 上面若走了兜底分支（chosen_psc==0），算出来的超时会远小于目标值——
     * 那意味着"以为有 12 秒保护，实际 1.5 秒就复位"，属于危险配置。
     * 所以这里反算一次实际超时，偏离超过 ±50% 就判定失败、不改 IWDG。 */
    {
        uint32_t actual_ms = ((rlr + 1U) * psc_value * 1000U) / f_lsi;
        uint32_t target_ms = APP_IWDG_TIMEOUT_S * 1000U;

        if (actual_ms < (target_ms / 2U) || actual_ms > (target_ms + (target_ms / 2U))) {
            if (res != NULL) { res->status = -4; res->measured_hz = f_lsi; }
            goto cleanup;
        }
    }

    /* ---------------- 6. 运行时改写 IWDG ----------------
     * 必须：解锁 → 等 PVU 清 → 写 PR → 等 RVU 清 → 写 RLR → 等 RVU 清
     *       → 重新装载计数 → 恢复写保护。
     * ⚠️ 漏掉 PVU/RVU 等待会让写入被静默忽略，现象是"代码写了但超时没变"。 */
    IWDG->KR = IWDG_KEY_WRITE_ACCESS_ENABLE;        /* 0x5555 解除写保护 */

    if (iwdg_wait_clear(IWDG_SR_PVU) != 0) {
        if (res != NULL) { res->status = -3; }
        goto relock;
    }
    IWDG->PR = chosen_psc;

    if (iwdg_wait_clear(IWDG_SR_RVU) != 0) {
        if (res != NULL) { res->status = -3; }
        goto relock;
    }
    IWDG->RLR = rlr;

    if (iwdg_wait_clear(IWDG_SR_RVU) != 0) {
        if (res != NULL) { res->status = -3; }
        goto relock;
    }

    __HAL_IWDG_RELOAD_COUNTER(&hiwdg);              /* 0xAAAA 重新装载 */
    IWDG->KR = IWDG_KEY_WRITE_ACCESS_DISABLE;       /* 0x0000 恢复写保护 */

    /* 同步回 HAL 句柄，保证后续 HAL_IWDG_Refresh 语义一致 */
    hiwdg.Init.Prescaler = chosen_psc;
    hiwdg.Init.Reload    = rlr;

    if (res != NULL) {
        res->measured_hz = f_lsi;
        res->reload      = rlr;
        res->prescaler   = (uint16_t)psc_value;
        res->status      = 0;
    }
    rc = 0;
    goto cleanup;

relock:
    /* 出错也要恢复写保护，否则 IWDG 寄存器一直处于可写状态 */
    IWDG->KR = IWDG_KEY_WRITE_ACCESS_DISABLE;

cleanup:
    /* 失败路径兜底：只在 TIM5 确实被启用过时才关闭，避免操作未初始化的句柄 */
    if (tim_on != 0U) {
        __HAL_TIM_DISABLE(&s_htim5);
    }
    __HAL_RCC_TIM5_CLK_DISABLE();
    return rc;
}

/* ==========================================================================
 * C. 低功耗：RTC 唤醒源 + HAL tick 补偿
 *
 * 背景（为什么这样做）：
 *   FreeRTOS 的 tickless idle 需要"一个能在 CPU 睡眠时继续计时、并把它唤醒"
 *   的定时器。STM32F4 上 **SysTick 不能担此任** —— 它属于内核，进 Sleep 后
 *   内核时钟停，SysTick 也停，无法唤醒自己。
 *
 *   所以流程是：挂起 SysTick → 启动 RTC 唤醒定时器 → WFI 睡下
 *   → RTC 中断唤醒 CPU → 补偿丢失的 tick → 恢复 SysTick。
 *
 *   选 RTC 而不是 TIM4 的理由：TIM4 是 HAL 时基本身，拿它当唤醒源会让
 *   "停止时基"和"定时唤醒"互相牵扯；而 RTC 由独立的 LSE 驱动，
 *   在低功耗下天然存活，正是为这个场景设计的。
 * ========================================================================== */

/** 唤醒定时器时钟：RTCCLK/2 = 32768/2 = 16384 Hz（DIV2 档） */
#define RTC_WUT_CLOCK_HZ     16384U

uint32_t AppRtcWakeup_Start(uint32_t ms)
{
    uint32_t ticks;

    if (ms == 0U) {
        return 0U;                      /* 0 毫秒没有意义，视为失败 */
    }
    if (ms > APP_TICKLESS_MAX_MS) {
        ms = APP_TICKLESS_MAX_MS;       /* 夹到 16 位计数器装得下的范围 */
    }

    /* 实际要睡多久必须回传：调用方要用它来补偿 tick。
     * ⚠️ 若这里偷偷截短了 ms 而调用方仍按原始值补偿，内核时间就会凭空快进。 */
    ticks = (ms * RTC_WUT_CLOCK_HZ) / 1000U;
    if (ticks == 0U) {
        ticks = 1U;                     /* 至少等 1 个计数单位 */
    }
    if (ticks > 0xFFFFU) {
        ticks = 0xFFFFU;                /* RTC_WUTR 只有 16 位 */
    }

    /* 先停掉上一次的配置：HAL 在 WUTE 已使能时直接改寄存器可能不生效。
     * 返回值不检查 —— 本来就没启动时 HAL 会返回错误，属正常情况。 */
    (void)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, ticks, RTC_WAKEUPCLOCK_RTCCLK_DIV2) != HAL_OK) {
        return 0U;
    }

    /* 回传"按计数单位换算回来的"实际毫秒数，保证补偿值与真实睡眠时长一致 */
    return (ticks * 1000U) / RTC_WUT_CLOCK_HZ;
}

void AppRtcWakeup_Stop(void)
{
    (void)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
}

void AppHalTick_Advance(uint32_t ms)
{
    if (ms != 0U) {
        uwTick += ms;
    }
}
