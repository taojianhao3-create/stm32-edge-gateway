/**
 * @file bsp_watchdog.h
 * @brief 独立看门狗（IWDG）板级封装：任务签到制喂狗 + LSI 实测校准
 *
 * 解决的问题（两个独立问题）：
 *
 * 1) 原来 Dog_task 无条件 HAL_IWDG_Refresh()，只能发现"整个系统停摆"，
 *    发现不了"单个任务死掉"——而后者才是 RTOS 最常见的故障。
 *    现在改为：每个关键任务跑完一轮调 AppDogCheckIn(位)，Dog_task 调
 *    AppDogTryRefresh()，只有所有位都齐了才真正喂狗；有任务卡住就不喂，
 *    让 IWDG 到期把整机复位。
 *
 * 2) LSI（IWDG 的时钟源）精度很差，STM32F4 实测跨度约 17~32 kHz，
 *    导致"20 秒超时"实际可能是 20~38 秒，随芯片而变。
 *    现在用 TIM5_CH4 **输入捕获测 LSI 周期**：TIM5 是 32 位定时器，
 *    PSC=0 时以定时器时钟（HSE 衍生，100MHz）自由计数，每个 LSI 上升沿
 *    硬件把 CNT 锁存进 CCR4；相邻两次捕获值之差就是一个 LSI 周期，
 *    由此算出本芯片的真实 LSI 频率，再反算 IWDG 分频与重装载值，
 *    使超时精确等于 APP_IWDG_TIMEOUT_S。
 *
 * ⚠️ 时基精度说明：LSI 误差含"器件离散性"（芯片间 ±30%，上电后固定）
 *    和"温度漂移"（±5~10%）两部分。本模块只校准前者（一次测量即可）。
 *    不做事后重测：在计数运行中改写 IWDG 参数有误复位风险，不值得。
 *
 * ⚠️ 为什么不用"固定窗口内数边沿"：那需要 CNT 被边沿清零（从模式复位），
 *    而触发源 TIM_TS_TI4FP 在本工程所用 HAL 里**没有定义**
 *    （HAL 只提供 TIM_TS_TI1FP1），无法可靠配置。捕获周期法只用 CCR4
 *    的相邻差值，既有 32 位分辨率（约 0.003% 量化误差），又不依赖从模式。
 */
#ifndef BSP_WATCHDOG_H
#define BSP_WATCHDOG_H

#include <stdint.h>

/**
 * 校准过程中的诊断信息。
 * 之所以通过结构体带出来、而不是在函数里直接打日志：
 * 本函数在调度器启动前调用，此时 TIM4 中断已经在跑 HAL_GetTick()，
 * 在中断环境下操作 SPI Flash（Log_Write）不安全 —— 由调用方在
 * 调度器起来之后决定何时打印。
 */
typedef struct {
    uint32_t measured_hz;   /**< 实测 LSI 频率（Hz），仅在 status==0 时有效 */
    uint32_t reload;        /**< 实际写入 IWDG 的重装载值 */
    uint16_t prescaler;     /**< 实际写入 IWDG 的分频系数（如 256） */
    int8_t   status;        /**< 0=成功；-1=测量失败；
                                 -2=TIM5 初始化失败；
                                 -3=改写 IWDG 寄存器超时；
                                 -4=算出的超时偏离目标过大（危险配置，已放弃） */
} wdg_cal_result_t;

/**
 * @brief 测量 LSI 频率并据此重设 IWDG 超时（必须在调度器启动前调用）
 *
 * 流程：TIM5_CH4 捕获 LSI 上升沿（共 63 个周期）→ 取周期中值 → 算出实际频率
 *      → 解除 IWDG 写保护 → 写入匹配的分频/重装载值 → 恢复写保护。
 *
 * @param[out] res  诊断结果，允许传 NULL。
 * @return 0 成功；负值失败（失败时原 IWDG 配置保持不变，只是超时不准）。
 *
 * @note 会临时占用并释放 TIM5（含 TIM5_IRQn）：测量结束后断电定时器时钟、
 *       关 CC4 中断、关 NVIC 通道，调用后 TIM5 完全归还，本工程其它地方不用它。
 * @note 函数内部不调用 Log_Write，不依赖调度器（用 HAL_GetTick 做超时）。
 */
int AppWatchdog_Calibrate(wdg_cal_result_t *res);

/** @brief 由各任务调用：跑完一轮，签上自己那一位（bit 取 DOG_BIT_xxx） */
void AppDogCheckIn(uint8_t bit);

/**
 * @brief 由 Dog_task 调用：所有任务都已签到则喂狗并清位，否则不喂
 * @return 1 本次喂狗了；0 有任务未签到，未喂（该任务若持续卡住将触发复位）
 *
 * @note 未喂时会把"缺失的签到位"累积到内部位图，并在恢复后写入日志
 *       （格式 "WDG stall bits=0x0C (MQTT,AT)"），用于现场定位是谁卡住了。
 */
int AppDogTryRefresh(void);

/**
 * @brief 把缺失签名位图转成可读的任务名（"UI,MB,MQ,AT" 的子集）
 * @param bits DOG_BIT_xxx 的或
 * @return 指向静态缓冲的字符串，仅用于日志打印（非线程安全，只许 Dog_task 用）
 */
const char *AppDogStallName(uint8_t bits);

/* ==========================================================================
 * C. 低功耗：RTC 唤醒源 + HAL tick 补偿（供 FreeRTOS tickless idle 使用）
 * ========================================================================== */

/**
 * 单次睡眠上限（ms）。唤醒定时器是 16 位计数器，DIV2 档时钟 16384Hz，
 * 65535/16384*1000 ≈ 3999.9ms；取 3900 留足余量（也避免乘法溢出）。
 * 空闲时间超过它就只睡这么多，醒来后由 FreeRTOS 再睡一次。
 */
#define APP_TICKLESS_MAX_MS   3900U

/**
 * @brief 启动 RTC 唤醒定时器，让 CPU 睡指定毫秒后被唤醒
 * @param ms 期望睡眠时长（毫秒），内部夹到 [1, APP_TICKLESS_MAX_MS]
 *
 * @note 唤醒定时器时钟 = RTCCLK/2 = 16384 Hz（RTCCLK = LSE 32768 Hz），
 *       每个计数单位约 61 微秒。
 * @note 内部先 Deactivate 上一次配置，避免重复启动出错。
 * @return ≥1 = **实际**设置的睡眠毫秒数（可能被上限截短）；
 *         0  = 失败（此时**绝对不能睡**，否则没人能唤醒 CPU）
 */
uint32_t AppRtcWakeup_Start(uint32_t ms);

/** @brief 关闭 RTC 唤醒定时器（醒来后调用，避免继续产生中断） */
void AppRtcWakeup_Stop(void);

/**
 * @brief HAL tick 补偿：把 uwTick 直接前推 ms 毫秒
 * @param ms 本次睡眠实际经过的毫秒数
 *
 * @note 为什么需要：HAL 时基是 TIM4（挂 APB1），进 Sleep 后内核时钟停，
 *       TIM4 也停 → HAL_GetTick() 在睡眠期间"冻结"。不补偿的话，
 *       所有 HAL 超时和日志时间戳都会随睡眠累积丢时间。
 * @note 直接写 uwTick 而不是循环调 HAL_IncTick()：后者只 +1，
 *       推 1000ms 要调 1000 次，在唤醒路径里不可接受。
 *       uwTick 在 stm32f4xx_hal.h:204 声明为 extern __IO uint32_t，
 *       32 位写入在 Cortex-M4 上是原子的。
 */
void AppHalTick_Advance(uint32_t ms);

#endif /* BSP_WATCHDOG_H */
