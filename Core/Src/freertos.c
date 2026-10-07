/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"       /* HAL 类型 + LCD/背光相关声明 */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "touch.h"
#include "stdio.h"
#include "string.h"
#include "lvgl.h"                  
#include "lv_port_disp.h"        
#include "lv_port_indev.h"       
#include "page_main.h"
#include "modbus.h"
#include "command.h"
#include "ESP_at.h"
#include "libemqtt.h"
#include "pal.h"
#include "app_tasks.h"   /* UI / Modbus / AT / MQTT / Dog_task 任务函数名 */
#include "bsp_watchdog.h"/* wdg_cal_result_t（校准结果类型） */
#include "flash.h"       /* Log_Write：任务创建失败时留下证据 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern TickType_t power_tick;

extern TickType_t idle_tick;
extern uint8_t lcd_bl_en ; //1亮屏 0熄屏

/* ---------------------------------------------------------------------------
 * 任务优先级
 *
 * 这里不再使用 CMSIS-RTOS2 的 osPriority* 枚举（那需要包含 cmsis_os2.h，
 * 而本工程已统一改为原生 FreeRTOS API）。下面这几个宏取的就是原先
 * CMSIS 枚举的数值（cmsis_os2.h: Normal=24, AboveNormal=32），
 * 因此任务的实际优先级与改动前完全一致，没有行为变化。
 *
 * 数值越大优先级越高（FreeRTOS 规则，0 保留给空闲任务）。
 * 提醒：这与 Cortex-M 的 NVIC 中断优先级相反（那里数值越小越高）。
 * ------------------------------------------------------------------------- */
#define APP_PRIO_NORMAL         (24)  /* 原 osPriorityNormal        */
#define APP_PRIO_ABOVE_NORMAL   (32)  /* 原 osPriorityAboveNormal   */
#define APP_PRIO_MODBUS         (APP_PRIO_NORMAL + 1)  /* 原 osPriorityNormal + 1 */
#define APP_PRIO_DOG            (APP_PRIO_ABOVE_NORMAL + 2) /* 原 osPriorityAboveNormal + 2 */

/* AT 任务刻意比 MQTT 低一级（24 → 23）。
 *
 * 原因：AT 和 MQTT 共用 uart1_mutex 保护 USART1 的发送通道。
 * 两者原来都是 24，属于"同优先级时间片轮转"关系——MQTT 拿到锁执行到一半，
 * 可能刚好撞上 SysTick 时间片切换（configTICK_RATE_HZ=1000，每 1ms 一次），
 * AT 被切进来后会阻塞在 uart1_mutex 上；反过来 AT 先拿到锁时，MQTT 也得等它。
 *
 * 让 AT 低一级后，MQTT 一旦拿到锁就不会被 AT 打断，锁能"用完即还"；
 * AT 也不会被饿死——它 99.99% 的时间都阻塞在 vTaskDelay(10000) 上，
 * 只有在 MQTT 也阻塞时才会运行，网络功能不受影响。 */
#define APP_PRIO_AT             (APP_PRIO_NORMAL - 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
/* 原生 FreeRTOS 用 TaskHandle_t；原来的 osThreadId_t 只是 CMSIS 对它的 typedef，
   osThreadAttr_t（name/stack_size/priority 三个字段）在原生 API 里直接展开成
   xTaskCreate() 的实参，因此不再需要这两个 CMSIS 结构体。 */
TaskHandle_t defaultTaskHandle = NULL;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN VPORT_SUPPORT_TICKS_AND_SLEEP */
/**
 * @brief FreeRTOS tickless idle：睡眠期间停掉 tick 中断，让 CPU 真正睡下去
 * @param xExpectedIdleTime 预期可睡眠的 tick 数（configTICK_RATE_HZ=1000 → 约等于毫秒）
 *
 * ── 为什么需要自己实现 ──────────────────────────────────────────────
 * 本工程 configUSE_TICKLESS_IDLE = 2，该模式要求**用户提供**本函数。
 * port.c 里那份实现被 `#if( configUSE_TICKLESS_IDLE == 1 )` 包住，
 * 对本工程**不参与编译**；而 CubeMX 只生成了一个空函数体
 * （原来的注释是 "TO BE COMPLETED"）—— 即"配了 tickless 却从不睡觉"。
 *
 * ── 思路（Sleep 模式，不碰 Stop）────────────────────────────────────
 *   ① 关 SysTick 的 TICKINT 位 → tick 不再中断（但计数器照常跑）
 *   ② 关中断后确认没有就绪任务/挂起切换，否则放弃睡眠（避免漏调度）
 *   ③ 启动 RTC 唤醒定时器，约 xExpectedIdleTime 毫秒后唤醒
 *   ④ WFI 睡下 → Sleep 模式只停内核时钟，外设与 SRAM/VBAT 域照常
 *   ⑤ 醒来：停唤醒定时器，用 xTaskCatchUpTicks() 推进内核 tick
 *   ⑥ 前推 HAL tick（TIM4 在睡眠期间冻结），恢复 SysTick 中断
 *
 * ── 为什么两种 tick 都要补 ─────────────────────────────────────────
 *   · FreeRTOS tick：不补则所有 vTaskDelay 的时间会随睡眠累积变短
 *   · HAL tick（TIM4，挂 APB1）：入 Sleep 后内核时钟停 → TIM4 停 →
 *     HAL_GetTick() 冻结。它是**日志时间戳**和全部 HAL 超时的基准，
 *     不补就会随睡眠累积丢时间（本工程日志用的正是 HAL_GetTick()）。
 *
 * ── 与看门狗的关系（重要）──────────────────────────────────────────
 *   IWDG 由**独立的 LSI** 计数，20 秒超时不受 CPU 睡眠影响。
 *   tickless 只会在"下一个任务到期之前"睡，正常远小于 20 秒；
 *   但若真出现任务卡死导致长时间睡眠，IWDG 仍会照常复位整机 ——
 *   这正是要的：低功耗不能牺牲可靠性。
 *
 * ── 唤醒源为什么是 RTC 而不是 SysTick ──────────────────────────────
 *   SysTick 属于 Cortex-M 内核，进 Sleep 后内核时钟停 → 它自己也停，
 *   **无法唤醒自己**。RTC 由独立的 LSE 驱动，低功耗下天然存活。
 *   注意 RTC 唤醒中断优先级是 5（见 rtc.c），高于
 *   configMAX_SYSCALL_INTERRUPT_PRIORITY 对应的屏蔽门限，
 *   所以它不会被 BASEPRI 屏蔽，能正常唤醒 —— 这是前提。
 */
void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
{
    TickType_t xLeaveTick;      /* 进入睡眠时刻（丢的部分 tick 最多 1ms） */
    TickType_t xElapsed;        /* 醒来后核对的 tick 差 */
    uint32_t   xSleep_ms;       /* 本次**实际**睡眠毫秒数 */

    /* 睡太短不值得：进出低功耗本身有开销 */
    if (xExpectedIdleTime < 2U) {
        return;
    }

    /* ① 关掉 SysTick 的 tick 中断。
     *    只清 TICKINT（不整条停掉 SysTick）：计数器继续走，
     *    省得恢复时还要重设 LOAD/VAL；唤醒后由 xTaskCatchUpTicks 统一补。 */
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;

    /* ② 关中断。
     *    这里用 __disable_irq() 而不是 taskENTER_CRITICAL()：
     *    后者会把所有优先级低于 configMAX_SYSCALL_INTERRUPT_PRIORITY
     *    的中断一并屏蔽，而被屏蔽的中断本来是可以把 CPU 唤醒的。
     *    RTC 唤醒中断优先级为 5，不在 BASEPRI 屏蔽范围内，仍能唤醒。 */
    __disable_irq();
    __DSB();
    __ISB();

    /* 若已有就绪任务或挂起的上下文切换，说明不该睡 */
    if (eTaskConfirmSleepModeStatus() == eAbortSleep) {
        SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
        __enable_irq();
        return;
    }

    xLeaveTick = xTaskGetTickCount();

    /* ③ 启动 RTC 唤醒定时器。
     *    返回**实际**睡眠毫秒数：唤醒计数器是 16 位，DIV2 下最长约 4 秒，
     *    超过 APP_TICKLESS_MAX_MS 的请求会被截短。必须用返回值来补偿 tick，
     *    否则内核时间会凭空快进（例如 AT 任务空闲 10 秒却按 10 秒补偿，
     *    而实际只睡了 3.9 秒）。 */
    xSleep_ms = AppRtcWakeup_Start((uint32_t)xExpectedIdleTime);
    if (xSleep_ms == 0U) {
        /* 唤醒源起不来就**绝不能睡**：否则没有东西能叫醒 CPU（死等） */
        SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
        __enable_irq();
        return;
    }

    /* ④ 睡下。用 HAL_PWR_EnterSLEEPMode 而非裸 __WFI()：
     *    它会先清 SCB->SCR 的 SLEEPDEEP 位，确保进的是 **Sleep** 而不是
     *    Stop —— Stop 会关掉大部分时钟，USART/SPI 时序和 AT 心跳都得重配。 */
    __enable_irq();         /* WFI 需要中断使能才能被唤醒；上方状态已确认安全 */
    HAL_PWR_EnterSLEEPMode( PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI );

    /* ⑤ 醒来：先停唤醒定时器，免得它继续产生中断 */
    __disable_irq();
    AppRtcWakeup_Stop();

    xElapsed = xTaskGetTickCount() - xLeaveTick;

    /* ⑥ 补偿内核 tick。
     *    xTaskCatchUpTicks() 是官方为"中断被长时间关闭"提供的接口
     *    （task.h:2501 注释原文），它会正确处理因此到期的阻塞任务，
     *    比 vTaskStepTick() 更适合本场景（后者不处理任务解除阻塞）。 */
    if (xSleep_ms > (uint32_t)xElapsed) {
        (void)xTaskCatchUpTicks( (TickType_t)( xSleep_ms - (uint32_t)xElapsed ) );
    }

    /* ⑦ HAL tick 同样前推 */
    AppHalTick_Advance( xSleep_ms );

    /* ⑧ 恢复 SysTick 中断与全局中断 */
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    __enable_irq();
}
/* USER CODE END VPORT_SUPPORT_TICKS_AND_SLEEP */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask —— 原生 xTaskCreate 替代 osThreadNew
     栈单位换算：CMSIS 的 .stack_size 是“字节”，FreeRTOS 的 usStackDepth 是“字(word)”。
     cmsis_os2.c 内部就是 stack = attr->stack_size / sizeof(StackType_t)，
     所以 128 * 4 字节  ==  128 字（sizeof(StackType_t) == 4，ARM_CM4F 端口）。
     原 CMSIS 属性 defaultTask_attributes 已随之删除，避免两套定义并存。*/
  if (xTaskCreate(StartDefaultTask, "defaultTask", 128, NULL,
                  (UBaseType_t) APP_PRIO_NORMAL, &defaultTaskHandle) != pdPASS)
  {
      Log_Write(LOG_ERROR, "defaultTask create fail");
      configASSERT(0);
  }

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	
		//  任务函数  任务的名字  栈大小,单位为word  调用任务函数时传入的参数  优先级  任务句柄, 以后使用它来操作这个任务

  /** 任务创建统一处理
   *
   *  xTaskCreate 返回 pdFAIL 的唯一常见原因是 heap_4 堆内存不足
   *  （本工程 configTOTAL_HEAP_SIZE = 15360 字节，而 5 个任务的栈就要
   *   1024+512*4 = 3072 字 = 12288 字节，加上 TCB/队列，余量很小）。
   *
   *  失败时先 Log_Write 把 "xxxTask create fail" 写进 W25Q64 日志分区留下证据，
   *  再 configASSERT。
   *
   *  ⚠️ 注意 configASSERT 的定义是"关中断 + 死循环"（FreeRTOSConfig.h:155），
   *  所以这里必然停机——但至少日志页能看到是哪个任务没建起来，
   *  比"任务静默消失、表现为某个功能莫名其妙不可用"好排查得多。
   *  如果不希望现场设备因建任务失败而卡死，把 configASSERT(0) 删掉即可
   *  （宁可少一个功能，也不要整机停机）。
   *
   *  注意：任务名是字符串字面量，不能用 (name) " create fail" 这种
   *  相邻字符串拼接（那会变成 ("UITask") " create fail" 而编译失败），
   *  所以这里用 snprintf 在运行时拼出消息。 */
  #define APP_CREATE_TASK(fn, name, stack, prio)                              \
      do {                                                                    \
          if (xTaskCreate((fn), (name), (stack), NULL, (prio), NULL) != pdPASS) { \
              char _msg[40];                                                  \
              snprintf(_msg, sizeof(_msg), "%s create fail", (name));          \
              Log_Write(LOG_ERROR, _msg);                                     \
              configASSERT(0);                                                \
          }                                                                   \
      } while (0)

  // UI 任务(LVGL) 栈必须大！  优先级 32
  APP_CREATE_TASK(UI, "UITask", 1024, APP_PRIO_ABOVE_NORMAL);

  // Modbus 任务  优先级 25
  APP_CREATE_TASK(Modbus, "SensorTask", 512, APP_PRIO_MODBUS);

  // 网络 AT 任务  优先级 23（刻意低于 MQTT，见 APP_PRIO_AT 的说明）
  APP_CREATE_TASK(AT, "NetworkTask", 512, APP_PRIO_AT);

  // MQTT 任务  优先级 24
  APP_CREATE_TASK(MQTT, "MQTTTask", 512, APP_PRIO_NORMAL);

  //看门狗   优先级最高(34)。检查周期 1 秒、超时 20 秒，详见 app_data.h 里的说明
  APP_CREATE_TASK(Dog_task, "Dog_task", 512, APP_PRIO_DOG);
	

power_tick = xTaskGetTickCount();

idle_tick = xTaskGetTickCount();

lcd_bl_en = 1;

  /* ==========================================================================
   * 上报 IWDG 校准结果
   *
   * AppWatchdog_Calibrate() 是在调度器启动【前】调用的（见 main.c）——
   * 那时 TIM4 中断已经在跑 HAL_GetTick()，在中断环境下写 SPI Flash 日志不安全，
   * 所以测量结果通过 wdg_cal 带到这里，等调度器起来后再打印。
   *
   * 三条日志都远短于 LOG_CONTENT_LEN(64)，sprintf 安全。
   * ======================================================================== */
  {
      extern wdg_cal_result_t wdg_cal;   /* 定义在 main.c */

      if (wdg_cal.status == 0) {
          char wdg_line[64];
          sprintf(wdg_line, "WDG cal OK LSI=%luHz psc=%u RLR=%lu",
                  (unsigned long)wdg_cal.measured_hz,
                  (unsigned)wdg_cal.prescaler,
                  (unsigned long)wdg_cal.reload);
          Log_Write(LOG_INFO, wdg_line);
      } else if (wdg_cal.status == -3) {
          Log_Write(LOG_WARN, "WDG cal: IWDG reg update timeout");
      } else if (wdg_cal.status == -4) {
          Log_Write(LOG_ERROR, "WDG cal: result unsafe, keep default");
      } else {
          Log_Write(LOG_WARN, "WDG cal failed, keep default");
      }
  }

  // ========================
  // 创建完成，删除自身
  // ========================
  vTaskDelete(NULL);

  // 下面永远不会执行

  /* Infinite loop */
  for(;;)
  {
    /* 原生 API：osDelay(1) 在 cmsis_os2.c 里就是 vTaskDelay(ticks) 的一层转发。
       注意本行永远执行不到（上面已 vTaskDelete(NULL)），保留只是为了保持结构完整。 */
    vTaskDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

