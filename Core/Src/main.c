/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "FreeRTOS.h"   /* configUSE_PREEMPTION / configTICK_RATE_HZ ... */
#include "task.h"       /* vTaskStartScheduler() —— 原生 API，替代 osKernelStart() */
#include "dma.h"
#include "iwdg.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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
#include "app_data.h"    /* Data_t：xQueueCreate 两个邮箱队列 */
#include "bsp_watchdog.h"/* IWDG 校准 + 签到制喂狗接口 */
#include "queue.h"
#include "semphr.h"
#include "bsp_flash.h"
#include "flash.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void Log_EraseAll(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;


extern QueueHandle_t data_queue;
extern SemaphoreHandle_t uart1_mutex;  
extern QueueHandle_t lvgl_data_queue;  
uint8_t a[256];



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE BEGIN 0 */
// ??????????(????)

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* IWDG 校准结果。测量在调度器启动前完成，日志推迟到这里之后打印 */
  wdg_cal_result_t wdg_cal = { 0U, 0U, 0U, -1 };

  /* USER CODE BEGIN SysInit */
//SysTick_Config(SystemCoreClock / 1000);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_IWDG_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	SPI_FLASH_Init();       // 使能 SPI + 配置 CS
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_IWDG_Init(&hiwdg);					//启动看门狗
	/* 校准 IWDG 超时：LSI 精度很差（F4 上芯片间跨度约 17~32kHz），
	 * 不校准的话"20 秒"实际可能是 20~38 秒。这里用 TIM5_CH4 输入捕获、
	 * 以 HSE 为时基测出本芯片的 LSI 频率，反算重装载值，使超时精确等于
	 * APP_IWDG_TIMEOUT_S（12 秒）。
	 *
	 * 放在这里的原因：IWDG 已启动（原配置 20 秒，够这次测量用 —— 校准过程
	 * 本身不喂狗，必须保证它短于当前超时）、而 DMA 接收和各种中断还没打开，
	 * 测量环境最干净。失败也没关系：内部会保留原配置，只是超时不准。
	 *
	 * 注意：这里不写 Flash 日志 —— 此时 TIM4 中断已在跑 HAL_GetTick()，
	 * 在中断环境下操作 SPI Flash 不安全。日志推迟到调度器启动后打印。 */
	(void)AppWatchdog_Calibrate(&wdg_cal);
	
	
	FlashCache_Init();  							//定位下一条数据的写入地址
	RS485_RX;  
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,a,sizeof(a));					//DMA接收空闲中断(ESP8266传数据到MCU 并放在环形缓冲区)	
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);						//关闭过半中断




// 创建队列：最多存 1 条温湿度数据 覆盖式写入 
data_queue = xQueueCreate(1, sizeof(Data_t));
lvgl_data_queue= xQueueCreate(1, sizeof(Data_t));

uart1_mutex =xSemaphoreCreateMutex();							//创建互斥锁 （谁上锁 谁才能开锁）
flash_mutex=xSemaphoreCreateMutex();

 //Log_EraseAll();							// 偶尔要擦除日志区

	

	
	
  /* USER CODE END 2 */

  /* Init scheduler */
  /* 原生 FreeRTOS 不需要像 CMSIS-RTOS2 那样先调 osKernelInitialize()：
     内核的数据结构由 vTaskStartScheduler() 内部初始化（pxCurrentTCB、
     就绪/延时链表、空闲任务、软件定时器任务）。原来的那行空壳调用已删除。 */
  MX_FREERTOS_Init();			//创建默认任务   进入就绪态，但还没开始运行（调度器未启动）
  /* Start scheduler */
  vTaskStartScheduler();		//启动 FreeRTOS 任务调度器，操作系统正式运行   开启调度，多任务并发运行

//不再往下执行了
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
/* USER CODE BEGIN WHILE */
while (1)
{
	/* USER CODE BEGIN WHILE */




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**  Cubmx自动生成的时钟树代码，用户不要修改*/
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}//Cubmx自动生成的代码，用户不要修改

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)					//定时器中断回调函数
{
  /* USER CODE BEGIN Callback 0 */

	if(htim==&htim3)									//lvgl的心跳函数
	{
		lv_tick_inc(1);									//告诉 LVGL 又过去了 1 毫秒
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();				//递增HAL库系统滴答计时 对应 HAL_GetTick()
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: u1_printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
