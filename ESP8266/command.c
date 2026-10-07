#include "command.h"
#include "usart.h"
#include "stm32f4xx.h"
#include "dma.h"
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
#include "flash.h"        /* Log_Write 等 */
#include "FreeRTOS.h"     /* 原生 API */
#include "task.h"         /* vTaskSuspendAll / xTaskResumeAll */

extern uint8_t a[256];
extern DMA_HandleTypeDef hdma_usart1_rx;
// 循环缓冲区大小

// 循环缓冲区
uint8_t buffer[BUFFER_SIZE];
// 循环缓冲区读索引
uint8_t readIndex = 0;
// 循环缓冲区写索引
uint8_t writeIndex = 0;

/**
 * @brief  清空环形缓冲区（重置读写指针）
 */
void RingBuf_Clear(void)
{
    readIndex = 0;
    writeIndex = 0;
}

/**
* @brief 增加读索引
* @param length 要增加的长度
*/
void Command_AddReadIndex(uint8_t length) {
    readIndex += length;
    readIndex %= BUFFER_SIZE;
}

/**
* @brief 读取第i位数据 超过缓存区长度自动循环
* @param i 要读取的数据索引
*/

uint8_t Command_Read(uint8_t i) {
    uint8_t index = i % BUFFER_SIZE;
    return buffer[index];
}

/*获取read下标对应的数据*/
uint8_t RingBuf_Read(uint8_t *out) {
    if (readIndex == writeIndex) {
        return 0;   // 缓冲区空
    }
    *out = buffer[readIndex];
    readIndex = (readIndex + 1) % BUFFER_SIZE;
    return 1;
}


uint8_t Command_GetLength() {
    return (writeIndex + BUFFER_SIZE - readIndex) % BUFFER_SIZE;
}


/* 查看第 index 个待读字节但不消费（index 从 0 起，0 = 下一个待读字节） */
uint8_t RingBuf_Peek(uint8_t index, uint8_t *out)
{
    if (out == NULL) {
        return 0;
    }
    if (index >= Command_GetLength()) {
        return 0;               /* 缓冲区里没有这么多字节 */
    }
    *out = buffer[(readIndex + index) % BUFFER_SIZE];
    return 1;
}

/* 消费（丢弃）n 个待读字节 */
void RingBuf_Advance(uint8_t n)
{
    uint8_t len = Command_GetLength();
    if (n > len) {
        n = len;
    }
    readIndex = (uint8_t)((readIndex + n) % BUFFER_SIZE);
}


/**
* @brief 计算缓冲区剩余空间
* @return 剩余空间
* @retval 0 缓冲区已满
* @retval 1~BUFFER_SIZE-1 剩余空间
* @retval BUFFER_SIZE 缓冲区为空
*/
uint8_t Command_GetRemain() {
    return BUFFER_SIZE - Command_GetLength();
}

/**	不会覆盖旧数据 满后停止写入
* @brief 向缓冲区写入数据
* @param data 要写入的数据指针
* @param length 要写入的数据长度
* @return 写入的数据长度
*/


uint8_t Command_Write(uint8_t *data, uint8_t len)
{
    // 入参校验
    if (data == NULL || len == 0)
        return 0;

    uint16_t remain = Command_GetRemain();			//剩余长度
    if (remain < len)
        return 0;

    /* 进入临界区：挂起调度器（等价于原来的 osKernelLock()）
       语义说明：vTaskSuspendAll() 只阻止"任务切换"，不屏蔽中断——这与
       osKernelLock() 完全一致，因此对 writeIndex 的读改写仍受保护：
         - 任务上下文：本函数执行期间不会被别的任务抢占；
         - 中断上下文（HAL_UARTEx_RxEventCallback 也会调用本函数）：
           writeIndex 的修改在 Cortex-M4 上是单条 STR 指令，天然原子。
       注意：本函数会被"中断"和"任务"两种上下文调用，
       所以不能用 taskENTER_CRITICAL()/portSET_INTERRUPT_MASK_FROM_ISR() 这类
       会屏蔽中断的写法（在中断里嵌套调用会破坏中断嵌套计数）。 */
    vTaskSuspendAll();

    uint16_t idx = writeIndex;
    // 分段写入环形缓冲区
    uint16_t part1 = BUFFER_SIZE - idx;					//写索引到末尾的剩余长度
    if (len <= part1)
    {
        memcpy(buffer + idx, data, len);
        writeIndex = (idx + len) % BUFFER_SIZE;
    }
    else			//绕回到开头 但并没有覆盖数据 前面的数据已经被读取
    {
        memcpy(buffer + idx, data, part1);
        memcpy(buffer, data + part1, len - part1);
        writeIndex = (len - part1) % BUFFER_SIZE;
    }

    /* 退出临界区（等价于原来的 osKernelUnlock()）。
       xTaskResumeAll() 的返回值是 xAlreadyYielded：
         pdTRUE  = 恢复调度时已经替我们完成了一次上下文切换
                   （挂起期间累积的 tick 处理过、或有高优先级任务被移入就绪链表）；
         pdFALSE = 没有发生切换。
       此处不需要据此再手动 yield，故显式 (void) 丢弃以消除告警——
       FreeRTOS 内核自己在空闲任务里也是这么写的： ( void ) xTaskResumeAll(); */
    (void) xTaskResumeAll();
    return len;
}




extern uint8_t screen_off_flag;
/**
* @brief 从缓冲区读取所有未处理的数据（不解析指令帧）
* @param command 存储读取数据的指针
* @return 实际读取的字节数（0 表示缓冲区为空）
*/
uint8_t Command_GetCommand(uint8_t *command) {
    uint8_t len = Command_GetLength();  // 获取当前未处理的数据长度
    if (len == 0) {
        return 0;
    }
    // 顺序读取 len 个字节到 command 中
    for (uint8_t i = 0; i < len; i++) {
        command[i] = Command_Read(readIndex + i);
    }
    Command_AddReadIndex(len);  // 移动读索引，表示这些数据已被消费
    return len;
}

extern TickType_t idle_tick;
extern uint8_t lcd_bl_en ; //1亮屏 0熄屏
/*接收完成回调*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart==&huart1)
	{
		Command_Write(a,Size);		//向缓冲区写入数据
		
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,a,sizeof(a));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
	}
}