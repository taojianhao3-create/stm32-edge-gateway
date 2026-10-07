/**
 * @file pal.c
 * @brief ESP8266 透传通道的低层收发（AT 指令用）
 *
 * 本文件的历史：原来这里放着 MQTT_Init() 和 RE_MQTT_Init() 两个【阻塞式】
 * 建链函数，单次调用最坏要 26.5~31.5 秒（一串 5 秒级的 AT 超时 + 多处
 * vTaskDelay）。它们已被 ESP8266/net_sm.c 里的分步非阻塞状态机取代：
 *
 *   MQTT_Init()    ->  NetSm_StartBoot()      + NetSm_Slice() 循环
 *   RE_MQTT_Init() ->  NetSm_StartReconnect() + NetSm_Slice() 循环
 *
 * 拆分动机见 net_sm.h 的文件头。本文件现在只保留状态机与 AT 任务都需要
 * 的通用收发原语：
 *
 *   MQTT_SendPing()     —— 发 MQTT PINGREQ（心跳）
 *   pal_tcp_recv_raw()  —— 从环形缓冲按字节收，带超时
 *
 * ⚠️ 注意 pal_tcp_recv_raw() 是【阻塞】的（内部 vTaskDelay(1) 让出 CPU），
 * 只适合"只需要几个字节"的场景（心跳 PINGRESP 2 字节）。状态机不使用它，
 * 而是用 RingBuf_Peek/Advance 做无阻塞分片读取。
 */
#include "esp_at.h"
#include "command.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include "stdlib.h"
#include "libemqtt.h"
#include "pal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "flash.h"
#include "app_data.h"   /* 跨任务共享数据（原 connect 标志已随阻塞式重连一并删除） */

extern uint8_t a[256];

/* ==================== MQTT 心跳包 ==================== */
/**
 * @brief 发送 MQTT PINGREQ（0xC0 0x00）
 * @note 调用方必须持有 uart1_mutex —— 这是真正的串口发送，
 *       与 MQTT 任务的 PUBLISH 共用 USART1，不加锁会在总线上交织。
 */
void MQTT_SendPing(void)
{
  uint8_t ping[] = {0xC0, 0x00};
  HAL_UART_Transmit(&huart1, ping, 2, 2000);
}

/**
 * @brief  透传模式下从 ESP8266 接收原始 TCP 数据
 * @param  sock       未使用（保留参数，兼容之前的接口设计）
 * @param  buf        接收缓冲区指针
 * @param  len        期望接收的字节数
 * @param  timeout_ms 超时时间（毫秒）
 * @return 实际接收到的字节数，超时或失败返回 -1
 *
 * @note 阻塞实现：靠 vTaskDelay(1) 主动让出 CPU，避免空转挤住其他任务。
 *       只用于"等少量确定字节"的场合；分片状态机请改用 RingBuf_Peek。
 */
int pal_tcp_recv_raw(int sock, uint8_t *buf, int len, int timeout_ms)
{
    uint32_t start = HAL_GetTick();
    int received = 0;
    while (received < len) {
        if (HAL_GetTick() - start > timeout_ms) {
            return -1;
        }
        uint8_t ch;
        while (RingBuf_Read(&ch) && received < len) {
            buf[received++] = ch;
        }
        vTaskDelay(1);
    }
    return received;
}
