#ifndef PAL_H
#define PAL_H

#include "main.h"
#include <string.h>
#include "usart.h"
#include "stm32f4xx.h"
#include "stdio.h"




// 巴法云参数配置
// ⚠️ 以下均为占位符，使用前请替换成自己的 Wi-Fi 与巴法云凭据。
//    真实凭据不入库：明文密码/私钥一旦推到公开仓库就会永久留在 git 历史里。
#define WIFI_SSID   "YOUR_WIFI_SSID"
#define WIFI_PASS   "YOUR_WIFI_PASSWORD"
#define BEMFA_BROKER "bemfa.com"
#define BEMFA_PORT   9501
#define BEMFA_UID    "YOUR_BEMFA_UID"  // 巴法云控制台右上角的私钥
#define TOPIC_PUB    "stm32"   // 你创建的主题名



typedef struct {
    uint8_t client_id[50];
    uint8_t username[24];
    uint8_t password[24];
    int socket;
    uint32_t keepalive_interval;
    uint8_t clean_session;
    
    // 关键：函数指针成员，用于发送和接收数据
    int (*send)(void* sock, const void *buf, unsigned int len);
    int (*recv)(int sock, uint8_t *buf, int buf_len, int timeout);
    
    void *socket_info; 
} mqtt_broker_handle_t;



int pal_tcp_recv_raw(int sock, uint8_t *buf, int len, int timeout_ms);

/**
 * MQTT 心跳：发 PINGREQ（0xC0 0x00）。
 * 调用方须持有 uart1_mutex（这是真正的串口发送）。
 */
void MQTT_SendPing(void);

/* ---------------------------------------------------------------------------
 * 注意：原 MQTT_Init() / RE_MQTT_Init() 的声明已删除。
 * 这两个函数是阻塞式的（单次最坏 26~31 秒），已被 ESP8266/net_sm.c 的
 * 分步非阻塞状态机取代：
 *     MQTT_Init()    ->  NetSm_StartBoot()      + NetSm_Slice() 循环
 *     RE_MQTT_Init() ->  NetSm_StartReconnect() + NetSm_Slice() 循环
 * 声明一并删掉，避免"能编过、链接才失败"的坑。
 * ------------------------------------------------------------------------- */
#endif
