#ifndef COMMAND_H
#define COMMAND_H

#include "main.h"
#include <string.h>
#include "usart.h"
#include "stm32f4xx.h"
#include "stdio.h"

#define BUFFER_SIZE 300
#define COMMAND_MIN_LENGTH 4
uint8_t Command_Write(uint8_t *data, uint8_t length);

uint8_t Command_GetCommand(uint8_t *command);
uint8_t RingBuf_Read(uint8_t *out) ;
void RingBuf_Clear(void);

/**
 * @brief 查看第 index 个待读字节但不消费它（从 readIndex 起算）
 * @param index 相对读指针的偏移，0 = 下一个待读字节
 * @param out   输出
 * @return 1 成功；0 表示缓冲区里没有这么多字节
 *
 * @note 供"收齐整帧再整体消费"的场景使用：例如 MQTT 的 CONNACK 必须
 *       先按 4 个字节校验通过（0x20 0x02 xx 0x00）才应该把它们读走，
 *       否则校验失败时字节已经被吃掉，无法继续等待。
 */
uint8_t RingBuf_Peek(uint8_t index, uint8_t *out);

/** @brief 消费（丢弃）n 个待读字节 */
void RingBuf_Advance(uint8_t n);
#endif /* INC_COMMAND_H_ */
