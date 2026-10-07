#ifndef FLASH_H
#define FLASH_H


#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"


//分区封装Flash_Part_Write→BufferWrite→PageWrite，分层调用。

// ===================== 1. W25Q64 分区定义（固定不变）=====================
//设计优势：物理分区隔离，一个分区损坏不影响其他功能，符合工业设备设计规范。
/*前4M日志区 后4M补传缓存区*/

#define LOG_PART_ADDR       0x000000U  
#define LOG_PART_SIZE       (0x400000U-1)								//日志


#define CACHE_START_ADDR    (4 * 1024 * 1024)					//补传缓冲区
#define CACHE_END_ADDR      (8 * 1024 * 1024)
#define CACHE_SECTOR_SIZE   4096
#define CACHE_ENTRY_SIZE    128

#define MAGIC_VALID         0xDEADBEEF
#define MAGIC_EMPTY         0xFFFFFFFF

#define LOG_CONTENT_LEN     64U    // 单条日志内容最大长度
#define LOG_MAGIC           0x55AAU // 有效日志魔数

// ===================== 2. 日志等级 & 日志结构体 =====================
typedef enum
{
    LOG_INFO  = 0,   // 正常信息：联网、补传、启动
    LOG_WARN  = 1,   // 警告：断网、重连
    LOG_ERROR = 2    // 错误：Modbus超时、SPI故障
} Log_Level;






typedef struct
{
    uint32_t    time_stamp;    // 时间戳：系统运行滴答数 HAL_GetTick()
    Log_Level   level;         // 日志等级
    uint8_t     content[LOG_CONTENT_LEN]; // 日志文本内容
    uint16_t    magic;         // 魔数：判断这条日志是否有效
} Log_Obj;

#define LOG_OBJ_SIZE        sizeof(Log_Obj)					//76

// ===================== 3. 全局互斥锁（Flash 全局保护）=====================
extern SemaphoreHandle_t flash_mutex;

// ===================== 4. 分区通用读写接口（新增）=====================
uint8_t Flash_Part_Read(uint32_t part_base, uint32_t offset, uint8_t *buf, uint16_t len);
uint8_t Flash_Part_Write(uint32_t part_base, uint32_t offset, uint8_t *buf, uint16_t len);

// ===================== 5. 日志接口（新增）=====================
void Log_Write(Log_Level level, const char *str);								//全工程通用日志写入入口
uint8_t Log_ReadOne(uint32_t log_offset, Log_Obj *log_obj);			//单条日志读取，供 LVGL 界面使用。

// ===================== 6. 原有环形缓存接口（全部保留）=====================
extern volatile uint8_t g_mqtt_connected;

void FlashCache_Init(void);
int  FlashCache_Write(const char *json, uint16_t len);
int  FlashCache_Read(char *json, uint16_t *len);
int  FlashCache_HasData(void);
void FlashCache_Clear(void);
void MQTT_PublishJson(const char *json);
void MQTT_SendTempHumi_WithCache(float temp, float humi);

void FlashCache_MarkSent(void);
int FlashCache_ReadAndConsume(char *json, uint16_t *len);
void my_clear();
#endif

