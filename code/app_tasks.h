/**
 * @file app_tasks.h
 * @brief 应用任务函数声明（原 Taskhandel.h 拆分出来的"任务接口"部分）
 *
 * 谁需要它：
 *   - Core/Src/freertos.c  —— 在 StartDefaultTask() 里用 xTaskCreate() 创建这些任务
 *   - code/Taskhandel.c    —— 这些函数的定义处
 *
 * 这里只放"任务函数名"，不放数据（数据结构见 app_data.h）。
 *
 * 注意：这些函数是 FreeRTOS 任务入口，形参必须是 void*，但当前实现写成无参。
 * 为不改动实现文件，这里按 C 的兼容写法声明为 void f(void)。
 * 若将来要符合 vTaskFunction_t 原型，定义处应改为 void UI(void *argument)。
 */
#ifndef APP_TASKS_H
#define APP_TASKS_H

#include <stdint.h>

/* ---- 5 个 FreeRTOS 任务入口 ------------------------------------------- */

/**
 * 看门狗任务：按 APP_IWDG_REFRESH_MS（1 秒）检查一轮，
 * 只有所有关键任务都签到过才喂狗；有任务卡住就不喂，IWDG 到期复位整机。
 * 超时值 APP_IWDG_TIMEOUT_S（20 秒）由 LSI 实测校准，见 bsp_watchdog.h。
 */
void Dog_task(void);

/** MQTT 任务：在线发实时帧 + 补传 Flash 缓存；离线写缓存 */
void MQTT(void);

/** AT 任务：ESP8266 建链、10 秒心跳保活、断线重连 */
void AT(void);

/** UI 任务：LVGL 刷新、队列取数上屏、连接状态、运行时长、中断息屏 */
void UI(void);

/** Modbus 任务：RS485 轮询各从机，把采集结果写入 data_queue */
void Modbus(void);

/* ---- 保留声明 --------------------------------------------------------- */

/**
 * 刷新日志页列表（从 W25Q64 日志分区倒序读取，新日志置顶）。
 * 实现与前置声明都在 Lvgl/LVGL_myGUI/page_main.c，此处仅为兼容历史引用保留，
 * 当前工程内没有任何调用点。
 */
void page_log_refresh(void);

#endif /* APP_TASKS_H */
