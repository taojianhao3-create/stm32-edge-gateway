/**
 * @file Taskhandel.h
 * @brief 任务层汇总入口（兼容保留）
 *
 * 本文件原先把"任务函数声明 + 跨任务共享数据"混在一起，已拆分：
 *
 *   app_tasks.h  —— 5 个 FreeRTOS 任务函数名（给 Core/Src/freertos.c 的 xTaskCreate 用）
 *   app_data.h   —— Data_t 结构体 + connect 状态（给队列/数据共享用）
 *
 * 保留本文件的原因：
 *   1. Keil 工程 MDK-ARM/F4project.uvprojx 里显式登记了 ..\code\Taskhandel.h；
 *   2. 历史代码/笔记可能仍以 "Taskhandel.h" 作为任务层的统一入口。
 *
 * 新代码请按需直接包含 app_tasks.h 或 app_data.h，不要只为了一个符号
 * 就把两个头文件一起拉进来（那正是这次拆分要消除的问题）。
 */
#ifndef TASKHANDEL_H
#define TASKHANDEL_H

#include "app_tasks.h"
#include "app_data.h"

#endif /* TASKHANDEL_H */
