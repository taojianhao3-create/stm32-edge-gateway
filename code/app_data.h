/**
 * @file app_data.h
 * @brief 跨任务共享数据（原 Taskhandel.h 拆分出来的"数据共享"部分）
 *
 * 谁需要它：
 *   - Core/Src/main.c    —— xQueueCreate(1, sizeof(Data_t)) 创建两个邮箱队列
 *   - model/modbus.c     —— xQueueOverwrite(data_queue / lvgl_data_queue, &Data_t)
 *   - code/Taskhandel.c  —— xQueueReceive 取出 Data_t 上屏
 *
 * 这里只放"共享的数据类型与状态变量"，不放任务函数名（见 app_tasks.h）。
 */
#ifndef APP_DATA_H
#define APP_DATA_H

#include <stdint.h>

/* ---- 采集数据结构 ----------------------------------------------------- */

/**
 * 一帧传感器数据，同时是 data_queue 与 lvgl_data_queue 的元素类型。
 * 两个队列长度都是 1，写入用 xQueueOverwrite，永远只保留最新一帧。
 */
typedef struct {
    float temp;    /**< 01# 温湿度传感器：温度（℃） */
    float shi;     /**< 01# 温湿度传感器：湿度（%） */
    float light;   /**< 02# 光照传感器：光照（lx） */
} Data_t;

/* ---- 跨任务共享状态 --------------------------------------------------- */

/* 注：原有一个 `extern volatile uint8_t connect` 标志，由阻塞式 RE_MQTT_Init()
 * 在重连成功时置 1、AT 任务用来避免"刚重连上就被旧计数判死"。
 * 重连改由 net_sm 状态机负责（成功后直接清零 heart_fail）后，该标志已无用途，
 * 现已删除。MQTT 在线状态统一看 flash.h 里的 g_mqtt_connected。 */

/* =======================================================================
 * 看门狗（IWDG）相关
 *
 * 设计要点：喂狗不再是"独立任务无条件喂"，而是"每个关键任务签到 → 全都签到了
 * 才喂"。原因：无条件喂狗只能发现"整个系统停摆"，发现不了"单个任务死掉"——
 * 而后者才是 RTOS 里最常见的故障（UI 死等信号量、MQTT 卡在锁上等）。
 * 改成签到制后，任何一个关键任务卡住超过一个超时周期，整机自动复位。
 * ======================================================================= */

/** 各任务的签到位。任务跑完一轮就往 dog_bits 里置上自己那一位 */
#define DOG_BIT_UI       (1u << 0)   /**< UI 任务：LVGL 刷新 + 上屏 */
#define DOG_BIT_MODBUS   (1u << 1)   /**< Modbus 任务：RS485 轮询一圈 */
#define DOG_BIT_MQTT     (1u << 2)   /**< MQTT 任务：一轮收发/补传 */
#define DOG_BIT_AT       (1u << 3)   /**< AT 任务：一次心跳 */

/** "全部签到"的掩码。Dog_task 只在 dog_bits 等于它时才喂狗 */
#define DOG_ALL_BITS     (DOG_BIT_UI | DOG_BIT_MODBUS | DOG_BIT_MQTT | DOG_BIT_AT)

/**
 * IWDG 目标超时（秒）。
 *
 * ⚠️ 这个值是"任务周期"和"硬件能力"两方面的共同约束下选出来的，改之前请看完：
 *
 * 【约束一：任何一颗芯片都要能精确实现它】
 * IWDG_RLR 只有 12 位，最大可达超时 = 4096 × 最大分频 / LSI 频率：
 *     LSI=17kHz（最坏）→ 61.7 s
 *     LSI=25kHz        → 41.9 s
 *     LSI=32kHz（标称）→ 32.8 s   ← 真正的瓶颈
 * 实测验证：目标 ≤32 秒时，全 LSI 范围的超时误差都是 0.00%；
 * 35 秒以上会被天花板截断（40 秒时误差已达 18%）。
 * 所以**上限就是 32 秒左右**。
 *
 * 【约束二：必须大于参与签到的任务的周期】
 *   Modbus : 2 从机 ×(800ms+30ms) + 800ms          ≈  2.5 s
 *   UI     : 5ms/轮                                 ≈  0.005 s
 *   MQTT   : vTaskDelay(5000)+补传5×800+出队        ≈  9.0 s
 *   AT     : vTaskDelay(10000) + 心跳收发(≤1s)      ≈ 11.0 s  ← 最紧
 * 取 **20 秒**：对 AT 留 9 秒余量，同时满足约束一（误差 0%）。
 *
 * 【为什么 AT 的重连路径不参与签到】
 * RE_MQTT_Init() 内部是一串阻塞 AT 指令，最坏约 26.5 秒（含 5 秒级的
 * CWJAP/CIPSTART/CIPSEND 超时），加上 10 秒延时与心跳，单轮最坏约 38 秒，
 * 超过上面 32 秒的硬件上限。若要求"重连完成才算签到"，就只能把超时设到
 * 40 秒以上，而那时校准已经失效（误差 18%+），得不偿失。
 * 因此 AT 只在"完成一次心跳往返"后签到，重连路径不签到 —— 见 Taskhandel.c
 * 里的说明。这样正常运行时覆盖 AT 挂死，断网时即使因重连超时被复位也无害
 * （本来就没有网可用）。
 */
#define APP_IWDG_TIMEOUT_S       20U

/** 喂狗检查周期（ms）。越小则实际超时越接近目标值，但唤醒更频繁 */
#define APP_IWDG_REFRESH_MS      1000U

#endif /* APP_DATA_H */
