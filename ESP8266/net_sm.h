/**
 * @file net_sm.h
 * @brief ESP8266 连接流程的"分步非阻塞状态机"接口
 *
 * 背景（为什么要做这件事）：
 *   原来的 MQTT_Init() / RE_MQTT_Init() 是一串阻塞式 AT 指令，单次调用最坏
 *   要 26.5~31.5 秒（AT+CWJAP/CIPSTART/CIPSEND 各 5 秒超时 + 多处 vTaskDelay）。
 *   这带来三个问题：
 *     1. 独占 AT 任务 26 秒以上，期间看门狗只能靠"无条件喂狗"敷衍；
 *     2. 看门狗超时被这一条路径顶到 32 秒以上，而 IWDG 受 12 位 RLR 限制
 *        在标称 LSI 下最大只能做 32.8 秒 —— 两者无法共存；
 *     3. 串口互斥锁长时间被占，MQTT 任务发不出数据。
 *
 * 现在把它拆成"每次调用只干一小段（≤50ms）"的状态机：
 *     while (!NetSm_IsDone()) {
 *         NetSm_Slice(NET_SM_SLICE_MS);   // 最多占用约 50ms
 *         AppDogCheckIn(DOG_BIT_AT);      // 每小段都签到 → 看门狗能精确覆盖
 *     }
 *   这样"最长静默时间"从 26 秒降到 50ms 量级，IWDG 超时可以安全收到 12~20 秒。
 *
 * 锁的处理：调用方（AT 任务）在每个 slice 前后 take/give uart1_mutex，
 * 因为 slice 内部会往 huart1 写字节。
 */
#ifndef NET_SM_H
#define NET_SM_H

#include <stdint.h>

/** 单个 slice 的时间预算（ms）。越小越"软"，但状态机推进越慢 */
#define NET_SM_SLICE_MS     50U

/** 整条流程允许的最长时间（ms）。超时即判失败，交给上层重试 */
#define NET_SM_TIMEOUT_MS   30000U

/** 结果码 */
#define NET_SM_OK           0
#define NET_SM_ERR_TIMEOUT  (-1)
#define NET_SM_ERR_AT       (-2)   /**< 某条 AT 指令返回 ERROR */
#define NET_SM_ERR_CONNACK  (-3)   /**< MQTT CONNACK 校验失败 */
#define NET_SM_ERR_BUSY     (-4)   /**< 状态机未启动就调用 Slice */

/**
 * @brief 启动"开机建链"流程（不重启模块，顺序同原 MQTT_Init）
 * @note  进入前请确认不是透传状态；本流程第一步就是发 "+++" 退出透传。
 */
void NetSm_StartBoot(void);

/**
 * @brief 启动"断线重连"流程（顺序同原 RE_MQTT_Init，含两次 AT+RST）
 */
void NetSm_StartReconnect(void);

/**
 * @brief 推进状态机，最多占用 budget_ms
 * @param budget_ms 本次时间预算（建议 NET_SM_SLICE_MS）
 * @return NET_SM_OK 表示整条流程已完成且成功；
 *         其他负值表示已完成但失败；
 *         0 以外的"未完成"用 NetSm_IsDone() 判断，本函数在未完成时也返回 0
 *
 * @note 未完成时返回 NET_SM_OK(0)，请配合 NetSm_IsDone() 判断是否真正结束。
 */
int NetSm_Slice(uint32_t budget_ms);

/** @brief 流程是否已结束（成功或失败） */
int NetSm_IsDone(void);

/** @brief 结束时的结果码（未结束时无意义） */
int NetSm_Result(void);

/**
 * @brief 结束后的结果文本，便于写日志
 * @return "ok" / "timeout" / "at error" / "connack fail" / "idle"
 */
const char *NetSm_ResultText(void);

#endif /* NET_SM_H */
