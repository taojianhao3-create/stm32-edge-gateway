/**
 * @file net_sm.c
 * @brief ESP8266 连接流程的分步非阻塞状态机（替代原 MQTT_Init / RE_MQTT_Init）
 *
 * 设计：把整条建链流程做成一张"命令表"，每个状态一项。
 *   每个 slice 只推进"当前状态"的一小段（最多 budget_ms），然后立刻返回。
 *
 * 状态有两类：
 *   AT_STEP_ESCAPE : 发 "+++" 并静默 1 秒（退出透传专用，模组要求 1 秒保护时间）
 *   AT_STEP_CMD    : 发一条 AT 指令 → 分片收应答 → 匹配关键词/ERROR → 定结果
 *
 * 每一步结束时都会把响应缓冲清空（RingBuf_Clear，带临界区），
 * 保证下一条指令不会读到上一条的残留 —— 这与原实现"每条指令前清缓冲"等价。
 */
#include "net_sm.h"

#include <string.h>

#include "stm32f4xx_hal.h"
#include "usart.h"       /* huart1 */
#include "command.h"     /* RingBuf_Peek / RingBuf_Advance / RingBuf_Clear / Command_GetLength */
#include "flash.h"       /* Log_Write */
#include "FreeRTOS.h"
#include "task.h"        /* taskENTER_CRITICAL */

/* ==========================================================================
 * 命令表
 * ========================================================================== */

#define AT_STEP_CMD      0U      /**< 发 AT 指令并等关键词 */
#define AT_STEP_ESCAPE   1U      /**< 发 "+++" 后静默 1 秒 */
#define AT_STEP_RECV_AT  2U      /**< 只收字节：等待 MQTT CONNACK（不做关键词匹配） */
#define AT_STEP_BLOB     3U      /**< 透传模式下直接发一段二进制报文 */

typedef struct {
    uint8_t     kind;
    const char *tx;         /**< 要发送的字符串（不含 CRLF，由代码补；ESCAPE 用 "+++"） */
    const char *expect;     /**< 期望出现的关键词；NULL 表示只看 OK/ERROR */
    uint16_t    timeout_ms; /**< 该步的应答等待上限 */
    uint16_t    gap_ms;     /**< 该步完成后等待的静默时间（替代原 vTaskDelay） */
    const char *tag;        /**< 日志用名字 */
} AtStep_t;

/* 回复缓冲：AT 应答最长可能一百多字节（CWJAP 会打印多行） */
#define AT_RESP_MAX     160U

/* ---- 开机建链流程（对应原 MQTT_Init）----
 *
 * ⚠️ 使用前请把下面 CWJAP 步骤里的 YOUR_WIFI_SSID / YOUR_WIFI_PASSWORD
 *    替换成自己的 Wi-Fi 名与密码。
 *    仓库里刻意留占位符、不提交真实凭据 —— 明文密码一旦推到公开仓库，
 *    就会永久留在 git 历史里，事后无法真正删除。
 */
static const AtStep_t boot_steps[] = {
  /* tx, expect, timeout, gap, tag */
  { AT_STEP_ESCAPE, "+++",            NULL,              0U,    1000U, "escape"     },
  { AT_STEP_CMD,    "AT",             "OK",              2000U,  200U, "AT"         },
  { AT_STEP_CMD,    "AT+CWMODE=1",    "OK",              2000U,    0U, "CWMODE"     },
  { AT_STEP_CMD,    "AT+CWJAP=\"YOUR_WIFI_SSID\",\"YOUR_WIFI_PASSWORD\"", "WIFI GOT IP", 5000U, 500U, "CWJAP" },
  { AT_STEP_CMD,    "AT+CIPSTART=\"TCP\",\"bemfa.com\",9501", "CONNECT", 5000U, 500U, "CIPSTART" },
  { AT_STEP_CMD,    "AT+CIPMODE=1",   "OK",              2000U,    0U, "CIPMODE"    },
  { AT_STEP_CMD,    "AT+CIPSEND",     ">",               5000U,  500U, "CIPSEND"    },
  /* 进入透传后：发 MQTT CONNECT 报文，然后收 4 字节 CONNACK */
  { AT_STEP_BLOB,   NULL,             NULL,                 0U,  300U, "MQTT CONNECT" },
  { AT_STEP_RECV_AT,NULL,             NULL,              3000U,    0U, "CONNACK"    },
};

/* ---- 断线重连流程（对应原 RE_MQTT_Init，前面多两次 AT+RST）---- */
static const AtStep_t reconnect_steps[] = {
  { AT_STEP_ESCAPE, "+++",            NULL,              0U,    1000U, "escape"     },
  { AT_STEP_CMD,    "AT+RST",         "OK",              2000U, 1000U, "RST1"       },
  { AT_STEP_CMD,    "AT+RST",         "OK",              2000U, 1000U, "RST2"       },
  { AT_STEP_CMD,    "AT+CWMODE=1",    "OK",              2000U,    0U, "CWMODE"     },
  { AT_STEP_CMD,    "AT+CWJAP=\"YOUR_WIFI_SSID\",\"YOUR_WIFI_PASSWORD\"", "WIFI GOT IP", 5000U, 500U, "CWJAP" },
  { AT_STEP_CMD,    "AT+CIPSTART=\"TCP\",\"bemfa.com\",9501", "CONNECT", 5000U, 500U, "CIPSTART" },
  { AT_STEP_CMD,    "AT+CIPMODE=1",   "OK",              2000U,    0U, "CIPMODE"    },
  { AT_STEP_CMD,    "AT+CIPSEND",     ">",               5000U,  500U, "CIPSEND"    },
  { AT_STEP_BLOB,   NULL,             NULL,                 0U,  300U, "MQTT CONNECT" },
  { AT_STEP_RECV_AT,NULL,             NULL,              3000U,    0U, "CONNACK"    },
};

/* ==========================================================================
 * 模块状态
 * ========================================================================== */

static const AtStep_t *s_steps   = NULL;   /* 当前流程的命令表 */
static uint16_t        s_step_cnt = 0U;
static uint16_t        s_step      = 0U;   /* 当前步号 */

static uint8_t  s_active   = 0U;           /* 状态机是否在运行 */
static uint8_t  s_done     = 0U;           /* 是否已结束 */
static int      s_result   = NET_SM_OK;

static uint32_t s_step_t0  = 0U;           /* 当前步的开始时刻 */
static uint32_t s_sm_t0    = 0U;           /* 整条流程的开始时刻 */

static uint8_t  s_tx_done  = 0U;           /* 当前步的发送是否已完成 */
static uint32_t s_ok_tick  = 0U;           /* 当前步应答首次满足的时刻（0=尚未满足） */

/* 当前步的接收缓冲（累积，便于匹配跨片的关键词） */
static char     s_resp[AT_RESP_MAX];
static uint16_t s_resp_len = 0U;

/* MQTT 报文（与 pal.c 里的字节序列保持一致） */
static const uint8_t mqtt_connect_pkt[] = {
    0x10, 0x2E,
    0x00, 0x06,
    0x4D, 0x51, 0x49, 0x73, 0x64, 0x70,
    0x03, 0x02, 0x00, 0x78,
    0x00, 0x20,
    '4','1','f','3','8','7','6','7',
    '2','0','a','d','4','a','e','8',
    '6','0','d','b','0','6','e','3',
    '5','6','3','1','a','8','5','5'
};

static const uint8_t mqtt_sub_pkt[] = {
    0x82, 0x0A,
    0x00, 0x01,
    0x00, 0x05,
    's','t','m','3','2',
    0x01
};

/* ==========================================================================
 * 小工具
 * ========================================================================== */

static void resp_reset(void)
{
    /* 与原来"每条 AT 指令前清缓冲"等价。清的是模块回显，
       必须屏蔽中断：DMA 空闲中断会并发改 writeIndex。 */
    taskENTER_CRITICAL();
    RingBuf_Clear();
    taskEXIT_CRITICAL();
    s_resp_len = 0U;
}

static void uart_send(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 1000);
}

/** 判断累积到的应答是否已可定论：返回 AT_RES_* */
#define AT_RES_NONE     0
#define AT_RES_OK       1
#define AT_RES_ERROR    2

static int resp_classify(const AtStep_t *st)
{
    /* ERROR 优先判定 */
    if (strstr(s_resp, "ERROR") != NULL) {
        return AT_RES_ERROR;
    }
    if (st->expect != NULL) {
        return (strstr(s_resp, st->expect) != NULL) ? AT_RES_OK : AT_RES_NONE;
    }
    /* expect == NULL：只要出现 OK 即算成功 */
    return (strstr(s_resp, "OK") != NULL) ? AT_RES_OK : AT_RES_NONE;
}

/** 把环形缓冲里现有字节搬到 s_resp（不清缓冲，由调用方决定何时清） */
static void resp_drain(void)
{
    uint8_t b;
    while (s_resp_len < (AT_RESP_MAX - 1U) && RingBuf_Peek(0U, &b) != 0U) {
        s_resp[s_resp_len++] = (char)b;
        s_resp[s_resp_len]   = '\0';
        RingBuf_Advance(1U);
    }
}

/* ==========================================================================
 * 状态机推进
 * ========================================================================== */

void NetSm_StartBoot(void)
{
    s_steps    = boot_steps;
    s_step_cnt = (uint16_t)(sizeof(boot_steps) / sizeof(boot_steps[0]));
    s_step     = 0U;
    s_active   = 1U;
    s_done     = 0U;
    s_result   = NET_SM_OK;
    s_tx_done  = 0U;
    s_ok_tick  = 0U;
    s_sm_t0    = HAL_GetTick();
    s_step_t0  = s_sm_t0;
    resp_reset();
}

void NetSm_StartReconnect(void)
{
    s_steps    = reconnect_steps;
    s_step_cnt = (uint16_t)(sizeof(reconnect_steps) / sizeof(reconnect_steps[0]));
    s_step     = 0U;
    s_active   = 1U;
    s_done     = 0U;
    s_result   = NET_SM_OK;
    s_tx_done  = 0U;
    s_ok_tick  = 0U;
    s_sm_t0    = HAL_GetTick();
    s_step_t0  = s_sm_t0;
    resp_reset();
}

int NetSm_IsDone(void)   { return (int)s_done; }
int NetSm_Result(void)   { return s_result; }

const char *NetSm_ResultText(void)
{
    if (s_active == 0U && s_done == 0U) { return "idle"; }
    switch (s_result) {
        case NET_SM_OK:          return "ok";
        case NET_SM_ERR_TIMEOUT: return "timeout";
        case NET_SM_ERR_AT:      return "at error";
        case NET_SM_ERR_CONNACK: return "connack fail";
        case NET_SM_ERR_BUSY:    return "busy";
        default:                 return "unknown";
    }
}

/** 进入下一步（并清空响应缓冲） */
static void next_step(void)
{
    s_step++;
    s_tx_done = 0U;
    s_ok_tick = 0U;
    s_step_t0 = HAL_GetTick();
    resp_reset();

    if (s_step >= s_step_cnt) {
        s_active = 0U;
        s_done   = 1U;
        s_result = NET_SM_OK;
    }
}

static void fail(int code)
{
    s_active = 0U;
    s_done   = 1U;
    s_result = code;
}

int NetSm_Slice(uint32_t budget_ms)
{
    uint32_t slice_t0;
    const AtStep_t *st;

    if (s_active == 0U) {
        /* 没在跑：若从未启动过返回 BUSY，已结束则直接回结果 */
        return s_done ? s_result : NET_SM_ERR_BUSY;
    }

    /* 整条流程超时保护 */
    if ((HAL_GetTick() - s_sm_t0) > NET_SM_TIMEOUT_MS) {
        fail(NET_SM_ERR_TIMEOUT);
        return s_result;
    }

    slice_t0 = HAL_GetTick();
    st = &s_steps[s_step];

    do {
        switch (st->kind) {

        /* ---------- "+++" 退出透传：发完必须静默 1 秒 ---------- */
        case AT_STEP_ESCAPE:
            if (s_tx_done == 0U) {
                /* 原实现发的是 "+++\r\n" 共 3 字节（字符串字面量的 NUL 不发送） */
                uart_send((const uint8_t *)"+++\r\n", 3U);
                s_tx_done = 1U;
            }
            if ((HAL_GetTick() - s_step_t0) >= st->timeout_ms) {
                /* 静默时间到（这里借 timeout_ms 存 1000ms 保护时间） */
                next_step();
            } else {
                return NET_SM_OK;   /* 本片就等在这里，把 CPU 让出去 */
            }
            break;

        /* ---------- 普通 AT 指令：发 → 分片收 → 匹配 ---------- */
        case AT_STEP_CMD:
            if (s_tx_done == 0U) {
                char line[96];
                size_t n = strlen(st->tx);
                if (n > (sizeof(line) - 3U)) { n = sizeof(line) - 3U; }
                memcpy(line, st->tx, n);
                line[n++] = '\r';
                line[n++] = '\n';
                uart_send((const uint8_t *)line, (uint16_t)n);
                s_tx_done = 1U;
            }

            resp_drain();
            {
                int cls = resp_classify(st);

                if (cls == AT_RES_ERROR) {
                    Log_Write(LOG_WARN, "AT error");
                    fail(NET_SM_ERR_AT);
                    return s_result;
                }

                if (cls == AT_RES_OK) {
                    /* 第一次匹配成功时记下时刻；之后的静默时间从这一刻算起 */
                    if (s_ok_tick == 0U) {
                        s_ok_tick = HAL_GetTick();
                    }
                    if ((HAL_GetTick() - s_ok_tick) >= (uint32_t)st->gap_ms) {
                        next_step();
                    } else {
                        return NET_SM_OK;   /* 等静默时间，本片到此为止 */
                    }
                    break;
                }

                /* 尚未出结果：检查该步是否超时 */
                if ((HAL_GetTick() - s_step_t0) > st->timeout_ms) {
                    Log_Write(LOG_WARN, "AT timeout");
                    fail(NET_SM_ERR_TIMEOUT);
                    return s_result;
                }
            }
            break;

        /* ---------- 只有 OK/ERROR 的简单指令（复用 AT_STEP_CMD 逻辑，语义区分） ---------- */
        case AT_STEP_BLOB:
            if (s_tx_done == 0U) {
                uart_send(mqtt_connect_pkt, (uint16_t)sizeof(mqtt_connect_pkt));
                s_tx_done = 1U;
            }
            if ((HAL_GetTick() - s_step_t0) >= st->gap_ms) {
                next_step();
            } else {
                return NET_SM_OK;
            }
            break;

        /* ---------- 只收：等待 MQTT CONNACK（4 字节，非文本） ---------- */
        case AT_STEP_RECV_AT:
        {
            if (s_tx_done == 0U) {
                /* 本步开始前清一次，丢掉 CIPMODE/CIPSEND 的残留回显。
                   注意 ">" 提示符可能只剩一部分没被 drain 掉，所以下面
                   仍用"滑动扫描"而不是"按固定偏移读 4 字节"。 */
                resp_reset();
                s_tx_done = 1U;
            }

            /* 滑动扫描：每次看清缓冲头部 4 字节，不匹配就丢掉 1 个字节继续找。
             * 这样对前面残留的 ">"、"\r\n"、模块杂音都免疫；匹配成功才消费。 */
            if (Command_GetLength() >= 4U) {
                uint8_t b0, b1, b2, b3;
                (void)RingBuf_Peek(0U, &b0);
                (void)RingBuf_Peek(1U, &b1);
                (void)RingBuf_Peek(2U, &b2);
                (void)RingBuf_Peek(3U, &b3);

                if (b0 == 0x20U && b1 == 0x02U && b3 == 0x00U) {
                    RingBuf_Advance(4U);
                    /* 连接成功：立刻补发 SUBSCRIBE（QoS1，主题 "stm32"） */
                    uart_send(mqtt_sub_pkt, (uint16_t)sizeof(mqtt_sub_pkt));
                    s_active = 0U;
                    s_done   = 1U;
                    s_result = NET_SM_OK;
                    return s_result;
                }

                /* 不是 CONNACK 头：丢掉 1 个字节，整体前移继续找 */
                RingBuf_Advance(1U);
            }

            if ((HAL_GetTick() - s_step_t0) > st->timeout_ms) {
                fail(NET_SM_ERR_CONNACK);
                return s_result;
            }
            break;
        }

        default:
            fail(NET_SM_ERR_AT);
            return s_result;
        }

        /* 某一步在片内完成后，重新取当前步继续（可能连着推进）
           但要受时间预算约束 */
        if (s_active == 0U) {
            return s_result;
        }
        st = &s_steps[s_step];

    } while ((HAL_GetTick() - slice_t0) < budget_ms);

    return NET_SM_OK;
}
