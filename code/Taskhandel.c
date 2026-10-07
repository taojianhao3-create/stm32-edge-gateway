#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
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
#include "app_tasks.h"   /* 本文件定义了这 5 个任务函数 */
#include "app_data.h"    /* Data_t + 看门狗相关的 DOG_BIT_xxx / APP_IWDG_TIMEOUT_S 宏 */
#include "bsp_watchdog.h"/* AppDogCheckIn / AppDogTryRefresh */
#include "net_sm.h"      /* 分步非阻塞的连接状态机（替代阻塞式 MQTT_Init/RE_MQTT_Init） */
#include "bsp_flash.h"
#include "flash.h"


// 空闲熄屏相关
TickType_t idle_tick;
uint8_t lcd_bl_en = 1; //1亮屏 0熄屏
uint8_t power_on_lock = 1;
TickType_t power_tick;
lv_obj_t *main_scr;

 uint8_t fail_count = 0; // 记录连续失败次数
 uint8_t count;
QueueHandle_t     data_queue;
SemaphoreHandle_t uart1_mutex;

QueueHandle_t lvgl_data_queue;   
uint32_t last_ping = 0;

extern IWDG_HandleTypeDef hiwdg;
 volatile uint8_t g_mqtt_connected;

// ==============================================
// Modbus 采集
// ==============================================
//void Modbus(void)
//{
//    Data_t data;				//结构体变量

//    while(1)
//    {
//        Modbus_Get_Temp(&data.temp, &data.shi);		//采集温湿度            
//				xQueueOverwrite(data_queue, &data);					// 永远只保留最新一次采集结果，旧值被直接覆盖 该队列长度为1
//        vTaskDelay(500);
//    }
//}

void Modbus(void)
{
    while(1)
    {
			
        Modbus_Poll_AllDev();					//读到的数据赋值给结构体数组
        
        Modbus_Upload_Json();					//将采集到的数据赋值给结构体 然后写入队列

        AppDogCheckIn(DOG_BIT_MODBUS);			//看门狗签到：本任务跑完一轮

        vTaskDelay(pdMS_TO_TICKS(800)); //秒一轮轮询所有设备（时间短点测数据就更灵敏）
    }
}

/*补传绿灯亮 正常红灯亮 离线存储黄灯亮*/
/*
正常在线：先补传历史数据，再发送实时数据，通知 UI 更新。

断网期间：把数据存入 SPI Flash，等待恢复。

重连后：自动补传缓存数据（每次最多 5 条，分批补传）。

从数组里获取数据保证数据是最新的 
*/
void MQTT(void)
{
    vTaskDelay(5000);
    Data_t data_q;
    while (1)
    {
				 
		//	xSemaphoreTake(uart1_mutex, portMAX_DELAY);				//获取锁
			
			xQueueReceive(data_queue, &data_q,portMAX_DELAY);
			

            if (g_mqtt_connected)			//在线
            {
                char cached[512];
                uint16_t len;
                int count = 0;

                /* ------------------------------------------------------------------
                 * 补传：临界区只包住"一帧"的发送
                 *
                 * 原来是把 uart1_mutex 拿在手上，然后在锁【内部】vTaskDelay(800)
                 * 逐条补传 5 条 —— 而且 MQTT_PublishJson() 里还藏着一个
                 * vTaskDelay(500)，于是每补传一条锁被持有约 1.3 秒，
                 * 5 条就是约 6.5 秒。
                 *
                 * 后果：AT 任务（10 秒心跳）如果来拿这把锁，最多要等 6.5 秒，
                 * 心跳节拍被拖得忽长忽短，严重时一轮心跳要 7.5 秒。
                 *
                 * 现在改成"锁内只发、锁外等"：每次只把一帧的发送包在临界区里，
                 * 帧与帧之间的间隔留在锁外，锁的持有时间从 6.5 秒降到几十毫秒。
                 * ------------------------------------------------------------------ */
                while (FlashCache_HasData() && count < 5)
                {
                    if (FlashCache_Read(cached, &len) == 0)		/*从读指针位置读一条缓存数据*/
                    {
                        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);

                        xSemaphoreTake(uart1_mutex, portMAX_DELAY);	/* 锁内只有发送 */
                        MQTT_PublishJson(cached);
                        FlashCache_MarkSent();			//只改魔数，不擦除、改魔数标记已补传
                        xSemaphoreGive(uart1_mutex);

                        count++;
                        if (count > 4)
                            Log_Write(LOG_INFO, "BUCHUAN Finish");

                        vTaskDelay(pdMS_TO_TICKS(800));	/* ← 间隔移到锁外，让别人能拿到锁 */
                    }
                    else
                    {
                        my_clear();		//补传五次数据完成后 清空缓存 让读地址等于写地址
                        break;
                    }
                }

                // 发实时数据（用从 data_queue 取到的那一帧）
                xSemaphoreTake(uart1_mutex, portMAX_DELAY);		/* 锁内只有发送 */
                MQTT_SendAllDev_NoWait(&data_q);
                xSemaphoreGive(uart1_mutex);					//释放锁

                HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
            }
            else	//断线中
            {
                // 离线缓存
                char json[128];
                sprintf(json, "{\"temperature\":%.1f,\"shidu\":%.1f,\"light\":%.0f}", data_q.temp, data_q.shi,data_q.light);
							HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
                FlashCache_Write(json, strlen(json));
            }

        /* 看门狗签到：本轮（在线补传+实时发送，或离线缓存）已完成。
         * 放在 vTaskDelay(2000) 之前，确保"干了活"才签到——
         * 如果本任务卡在队列等待或补传循环里，就不会走到这里，看门狗会复位。 */
        AppDogCheckIn(DOG_BIT_MQTT);

        vTaskDelay(2000);
    }
}


/*
每10秒发心跳检测 然后读心跳应答 看是否正常
连续3次心跳应答错误或失败则判断离线
离线后 每10秒执行重连函数直到连接成功
*/
void AT(void)
{
    uint8_t heart_fail = 0;  // 心跳连续失败次数
    uint8_t need_boot  = 1;  // 首次进入先跑"开机建链"流程

    for (;;)
    {
        /* ==================================================================
         * 阶段 A：连接流程（开机建链 / 断线重连）
         *
         * 这里替换了原来的阻塞式 MQTT_Init() / RE_MQTT_Init()。原实现单次调用
         * 最坏要 26.5~31.5 秒（一串 5 秒级的 AT 超时 + 多处 vTaskDelay），
         * 带来三个问题：占死 AT 任务、把看门狗超时顶到 32 秒以上（超过 IWDG
         * 在标称 LSI 下 32.8 秒的硬件上限）、长时间占着串口锁。
         *
         * 现在改成 NetSm_* 状态机，每个 slice 最多约 50ms：
         *   - 每片前后 take/give uart1_mutex（片内要写 huart1）；
         *   - 每片结束都 AppDogCheckIn()，于是"最长静默时间"从 26 秒降到 50ms；
         *   - 期间 MQTT 任务能正常拿到串口锁发数据。
         * ================================================================== */
        if (need_boot != 0U || g_mqtt_connected == 0U)
        {
            if (need_boot != 0U) {
                NetSm_StartBoot();      /* 首轮：开机建链（不重启模块） */
            } else {
                NetSm_StartReconnect(); /* 断线：重连（含两次 AT+RST） */
            }
            need_boot = 0U;

            while (NetSm_IsDone() == 0) {
                xSemaphoreTake(uart1_mutex, portMAX_DELAY);
                (void)NetSm_Slice(NET_SM_SLICE_MS);
                xSemaphoreGive(uart1_mutex);

                AppDogCheckIn(DOG_BIT_AT);      /* 每片都签到 → 看门狗能精确覆盖 */
                vTaskDelay(1);                  /* 让出 CPU，别把低优先级任务挤住 */
            }

            if (NetSm_Result() == NET_SM_OK) {
                g_mqtt_connected = 1;
                heart_fail       = 0;
                Log_Write(LOG_INFO, "CONNECT FINISH");
            } else {
                g_mqtt_connected = 0;
                Log_Write(LOG_WARN, NetSm_ResultText());
            }
        }

        /* ==================================================================
         * 阶段 B：等到下一个心跳周期
         *
         * 注意这里【不签到】：本任务 99% 的时间就睡在这个 10 秒延时上，
         * 若在此处签到，看门狗就永远发现不了"AT 任务卡死"。
         * 所以签到只发生在"确实完成了一轮工作"之后（心跳往返 / 连接流程每片）。
         * ================================================================== */
        vTaskDelay(10000);

        /* ==================================================================
         * 阶段 C：一次心跳往返
         *
         * 临界区划分原则：uart1_mutex 保护的是 USART1 的【发送】通道——
         * 防止 AT 的 PINGREQ 与 MQTT 的 PUBLISH 在串口上交织成垃圾报文。
         *
         *   ✅ 必须持锁：MQTT_SendPing()（往 huart1 写字节）
         *   ❌ 不该持锁：RingBuf_Clear()（只动环形缓冲下标）、
         *               pal_tcp_recv_raw()（只从环形缓冲读，且只有本任务会读）
         *
         * 原来 pal_tcp_recv_raw() 最长要等 1 秒（PINGRESP 超时）却被包在锁里，
         * 导致 MQTT 那一秒内发不出任何数据。现在等待应答在锁外。
         * ================================================================== */

        /* 清空环形缓冲：把 readIndex / writeIndex 都置 0。
         * 必须包在临界区里 —— RingBuf_Clear() 只是两条赋值，而 writeIndex 会被
         * DMA 空闲中断（HAL_UARTEx_RxEventCallback → Command_Write）随时改写；
         * 若两条赋值之间插入一次中断写，会出现"readIndex 已清零、writeIndex
         * 却非零"的瞬间不一致，把新到的字节算成旧数据。
         * 这里用 taskENTER_CRITICAL() 而非 vTaskSuspendAll()，因为本函数只在
         * AT 任务里调用（纯任务上下文），而目标包含"防中断插进两条赋值中间"。 */
        taskENTER_CRITICAL();
        RingBuf_Clear();
        taskEXIT_CRITICAL();

        if (g_mqtt_connected)
        {
            // 发送 PINGREQ（需要锁：这是真正的串口发送）
            xSemaphoreTake(uart1_mutex, portMAX_DELAY);
            MQTT_SendPing();
            xSemaphoreGive(uart1_mutex);

            // 等待 PINGRESP（1 秒超时）—— 放在锁外，期间 MQTT 任务可正常发数据
            uint8_t pong[2];
            int ret = pal_tcp_recv_raw(0, pong, 2, pdMS_TO_TICKS(1000));

            /* 看门狗签到：完成一次心跳往返（含超时失败）就算本轮完成。
             *
             * 签在这里而不是循环末尾，是刻意的：
             * 循环末尾那个 vTaskDelay(10000) 占据了本任务 99% 的时间，
             * 若在那里签到，看门狗就永远发现不了"AT 任务卡死"。
             * 而连接流程（阶段 A）耗时较长，那里是按 slice 逐片签到的。
             *
             * 这样安排的效果：
             *   - 在线时：心跳是每轮必经的短路径，它卡住就会被看门狗发现 ✔
             *   - 建链/重连时：状态机每 50ms 签一次，不会误复位 ✔ */
            AppDogCheckIn(DOG_BIT_AT);

            if (ret>=2&& pong[0] == 0xD0 && pong[1] == 0x00)
            {
                // 心跳正常
                heart_fail = 0;
            }
            else							// 心跳超时或数据错误
            {
                /* 连续 3 次失败（约 30 秒）判定断网。
                 *
                 * 原来这里外面还套了一层 if(connect==0)：那个 connect 标志由
                 * 阻塞式 RE_MQTT_Init() 在重连成功时置 1，用来避免"刚重连上就
                 * 被旧计数判死"。现在重连由 net_sm 状态机负责、成功时会把
                 * heart_fail 清零（见阶段 A），这层保护已无必要，一并删除。 */
                heart_fail++;
                if (heart_fail >= 3)  // 连续 3 次失败，约 30 秒
                {
                    g_mqtt_connected = 0;					//断网
										Log_Write(LOG_WARN, "MQTT disconnect");
										HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
										HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
                    heart_fail = 0;
                }
            }
        }
				else
				{
                /* 离线：这里【不再】做重连。
                 * 重连是一个 26 秒量级的阻塞过程，放在这里会把心跳路径拖长、
                 * 让看门狗超时无法收紧。现在统一由循环开头的阶段 A
                 * （NetSm_StartReconnect + 按 slice 推进）处理，
                 * g_mqtt_connected 置 0 后下一轮自然会走那条路径。 */
				}
    }
}


// ==============================================
// UI 显示
// ==============================================
void UI(void)
{
    Data_t disp_data;

    LCD_Init();
    TP_Init();
    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();

    main_scr = page_main_create();		//屏幕界面
    lv_scr_load(main_scr);				//加载界面

    uint32_t start_tick = xTaskGetTickCount();
    
    uint8_t  last_status = 0xFF;      // 记录上一次的 MQTT 状态
    uint32_t last_uptime  = 0xFFFFFFFF; // 记录上一次的运行时间    魔法数
		
		uint32_t log_tick =0; //定义局部计时变量
	
	while (1)				//任务函数 不会退出该函数 故局部变量不会被清空
    {
		
				// 空闲10s无动作 → 息屏
        if(lcd_bl_en == 1 && (xTaskGetTickCount() - idle_tick >= pdMS_TO_TICKS(10000)))
        {
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
            lcd_bl_en = 0;
        }

        // 数据变化 则亮屏 + 重置计时   
        if (xQueueReceive(lvgl_data_queue, &disp_data, 0) == pdPASS)
        {
            page_main_update_data(disp_data.temp, disp_data.shi,disp_data.light);

        }

        // MQTT连接状态变更也刷新亮屏
        if (g_mqtt_connected != last_status)
        {
            last_status = g_mqtt_connected;
						page_main_update_status(g_mqtt_connected);				//更新ui状态显示
					
            idle_tick = xTaskGetTickCount();
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
            lcd_bl_en = 1;
        }

				
				//更新运行时间
        uint32_t uptime = (xTaskGetTickCount() - start_tick) / configTICK_RATE_HZ;			//秒数
        if (uptime != last_uptime)
        {
            last_uptime = uptime;
            page_main_update_uptime(uptime);					//更新ui时间显示
        }
				
        lv_timer_handler();			//5毫秒刷新函数

        AppDogCheckIn(DOG_BIT_UI);	//看门狗签到：本任务跑完一轮

        vTaskDelay(5);
    }
}


/* ============================================================================
 * 看门狗任务（签到制喂狗）
 *
 * 与原来的区别：原来无条件 HAL_IWDG_Refresh()，只能发现"整个系统停摆"，
 * 发现不了"单个任务死掉"——而后者才是 RTOS 最常见的故障。
 *
 * 现在改为：每个关键任务跑完一轮就签上自己那一位（AppDogCheckIn），
 * 本任务只在"所有位都齐了"时才喂狗（AppDogTryRefresh 返回 1）；
 * 只要有任务卡住，就不喂，让 IWDG 到期把整机复位。
 *
 * 超时值：APP_IWDG_TIMEOUT_S = 12 秒（在 app_data.h 里定义，依据是最慢的
 * AT 任务约 11 秒一轮）。该超时由 AppWatchdog_Calibrate() 按实测 LSI 频率
 * 精确设定，所以"12 秒"是可信的，不像原来"20 秒"实际可能是 20~38 秒。
 * ========================================================================== */
void Dog_task()
{
	while(1)
	{
		AppDogTryRefresh();			//所有任务都签到过才喂狗，否则等 IWDG 复位
		vTaskDelay(pdMS_TO_TICKS(APP_IWDG_REFRESH_MS));
	}
}
