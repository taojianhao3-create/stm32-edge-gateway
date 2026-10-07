#include "modbus.h"
#include "stm32f4xx.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "stm32f4xx_hal_def.h"
#include "FreeRTOS.h"
#include "task.h"
#include "flash.h"
#include "app_data.h"   /* Data_t：写入 data_queue / lvgl_data_queue */
/* ================================================================
 *	查表法
 *  CRC 高字节表 (高 8 位部分)
 *  由多项式 0x8005 预计算生成，共 256 项
 * ================================================================ */
 
 extern QueueHandle_t     data_queue;
 extern QueueHandle_t     lvgl_data_queue;   /* UI 队列：由 Modbus_Upload_Json 投递 */
 extern UART_HandleTypeDef huart2;

static const uint8_t auchCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

static const uint8_t auchCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};



/****************************************************手敲*************************************************************/
/*CRC16校验*/

/*
一般情况 低地址在前 小端排序（低位字节在低地址）低字节在前  大端排序 高字节在前
*/
uint16_t CRC_16_MODBUS(uint8_t *data,uint8_t len,uint8_t flag)
{
	if(flag)
	{
		/*按位循环法*/
		uint16_t crc=0xffff;
		while(len--)
		{
			crc^=*data++;
			for(int i=0;i<8;i++)
			{
				if(crc &1)
				{
					crc=(crc>>1)^0xA001;
				}
				else
				{
					crc>>=1;
				}
			}
		}
		return crc;
	}
	else
	{
		/*查表法*/
		uint8_t CRCL=0xff;
		uint8_t CRCH=0xff;
		uint8_t index=0;
		while(len--)
		{
			index=CRCL^*data++;
			CRCL=CRCH^auchCRCHi[index];
			CRCH=auchCRCLo[index];
		}
		return (CRCH<<8)|CRCL;
	}
}


// 结构体数组 多从机配置：地址、功能码、起始寄存器、寄存器数量、错误计数、数据
 ModDev_t dev_list[]={
    {0x01,0x03,0x0001,2,0,0,0}, //1#温湿度
    {0x02,0x03,0x0002,2,0,0,0} //2#光照
};
uint8_t dev_cnt = sizeof(dev_list)/sizeof(ModDev_t);
#define MAX_ERR_CNT 5   //连续失败5次临时跳过该设备

/* 等待 USART2 发送完成的最长时间。
 * 9600bps 下一个字节 = 10 位 ≈ 1.04ms，最长帧 8 字节 ≈ 8.3ms；
 * HAL_UART_Transmit() 返回时最后一个字节已进移位寄存器，还差 ≤1.04ms 移出；
 * 这里给 10ms：够覆盖 8 字节帧 + 余量，又不会在串口异常时死等。 */
#define RS485_TC_TIMEOUT_MS  10U

/**
 * @brief  等待 USART2 最后一个字节真正发送完毕（TC：Transmission Complete）
 *
 * @note 为什么必须等 TC，而不是靠 vTaskDelay(1)：
 *
 *   HAL_UART_Transmit() 的返回条件是 TXE（发送数据寄存器空），**不是** TC
 *   （发送完成）。函数返回时，最后一个字节可能还在移位寄存器里往外移，
 *   此时立刻拉低 RS485 的 DE（切回接收）会导致：
 *     1. 发送器进入高阻，A/B 差分电平塌陷 → 总线上出现非法电平；
 *     2. 最后一个字节被截断 → 从机收到的 Modbus 帧 CRC 必然错误，不予应答。
 *
 *   典型表现是"偶发通信失败 / CRC 错"，且随 tick 相位随机出现，极难排查。
 *   原来的 vTaskDelay(1) 属于碰运气：它等 0~1 个 tick（1kHz 下约 0~1ms 抖动），
 *   而 9600bps 下最后一字节需要 1.04ms 移出 —— 两者同量级，边界情况必然失败。
 *   等 TC 标志则是确定性的。
 */
static void RS485_WaitTxComplete(void)
{
    uint32_t t0 = HAL_GetTick();
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET)
    {
        if ((HAL_GetTick() - t0) > RS485_TC_TIMEOUT_MS)
            break;      /* 超时兜底：宁可丢一帧，也不能把 Modbus 任务卡死 */
    }
}



/**
 * @brief 通用Modbus03读寄存器   采集数据
 * @param addr:从机地址 func:功能码 reg:起始地址 regcnt:寄存器数量 buf:接收缓存
 * @retval 0失败 1成功
*/
uint8_t Modbus_Read_Dev(uint8_t addr,uint8_t func,uint16_t reg,uint16_t regcnt,uint16_t *buf)
{
    uint8_t tx[8];
    uint8_t rx[32];
    uint8_t count=0;
    uint16_t crc=0;

    //组问询帧
    tx[0] = addr;
    tx[1] = func;
    tx[2] = (reg>>8)&0xFF;
    tx[3] = reg&0xFF;
    tx[4] = (regcnt>>8)&0xFF;
    tx[5] = regcnt&0xFF;

    crc=CRC_16_MODBUS(tx,6,0);
    tx[6]=crc&0xFF;
    tx[7]=(crc>>8)&0xFF;

    memset(rx,0,sizeof(rx));
	      // 关键：清空UART接收标志，清除上一轮残留
    __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE | UART_FLAG_ORE);
	
    RS485_TX;
    HAL_UART_Transmit(&huart2,tx,8,1000);
    RS485_WaitTxComplete();		/* 等最后一个字节真正发完，再放开总线 */
    RS485_RX;

    uint32_t start=HAL_GetTick();
    uint32_t upd=HAL_GetTick();
    count=0;
    //沿用你原有阻塞轮询接收逻辑完全不变
    while(1)
    {
        if(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_RXNE))
        {
            uint8_t ch=huart2.Instance->DR&0xFF;
            if(count<sizeof(rx)) rx[count++]=ch;
            upd=HAL_GetTick();
        }
        if(count>0 && (HAL_GetTick()-upd)>50) break;			//帧间隔3.5个字符 4ms左右
        if(HAL_GetTick()-start>800) break;								//绝对超时
        vTaskDelay(1);
    }

    //校验长度+地址+CRC
    uint8_t data_len = rx[2];			//数据字节数
    uint16_t crc_calc = CRC_16_MODBUS(rx,count-2,1);
    if(count == (3+data_len+2) && rx[0]==addr && rx[1]==func
        && (rx[count-2]==(crc_calc&0xFF)) && (rx[count-1]==((crc_calc>>8)&0xFF)))
    {
        //数据转存到buf
        for(uint8_t i=0;i<regcnt;i++)					//一个寄存器是两个字节 一个数据
        {
            buf[i] = (rx[3+i*2]<<8)|rx[4+i*2];
        }
        return 1;
    }
    return 0;
}




void Modbus_Poll_AllDev(void)
{
    uint16_t tmp_buf[8];
    for(uint8_t i=0;i<dev_cnt;i++)		//dev_cnt数组元素个数
    {
        ModDev_t *p = &dev_list[i];
			 memset(tmp_buf, 0, sizeof(tmp_buf)); // 新增：每次读取前清空缓存
        //连续错误超限，跳过本次轮询
//        if(p->err_cnt >= MAX_ERR_CNT)
//        {
//            continue;
//        }

        //发起读取
        if(Modbus_Read_Dev(p->dev_addr,p->func,p->reg_start,p->reg_num,tmp_buf)==1)
        {
         //   p->err_cnt = 0; //成功清零错误计数
            //根据从机地址解析数据
            switch(p->dev_addr)
            {
                case 0x01: //温湿度
                    p->val1 = tmp_buf[0]/10.0f;
                    p->val2 = tmp_buf[1]/10.0f;
                    break;
                case 0x02: //光照
                    p->val1 = ((uint32_t)tmp_buf[0]<<16 | tmp_buf[1])/1000;
                    p->val2 = 0;
                    break;
//                case 0x03: //继电器状态
//                    p->val1 = tmp_buf[0];
//                    p->val2 = 0;
//                    break;
                default:break;
            }
        }
//        else
//        {
//            p->err_cnt++;
////            if(p->err_cnt == MAX_ERR_CNT)
////            {
////             //   Log_Write(LOG_ERROR,"Mod dev%d offline suspend");
////            }
//        }
        vTaskDelay(pdMS_TO_TICKS(30)); //单从机间隔防总线干扰					重点时间不能过大！！！！！！！
    }
}




/* 将采集到的数据赋值给结构体，然后投递给两个消费者：
 *
 *   data_queue       → MQTT 任务（在线时上报，离线时写 Flash 缓存）
 *   lvgl_data_queue  → UI 任务（上屏显示）
 *
 * 两条路都从这里出发是刻意的：
 *   1) 快照一致性：先在一个 Data_t 里取齐 3 个值，再分别投递，
 *      避免"UI 读到本轮、MQTT 读到下一轮"的撕裂；
 *   2) 断网时 UI 仍能刷新：原来 lvgl_data_queue 由 MQTT 任务写，
 *      而断网分支根本不调上报函数，导致屏幕冻结在最后一帧，
 *      尽管 Modbus 还在正常采集。
 *   3) 职责清晰：采集 → 分发给 UI 与云端，而不是"UI 的数据要等云端发完才有"。
 *
 * 两个队列长度都是 1，用 xQueueOverwrite 覆盖写：只保留最新一帧，永不阻塞。 */
void Modbus_Upload_Json(void)
{
    Data_t uidata;

    /* 固定在同一个时刻取齐三个值 */
    uidata.temp  = dev_list[0].val1;
    uidata.shi   = dev_list[0].val2;
    uidata.light = dev_list[1].val1;

    xQueueOverwrite(data_queue,      &uidata);   /* → MQTT 任务 */
    xQueueOverwrite(lvgl_data_queue, &uidata);   /* → UI 任务   */
}


/**
 * @brief  把一帧数据组装成 MQTT PUBLISH 报文并发送（不等应答）
 * @param  d 要上报的数据帧
 *
 * @note 数据来源从"全局数组 dev_list[]"改成"参数传入"，是这次重构的关键：
 *       原来这里直接读 dev_list[] 拼报文，并在拼的过程中顺手写
 *       lvgl_data_queue 给 UI。那样做的后果是
 *         - UI 的刷新时机被绑在"MQTT 是否在线"上（断网即冻结）；
 *         - 数据来源隐式依赖全局数组，与 data_queue 传的帧可能不同步。
 *       现在它只负责"把给定的一帧发出去"，UI 队列由 Modbus 任务统一投递。
 *
 * @note 报文用 QoS0（固定报头 0x32 的低 2 位为 0），不等 PUBACK。
 */
void MQTT_SendAllDev_NoWait(const Data_t *d)
{
    char payload[256];
    int  p;

    if (d == NULL) {
        return;
    }

    /* JSON 用大括号包住所有字段，字段间用逗号分隔。
     * （原实现是"每个字段自带花括号、靠回退一个字符来闭合"，
     *   字段顺序一变就会产生非法 JSON；这里改成结构化的拼法。） */
    p  = sprintf(payload, "{");
    p += sprintf(payload + p, "\"temp\":%.1f,",  (double)d->temp);
    p += sprintf(payload + p, "\"humi\":%.1f,",  (double)d->shi);
    p += sprintf(payload + p, "\"light\":%.0f}", (double)d->light);

    /* 发布报文 */
    uint8_t buf[256];
    int pos = 0;
    buf[pos++] = 0x32;
    uint16_t tlen=5;
    uint16_t mlen=strlen(payload);
    uint16_t rem=2+tlen+2+mlen;
    do{
        uint8_t b=rem%128;
        rem/=128;
        if(rem>0)b|=0x80;
        buf[pos++]=b;
    }while(rem>0);
    buf[pos++]=0;buf[pos++]=tlen;
    memcpy(&buf[pos],"stm32",5);pos+=5;
    buf[pos++]=0;buf[pos++]=1;
    memcpy(&buf[pos],payload,mlen);pos+=mlen;
    HAL_UART_Transmit(&huart1,buf,pos,5000);
    vTaskDelay(10);
}

/*																	USART2
手动轮询RXNE：不依赖HAL库的阻塞接收或中断接收，零延迟读取每个字节。

双重超时保护：50ms帧间隔（Modbus标准3.5字符/4ms）和800ms绝对超时，保证不卡死。

vTaskDelay(1)：在FreeRTOS下主动让出CPU，保证UI和MQTT任务不被饿死。

没有开启串口中断接收 检查状态寄存器 有数据就读DR 读完后状态会变为0
*/
int Modbus_Get_Temp(float *temp,float *shi)
{
	uint8_t tx[8];
	uint8_t rx[20];
	uint8_t count=0;
	uint16_t crc=0;
	
	/*问询帧*/
	tx[0]=0x01;
	tx[1]=0x03;
	tx[2]=0x00;
	tx[3]=0x01;
	tx[4]=0x00;
	tx[5]=0x02;
	
	/*小端排序 低字节在前 （低字节在低地址 ，低地址默认在前）*/
	crc=CRC_16_MODBUS(tx,6,0);
	tx[6]=crc &0xff;
	tx[7]=(crc>>8)&0xff;
	
	memset(rx,0,sizeof(rx));
	RS485_TX;
	HAL_UART_Transmit(&huart2,tx,sizeof(tx),1000);			//发送问询帧
	RS485_WaitTxComplete();		/* 等最后一个字节真正发完，再放开总线 */
	
	RS485_RX;				//切换为接收模式
	
	uint32_t start_time=HAL_GetTick();
	uint32_t up_time=HAL_GetTick();
	
	while(1)
	{
		if(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_RXNE))
		{
			uint8_t ch=huart2.Instance->DR &0xff;		//DR寄存器有数据时 状态寄存器为1  读取DR寄存器的值 状态清零
			if(count<sizeof(rx))
			{
				rx[count++]=ch;
				up_time=HAL_GetTick();
			}
		}
		if(count>0 && (HAL_GetTick()-up_time)>50)					//帧与帧间隔3.5个字符 4ms
		{
			break;
		}
		if(HAL_GetTick()-start_time>800)										//绝对超时 不卡死
		{
			break;
		}
		vTaskDelay(1);						//释放CPU防止其他任务饿死
	}
	
	uint16_t crc_re=CRC_16_MODBUS(rx,count-2,1);
	if(count==9 && rx[0]==0x01 && rx[1]==0x03 && rx[2]==0x04 && rx[7]==(crc_re& 0xff) && rx[8]==((crc_re>>8)& 0xff))
	{
		uint16_t t=(rx[3]<<8)|rx[4];
		uint16_t s=(rx[5]<<8)|rx[6];
		*temp=t/10.0;
		*shi=s/10.0;
		return 1;
	}
	return 0;
}
