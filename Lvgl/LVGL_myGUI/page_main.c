#include "lvgl.h"
#include "flash.h" 
#include "stdio.h"
#include "stm32f4xx.h"
// 字体声明
extern lv_font_t font_chinese_9;


lv_obj_t *bar_light;
lv_obj_t *label_light;

lv_obj_t *page_main_create(void);
void page_log_refresh(void);
/* 全局控件 */
static lv_obj_t *label_temp;
static lv_obj_t *label_humi;
static lv_obj_t *label_status;
static lv_obj_t *label_uptime;
static lv_obj_t *bar_temp;
static lv_obj_t *bar_humi;
static lv_obj_t *btn_test;  // 按钮句柄

//=====新增日志页面全局控件=====
static lv_obj_t *scr_log = NULL;    //日志屏幕
static lv_obj_t *list_log = NULL;   //日志列表
static lv_obj_t *btn_back;         //日志页返回按钮
//static lv_obj_t *scr_main = NULL;    //日志屏幕

extern lv_obj_t *main_scr;
static void btn_back_click(lv_event_t *e);
void page_main_update_status(uint8_t connected);
extern  uint32_t log_write_offset ;

lv_obj_t *page_log_create(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);				//NULL 创建独立屏幕
    lv_obj_set_style_bg_color(scr,lv_color_hex(0xF8F9FA),0);
    lv_obj_clear_flag(scr,LV_OBJ_FLAG_SCROLLABLE);			//禁用 scr 这个 LVGL 屏幕对象的滚动功能

    //顶部返回按钮
    btn_back = lv_btn_create(scr);
    lv_obj_set_size(btn_back,65,35);
    lv_obj_align(btn_back,LV_ALIGN_TOP_LEFT,5,5);
    lv_obj_set_style_bg_color(btn_back,lv_color_hex(0x0F3460),0);
	
    lv_obj_t *lab_back = lv_label_create(btn_back);
    lv_label_set_text(lab_back,"返回");
		lv_obj_align(lab_back, LV_ALIGN_CENTER, 0, 0); 
    lv_obj_set_style_text_font(lab_back,&font_chinese_9,0);
    lv_obj_add_event_cb(btn_back,btn_back_click,LV_EVENT_CLICKED,NULL);	//事件回调

    //页面标题
    lv_obj_t *title_log = lv_label_create(scr);
    lv_label_set_text(title_log,"系统运行日志");
    lv_obj_align(title_log,LV_ALIGN_TOP_MID,0,8);
    lv_obj_set_style_text_font(title_log,&font_chinese_9,0);

    //日志列表（可滚动）
    list_log = lv_list_create(scr);
    lv_obj_set_size(list_log,lv_obj_get_width(scr)-10,lv_obj_get_height(scr)-60);
    lv_obj_align(list_log,LV_ALIGN_BOTTOM_MID,0,-5);
    lv_obj_set_style_radius(list_log,6,0);

    return scr;
}

//日志按钮事件回调
static void btn_click_jump_log(lv_event_t *e)
{
    if(scr_log == NULL)
        scr_log = page_log_create();
    lv_scr_load(scr_log);			//加载日志页面
    page_log_refresh(); //仅切页面刷新1次日志
}

//日志页【返回】按钮回调
static void btn_back_click(lv_event_t *e)
{

	lv_scr_load(main_scr);
	 page_main_update_status(g_mqtt_connected);				//返回主页面后更新连接状态
}


/**
 * @brief  创建主监控页面
 * @retval 返回页面根对象
 */
lv_obj_t *page_main_create(void)
{
    // 创建独立屏幕对象，NULL 表示不依附默认屏幕
    lv_obj_t *scr = lv_obj_create(NULL);			

    // 设置屏幕背景色：浅灰蓝
    lv_obj_set_style_bg_color(scr, lv_color_hex(0xE8EEF5), 0);
    // 禁用屏幕整体滑动功能
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);							
    // 关闭滚动条显示
    lv_obj_set_scrollbar_mode(scr, LV_SCROLLBAR_MODE_OFF);				 

    // ===================== 顶部主标题 =====================
    lv_obj_t *title = lv_label_create(scr);                 // 创建文本标签
    lv_label_set_text(title, "边缘网关监控");               // 设置标题文字
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 4);            // 顶部居中，Y偏移4
    lv_obj_set_style_text_color(title, lv_color_hex(0x2D3748), 0); // 文字深灰
    lv_obj_set_style_text_font(title, &font_chinese_9, 0);   // 设置中文字体

    // 定义三张卡片宽度、高度，320屏无缝均分
    const int card_w1 = 106;
    const int card_w2 = 106;
    const int card_w3 = 108;
    const int card_h = 90;

    // 计算卡片组水平居中偏移量
    int group_x = (320 - (card_w1 + card_w2 + card_w3)) / 2;
    // 卡片Y坐标：底部状态栏(32高)上方，预留间距
    int base_y = 240 - 32 - card_h - 42;

    // ===================== 左侧 温度卡片 =====================
    lv_obj_t *card_temp = lv_obj_create(scr);               // 创建卡片容器
    lv_obj_set_size(card_temp, card_w1, card_h);            // 设置卡片尺寸
    lv_obj_align(card_temp, LV_ALIGN_TOP_LEFT, group_x, base_y); // 定位卡片
    lv_obj_clear_flag(card_temp, LV_OBJ_FLAG_SCROLLABLE);   // 禁止卡片内部滑动
    lv_obj_set_style_bg_color(card_temp, lv_color_hex(0xFFFFFF), 0); // 卡片白底
    lv_obj_set_style_border_width(card_temp, 1, 0);         // 边框宽度1px
    lv_obj_set_style_border_color(card_temp, lv_color_hex(0xC9D8E8), 0); // 边框浅灰
    lv_obj_set_style_radius(card_temp, 0, 0);               // 直角卡片

    // 卡片顶部标签：温度
    lv_obj_t *icon_temp = lv_label_create(card_temp);
    lv_label_set_text(icon_temp, "温度");
    lv_obj_align(icon_temp, LV_ALIGN_TOP_MID, 0, 4);         // 靠近卡片顶部
    lv_obj_set_style_text_color(icon_temp, lv_color_hex(0xE53E3E), 0); // 红色文字
    lv_obj_set_style_text_font(icon_temp, &font_chinese_9, 0);

    // 温度数值显示标签
    label_temp = lv_label_create(card_temp);
    lv_label_set_text(label_temp, "--.-C");
    lv_obj_align(label_temp, LV_ALIGN_TOP_MID, 0, 32);
    lv_obj_set_style_text_color(label_temp, lv_color_hex(0xE53E3E), 0);
    lv_obj_set_style_text_font(label_temp, &font_chinese_9, 0);

    // 温度进度条
    bar_temp = lv_bar_create(card_temp);
    lv_obj_set_size(bar_temp, 85, 8);                       // 进度条尺寸
    lv_obj_align(bar_temp, LV_ALIGN_BOTTOM_MID, 0, 0);      // 紧贴卡片底部
    lv_bar_set_range(bar_temp, -10, 60);                    // 进度条量程
    lv_bar_set_value(bar_temp, 0, LV_ANIM_ON);              // 初始值0，开启动画
    lv_obj_set_style_bg_color(bar_temp, lv_color_hex(0xE2E8F0), LV_PART_MAIN); // 轨道底色
    lv_obj_set_style_bg_color(bar_temp, lv_color_hex(0xE53E3E), LV_PART_INDICATOR); // 进度色
    lv_obj_set_style_radius(bar_temp, 2, 0);                // 进度条圆角

    // ===================== 中间 光照卡片 =====================
    lv_obj_t *card_light = lv_obj_create(scr);
    lv_obj_set_size(card_light, card_w2, card_h);
    lv_obj_align(card_light, LV_ALIGN_TOP_LEFT, group_x + card_w1, base_y);
    lv_obj_clear_flag(card_light, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(card_light, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(card_light, 1, 0);
    lv_obj_set_style_border_color(card_light, lv_color_hex(0xC9D8E8), 0);
    lv_obj_set_style_radius(card_light, 0, 0);

    // 卡片顶部标签：光照
    lv_obj_t *icon_light = lv_label_create(card_light);
    lv_label_set_text(icon_light, "光照");
    lv_obj_align(icon_light, LV_ALIGN_TOP_MID, 0, 4);
    lv_obj_set_style_text_color(icon_light, lv_color_hex(0xD69E00), 0); // 黄色文字
    lv_obj_set_style_text_font(icon_light, &font_chinese_9, 0);

    // 光照数值显示标签
    label_light = lv_label_create(card_light);
    lv_label_set_text(label_light, "----");
    lv_obj_align(label_light, LV_ALIGN_TOP_MID, 0, 32);
    lv_obj_set_style_text_color(label_light, lv_color_hex(0xD69E00), 0);
    lv_obj_set_style_text_font(label_light, &font_chinese_9, 0);

    // 光照进度条
    bar_light = lv_bar_create(card_light);
    lv_obj_set_size(bar_light, 85, 8);
    lv_obj_align(bar_light, LV_ALIGN_BOTTOM_MID, 0, 0);      // 紧贴卡片底部
    lv_bar_set_range(bar_light, 0, 20000);                  // 光照量程 0~20000
    lv_bar_set_value(bar_light, 0, LV_ANIM_ON);
    lv_obj_set_style_bg_color(bar_light, lv_color_hex(0xE2E8F0), LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar_light, lv_color_hex(0xD69E00), LV_PART_INDICATOR);
    lv_obj_set_style_radius(bar_light, 2, 0);

    // ===================== 右侧 湿度卡片 =====================
    lv_obj_t *card_humi = lv_obj_create(scr);
    lv_obj_set_size(card_humi, card_w3, card_h);
    lv_obj_align(card_humi, LV_ALIGN_TOP_LEFT, group_x + card_w1 + card_w2, base_y);
    lv_obj_clear_flag(card_humi, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(card_humi, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(card_humi, 1, 0);
    lv_obj_set_style_border_color(card_humi, lv_color_hex(0xC9D8E8), 0);
    lv_obj_set_style_radius(card_humi, 0, 0);

    // 卡片顶部标签：湿度
    lv_obj_t *icon_humi = lv_label_create(card_humi);
    lv_label_set_text(icon_humi, "湿度");
    lv_obj_align(icon_humi, LV_ALIGN_TOP_MID, 0, 4);
    lv_obj_set_style_text_color(icon_humi, lv_color_hex(0x319795), 0); // 青绿色文字
    lv_obj_set_style_text_font(icon_humi, &font_chinese_9, 0);

    // 湿度数值显示标签
    label_humi = lv_label_create(card_humi);
    lv_label_set_text(label_humi, "--.-%");
    lv_obj_align(label_humi, LV_ALIGN_TOP_MID, 0, 32);
    lv_obj_set_style_text_color(label_humi, lv_color_hex(0x319795), 0);
    lv_obj_set_style_text_font(label_humi, &font_chinese_9, 0);

    // 湿度进度条
    bar_humi = lv_bar_create(card_humi);
    lv_obj_set_size(bar_humi, 85, 8);
    lv_obj_align(bar_humi, LV_ALIGN_BOTTOM_MID, 0, 0);      // 紧贴卡片底部
    lv_bar_set_range(bar_humi, 0, 100);                    // 湿度量程 0~100%
    lv_bar_set_value(bar_humi, 0, LV_ANIM_ON);
    lv_obj_set_style_bg_color(bar_humi, lv_color_hex(0xE2E8F0), LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar_humi, lv_color_hex(0x319795), LV_PART_INDICATOR);
    lv_obj_set_style_radius(bar_humi, 2, 0);

    // ===================== 底部状态栏 =====================
    lv_obj_t *footer = lv_obj_create(scr);
    lv_obj_set_width(footer, lv_obj_get_width(scr));         // 宽度铺满整屏
    lv_obj_set_height(footer, 32);                          // 状态栏高度32
    lv_obj_align(footer, LV_ALIGN_BOTTOM_MID, 0, 0);        // 贴屏幕底部
    lv_obj_set_style_bg_color(footer, lv_color_hex(0x121A2C), 0); // 深色背景
    lv_obj_set_style_radius(footer, 0, 0);
    lv_obj_set_style_border_width(footer, 1, 0);
    lv_obj_set_style_border_color(footer, lv_color_hex(0x121A2C), 0);
    lv_obj_clear_flag(footer, LV_OBJ_FLAG_SCROLLABLE);       // 禁止状态栏滑动
    lv_obj_set_scrollbar_mode(footer, LV_SCROLLBAR_MODE_OFF);			//关闭滑动条

    // 左侧：连接状态提示
    label_status = lv_label_create(footer);
    lv_label_set_text(label_status, "● 等待连接...");
    lv_obj_align(label_status, LV_ALIGN_LEFT_MID, 6, 0);
    lv_obj_set_style_text_color(label_status, lv_color_hex(0xE2E8F0), 0);
    lv_obj_set_style_text_font(label_status, &font_chinese_9, 0);

    // 右侧：系统运行时长
    label_uptime = lv_label_create(footer);
    lv_label_set_text(label_uptime, "运行 00:00:00");
    lv_obj_align(label_uptime, LV_ALIGN_RIGHT_MID, -6, 0);
    lv_obj_set_style_text_color(label_uptime, lv_color_hex(0xC9D8E8), 0);
    lv_obj_set_style_text_font(label_uptime, &font_chinese_9, 0);

    // ===================== 右上角 日志按钮 =====================
    btn_test = lv_btn_create(scr);                          // 创建按键
    lv_obj_set_size(btn_test, 65, 35);                      // 按键尺寸
    lv_obj_align(btn_test, LV_ALIGN_TOP_RIGHT, -3, 3);      // 屏幕右上角
    lv_obj_set_style_bg_color(btn_test, lv_color_hex(0x4A90E2), 0); // 蓝色按键
    lv_obj_set_style_radius(btn_test, 4, 0);                // 按键圆角

    // 按钮文字
    lv_obj_t *lab_btn = lv_label_create(btn_test);
    lv_label_set_text(lab_btn, "日志");
		lv_obj_align(lab_btn, LV_ALIGN_CENTER, 0, 0); 
    lv_obj_set_style_text_color(lab_btn, lv_color_white(), 0);
    lv_obj_set_style_text_font(lab_btn, &font_chinese_9, 0);

    // 绑定点击事件回调函数
    lv_obj_add_event_cb(btn_test, btn_click_jump_log, LV_EVENT_CLICKED, NULL);

    // 返回当前页面句柄
    return scr;
}
/*
事件触发→Log_Write→打包Log_Obj→W25Q存入→log_write_offset++
                                  ↓
点击日志按钮→page_log_refresh→从log_write_offset倒序向前读→渲染列表(新在上)

从 ** 日志尾部（即将写入的空白地址）** 往前一条一条读；
读到有效日志立刻加到列表，先读到的最新日志排在最上面；
最多 30 条，旧日志不会被擦除，但超出条数不再显示；
每次打开日志页清空列表，重新倒序刷新。
*/
//刷新日志列表，从flash读取日志      效果：新日志置顶，老日志自动不在列表展示，不用每次全擦分区。
void page_log_refresh(void)
{
    if(list_log == NULL) return;
    lv_obj_clean(list_log);		//清空列表原有所有条目

    Log_Obj log_tmp;
	uint32_t offset = log_write_offset;		//下一条要写入的空白地址，最后一条有效日志在它前面(日志写指针 为空白地址)
    char buf[128];
    const char *level_str;
    uint8_t cnt=0;		//计数，最多只显示 30 条日志

    // 倒着往前读，最多30条
    while(offset >= LOG_OBJ_SIZE && cnt<30)
    {
        offset -= LOG_OBJ_SIZE;					//地址先往前挪一条日志长度，指向当前最后一条有效日志的起始地址
        if(Log_ReadOne(offset,&log_tmp) == 0)
        {
            uint32_t sec = log_tmp.time_stamp / 1000U;
            uint32_t h = sec / 3600U;
            uint32_t m = (sec % 3600U) / 60U;
            uint32_t s = sec % 60U;

            switch(log_tmp.level)
            {
                case LOG_INFO:  level_str="INFO";break;
                case LOG_WARN:  level_str="WARN";break;
                case LOG_ERROR: level_str="ERROR";break;
                default:level_str="UNK";break;
            }
            snprintf(buf,sizeof(buf),"%02u:%02u:%02u %s %s",h,m,s,level_str,log_tmp.content);
            lv_obj_t *item = lv_list_add_text(list_log,buf);
           // lv_obj_set_style_text_font(item,&font_chinese_9,0);
            cnt++;
        }
    }
}


extern TickType_t idle_tick;
extern uint8_t lcd_bl_en ; //1亮屏 0熄屏
// 更新函数
void page_main_update_data(float temp, float humi, float light)
{
    static float temp_last=-999;
    static float humi_last=-999;
    static float light_last=-99999;

		//数据变化才更新
    if(temp!=temp_last || humi!=humi_last || light!=light_last)
    {
        lv_label_set_text_fmt(label_temp, "%.1fC", temp);
        lv_label_set_text_fmt(label_humi, "%.1f%%", humi);
        lv_label_set_text_fmt(label_light, "%.0fL", light);

				//改变进度条的值
        lv_bar_set_value(bar_temp, (int)temp, LV_ANIM_ON);
        lv_bar_set_value(bar_humi, (int)humi, LV_ANIM_ON);
        lv_bar_set_value(bar_light, (int)light, LV_ANIM_ON);

			if((temp-temp_last)>3 || (temp-temp_last)<-3 || (humi-humi_last)>3 || (humi-humi_last)<-3|| (light-light_last)>10 || (light-light_last)<-10)
			{
			//数据改变后亮屏
			      idle_tick = xTaskGetTickCount();
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
            lcd_bl_en = 1;
			}
			
			
        temp_last=temp;
        humi_last=humi;
        light_last=light;
			
    }
}



// 更新连接状态
void page_main_update_status(uint8_t connected)
{
	if(!label_status) return;																									//创建成功为1 label_status 底部部件文本
    if(connected)
    {
        lv_label_set_text(label_status, "在线");
        lv_obj_set_style_text_color(label_status, lv_color_hex(0x4ECDC4), 0);
    }
    else
    {
        lv_label_set_text(label_status, "离线");
        lv_obj_set_style_text_color(label_status, lv_color_hex(0xFF6B6B), 0);
    }
}

// 更新运行时间
void page_main_update_uptime(uint32_t seconds)
{
    if(!label_uptime) return;
    uint32_t h = seconds / 3600;
    uint32_t m = (seconds % 3600) / 60;
    uint32_t s = seconds % 60;
    lv_label_set_text_fmt(label_uptime, "运行 %02d:%02d:%02d", h, m, s);
}


