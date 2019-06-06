/*
 * ESPRSSIF MIT License
 *
 * Copyright (c) 2015 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "esp_common.h"
#include "../include/gpio.h"
#include "espressif/espconn.h"

char IOZT=1;
int  count = 0;

os_timer_t timer_led;
os_timer_t timer_count;
os_timer_t breakHeart;
os_timer_t timer_smartconfig;
struct espconn user_tcp_conn;
struct espconn *pespconn;
extern void uart0_write_string(char *pdata, unsigned short len);
void send_msg_to_server(uint8 *psent,uint16 length)
{
	uint8 returnCode = espconn_sent(pespconn, psent, length);
	if(returnCode == 0)
	{

	}
	else if(returnCode == ESPCONN_ARG)
	{
		espconn_connect(&user_tcp_conn);
	}
}

void stop_smartconfig()
{
	uint8 getState;
	getState = wifi_station_get_connect_status();
	//查询 ESP8266 WiFi station 接口连接 AP 的状态
	if (getState != STATION_GOT_IP)
	{
		smartconfig_stop();
	}
}

void timer_led_callback()
{
	if(IOZT == 1)
	{
		//GPIO_OUTPUT_SET(GPIO_ID_PIN(2), 0);
		os_printf("led on");
		IOZT = 0;
	}
	else
	{
		//GPIO_OUTPUT_SET(GPIO_ID_PIN(2), 1);
		os_printf("led off");
		IOZT = 1;
	}
}

void timer_count_callback()
{
	count++;
}

void timer_callback()
{
	espconn_sent(pespconn, "heart=machine!", strlen("heart=machine!"));
}

void ICACHE_FLASH_ATTR user_tcp_sent_cb(void *arg)  //发送
{
	os_printf("send suceed");
}

void ICACHE_FLASH_ATTR user_tcp_discon_cb(void *arg)  //断开
{
	os_printf("disconnect suceed!");
	espconn_connect((struct espconn *) arg);
}

void ICACHE_FLASH_ATTR user_tcp_recv_cb(void *arg,  //接收
		char *pdata, unsigned short len) {

	os_printf("recv：%s\r\n", pdata);
	uart0_write_string(pdata,len);
}

void ICACHE_FLASH_ATTR user_tcp_recon_cb(void *arg, sint8 err) //注册 TCP 连接发生异常断开时的回调函数，可以在回调函数中进行重连
{
	os_printf("connect error %d\r\n", err);
	os_timer_disarm(&breakHeart);
}

void ICACHE_FLASH_ATTR user_tcp_connect_cb(void *arg)  //注册 TCP 连接成功建立后的回调函数
{
	char yladdr[6];
	char DeviceBuffer[40]={0};
	int keep_alive = 6;
    pespconn = arg;
    espconn_set_opt(arg, ESPCONN_KEEPALIVE);
	espconn_regist_recvcb(arg, user_tcp_recv_cb);  //接收
	espconn_regist_sentcb(arg, user_tcp_sent_cb);  //发送
	espconn_regist_disconcb(arg, user_tcp_discon_cb);  //断开
	wifi_get_macaddr(STATION_IF,yladdr);
	sprintf(DeviceBuffer,MAC_STR",machine!",MAC2STR(yladdr));

	espconn_sent(pespconn, DeviceBuffer, strlen(DeviceBuffer));
	//keep alive checking per 30s

	espconn_set_keepalive(arg, ESPCONN_KEEPIDLE, &keep_alive);
	keep_alive = 5; //repeat interval = 5s
	espconn_set_keepalive(arg, ESPCONN_KEEPINTVL, &keep_alive);
	keep_alive = 2;//repeat 2times
	espconn_set_keepalive(arg, ESPCONN_KEEPCNT, &keep_alive);
	os_timer_arm(&breakHeart, 1000, TRUE);
}

void ICACHE_FLASH_ATTR my_station_init(struct ip_addr *remote_ip,
		struct ip_addr *local_ip, int remote_port)
{
	user_tcp_conn.proto.tcp = (esp_tcp *) os_zalloc(sizeof(esp_tcp));  //分配空间
	user_tcp_conn.type = ESPCONN_TCP;  //设置类型为TCP协议
	memcpy(user_tcp_conn.proto.tcp->local_ip, local_ip, 4);
	memcpy(user_tcp_conn.proto.tcp->remote_ip, remote_ip, 4);
	user_tcp_conn.proto.tcp->local_port = espconn_port();  //本地端口
	user_tcp_conn.proto.tcp->remote_port = remote_port;  //目标端口
	//注册连接成功回调函数和重新连接回调函数
	espconn_regist_connectcb(&user_tcp_conn, user_tcp_connect_cb);//注册 TCP 连接成功建立后的回调函数
	espconn_regist_reconcb(&user_tcp_conn, user_tcp_recon_cb);//注册 TCP 连接发生异常断开时的回调函数，可以在回调函数中进行重连

	//启用连接
	espconn_connect(&user_tcp_conn);
}

void tcp_client_init()
{
	os_timer_disarm(&timer_led);//关闭定时器，相当于清零计时器计数
	//GPIO_OUTPUT_SET(GPIO_ID_PIN(2), 0);
	struct ip_info info;//47, 97, 252, 149
	const char remote_ip[4] = { 192, 168, 1, 100 };//目标IP地址,必须要先从手机获取，否则连接失败.
	wifi_get_ip_info(STATION_IF, &info);	//查询 WiFi模块的 IP 地址
	my_station_init((struct ip_addr *) remote_ip, &info.ip, 1008);//连接到目标服务器的1008端口
}

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 user_rf_cal_sector_set(void)
{
    flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

void ICACHE_FLASH_ATTR
smartconfig_done(sc_status status, void *pdata)
{
    switch(status) {
        case SC_STATUS_WAIT:
        	os_printf("SC_STATUS_WAIT\n");
            break;
        case SC_STATUS_FIND_CHANNEL:
        	os_printf("SC_STATUS_FIND_CHANNEL\n");
            break;
        case SC_STATUS_GETTING_SSID_PSWD:
        	os_printf("SC_STATUS_GETTING_SSID_PSWD\n");
            sc_type *type = pdata;
            if (*type == SC_TYPE_ESPTOUCH) {
            	os_printf("SC_TYPE:SC_TYPE_ESPTOUCH\n");
            } else {
            	os_printf("SC_TYPE:SC_TYPE_AIRKISS\n");
            }
            break;
        case SC_STATUS_LINK:
        	os_printf("SC_STATUS_LINK\n");
            struct station_config *sta_conf = pdata;
	        wifi_station_set_config(sta_conf);
	        wifi_station_disconnect();
	        wifi_station_set_auto_connect(TRUE);
	        wifi_station_connect();
            break;
        case SC_STATUS_LINK_OVER:
        	os_printf("SC_STATUS_LINK_OVER\n");
            if (pdata != NULL) {
				//SC_TYPE_ESPTOUCH
                uint8 phone_ip[4] = {0};

                memcpy(phone_ip, (uint8*)pdata, 4);
                printf("Phone ip: %d.%d.%d.%d\n",phone_ip[0],phone_ip[1],phone_ip[2],phone_ip[3]);

            } else {
            	//SC_TYPE_AIRKISS - support airkiss v2.0
				//airkiss_start_discover();
			}
            smartconfig_stop();
            break;
    }

}

/******************************************************************************
 * FunctionName : wifi_handle_event_cb
 * Description  : wifi event callback
 * Parameters   : evt,事件
 * Returns      : none
*******************************************************************************/
void ICACHE_FLASH_ATTR
wifi_handle_event_cb(System_Event_t *evt)
{
	os_printf("event %x\n", evt->event_id);
	switch (evt->event_id) {
		case EVENT_STAMODE_CONNECTED:
			os_printf("connect to ssid %s, channel %d\n",
					evt->event_info.connected.ssid,
					evt->event_info.connected.channel);
			break;
		case EVENT_STAMODE_DISCONNECTED:
			os_printf("disconnect from ssid %s, reason %d\n",
					evt->event_info.disconnected.ssid,
					evt->event_info.disconnected.reason);
			break;
		case EVENT_STAMODE_AUTHMODE_CHANGE:
			os_printf("mode: %d -> %d\n",
					evt->event_info.auth_change.old_mode,
					evt->event_info.auth_change.new_mode);
			break;
		case EVENT_STAMODE_GOT_IP:
			os_printf("WIFI connect suceed\r\n");
			tcp_client_init();
			break;
		case EVENT_SOFTAPMODE_STACONNECTED:
			os_printf("station: " MACSTR "join, AID = %d\n",
					MAC2STR(evt->event_info.sta_connected.mac),
					evt->event_info.sta_connected.aid);
			break;
		case EVENT_SOFTAPMODE_STADISCONNECTED:
			os_printf("station: " MACSTR "leave, AID = %d\n",
					MAC2STR(evt->event_info.sta_disconnected.mac),
					evt->event_info.sta_disconnected.aid);
			break;
		default:
			break;
	}
}



void ICACHE_FLASH_ATTR
smartconfig_task(void *pvParameters)
{
	wifi_station_set_auto_connect(FALSE);
	wifi_station_disconnect();
	smartconfig_stop();
    smartconfig_start(smartconfig_done);
    os_timer_disarm(&timer_led);//关闭定时器，相当于清零计时器计数
    os_timer_arm(&timer_led, 1000, TRUE);
    vTaskDelete(NULL);
}

static void gpio_intr_handler()
{
	_xt_isr_mask(1<<ETS_GPIO_INUM);    //disable interrupt

	os_delay_us(30*1000);
	if(!GPIO_INPUT_GET(GPIO_ID_PIN(4)))//判断中断针脚
	{
		os_printf("PressButton\n");
		os_timer_arm(&timer_count, 1000, TRUE);
	}
	else
	{
		os_printf("ReleaseButton %d\n",count);
		os_timer_disarm(&timer_count);
		if(count >= 3 && count <= 6)
		{
			os_printf("EnterConfig\n");
			xTaskCreate(smartconfig_task, "smartconfig_task", 256, NULL, 2, NULL);
		}
		count = 0;
	}

	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, GPIO_Pin_4);
	_xt_isr_unmask(1 << ETS_GPIO_INUM);
}

void init_timer()
{
	os_timer_disarm(&timer_led);//关闭定时器，相当于清零计时器计数
	os_timer_setfn(&timer_led, (os_timer_func_t *)timer_led_callback, NULL);//初始化定时器
	os_timer_arm(&timer_led, 3000, TRUE);

	os_timer_disarm(&timer_count);//关闭定时器，相当于清零计时器计数
	os_timer_setfn(&timer_count, (os_timer_func_t *)timer_count_callback, NULL);//初始化定时器
	os_timer_disarm(&timer_smartconfig);//关闭定时器，相当于清零计时器计数
	os_timer_setfn(&timer_smartconfig, (os_timer_func_t *)stop_smartconfig, NULL);//初始化定时器
	os_timer_disarm(&breakHeart);//关闭定时器，相当于清零计时器计数
	os_timer_setfn(&breakHeart, (os_timer_func_t *)timer_callback, NULL);//初始化定时器
}

void init_key()
{
	GPIO_ConfigTypeDef gpio_in_cfg;    //Define GPIO Init Structure
	gpio_in_cfg.GPIO_IntrType = GPIO_PIN_INTR_ANYEDGE;    //
	gpio_in_cfg.GPIO_Mode = GPIO_Mode_Input;    //Input mode
	gpio_in_cfg.GPIO_Pullup = GPIO_PullUp_EN;
	gpio_in_cfg.GPIO_Pin =  GPIO_Pin_4;    // Enable GPIO
	gpio_config(&gpio_in_cfg);    //Initialization function

	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, GPIO_Pin_4);
	gpio_intr_handler_register(gpio_intr_handler, NULL); // Register the interrupt function
	_xt_isr_unmask(1 << ETS_GPIO_INUM);    //Enable the GPIO interrupt
}


/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void user_init(void)
{
	uart_init_new();
	//PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U,FUNC_GPIO2);//选择GPIO2
	//GPIO_DIS_OUTPUT(GPIO_ID_PIN(2)) ;
	wifi_set_opmode(STATION_MODE);
	wifi_station_set_auto_connect(TRUE);
	wifi_set_event_handler_cb(wifi_handle_event_cb);
	init_timer();
	init_key();
	espconn_init();
}
