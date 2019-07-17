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
/****************************
*   TCP SERVER FUNCTIONS   *
****************************/
/**********************************
*  TCP SERVER STATIC PROTOTYPES  *
**********************************/
//static void tcp_server_sent_cb(void *arg);
//static void tcp_server_recv_cb(void *arg,char *pdata,unsigned short length);
//static void tcp_server_recon_cb(void *arg,sint8 error);
//static void tcp_server_discon_cb(void *arg);
//static void tcp_server_listen_cb(void *arg);
typedef enum {
	timer_invalid = -1,
    timer_wait_config,
	timer_smartconfiging,
	timer_wifi_error,
	timer_server_error,
	timer_stop
} LED_STATUS;

BOOL bIsConnectRemote = FALSE;
BOOL bIsNeedToConnect = TRUE;

LED_STATUS led_status;

char cmd[4]={0};
int  time_used = 0;

bool bIsSmartConfiging = FALSE;

os_timer_t timer_task;

struct espconn user_tcp_conn;
struct espconn *pespconn;

/**********************************
 *   TCP SERVER STATIC VARIABLES  *
**********************************/
//os_timer_t tcp_server_send_data_timer;
//struct espconn tcp_server;
//uint8 z;

 /**********************************
  *   TCP server STATIC FUNCTIONS  *
  **********************************/

 /**
  * TCP Server数据发送回调函数
  */
// static void ICACHE_FLASH_ATTR
// tcp_server_sent_cb(void *arg){
//     os_printf("tcp server send data successful\r\n");
//
// }

 /**
  * TCP Server数据接收回调函数，可以在这处理收到Client发来的数据
  */
// static void ICACHE_FLASH_ATTR
// tcp_server_recv_cb(void *arg,char *pdata,unsigned short len){
//	 int i;
//
//     os_printf("length: %d \r\ndata: %s\r\n",len,pdata);
//
//     struct station_config data;
//     data.bssid_set = 0;
//     for(i = 0;i < len;i++)
//     {
//    	 if(pdata[i] == '=')
//    	 {
//    		 break;
//    	 }
//     }
//
//     memset(data.bssid,0,sizeof(data.bssid));
//     memset(data.password,0,sizeof(data.password));
//     memcpy(data.bssid,pdata,i);
//     memcpy(data.password,pdata+i+1,len-i-1);
//     espconn_send(arg,"get wifiname",12);
////     wifi_set_opmode(STATION_MODE);
////     wifi_station_set_auto_connect(true);
////     wifi_station_set_config(&data);
// }

 /**
  * TCP Server重连回调函数，可以在此函数里做重连接处理
  */
// static void ICACHE_FLASH_ATTR
// tcp_server_recon_cb(void *arg,sint8 error){
//     os_printf("tcp server connect tcp client error %d\r\n",error);
//     os_timer_disarm(&tcp_server_send_data_timer);
// }

 /**
  * TCP Server断开连接回调函数
  */
// static void ICACHE_FLASH_ATTR
// tcp_server_discon_cb(void *arg){
//     os_printf("tcp server disconnect tcp client successful\r\n");
//     os_timer_disarm(&tcp_server_send_data_timer);
// }

 /**
  * TCP Server监听Client连接回调函数
  */
// static void ICACHE_FLASH_ATTR
// tcp_server_listen_cb(void *arg){
//     struct espconn *pespconn = arg;
//
//     os_printf("tcp server have tcp client connect\r\n");
//     espconn_regist_recvcb(pespconn,tcp_server_recv_cb);//注册收到数据回调函数
//     espconn_regist_sentcb(pespconn,tcp_server_sent_cb);//注册发送完数据回调函数
//     espconn_regist_disconcb(pespconn,tcp_server_discon_cb);//注册断开连接回调函数
//
//     os_timer_disarm(&tcp_server_send_data_timer);
//     //os_timer_setfn(&tcp_server_send_data_timer, (os_timer_func_t *) tcp_server_send_data,NULL);//注册Server定时发送数据回调函数
//     os_timer_arm(&tcp_server_send_data_timer, 1000, true);//设置时间为1s
// }

 /**********************************
  *   TCP CLIENT GLOBAL FUNCTIONS  *
  **********************************/

 /**
  * TCP Server定时发送数据回调函数
  */
// void ICACHE_FLASH_ATTR
// tcp_server_send_data(void){
//     char buf[256],length;
//     os_printf("tcp server send data tcp client\r\n");
//     length = sprintf(buf,(char *)"Hi this is ESP8266 server! message num %d",z);
//     z++;
//     espconn_send(&tcp_server,buf,length);
// }

 /**
  * TCP Server初始化函数
  * @local_port 本地监听端口号，与Client连接的端口号一致
  */
// void ICACHE_FLASH_ATTR
// tcp_server_init(uint16 local_port){
//
//     os_printf("tcp server waiting tcp client connect!\r\n");
//     tcp_server.proto.tcp = (esp_tcp *)os_zalloc(sizeof(esp_tcp));
//     tcp_server.type = ESPCONN_TCP;
//     tcp_server.proto.tcp->local_port = local_port;//设置本地监听的端口号，等待Client连接
//
//     espconn_regist_connectcb(&tcp_server,tcp_server_listen_cb);//注册Server监听回调函数
//     espconn_regist_reconcb(&tcp_server,tcp_server_recon_cb);//注册断连重新连接回调函数
//
//     espconn_accept(&tcp_server);//创建Server,开始监听
//     espconn_regist_time(&tcp_server,360,0);//超时断开连接时间
//}

void change_timer_task(LED_STATUS led_status_para,int nSec)
{
	time_used = 0;
	os_timer_disarm(&timer_task);
	led_status = led_status_para;
	os_timer_arm(&timer_task, nSec, TRUE);
}

void timer_task_callback()
{
	 time_used++;
     switch(led_status)
     {
     case timer_invalid:
    	 if(!GPIO_INPUT_GET(GPIO_ID_PIN(4)))
    	 {
    		 bIsNeedToConnect = FALSE;
    		 if(bIsConnectRemote)
    		 {
    			 //while(espconn_sent(pespconn, "BB02=machine!", strlen("BB02=machine!")) != 0);
    			 espconn_sent(pespconn, "BB02=machine!", strlen("BB02=machine!"));
    			 struct station_config data;
    			 wifi_station_set_config(&data);
    			 os_delay_us(20*1000);
    			 espconn_disconnect(pespconn);
    		 }
    		 change_timer_task(timer_smartconfiging,300);
    		 return;
    	 }
    	 break;
     case timer_wait_config:
    	 if(!GPIO_INPUT_GET(GPIO_ID_PIN(4)))
    	 {
    		 bIsNeedToConnect = FALSE;
    		 if(bIsConnectRemote)
    		 {
    			 while(espconn_sent(pespconn, "BB02=machine!", strlen("BB02=machine!")) != 0);
    			 struct station_config data;
    			 wifi_station_set_config(&data);
    			 os_delay_us(20*1000);
    			 espconn_disconnect(pespconn);
    		 }
    		 change_timer_task(timer_smartconfiging,300);
    		 return;
    	 }
    	 GPIO_OUTPUT_SET(GPIO_ID_PIN(2), 0);
    	 os_delay_us(10000);
    	 GPIO_OUTPUT_SET(GPIO_ID_PIN(2), 1);
    	 if(time_used >= 7)
    	 {
    		 os_timer_disarm(&timer_task);
    		 system_deep_sleep(1000*1000*10);
    	 }
    	 break;
     case timer_smartconfiging:
    	 GPIO_OUTPUT_SET(GPIO_ID_PIN(2), 0);
    	 os_delay_us(10000);
    	 GPIO_OUTPUT_SET(GPIO_ID_PIN(2), 1);
    	 if(time_used >= 1000)
    	 {
    		 os_timer_disarm(&timer_task);
    		 system_deep_sleep(1000*1000*10);
    	 }
    	 break;
     case timer_server_error:
    	 GPIO_OUTPUT_SET(GPIO_ID_PIN(2), 0);
    	 os_delay_us(10000);
    	 GPIO_OUTPUT_SET(GPIO_ID_PIN(2), 1);

    	 if(time_used % 5 == 0)
    	 {
    		 if(wifi_station_get_connect_status() != STATION_GOT_IP)
    		 {
    			 //espconn_connect(&user_tcp_conn);
    			 wifi_station_connect();
    		 }
    		 else
    		 {
    			 espconn_connect(&user_tcp_conn);
    		 }

    	 }

    	 break;
     default:
    	 break;
     }
}


extern void uart0_write_string(char *pdata, unsigned short len);
//void send_msg_to_server(uint8 *psent,uint16 length)
//{
//	uint8 returnCode = espconn_sent(pespconn, psent, length);
//	if(returnCode == 0)
//	{
//        //portDISABLE_INTERRUPTS();
//	}
//	else if(returnCode == ESPCONN_ARG)
//	{
//		espconn_connect(&user_tcp_conn);
//	}
//}

void timer_led_callback()
{
	GPIO_OUTPUT_SET(GPIO_ID_PIN(2), 0);
	os_delay_us(5000);
	GPIO_OUTPUT_SET(GPIO_ID_PIN(2), 1);
}

void ICACHE_FLASH_ATTR user_tcp_sent_cb(void *arg)  //发送
{
	os_printf("send suceed");
}

void ICACHE_FLASH_ATTR user_tcp_discon_cb(void *arg)  //断开
{
	os_printf("disconnect suceed!");
	bIsConnectRemote = FALSE;

	//如果是按配网断开的就不要重新连接了
	if(bIsNeedToConnect)
	{
		espconn_connect((struct espconn *) arg);
	}

}

void ICACHE_FLASH_ATTR user_tcp_recv_cb(void *arg,  //接收
		char *pdata, unsigned short len) {

    char DeviceBuffer[20]={0};

    sprintf(DeviceBuffer,"success=%d!",len);
    bIsConnectRemote = TRUE;

	if(pdata[0] == 0xAA)
	{
		memcpy(cmd,pdata,4);
	    uart0_write_string(cmd,4);
	}
	else if(pdata[0] == 0xBB)
	{
        if(pdata[1] == 0x01)//wake up
        {

        }
        else if(pdata[1] == 0x02)//clear
        {
        	system_restore();
        	system_restart();
        }
        else if(pdata[1] == 0x03)//sleep
        {

        }


	}
	else
	{
		uart0_write_string(cmd,4);
	}

	espconn_sent(arg, DeviceBuffer, strlen(DeviceBuffer));
}

void ICACHE_FLASH_ATTR user_tcp_recon_cb(void *arg, sint8 err) //注册 TCP 连接发生异常断开时的回调函数，可以在回调函数中进行重连
{
	os_printf("connect error %d\r\n", err);
	change_timer_task(timer_server_error,1000);
}

void ICACHE_FLASH_ATTR user_tcp_connect_cb(void *arg)  //注册 TCP 连接成功建立后的回调函数
{
	char yladdr[6];
	char DeviceBuffer[40]={0};

	int keep_alive = 3;

	led_status = timer_stop;
	os_timer_disarm(&timer_task);
	GPIO_OUTPUT_SET(GPIO_ID_PIN(2), 0);
    pespconn = arg;
	bIsNeedToConnect = TRUE;

	espconn_regist_recvcb(arg, user_tcp_recv_cb);  //接收
	espconn_regist_sentcb(arg, user_tcp_sent_cb);  //发送
	espconn_regist_disconcb(arg, user_tcp_discon_cb);  //断开
	wifi_get_macaddr(STATION_IF,yladdr);
	sprintf(DeviceBuffer,MAC_STR",machine!",MAC2STR(yladdr));

	espconn_sent(pespconn, DeviceBuffer, strlen(DeviceBuffer));

	bIsConnectRemote = TRUE;
	//keep alive checking per 30s

	espconn_set_keepalive(arg, ESPCONN_KEEPIDLE, &keep_alive);
	keep_alive = 5; //repeat interval = 5s
	espconn_set_keepalive(arg, ESPCONN_KEEPINTVL, &keep_alive);
	keep_alive = 2;//repeat 2times
	espconn_set_keepalive(arg, ESPCONN_KEEPCNT, &keep_alive);
	espconn_set_opt(arg, ESPCONN_KEEPALIVE);
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
	struct ip_info info;//47, 97, 252, 149
	const char remote_ip[4] = { 47, 97, 252, 149 };//目标IP地址,必须要先从手机获取，否则连接失败.
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
	bIsSmartConfiging = TRUE;
    wifi_station_disconnect();
	smartconfig_stop();
	smartconfig_set_type(SC_TYPE_ESPTOUCH);
    smartconfig_start(smartconfig_done);
    vTaskDelete(NULL);
}

//void ICACHE_FLASH_ATTR
//APConfig_task(void *pvParameters)
//{
////	network_type = 1;
////	smartconfig_stop();
////    os_timer_disarm(&timer_led);//关闭定时器，相当于清零计时器计数
////    os_timer_arm(&timer_led, 2000, TRUE);
////    wifi_set_opmode(STATIONAP_MODE);
////    tcp_server_init(8080);
////    vTaskDelete(NULL);
//}

static void gpio_intr_handler()
{
	_xt_isr_mask(1<<ETS_GPIO_INUM);    //disable interrupt

	os_delay_us(30*1000);
	if(!GPIO_INPUT_GET(GPIO_ID_PIN(4)))//判断中断针脚
	{
		os_printf("PressButton\n");
		if(led_status != timer_smartconfiging)
		{
			if(led_status != timer_wait_config)
			{
				change_timer_task(timer_invalid,3000);
			}
			else
			{
				change_timer_task(timer_wait_config,3000);
			}
		}
	}
	else
	{
		os_printf("ReleaseButton\n");
		if(led_status == timer_smartconfiging)
		{
			os_printf("EnterConfig\n");
			xTaskCreate(smartconfig_task, "smartconfig_task", 256, NULL, 2, NULL);
		}
		else
		{
			if(bIsConnectRemote)
			{
				led_status = timer_stop;
				os_timer_disarm(&timer_task);
			}
			else
			{
				change_timer_task(timer_wait_config,3000);
			}
		}
	}

	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, GPIO_Pin_4);
	_xt_isr_unmask(1 << ETS_GPIO_INUM);
}

void init_timer()
{
	os_timer_disarm(&timer_task);
	os_timer_setfn(&timer_task, (os_timer_func_t *)timer_task_callback, NULL);
	change_timer_task(timer_wait_config,3000);
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
void ICACHE_FLASH_ATTR user_init(void)
{
	uart_init_new();

	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U,FUNC_GPIO2);//选择GPIO2
	GPIO_DIS_OUTPUT(GPIO_ID_PIN(2));
	wifi_set_opmode(STATION_MODE);
	wifi_set_sleep_type(LIGHT_SLEEP_T);
	//如果flash中存在wifi信息，自动连接wifi然后连接到服务器

	wifi_station_set_auto_connect(TRUE);

	wifi_set_event_handler_cb(wifi_handle_event_cb);

	//等待配网
	init_key();
	init_timer();
	espconn_init();

}
