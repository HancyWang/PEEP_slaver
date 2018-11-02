

/**
********************************************************************************
* 版權：
* 模块名称：app.c
* 模块功能：
* 创建日期：
* 创 建 者：
* 声    明：
********************************************************************************
**/

/***********************************
* 头文件
***********************************/
#include "app.h"
#include "datatype.h"
#include "comm_task.h"
#include "CMD_Receive.h"
#include "serial_port.h"
#include "hardware.h"
#include "Motor_pwm.h"
#include "fifo.h"
#include "protocol_module.h"
#include "stm32f0xx_rtc.h"
#include "key_power_on_task.h"
//#include "exp_task.h" 
#include "stm32f0xx_usart.h"
//#include "store_fifo.h"
#include "send_data_to_phone.h"
#include "i2c.h"
#include "delay.h"
//#include "honeywell_sampling_data.h"
//#include "MS5525DSO_sampling_data.h"
#include "SDP31_sampling_data.h"
#include "MPXV70_sampling_data.h"

//#include <stdint.h>
//#include <math.h>
/**********************************
*宏定义
***********************************/

/***********************************
* 全局变量
***********************************/
 // 命令接收控制对象
extern CMD_Receive g_CmdReceive; 

//發送數據FIFO
extern FIFO_TYPE send_fifo;
extern uint8_t send_buf[SEND_BUF_LEN];

extern BOOL b_Is_PCB_PowerOn;

extern int16_t zero_point_of_pressure_sensor;
//保存數據
//extern FIFO_TYPE train_store_fifo;
//extern STORE_HEAD exp_store_head;
//extern TRAIN_STORE_DATA_PAGE train_store_data;
/***********************************
* 局部变量
***********************************/

/***********************************
* 函数声明
***********************************/
void test_task(void);
/***********************************
* 函数定义
***********************************/


//初始化任务
void init_task(void)
{
	//初始化硬件
	init_hardware();	
	//Motor_PWM_Init();

#if 0
//#ifdef _DEBUG
//	Calibrate_pressure_sensor(&zero_point_of_pressure_sensor);
//#endif
	
//	//初始化通信相关
//	fifoInit(&send_fifo,send_buf,SEND_BUF_LEN);
//	UARTInit(g_CmdReceive.m_Buf1, BUF1_LENGTH);	
//	Init_Receive(&g_CmdReceive);

//	os_create_task(TaskDataSend, OS_TRUE, SEND_TASK_ID);
//	os_create_task(CMD_ProcessTask, OS_TRUE, RECEIVE_TASK_ID);

	//os_create_task(test, OS_TRUE, TASK_TEST_ID);
	//os_create_task(test_task,OS_TRUE,TASK_TEST_ID);
//	//这两个sensor不用了，换成sensirion和MPXV70
//	os_create_task(honeywell_sampling_data, OS_TRUE, HONEYWELL_SAMPLING_DATA_TASK_ID);
//	os_create_task(MS5525DSO_sampling_data, OS_TRUE, MS5525DSO_SAMPLING_DATA_TASK_ID);
#endif
	os_create_task(key_power_on_task, OS_TRUE, KEY_LED_TASK_ID);
	os_create_task(SDP31_sampling_data, OS_TRUE, SDP31_SAMPLING_DATA_TASK_ID);
	os_create_task(MPXV70_sampling_data, OS_TRUE, MPXV70_SAMPLING_DATA_TASK_ID);
	
	os_create_task(send_data_to_phone_task, OS_TRUE, KEY_SEND_DATA_TO_PHONE_TASK_ID);
	
	//os_create_task(get_switch_mode,OS_TRUE,TASK_GET_SWITCH_MODE);
	//os_create_task(check_selectedMode_ouputPWM,OS_TRUE,TASK_OUTPUT_PWM);
	//os_create_task(ReleaseGas, OS_TRUE, TASK_RELEASE_GAS_ID);
	//os_create_task(DetectPalm, OS_TRUE, TASK_DETECT_PALM_ID);
	//os_create_task(Detect_battery_and_tmp,OS_TRUE,TASK_DETECT_BATTERY_ID);
	//os_create_task(led_blink_beep,OS_TRUE,TASK_LED_BINK_BEEP);
	//os_create_task(usb_charge_battery,OS_TRUE,TASK_USB_CHARGE_BAT);
	//os_create_task(self_test,OS_TRUE,TASK_SELF_TEST);

	os_pend_task(INIT_TASK_ID);
}

//测试任务，专门用来调试代码
extern uint32_t os_ticks;
//static uint32_t debug_prev_os_tick;
//static uint32_t debug_after_os_tick;
void test_task(void)
{
	//测试honeywell sensor
	#if 0
	static unsigned int data,d1,d2;
	static UINT16 c1,c2,c3,c4,c5,c6;  //6个PROM的值
	Init_honeywell_sensor();
	Delay_ms(5);
////	Honeywell_ready_for_read();
//////		Init_ADS115();
//		Delay_ms(10);
	
	//复位MS5525DSO
	Init_MS5525DSO_sensor();
		Delay_ms(5);
	//读取PROM值，一共6个值

	c1=MS5525DSO_PROM_CX(C1);
	c2=MS5525DSO_PROM_CX(C2);
	c3=MS5525DSO_PROM_CX(C3);
	c4=MS5525DSO_PROM_CX(C4);
	c5=MS5525DSO_PROM_CX(C5);
	c6=MS5525DSO_PROM_CX(C6);
	
	while(1)
	{
		Init_honeywell_sensor();
		MS5525DSO_prepare_to_read(D1);
		Delay_ms(1);
		d1=MS5525DSO_readByte();
		
		MS5525DSO_prepare_to_read(D2);
		Delay_ms(1);
		d2=MS5525DSO_readByte();
		
		Delay_ms(3);
		
		debug_prev_os_tick=os_ticks;
		data=honeywell_readByte();
		//计算压力值
		static __int64 dT,TEMP,OFF,SENS;
		static int P;
		dT=d2-(c5<<7);  //D2 - C5 * 2^7
		TEMP=2000+((dT*c6)>>21);  //TEMP=2000+dT*C6/2^21
		OFF=(c2<<17)+((c4*dT)>>5);   //OFF=C2*2^17 +(C4*dT)/2^5
		SENS=(c1<<15)+((c3*dT)>>7); //C1*2^15 +(C3*dT)/2^7
		P=(d1*(SENS>>21)-OFF)>>15;    //(D1*SENS/2^21 -OFF)/2^15
		 debug_after_os_tick=os_ticks;
	}	
	
//	uint8_t data=0x34;
//	while(1)
//	{
//		USART_WIFI_SendBuf(&data,1);
//		Delay_ms(100);
//	}
	#endif
	
	os_delay_100us(TEST_TASK_ID, 50);
	//os_delay_ms(TEST_TASK_ID, 50);
}
