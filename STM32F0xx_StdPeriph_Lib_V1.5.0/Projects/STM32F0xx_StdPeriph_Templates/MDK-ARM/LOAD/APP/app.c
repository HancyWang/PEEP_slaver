

/**
********************************************************************************
* ��ࣺ
* ģ�����ƣ�app.c
* ģ�鹦�ܣ�
* �������ڣ�
* �� �� �ߣ�
* ��    ����
********************************************************************************
**/

/***********************************
* ͷ�ļ�
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
/**********************************
*�궨��
***********************************/

/***********************************
* ȫ�ֱ���
***********************************/
 // ������տ��ƶ���
extern CMD_Receive g_CmdReceive; 

//�l�͔���FIFO
extern FIFO_TYPE send_fifo;
extern uint8_t send_buf[SEND_BUF_LEN];

extern BOOL b_Is_PCB_PowerOn;

extern int16_t zero_point_of_pressure_sensor;
//���攵��
//extern FIFO_TYPE train_store_fifo;
//extern STORE_HEAD exp_store_head;
//extern TRAIN_STORE_DATA_PAGE train_store_data;
/***********************************
* �ֲ�����
***********************************/

/***********************************
* ��������
***********************************/
void test_task(void);
/***********************************
* ��������
***********************************/


//��ʼ������
void init_task(void)
{
	//��ʼ��Ӳ��
	init_hardware();	
	//Motor_PWM_Init();

	
//#ifdef _DEBUG
//	Calibrate_pressure_sensor(&zero_point_of_pressure_sensor);
//#endif
	
//	//��ʼ��ͨ�����
//	fifoInit(&send_fifo,send_buf,SEND_BUF_LEN);
//	UARTInit(g_CmdReceive.m_Buf1, BUF1_LENGTH);	
//	Init_Receive(&g_CmdReceive);
//	//init_system(b_Is_PCB_PowerOn);

//	os_create_task(TaskDataSend, OS_TRUE, SEND_TASK_ID);
//	os_create_task(CMD_ProcessTask, OS_TRUE, RECEIVE_TASK_ID);

	//os_create_task(test, OS_TRUE, TASK_TEST_ID);TASK_TEST_ID
	os_create_task(test_task,OS_TRUE,TASK_TEST_ID);
	//os_create_task(key_power_on_task, OS_TRUE, KEY_LED_TASK_ID);
//	os_create_task(send_data_to_phone_task, OS_TRUE, KEY_SEND_DATA_TO_PHONE_TASK_ID);
	
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

//��������ר���������Դ���
void test_task(void)
{
//	//GPIO_ResetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_4);
//	//����PB8
//	GPIO_InitTypeDef  GPIO_InitStructure;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	GPIO_SetBits(GPIOB,GPIO_Pin_8);
//	Delay_ms(10);
	
	//����honeywell sensor
	static UINT32 data,data1;
	Init_honeywell_sensor();
	Delay_ms(5);
////	Honeywell_ready_for_read();
//////		Init_ADS115();
//		Delay_ms(10);
	
	Init_MS5525DSO_sensor();
		Delay_ms(5);
	
	while(1)
	{
		Init_honeywell_sensor();
		//while(0x60==Honeywell_ready_for_read()){}    //����ʹ�����while��䣬Ҳ����ʹ��delay_ms(5)
		Delay_ms(5);
		data=honeywell_readByte();
//		Delay_ms(5);
		
		
		MS5525DSO_prepare_to_read();
		Delay_ms(8);
		data1=MS5525DSO_readByte();
	}	
	
//	uint8_t data=0x34;
//	while(1)
//	{
//		USART_WIFI_SendBuf(&data,1);
//		Delay_ms(100);
//	}
	
	
	os_delay_ms(TEST_TASK_ID, 50);
}
