#ifndef __APP_H
#define __APP_H

#include "os_cfg.h"

/***********************************
* ͷ�ļ�
***********************************/

/**********************************
*�궨��
***********************************/

/***********************************
* ȫ�ֱ���
***********************************/

/***********************************
* ��Ͷ��x
***********************************/
#define BOOL unsigned char
	
typedef enum{
	INIT_TASK_ID = 0,
	KEY_LED_TASK_ID,
	TASK_TEST_ID,               //���ID������Ϊ�˲����õ�
//	TASK_PROCESS_INTERRUPT,
	//KEY_LED_TASK_ID,
	TASK_DETECT_BATTERY_ID,
	
	KEY_SEND_DATA_TO_PHONE_TASK_ID,
	TASK_USB_CHARGE_BAT,
	TASK_GET_SWITCH_MODE,
	//KEY_LED_TASK_ID,
	TASK_DETECT_PALM_ID,
	TASK_OUTPUT_PWM,
//	TASK_USB_CHARGE_BAT,
	TASK_LED_BINK_BEEP,
//	TASK_GET_SWITCH_MODE,
	SEND_TASK_ID,
	//TASK_OUTPUT_PWM,
	RECEIVE_TASK_ID,
	TASK_SELF_TEST,
	//KEY_LED_TASK_ID,
	//TASK_OUTPUT_PWM,
	TASK_RELEASE_GAS_ID,
	EXP_DETECT_SAVE_TASK_ID,
	EXP_READ_SEND_TASK_ID,
	TEST_TASK_ID,
	TASK_MAX_ID
}TASK_ID;

typedef struct{
	uint8_t run_status;//
	
}CONFIG_TYPE;
/***********************************
* �ⲿ����
***********************************/
void init_task(void);
//void init_system(void);
void init_system(BOOL bWakeUp);
#endif
