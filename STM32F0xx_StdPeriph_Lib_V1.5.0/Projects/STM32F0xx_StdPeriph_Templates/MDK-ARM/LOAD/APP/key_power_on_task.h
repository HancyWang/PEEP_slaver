#ifndef __KEY_TASK_H_
#define __KEY_TASK_H_
/***********************************
* ͷ�ļ�
***********************************/

/**********************************
*�궨��
***********************************/
#define KEY_LED_PERIOD  2000
#define CHECK_MODE_OUTPUT_PWM 10
#define ONE_SEC_KEY_TIME   1000/KEY_LED_PERIOD
/***********************************
* ȫ�ֱ���
***********************************/
typedef enum
{
	POWER_ON,
	POWER_OFF
}MCU_STATE;

typedef enum
{
	KEY_STOP_MODE,  //����״̬����͹���ģʽ
	KEY_UPING,
	KEY_DOWNING,
	KEY_FAIL_WAKEUP,  //��������û��������������͹���(Ҳ���ǰ���ʱ�򲻹�)
//	KEY_DOWN_UP,
	KEY_WAKE_UP
	//KEY_UP_DOWN
}KEY_STATE;

typedef enum
{
	USB_PUSH_IN,
	USB_FAIL_INSERT,
	USB_PULL_UP,
	USB_NOT_DETECT,
	USB_INSERTED
}USB_DETECT_STATE;

/***********************************
* ��Ͷ��x
***********************************/

/***********************************
* �ⲿ����
***********************************/
void key_power_on_task(void);
void EnterStopMode(void);
void CfgWFI(void);
void CfgALLPins4StopMode(void);
void init_system_afterWakeUp(void);
void test(void);
//extern INT8U I2C_RecByte(void);



#endif
