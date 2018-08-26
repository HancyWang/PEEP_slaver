//////////////////////////////////////////////////////////////////////////////////	 
			  
//////////////////////////////////////////////////////////////////////////////////
#include "comm_task.h"
#include "fifo.h"
#include "CMD_Receive.h"
#include "os_cfg.h"
#include "stdio.h"
#include "delay.h"
#include "string.h"
#include "app.h"
#include "serial_port.h"
#include "protocol_module.h"
#include "key_power_on_task.h"
#include "Motor_pwm.h"
#include "i2c.h"
#include "hardware.h"
#include "iwtdg.h"


extern int16_t zero_point_of_pressure_sensor;

//ȫ�ֱ���
CMD_Receive g_CmdReceive;  // ������տ��ƶ���
FIFO_TYPE send_fifo;//�l�͔���FIFO
UINT8 send_buf[SEND_BUF_LEN];

//����������λ���������Ĳ���
UINT8 parameter_buf[PARAMETER_BUF_LEN]; 

UINT8 buffer[PARAMETER_BUF_LEN];

UINT16 check_sum;
extern BOOL b_Is_PCB_PowerOn;
extern MCU_STATE mcu_state;
extern BOOL rcvParameters_from_PC;
extern KEY_STATE key_state;
extern const uint8_t default_parameter_buf[PARAMETER_BUF_LEN];

extern uint16_t RegularConvData_Tab[2];

extern uint32_t os_ticks;

//extern BOOL b_usb_charge_bat;
extern USB_CHARGING_STATE usb_charging_state;
extern uint8_t led_beep_ID;

extern USB_DETECT_STATE usb_detect_state;

extern BOOL b_LED_ON_in_turn;
//extern BOOL b_usb_intterruptHappened;
//extern BOOL b_KeyWkUP_InterrupHappened;
//BOOL b_start_powerOn_check=FALSE;


BOOL b_release_gas=FALSE;
BOOL b_palm_checked=FALSE;
BOOL b_bat_detected_ok=FALSE;

BOOL b_no_hand_in_place=FALSE;
BOOL b_end_of_treatment=FALSE;

 uint32_t detectPalm_cnt=0;
 uint32_t noPalm_cnt=0;

 BOOL PWM1_timing_flag=TRUE;
 BOOL PWM2_timing_flag=TRUE; 
 BOOL PWM3_timing_flag=TRUE;



 BOOL led_bink_timing_flag=TRUE;
 BOOL beep_timing_flag=TRUE;
 BOOL usb_charge_timing_flag=TRUE;
 BOOL key_Press_or_Release_timing_flag=TRUE;
 BOOL key_self_test_timing_flag=TRUE;
//static BOOL switch_bnt_timing_flag=TRUE;
 BOOL b_releaseGas_timing_flag=TRUE;
//static BOOL b_detect_palm=TRUE;




BOOL b_check_bnt_release=FALSE;

 uint16_t led_bink_cnt;
	uint16_t beep_cnt;
	uint16_t delay_cnt;
	
 BOOL b_Motor_Ready2Shake=TRUE;
 BOOL b_Motor_shake=FALSE;

//uint32_t prev_switchBtn_os_tick;
//uint32_t prev_detect_palm_flag;
uint32_t prev_releaseGas_os_tick;
uint32_t prev_ledBlink_os_tick;
uint32_t prev_selfTest_os_tick;
uint32_t prev_keyPressOrRelease_os_tick;
uint32_t prev_usbCharge_os_tick;
uint32_t prev_beep_os_tick;
uint32_t prev_WaitBeforeStart_os_tick;
uint32_t prev_PWM1_os_tick;
uint32_t prev_PWM2_os_tick;
uint32_t prev_PWM3_os_tick;
uint32_t prev_PWM4_os_tick;
uint32_t prev_PWM5_os_tick;
uint32_t* p_prev_os_tick;


uint8_t mode=1;
//uint16_t prev_cnt;
//uint16_t cnt;


PWM_STATE pwm1_state=PWM_NONE;
PWM_STATE pwm2_state=PWM_NONE;
PWM_STATE pwm3_state=PWM_NONE;

//static uint16_t* p_PWM3_threshold; //ֻ��PWM3����threshold

//static uint16_t* p_PWM_period_cnt;
//static uint16_t* p_PWM_waitBetween_cnt;
//static uint16_t* p_PWM_waitAfter_cnt;



uint16_t PWM_waitBeforeStart_cnt=0;

//uint16_t PWM1_period_cnt=0;
//uint16_t PWM2_period_cnt=0;
//uint16_t PWM3_period_cnt=0;

//uint16_t PWM1_waitBetween_cnt=0;
//uint16_t PWM2_waitBetween_cnt=0;
//uint16_t PWM3_waitBetween_cnt=0;

//uint16_t PWM1_waitAfter_cnt=0;
//uint16_t PWM2_waitAfter_cnt=0;
//uint16_t PWM3_waitAfter_cnt=0;

uint8_t PWM1_numOfCycle=0;
uint8_t PWM2_numOfCycle=0;
uint8_t PWM3_numOfCycle=0;

uint8_t PWM1_serial_cnt=0;
uint8_t PWM2_serial_cnt=0;
uint8_t PWM3_serial_cnt=0;

//volatile CHCKMODE_OUTPUT_PWM state=WAIT_BEFORE_START;
CHCKMODE_OUTPUT_PWM state=LOAD_PARA;
//uint16_t mode;                      

//uint8_t pwm_buffer[144]={0};
//uint16_t	mode;

//uint8_t pressure;
uint16_t checkPressAgain_cnt=0;
uint8_t wait_cnt=0;


 BOOL b_self_test=FALSE;

BOOL b_Palm_check_complited=FALSE;


USB_CHARGING_STATE usb_charging_state=USB_CHARGE_NONE;
 BOOL b_usb_push_in;
 BOOL b_usb_pull_up;
BOOL b_stop_current_works=FALSE;
BOOL b_LED_ON_in_turn=FALSE;

SELF_TEST_STATE self_tet_state=SELF_TEST_NONE;
//SELF_TEST_STATE self_tet_state=SELF_TEST_DELAY_BEFORE_START; //debug
LED_IN_TURN_STATE led_In_Turn_state=LED_IN_TURN_NONE;


 uint8_t selfTest_delay_Cnt;
 uint8_t nLED_ON_in_turn;
 uint16_t inflate_cnt;
 uint16_t hold_cnt;
 uint8_t deflate_cnt;
//static uint8_t bat_detect_cnt=0;

//typedef enum
//{
//	INIT,
//	BLINK,
//	BEEP
//}LED_BEEP_STATE;

//LED_BEEP_STATE led_beep_state=INIT;

uint8_t sample_cnt;
uint32_t sample_sum;

LED_STATE led_state=LED_INIT;
BEEP_STATE beep_state=BEEP_INIT;
 uint8_t led_beep_ID=0;
 
 //�Լ�
  uint8_t deflate_cnt;
 uint16_t selfTest_inflate_record_1;
 uint16_t selfTest_inflate_record_2;

 uint16_t selfTest_hold_record_1;
 uint16_t selfTest_hold_record_2;
 uint16_t selfTest_deflate_record_1;
 uint16_t selfTest_deflate_record_2;
 uint8_t selfTest_fail_Cnt;
 uint8_t selfTest_fail_period_H;
 uint8_t selfTest_fail_period_L;
 uint8_t selfTest_end_Cnt;
 
 BOOL b_detect_hand_before_system_running=TRUE;
// BOOL b_stop_motors=FALSE;
uint16_t wait_between_total_cnt=0;
 uint8_t value=0;
 
//*********************debug*******************
//cycles_record���������������Եģ���ʽ�İ汾����Ҫ���
#ifdef _DEBUG_TEST_CYCLES
//�ֱ��¼���ܵ�Ȧ��,os_ticks(ÿ����һȦ��¼һ�ε�ǰʱ��),os_ticks(�ܵ�û���ʱ���¼ʱ��)
uint32_t debug_cycles_record[3]={0}; 
//uint32_t debug_cycle_cnt=0; 
#else
#endif
//*********************debug*******************

/*******************************************************************************
*                                �ڲ���������
*******************************************************************************/
static BOOL ModuleUnPackFrame(void);
static BOOL ModuleProcessPacket(UINT8 *pData);
static UINT8 CheckCheckSum(UINT8* pData, UINT8 nLen);



void CalcCheckSum(UINT8* pPacket)
{
	UINT16 dataLen = pPacket[1];
	UINT16 checkSum = 0;
	UINT16 i;

	for (i = 1; i < dataLen; i++)
	{
		checkSum += pPacket[i];
	}

	pPacket[dataLen] = checkSum >> 8;
	pPacket[dataLen+1] = checkSum&0xFF;
}

/*******************************************************************************
** ��������: ModuleUnPackFrame
** ��������: ������մ���
** �䡡  ��: ��
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
*******************************************************************************/
 UINT8 sDataBuff[CMD_BUFFER_LENGTH] = {0};	
 UINT8 sBackBuff[CMD_BUFFER_LENGTH] = {0};
BOOL ModuleUnPackFrame(void)
{
	static BOOL sPacketHeadFlag = FALSE;
	static BOOL sPacketLenFlag = FALSE;
	static UINT8 sCurPacketLen = 0;
	static UINT8 sResetByte = 0;
	//static UINT8 sDataBuff[CMD_BUFFER_LENGTH] = {0};	
	//static UINT8 sBackBuff[CMD_BUFFER_LENGTH] = {0};

	UINT8 *pBuff = (UINT8 *)sDataBuff;
	UINT16 dwLen = 0;
	UINT8 byCurChar;

	// �Ӵ��ڻ������ж�ȡ���յ�������
	dwLen = GetBuf2Length(&g_CmdReceive);

	// �����ݽ��н���
	while(0 < dwLen)
	{
		byCurChar = Buf2Read(&g_CmdReceive);

		if (sPacketHeadFlag)
		{
			// ��������ͷ
			if(sPacketLenFlag)
			{
				// ������������
				pBuff[sCurPacketLen] = byCurChar;
				sCurPacketLen ++;
				sResetByte --;

				if (0 >= sResetByte)
				{
					// �������
					// ����У��ͱȽ�
					if (CheckCheckSum(pBuff, pBuff[1]))
					{
						// ������һ����Ч���ݰ�
						memcpy(sBackBuff, sDataBuff, CMD_BUFFER_LENGTH);
						ModuleProcessPacket(sBackBuff);//������*********************************
						//��ֹ�������ն�������ʱ������Ӧ���źų�����
						delay_ms(2);
					}

					sPacketHeadFlag = FALSE;
					sPacketLenFlag = FALSE;
					memset(&sDataBuff, 0x00, CMD_BUFFER_LENGTH);
				}													
			}
			else
			{
				if((CMD_BUFFER_LENGTH-1 > byCurChar) && (0 < byCurChar ))// �ݴ�����ֹ���ݰ���Ϊ49��0ʱ��� ����X5����������Ͱ�Ϊ15
				{
					// ������ģ��ĳ���
					sDataBuff[sCurPacketLen] = byCurChar;
					sResetByte = byCurChar;			
					sPacketLenFlag = TRUE;
					sCurPacketLen ++;
				}
				else
				{
					//û�н�����ģ��ĳ���, ���½���
					sPacketHeadFlag = FALSE;
					sPacketLenFlag = FALSE;					
				}
			}
		}
		
		else if (PACK_HEAD_BYTE == byCurChar)		
		{
			// ��������ͷ
			sDataBuff[0] = byCurChar;
			sPacketHeadFlag = TRUE;
			sPacketLenFlag = FALSE;			
			sCurPacketLen = 1;
			sResetByte = 0;
		}

		//pData ++;
		dwLen --;
	}
	return TRUE;
}


/*******************************************************************************
** ��������: CheckCheckSum
** ��������: ��У��
** �䡡  ��: pData ���� nLen����
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
*******************************************************************************/
UINT8 CheckCheckSum(UINT8* pData, UINT8 nLen)
{
	UINT16 bySum = 0;
	int i;
	// �������ݵ�У���	
	for(i = 1; i < nLen; i++)
	{
		bySum += pData[i];
	}		

	if (bySum == (pData[nLen] << 8)+ pData[nLen + 1])
	{
		return TRUE;
	}
	else
	{
		return FALSE;	
	}
}

/*******************************************************************************
** ��������: ModuleProcessPacket
** ��������: ������
** �䡡  ��: pData ����
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
*******************************************************************************/
BOOL ModuleProcessPacket(UINT8 *pData)
{	
	protocol_module_process(pData);

	return TRUE;     	
}


/*******************************************************************************
* �������� : TaskDataSend
* �������� : ���ݷ�������5msִ��һ��
* ������� : arg  ��������ʱ���ݵĲ���
* ������� : ��
* ���ز��� : ��
*******************************************************************************/
void TaskDataSend (void)
{
    UINT8 send_data_buf[SEND_DATA_BUF_LENGTH] = {0};
    UINT16  len;
		
		//protocol_module_send_exp_flag(1);
#ifdef _DEBUG
#else
		if(mcu_state==POWER_ON)
#endif
		
		{
			//ѭ�h
			len = fifoReadData(&send_fifo, send_data_buf, SEND_DATA_BUF_LENGTH);
			if(len)
			{
					UartSendNBytes(send_data_buf, len);
			}
		}
		
		
		os_delay_ms(SEND_TASK_ID, 28);  //markһ��
}





/*******************************************************************************
** ��������: CMD_ProcessTask
** ��������: �����������
** �䡡  ��: arg  ��������ʱ���ݵĲ���
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
*******************************************************************************/
void CMD_ProcessTask (void)
{
#ifdef _DEBUG
#else
	if(mcu_state==POWER_ON)
#endif
	
	{
		//ѭ�h
		ReceiveData(&g_CmdReceive);//�������ݵ�������
		ModuleUnPackFrame();//�����
	}

	os_delay_ms(RECEIVE_TASK_ID, 100);
}
