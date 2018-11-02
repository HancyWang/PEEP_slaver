#include "send_data_to_phone.h"
#include "serial_port.h"
#include "key_power_on_task.h"
#include "delay.h"
#include "i2c.h"
#include "comm_task.h"
//#include "honeywell_sampling_data.h"
//#include "MS5525DSO_sampling_data.h"
#include <string.h>
#include <math.h>
#include "SDP31_sampling_data.h"
#include "MPXV70_sampling_data.h"
#include "app.h"

BOOL DELAY_BLE_SEND_TIMING_20ms_flag=TRUE;
uint32_t DELAY_prev_BLE_SEND_TIMING_20ms_os_tick;

extern MCU_STATE mcu_state;
#if 0
//#define HONEYWELL_RATE			11110   //б��
//#define HONEYWELL_INTERCEPT 383196	//�ؾ�
//uint8_t flag=1;
//static int send_cnt;

//unsigned int d1,d2;
//uint16_t c1,c2,c3,c4,c5,c6;  //6��PROM��ֵ


//static BOOL b_initADC_sensor=1;


//unsigned int honeywell_data; 	 //ѹ����������ֵ
//int p_diff;										//ѹ�������ֵ

//static uint16_t send_cnt;							//
//static uint16_t honeywell_buff[4];   
//static uint16_t MS5525DSO_buff[4];


//extern HONEYWELL_STATE honeywell_state;
//extern MS5525DSO_STATE MS5525DSO_state;
#endif 

static SEND_STATE send_state=SEND_PRE;
extern SDP31_STATE SDP31_state;
extern MPXV70_STATE MPXV70_state;

extern int16_t SDP31_buffer[SDP31_SAMPLE_CNT];
extern int16_t MPVX70_buffer[MPXV70_SAMPLE_CNT];
//���巢�͵��ֻ���buffer
uint8_t send2Phone_buffer[1+(SDP31_SAMPLE_CNT+MPXV70_SAMPLE_CNT)*2+2];  

//��5�βɼ�����������䵽ָ����p_buffer��
void fill_buffer_send(uint8_t* p_buffer)
{
	uint16_t sum=0;
	sum+=p_buffer[0];
	*p_buffer++=0xFF;  //֡ͷ
	
	for(uint8_t i=0;i<MPXV70_SAMPLE_CNT;i++)   //���ѹMPXV7���ݣ�ѹ��
	{
		sum+=MPVX70_buffer[i];
		*p_buffer++=MPVX70_buffer[i]>>8;
		*p_buffer++=MPVX70_buffer[i]%256;
	}
	
	for(uint8_t i=0;i<SDP31_SAMPLE_CNT;i++)   //���SDP31����,ѹ��
	{
		sum+=SDP31_buffer[i];
		*p_buffer++=SDP31_buffer[i]>>8;
		*p_buffer++=SDP31_buffer[i]%256;
	}
	*p_buffer++=sum>>8;  //���checksum
	*p_buffer=sum%256;
	
	//���buffer
	memset(SDP31_buffer,0,SDP31_SAMPLE_CNT*2);
	memset(MPVX70_buffer,0,MPXV70_SAMPLE_CNT*2);
}

#if 0
////Honeywell,��3���ֽڵ�ADC����ת����2�ֽڵ�PSI����
//uint16_t cal_honeywell_data(int data)
//{
//	int diff;
////	diff=data-419430;
//	diff=data-HONEYWELL_ZERO_POINT;
//	if(diff>0)
//	{
//		//3999=5.8*6.8948*100,(����30kpa=5.8psi,1psi=6.8948kpa,1000��ʾ�Ŵ�1000������λ����Ҫ�Լ�����1000����ԭ)
//		//����diff*3999�ᳬ��int��������(24��),��uinsigned int Ҳ��42�ڣ�������
//		//���ڽ�3999�ĳ�400*10
//		return diff*400/3355443*100;  
////		return diff*1900/11110;
//	}
//	else
//	{
//		return 0;
//	}
//}

////MS5525DSO,��3�ֽڵ�ADC����ת����2�ֽڵ���������
////����㷨��д
//uint16_t cal_MS5525DSO_data(int data)
//{
//	static int diff;
//	static int result;
//	diff=data+(0-MS5525DSO_ZERO_POINT);
//	if(diff>0)
//	{
//		result= sqrt(2*diff*13)*51*125/100;
//		return result;
//	}
//}
#endif

void send_data_to_phone_task()
{
	#ifdef _DEBUG
	#else
	if(mcu_state==POWER_ON)
	#endif
	{
		//1.׼��
		if(send_state==SEND_PRE)  //�ӳ�210ms,������׼����
		{
			if(Is_timing_X10us(200*100,DELAY_BEFORE_START))
			{
				send_state=SEND_NONE;
			}
		}
		
		//��鵽����sensor�����ݶ��ɼ����
		if(SDP31_state==SDP31_SAMPLE_FINISH&&MPXV70_state==MPXV70_SAMPLE_FINISH)
		{
			SDP31_state=SDP31_READ_DATA;
			MPXV70_state=MPXV70_SAMPLE_DATA; //����ʹ����sensor��ʼ�ɼ�����
			
			send_state=SEND_DATA_TO_PHONE;
		}
		
		if(send_state==SEND_DATA_TO_PHONE)
		{
			//20ms����һ������
			if(Is_timing_X10us(2000,DELAY_BLE_SEND_TIMING_20ms))
			{
				fill_buffer_send(send2Phone_buffer);
				for(uint8_t i=0;i<sizeof(send2Phone_buffer);i++)
				{ 
					while(USART_GetFlagStatus(USART1,USART_FLAG_TC )==RESET);	
					USART_SendData(USART1,send2Phone_buffer[i]);
				}
				send_state=SEND_NONE;
			}
		}
	}
		
#if 0	
		//ͨ�������������������ݵ��ֻ����˴���ר������debug
			uint8_t data1[19]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x10,
												0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19};
//			0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x30,
//			0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x40,
//												0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x50,
//												0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x60,
//				0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x70,
//				0x71,0x72,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x80,
//				0x81,0x82,0x83};
												
		for(uint8_t i=0;i<sizeof(data1);i++)
		{ 
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC )==RESET);	
			USART_SendData(USART1,data1[i]);
		}
												
//			USART_BLUETOOTH_SendBuf(data1);
//			send_cnt+=sizeof(data1);	
			//GPIO_ResetBits(GPIOB,GPIO_Pin_9);
#endif
		
		os_delay_10us(KEY_SEND_DATA_TO_PHONE_TASK_ID, SEND_DATA_TO_PHONE_PERIOD);//10*10us=100us
}

