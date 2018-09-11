#include "send_data_to_phone.h"
#include "serial_port.h"
#include "key_power_on_task.h"
#include "delay.h"
#include "i2c.h"
#include "comm_task.h"

#define HONEYWELL_RATE			11110   //б��
#define HONEYWELL_INTERCEPT 383196	//�ؾ�

extern MCU_STATE mcu_state;
//uint8_t flag=1;
//static int send_cnt;

typedef enum 
{
	PRE_SEND,
	COLLECT_ADC_DATA_AND_SEND,
	SEND_NONE
}SEND_STATE;

static SEND_STATE send_state=PRE_SEND;
static BOOL b_initADC_sensor=1;
static unsigned int d1,d2;
static uint16_t c1,c2,c3,c4,c5,c6;  //6��PROM��ֵ
static uint8_t delay_cnt;
static uint8_t MS5525DSO_cnt;

static unsigned int honeywell_data; 	 //ѹ����������ֵ
static int p_diff;										//ѹ�������ֵ
static uint16_t send_cnt;							//
static uint16_t honeywell_buff[4];   
static uint16_t MS5525DSO_buff[4];

static uint32_t debug_os_tick_prev;
static uint32_t debug_os_tick_after;
extern uint32_t os_ticks;
static uint8_t delay_flag=1;

int cal_diff_pressure_value()
{
	 __int64 dT,TEMP,OFF,SENS;
	 int P;
	dT=d2-c5*128;  //D2 - C5 * 2^7
	TEMP=2000+dT*c6/2097152;  //TEMP=2000+dT*C6/2^21
	OFF=c2*131072+(c4*dT)/32;   //OFF=C2*2^17 +(C4*dT)/2^5
	SENS=c1*32768+(c3*dT)/128; //C1*2^15 +(C3*dT)/2^7
	P=(d1*SENS/2097152-OFF)/32768;    //(D1*SENS/2^21 -OFF)/2^15
	return P;
}

void fill_buffer_send(uint8_t* p_buffer)
{
	uint16_t sum=0;
	sum+=p_buffer[0];
	*p_buffer++=0xFF;  //֡ͷ
	for(uint8_t i=0;i<4;i++)   //���honeywell������
	{
		sum+=honeywell_buff[i];
		*p_buffer++=honeywell_buff[i]/256;
		*p_buffer++=honeywell_buff[i]%256;
	}
	for(uint8_t i=0;i<4;i++)   //���ѹ�������
	{
		sum+=MS5525DSO_buff[i];
		*p_buffer++=MS5525DSO_buff[i]/256;
		*p_buffer++=honeywell_buff[i]%256;
	}
	*p_buffer++=sum/256;  //���checksum
	*p_buffer=sum%256;
}

//static uint8_t delay_3ms_flag=1;
//static uint8_t delay_6ms_flag=1;
//static uint8_t delay_24ms_flag=1;

void send_data_to_phone_task()
{
	//if(mcu_state==POWER_ON)
	{
		//1.׼��
		if(send_state==PRE_SEND)  //�ӳ�210ms,������׼����,��������ʼ��
		{
			if(Is_timing_Xmillisec(200,DELAY_BEFORE_START))
			{
				send_state=COLLECT_ADC_DATA_AND_SEND;   //����ɼ����ݽ׶�
				
				//��ȡѹ�����PROMֵ��һ��6��ֵ
				c1=MS5525DSO_PROM_CX(C1);
				c2=MS5525DSO_PROM_CX(C2);
				c3=MS5525DSO_PROM_CX(C3);
				c4=MS5525DSO_PROM_CX(C4);
				c5=MS5525DSO_PROM_CX(C5);
				c6=MS5525DSO_PROM_CX(C6);
			}
			else
			{
				if(b_initADC_sensor)
				{
					Init_honeywell_sensor();	//����honeywell sensor
					Init_MS5525DSO_sensor(); //��λMS5525DSO
					b_initADC_sensor=0;
				}
			}
		}
		
		//2.����flow sensor��pressure sensor��ADCֵ,��������
		if(send_state==COLLECT_ADC_DATA_AND_SEND)  //�ɼ�adcֵ
		{
//		
//			if(Is_timing_Xmillisec(3,DELAY_3ms))
//			{
//				send_cnt++;
//				d1=MS5525DSO_readByte();
//				MS5525DSO_prepare_to_read(D2);
//			}
//			else
//			{
//				if(delay_flag)
//				{
//					Init_honeywell_sensor();
//					MS5525DSO_prepare_to_read(D1);
//					delay_flag=0;
//				}
//			}
//			

			
			
			
			
			
			
			
//			if(Is_timing_Xmillisec(6,DELAY_6ms))
//			{

//				honeywell_data=honeywell_readByte();  
//				d2=MS5525DSO_readByte();  
//				p_diff=cal_diff_pressure_value();
//				uint16_t honeywell_psi=190*(honeywell_data-HONEYWELL_INTERCEPT)/HONEYWELL_RATE;   //�����ʽ����֤
//				honeywell_buff[send_cnt]=honeywell_psi; 
//				MS5525DSO_buff[send_cnt]=p_diff;
//			}
//		
//			
//			if(Is_timing_Xmillisec(24,DELAY_24ms))
//			{
//				uint8_t buffer_send[19];
//				//���buffer_send[19]
//				fill_buffer_send(buffer_send);
//				//����
//				for(uint8_t i=0;i<sizeof(buffer_send);i++)
//				{ 
//					while(USART_GetFlagStatus(USART1,USART_FLAG_TC )==RESET);	
//					USART_SendData(USART1,buffer_send[i]);
//				}
//			}
//			

			
			
			
#if 0		
			if(MS5525DSO_cnt==2)
			{
				MS5525DSO_cnt=0;
				honeywell_data=honeywell_readByte();   //honeywell����Ҫ5ms��ʱ�����ɼ�ת������
				d2=MS5525DSO_readByte();  //��ȡѹ�����d2ֵ
				
				//����ѹ��ֵ
				p_diff=cal_diff_pressure_value();
				
				if(2*send_cnt*SEND_DATA_TO_PHONE_PERIOD==24)
				{
					uint8_t buffer_send[19];
					//���buffer_send[19]
					fill_buffer_send(buffer_send);
					//����
					for(uint8_t i=0;i<sizeof(buffer_send);i++)
					{ 
						while(USART_GetFlagStatus(USART1,USART_FLAG_TC )==RESET);	
						USART_SendData(USART1,buffer_send[i]);
					}
					debug_os_tick_after=os_ticks;
					send_cnt=0;
				}
				else
				{
					//ת�����ݳ�psi,����������ģ����ݻ�ûת��������
					uint16_t honeywell_psi=190*(honeywell_data-HONEYWELL_INTERCEPT)/HONEYWELL_RATE;   //�����ʽ����֤
					honeywell_buff[send_cnt]=honeywell_psi; 
					MS5525DSO_buff[send_cnt]=p_diff;
					send_cnt++;
				}
			}
			else
			{
				if(MS5525DSO_cnt==0)
				{
					debug_os_tick_prev=os_ticks;
					Init_honeywell_sensor();
					MS5525DSO_prepare_to_read(D1);
				}
				
				if(MS5525DSO_cnt==1)
				{
					d1=MS5525DSO_readByte();  //�ӳ�3ms��ȡѹ�����d1ֵ
					MS5525DSO_prepare_to_read(D2);  //׼����ȡd2
				}
				
				MS5525DSO_cnt++;
			}
#endif
		}
	}
		
#if 0		
//		//2.ͨ�������������������ݵ��ֻ�
//			uint8_t data1[19]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x10,
//												0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19};
////			0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x30,
////			0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x40,
////												0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x50,
////												0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x60,
////				0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x70,
////				0x71,0x72,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x80,
////				0x81,0x82,0x83};
//												
//		for(uint8_t i=0;i<sizeof(data1);i++)
//		{ 
//			while(USART_GetFlagStatus(USART1,USART_FLAG_TC )==RESET);	
//			USART_SendData(USART1,data1[i]);
//		}
//												
////			USART_BLUETOOTH_SendBuf(data1);
//			send_cnt+=sizeof(data1);	
//			//GPIO_ResetBits(GPIOB,GPIO_Pin_9);
#endif
		
		
		os_delay_ms(KEY_SEND_DATA_TO_PHONE_TASK_ID, SEND_DATA_TO_PHONE_PERIOD); //3ms��һ��
}
	

