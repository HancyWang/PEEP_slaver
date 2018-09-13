#include "send_data_to_phone.h"
#include "serial_port.h"
#include "key_power_on_task.h"
#include "delay.h"
#include "i2c.h"
#include "comm_task.h"
#include "honeywell_sampling_data.h"
#include "MS5525DSO_sampling_data.h"
#include <string.h>
#include <math.h>

#define HONEYWELL_RATE			11110   //斜率
#define HONEYWELL_INTERCEPT 383196	//截距

extern MCU_STATE mcu_state;
//uint8_t flag=1;
//static int send_cnt;

typedef enum 
{
	SEND_PRE,
	SEND_INIT_TWO_SENSORS,
	SEND_DELAY_1000us_1,
	SEND_DELAY_1000us_2,
	SEND_DELAY_3000us,
	SEND_DATA_TO_PHONE,
	SEND_NONE
}SEND_STATE;


//unsigned int d1,d2;
uint16_t c1,c2,c3,c4,c5,c6;  //6个PROM的值

static SEND_STATE send_state=SEND_PRE;
static BOOL b_initADC_sensor=1;


unsigned int honeywell_data; 	 //压力传感器数值
int p_diff;										//压差传感器数值

static uint16_t send_cnt;							//
static uint16_t honeywell_buff[4];   
static uint16_t MS5525DSO_buff[4];


extern HONEYWELL_STATE honeywell_state;
extern MS5525DSO_STATE MS5525DSO_state;
extern int HONEYWELL_ZERO_POINT;
extern int MS5525DSO_ZERO_POINT;

//将4次采集到的数据填充到指定的p_buffer中
void fill_buffer_send(uint8_t* p_buffer)
{
	uint16_t sum=0;
	sum+=p_buffer[0];
	*p_buffer++=0xFF;  //帧头
	for(uint8_t i=0;i<4;i++)   //填充honeywell的数据
	{
		sum+=honeywell_buff[i];
		*p_buffer++=honeywell_buff[i]>>8;
		*p_buffer++=honeywell_buff[i]%256;
	}
	for(uint8_t i=0;i<4;i++)   //填充压差的数据
	{
		sum+=MS5525DSO_buff[i];
		*p_buffer++=MS5525DSO_buff[i]>>8;
		*p_buffer++=MS5525DSO_buff[i]%256;
	}
	*p_buffer++=sum>>8;  //填充checksum
	*p_buffer=sum%256;
	
	//清空buffer
	memset(honeywell_buff,0,4);
	memset(MS5525DSO_buff,0,4);
}
//Honeywell,将3个字节的ADC数据转换成2字节的PSI数据
uint16_t cal_honeywell_data(int data)
{
	int diff;
//	diff=data-419430;
	diff=data-HONEYWELL_ZERO_POINT;
	if(diff>0)
	{
		//3999=5.8*6.8948*100,(量程30kpa=5.8psi,1psi=6.8948kpa,1000表示放大1000倍，上位机需要自己除以1000来还原)
		//由于diff*3999会超过int的正量程(24亿),用uinsigned int 也就42亿，不够大
		//现在将3999改成400*10
		return diff*400/3355443*100;  
//		return diff*1900/11110;
	}
	else
	{
		return 0;
	}
}


//MS5525DSO,将3字节的ADC数据转换成2字节的流量数据
//这个算法待写
uint16_t cal_MS5525DSO_data(int data)
{
	static int diff;
	static int result;
	diff=data+(0-MS5525DSO_ZERO_POINT);
	if(diff>0)
	{
		result= sqrt(2*diff*13)*51*125/100;
		return result;
	}
}

void send_data_to_phone_task()
{
	//if(mcu_state==POWER_ON)
	{
		//1.准备
		if(send_state==SEND_PRE)  //延迟210ms,让蓝牙准备好,传感器初始化
		{
			if(Is_timing_X10us(200*100,DELAY_BEFORE_START))
			{
				send_state=SEND_NONE;
				
				//读取压差传感器PROM值，一共6个值
				c1=MS5525DSO_PROM_CX(C1);
				c2=MS5525DSO_PROM_CX(C2);
				c3=MS5525DSO_PROM_CX(C3);
				c4=MS5525DSO_PROM_CX(C4);
				c5=MS5525DSO_PROM_CX(C5);
				c6=MS5525DSO_PROM_CX(C6);
				
				honeywell_state=HONEYWELL_START;
				MS5525DSO_state=MS5525DSO_START;
			}
			else
			{
				if(b_initADC_sensor)
				{
					Init_honeywell_sensor();	//测试honeywell sensor
					Init_MS5525DSO_sensor(); //复位MS5525DSO
					b_initADC_sensor=0;		
				}
			}
		}
		
		if(honeywell_state==HONEYWELL_SAMPLE_DATA_FINISH&&MS5525DSO_state==MS5525DSO_SAMPLE_DATA_FINISH)
		{
				honeywell_data=honeywell_readByte();
				p_diff=cal_diff_pressure_value();
				
				honeywell_buff[send_cnt]=cal_honeywell_data(honeywell_data);   //需要计算
				MS5525DSO_buff[send_cnt]=cal_MS5525DSO_data(p_diff);    
				send_cnt++;
				
				if(send_cnt==4)
				{
					send_state=SEND_DATA_TO_PHONE;
					
					honeywell_state=HONEYWELL_START;
					MS5525DSO_state=MS5525DSO_START;
				}
		}
		
		if(send_state==SEND_DATA_TO_PHONE)
		{
			static uint8_t buffer[19];
			fill_buffer_send(buffer);
			for(uint8_t i=0;i<sizeof(buffer);i++)
			{ 
				while(USART_GetFlagStatus(USART1,USART_FLAG_TC )==RESET);	
				USART_SendData(USART1,buffer[i]);
			}
			static int debug_data_cnt;
			debug_data_cnt+=19;
			
			send_cnt=0;
			send_state=SEND_NONE;
			
		}
	}
		
#if 0		
//		//通过串口蓝牙，发送数据到手机，此代码专门用来debug
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
		
		os_delay_10us(KEY_SEND_DATA_TO_PHONE_TASK_ID, SEND_DATA_TO_PHONE_PERIOD);//10*10us=100us
}
	

