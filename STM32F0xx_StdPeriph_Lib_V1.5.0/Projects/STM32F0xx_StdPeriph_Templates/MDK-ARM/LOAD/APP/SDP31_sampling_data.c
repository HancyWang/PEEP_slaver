#include "SDP31_sampling_data.h"
#include "app.h"
#include "i2c.h"
#include "delay.h"
#include "datatype.h"
#include "comm_task.h"
#include <math.h>

int16_t SDP31_buffer[SDP31_SAMPLE_CNT];

SDP31_STATE SDP31_state=SDP31_CONFIG_IO;

BOOL b_send_cmd15_8=1;
BOOL b_reset=1;

//控制SDP31定时时间的相关变量
BOOL SDP31_dealy_after_reset_flag=TRUE;
BOOL SDP31_SEND_CMD2_flag=TRUE;
BOOL SDP31_READ_DATA_flag=TRUE;

uint32_t SDP31_prev_DELAY_20000us_os_tick;
uint32_t SDP31_prev_SEND_CMD2_20000us_os_tick;
uint32_t SDP31_prev_READ_DATA_1000us_os_tick;

uint8_t SDP31_sample_Cnt=0;  //记录采样次数

//uint16_t switch_2_flow_data_from(int data)
//{
//	static int diff;
//	static int result;
////	diff=data+(0-MS5525DSO_ZERO_POINT);
////	if(diff>0)
//	{
//		result= sqrt(2*data*13)*51*125/100;  //这个公式还没有意义，等结构件出来才可以
//		return result;
//	}
//}

//SDP31，压差
void SDP31_sampling_data()
{
	#if 0
//	//debug
//	while(1)
//	{
//		SDP31_I2C_Configuration();
//		SDP31_reset();
//		delay_ms(20);
//		SDP31_send_read_cmd_15_8();
//		delay_ms(7);
//		SDP31_send_read_cmd_7_0();
//	}
	#endif

	//经过调试发现
	//1.要先初始化reset这个sensor(它有continue和trigger模式,选了一种想用另一种的话要先初始化)
	//2.初始化之后发送命令，这个命令只响应一次，重复发送就不响应了(这个好坑)
	if(SDP31_state==SDP31_CONFIG_IO)  //配置端口
	{
		SDP31_I2C_Configuration();
		SDP31_state=SDP31_RESET;
	}

	if(SDP31_state==SDP31_RESET)   	//使用前复位
	{ 
		if(Is_timing_X10us(2000,DELAY_SDP31_DELAY_AFTER_RESET))  //定时20ms
		{
			SDP31_state=SDP31_SEND_READ_CMD;
		}
		else
		{
			if(b_reset==1)
			{
				SDP31_reset();
				b_reset=0;
			}
		}
	}
	
	if(SDP31_state==SDP31_SEND_READ_CMD)
	{
		//SDP31发送读取命令
		if(Is_timing_X10us(2000,DELAY_SDP31_SEND_CMD2))  //定时20ms
		{
			SDP31_send_read_cmd_7_0();
			SDP31_state=SDP31_READ_DATA;
		}
		else
		{
			if(b_send_cmd15_8==1)   //发送第一个命令,发送完毕之后就不进入
			{
				SDP31_send_read_cmd_15_8();  
				b_send_cmd15_8=0;
			}
		}
	}
	else if(SDP31_state==SDP31_READ_DATA)
	{
		static int32_t nData=0;
		static int16_t result;
		uint16_t compensation;
		if(Is_timing_X10us(100,DELAY_SDP31_READ_DATA))  //1ms采集一次
		{
			result=SDP31_read_data();
			if(result>=0)
			{
				nData=result;
				compensation=17;
				
				SDP31_buffer[SDP31_sample_Cnt]=(int16_t)(43*sqrt(2*nData*100/60*13)*compensation/100);
			}
			else
			{
				nData=0-result;
				compensation=21;
				
				SDP31_buffer[SDP31_sample_Cnt]=0-(int16_t)(43*sqrt(2*nData*100/60*13)*compensation/100);
			}
//			SDP31_buffer[SDP31_sample_Cnt]=(int16_t)(43*sqrt(2*nData*100/60*13)*compensation/100);

			SDP31_sample_Cnt++;
		}
	}
	
	if(SDP31_sample_Cnt==SDP31_SAMPLE_CNT)   //采样5次之后停止
	{
		SDP31_sample_Cnt=0;
		SDP31_state=SDP31_SAMPLE_FINISH;
	}
	
	os_delay_10us(SDP31_SAMPLING_DATA_TASK_ID, SDP31_SAMPLING_DATA_PERIOD);  
}
