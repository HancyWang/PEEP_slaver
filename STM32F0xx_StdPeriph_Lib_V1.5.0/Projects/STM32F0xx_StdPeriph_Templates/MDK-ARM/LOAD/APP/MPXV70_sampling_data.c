#include "MPXV70_sampling_data.h"
#include "app.h"
#include "datatype.h"
#include "comm_task.h"

int16_t MPVX70_buffer[MPXV70_SAMPLE_CNT];  // 准备数据buffer

MPXV70_STATE MPXV70_state=MPXV70_SAMPLE_DATA;

extern uint16_t RegularConvData_Tab[1];

BOOL MPXV70_SAMPLE_DATA_flag=TRUE;
uint32_t MPXV70_prev_SAMPLE_DATA_1000us_os_tick;

uint8_t MPXV70_sample_Cnt=0;

//MPXV70,压力
void MPXV70_sampling_data()
{
//	static int16_t pressure_value=0;
	if(MPXV70_state==MPXV70_SAMPLE_DATA)
	{
		if(Is_timing_X10us(100,DELAY_MPXV70_SAMPLE_DATA))   //1ms
		{
			MPVX70_buffer[MPXV70_sample_Cnt]=(10000*RegularConvData_Tab[0]/4096-5000)*5;  //放大10000倍，500=0.05pa，10000=1Kpa
			MPXV70_sample_Cnt++;
		}
	}
	
	if(MPXV70_sample_Cnt==MPXV70_SAMPLE_CNT)  //采集5次
	{
		MPXV70_sample_Cnt=0;
		MPXV70_state=MPXV70_SAMPLE_FINISH;
	}
	
	os_delay_10us(MPXV70_SAMPLING_DATA_TASK_ID, MPXV70_SAMPLING_DATA_PERIOD);
}
