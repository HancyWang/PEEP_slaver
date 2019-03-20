#include "MPXV70_sampling_data.h"
#include "app.h"
#include "datatype.h"
#include "comm_task.h"
#include "key_power_on_task.h"

#define MPVX70_DIVIDE 825

int16_t MPVX70_buffer[MPXV70_SAMPLE_CNT];  // ׼������buffer

MPXV70_STATE MPXV70_state=MPXV70_SAMPLE_DATA;

extern uint16_t RegularConvData_Tab[1];

BOOL MPXV70_SAMPLE_DATA_flag=TRUE;
uint32_t MPXV70_prev_SAMPLE_DATA_1000us_os_tick;

uint8_t MPXV70_sample_Cnt=0;
int16_t MPXV70_zero_point;  //���
BOOL b_MPXV70_get_zero_point=FALSE;
static int16_t pressure_value;

extern MCU_STATE mcu_state;
//MPXV70,ѹ��
void MPXV70_sampling_data()
{
	#ifdef _DEBUG
	#else
	if(mcu_state==POWER_ON)
	#endif
	{
		if(MPXV70_state==MPXV70_SAMPLE_DATA)
		{
			if(Is_timing_X10us(200,DELAY_MPXV70_SAMPLE_DATA))   //200����2ms�ɼ�һ��
			{
				if(!b_MPXV70_get_zero_point)
				{
					MPXV70_zero_point=RegularConvData_Tab[0];
					b_MPXV70_get_zero_point=TRUE;
				}
				else
				{
					//���ݹ�ʽy=825x+zero_point���ƹ������� x=(y-zero_point)/825,����xΪѹ��ֵ��yΪadcֵ
					pressure_value=10000*(RegularConvData_Tab[0]-MPXV70_zero_point)/MPVX70_DIVIDE;
					
//					pressure_value=(10000*(RegularConvData_Tab[0])/4096-5000)*5;
					MPVX70_buffer[MPXV70_sample_Cnt]=pressure_value;
	//				MPVX70_buffer[MPXV70_sample_Cnt]=(10000*RegularConvData_Tab[0]/4096-5000)*5;  //�Ŵ�10000����500=0.05pa��10000=1pa
					MPXV70_sample_Cnt++;
				}
			}
		}
		
		if(MPXV70_sample_Cnt==MPXV70_SAMPLE_CNT)  //�ɼ�5��
		{
			MPXV70_sample_Cnt=0;
			MPXV70_state=MPXV70_SAMPLE_FINISH;
		}
	}

	os_delay_10us(MPXV70_SAMPLING_DATA_TASK_ID, MPXV70_SAMPLING_DATA_PERIOD);
}
