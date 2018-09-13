#include "MS5525DSO_sampling_data.h"
#include "os_core.h"
#include "app.h"
#include "i2c.h"
#include "comm_task.h"


extern uint16_t c1,c2,c3,c4,c5,c6;  //6个PROM的值

unsigned int d1,d2;


extern int p_diff;			

MS5525DSO_STATE MS5525DSO_state=MS5525DSO_NONE;
 int MS5525DSO_ZERO_POINT;
static BOOL b_getMS5525DSO_ZeroPoint=1;

int cal_diff_pressure_value()
{
	//计算压力值
	 static __int64 dT,TEMP,OFF,SENS;
	 int P;
	dT=d2-(c5<<7);  //D2 - C5 * 2^7
	TEMP=2000+((dT*c6)>>21);  //TEMP=2000+dT*C6/2^21
	OFF=(c2<<17)+((c4*dT)>>5);   //OFF=C2*2^17 +(C4*dT)/2^5
	SENS=(c1<<15)+((c3*dT)>>7); //C1*2^15 +(C3*dT)/2^7
	P=(d1*(SENS>>21)-OFF)>>15;    //(D1*SENS/2^21 -OFF)/2^15
	return P;
}

void MS5525DSO_sampling_data()
{
	if(MS5525DSO_state==MS5525DSO_START)
	{
		MS5525DSO_prepare_to_read(D1);
		MS5525DSO_state=MS5525DSO_DELAY_1000us_1;
	}
	
	if(MS5525DSO_state==MS5525DSO_DELAY_1000us_1)
	{
		if(Is_timing_X10us(100,DELAY_1000us_1))
		{
			d1=MS5525DSO_readByte();
			MS5525DSO_prepare_to_read(D2);
			
			MS5525DSO_state=MS5525DSO_DELAY_1000us_2;
		}
	}
	
	if(MS5525DSO_state==MS5525DSO_DELAY_1000us_2)
	{
		if(Is_timing_X10us(100,DELAY_1000us_2))
		{
			d2=MS5525DSO_readByte();
			p_diff=cal_diff_pressure_value();
			if(b_getMS5525DSO_ZeroPoint)   //获取零点值，该值为负数
			{
				MS5525DSO_ZERO_POINT=p_diff;
				b_getMS5525DSO_ZeroPoint=0;
			}
			
			MS5525DSO_state=MS5525DSO_SAMPLE_DATA_FINISH;
		}
	}
	
	os_delay_10us(MS5525DSO_SAMPLING_DATA_TASK_ID, MS5525DSO_SAMPLING_DATA_PERIOD);
}


