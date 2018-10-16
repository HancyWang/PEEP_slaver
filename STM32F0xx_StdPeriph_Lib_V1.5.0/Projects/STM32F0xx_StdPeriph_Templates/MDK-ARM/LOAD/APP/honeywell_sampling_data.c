#include "honeywell_sampling_data.h"
#include "os_core.h"
#include "app.h"
#include "i2c.h"
#include "comm_task.h"

static int honeywelldata;
static BOOL b_getHoneywellZeroPoint=1;
HONEYWELL_STATE honeywell_state=HONEYWELL_NONE;
int HONEYWELL_ZERO_POINT;

//void honeywell_sampling_data()
//{
//	if(honeywell_state==HONEYWELL_START)
//	{
//		Init_honeywell_sensor();
//		honeywell_state=HONEYWELL_WAIT_5ms;
//	}
//	
//	if(honeywell_state==HONEYWELL_WAIT_5ms)
//	{
//		if(Is_timing_X10us(5*100-1,HONEYWELL_DELAY_5000us))
//		{
//			honeywell_state=HONEYWELL_READ_DATA;
//		}
//	}
//	
//	if(honeywell_state==HONEYWELL_READ_DATA)
//	{
//		honeywelldata=honeywell_readByte();
//		if(b_getHoneywellZeroPoint==1) //考虑到会有零飘,在第一次读数据的时候获取零点值
//		{
//			HONEYWELL_ZERO_POINT=honeywelldata;
//			b_getHoneywellZeroPoint=0;
//		}
//		honeywell_state=HONEYWELL_SAMPLE_DATA_FINISH;
//	}
//	
//	os_delay_10us(HONEYWELL_SAMPLING_DATA_TASK_ID, HONEYWELL_SAMPLING_DATA_PERIOD);
//}

