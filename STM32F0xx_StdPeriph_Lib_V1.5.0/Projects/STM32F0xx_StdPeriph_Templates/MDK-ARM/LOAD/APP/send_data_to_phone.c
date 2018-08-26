#include "send_data_to_phone.h"

void send_data_to_phone_task()
{
	//1.采样flow sensor和pressure sensor的ADC值
	
	//2.通过串口wifi，发送数据到手机
	
	
	os_delay_ms(KEY_SEND_DATA_TO_PHONE_TASK_ID, SEND_DATA_TO_PHONE_PERIOD);
}
