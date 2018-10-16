#ifndef _SEND_DATA_TO_PHONE_H
#define _SEND_DATA_TO_PHONE_H

#include "os_core.h"
#include "app.h"

#define SEND_DATA_TO_PHONE_PERIOD 10

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

void send_data_to_phone_task(void);


#endif //_SEND_DATA_TO_PHONE_H
