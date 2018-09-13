#ifndef _MS5525DSO_SAMPLING_DATA
#define _MS5525DSO_SAMPLING_DATA

#define MS5525DSO_SAMPLING_DATA_PERIOD 10


typedef enum
{
	MS5525DSO_START,
	MS5525DSO_NONE,
	MS5525DSO_DELAY_1000us_1,
	MS5525DSO_DELAY_1000us_2,
	MS5525DSO_SAMPLE_DATA_FINISH
	
}MS5525DSO_STATE;

void MS5525DSO_sampling_data(void);
int cal_diff_pressure_value(void);
#endif //_MS5525DSO_SAMPLING_DATA
