#ifndef _I2C_H
#define _I2C_H

#include "common.h"


//定义honewell传感器(采集压力值)
#define HONNEYWELL_IO_PORT		GPIOB
#define HONNEYWELL_I2C_SCL   	GPIO_Pin_4
#define HONNEYWELL_I2C_SDA		GPIO_Pin_3
#define HONNEYWELL_OFFSET								3         //根据SDA的引脚值

//#define HONNEYWELL_IO_PORT		GPIOB
//#define HONNEYWELL_I2C_SCL   	GPIO_Pin_4
//#define HONNEYWELL_I2C_SDA		GPIO_Pin_3
//#define HONNEYWELL_IO_PORT		GPIOA
//#define HONNEYWELL_I2C_SCL   	GPIO_Pin_9
//#define HONNEYWELL_I2C_SDA		GPIO_Pin_10

#define SCL_L          		      HONNEYWELL_IO_PORT->BRR  = HONNEYWELL_I2C_SCL
#define SCL_H          		      HONNEYWELL_IO_PORT->BSRR = HONNEYWELL_I2C_SCL
#define SDA_L          		      HONNEYWELL_IO_PORT->BRR  = HONNEYWELL_I2C_SDA
#define SDA_H          		      HONNEYWELL_IO_PORT->BSRR = HONNEYWELL_I2C_SDA
#define SDA_READ                (HONNEYWELL_IO_PORT->IDR &  HONNEYWELL_I2C_SDA) >> HONNEYWELL_OFFSET

//定义压差传感器(采集压力差，最终转换成流量)
#define MS5525DSO_IO_PORT		GPIOB
#define MS5525DSO_I2C_SCL   GPIO_Pin_6
#define MS5525DSO_I2C_SDA		GPIO_Pin_5
#define MS5525DSO_OFFSET						5        //根据SDA的引脚值

#define MS5525DSO_SCL_L          		      MS5525DSO_IO_PORT->BRR  = MS5525DSO_I2C_SCL
#define MS5525DSO_SCL_H          		      MS5525DSO_IO_PORT->BSRR = MS5525DSO_I2C_SCL
#define MS5525DSO_SDA_L          		      MS5525DSO_IO_PORT->BRR  = MS5525DSO_I2C_SDA
#define MS5525DSO_SDA_H          		      MS5525DSO_IO_PORT->BSRR = MS5525DSO_I2C_SDA
#define MS5525DSO_SDA_READ                (MS5525DSO_IO_PORT->IDR &  MS5525DSO_I2C_SDA) >> MS5525DSO_OFFSET

typedef enum 
{
	C1,
	C2,
	C3,
	C4,
	C5,
	C6
}PROM_CX;

typedef enum
{
	D1,
	D2
}DX;

extern void I2C_uConfiguration(void);
/*
extern void SENSOR_IIC_uWriteByte(INT8U slaveaddr,INT8U regaddr,INT8U dat);
extern void SENSOR_IIC_uMultipleRead(INT8U slaveaddr,INT8U regaddr,INT8U len,INT8U *pBuffer);
*/
//extern void HMCxx_uWriteByte(INT8U slaveaddr,INT8U writeaddr,INT8U data);
//extern void HMCxx_uMultipleRead(INT8U slaveaddr,INT8U readaddr,INT8U len,INT8U *pBuffer);
//extern INT8U AT24xx_uReadByte(INT8U slaveaddr,INT16U readaddr);
//extern void AT24xx_uWriteByte(INT8U slaveaddr,INT16U writeaddr,INT8U data);
//extern void AT24xx_uMultipleRead(INT8U slaveaddr,INT16U readaddr,INT8U len,INT8U *pBuffer);
//extern void AT24xx_uMultipleWrite(INT8U slaveaddr,INT16U writeaddr,INT8U len,INT8U *pBuffer);
//extern INT16U MS5525DSO_PROM_SENS_T1(void);
//extern INT16U MS5525DSO_PROM_OFF_T1(void);
//extern INT16U MS5525DSO_PROM_TCS(void);
//extern INT16U MS5525DSO_PROM_TCO(void);
//extern INT16U MS5525DSO_TREF(void);
//extern INT16U MS5525DSO_TEMPSENS(void);
extern INT16U MS5525DSO_PROM_CX(PROM_CX cx);

extern void MS5525DSO_prepare_to_read(DX dx);
extern void Init_MS5525DSO_sensor(void);
extern INT32S MS5525DSO_readByte(void);
extern void ADS115_enter_power_down_mode(void);
extern void Init_honeywell_sensor(void);
extern void ADS115_writeByte(INT8U slaveaddr,INT8U data);
extern INT16U ADS115_readByte(INT8U slaveaddr);
void ADS115_enter_power_down_mode(void);
extern INT32U honeywell_readByte(void);
extern INT8U Honeywell_ready_for_read(void);
#endif

