#ifndef _I2C_H
#define _I2C_H

#include "common.h"

#define SCL_L          		      GPIOA->BRR  = GPIO_Pin_9
#define SCL_H          		      GPIOA->BSRR = GPIO_Pin_9
#define SDA_L          		      GPIOA->BRR  = GPIO_Pin_10
#define SDA_H          		      GPIOA->BSRR = GPIO_Pin_10
#define SDA_READ                (GPIOA->IDR &  GPIO_Pin_10) >> 10

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

extern void ADS115_enter_power_down_mode(void);
extern void Init_honeywell_sensor(void);
extern void ADS115_writeByte(INT8U slaveaddr,INT8U data);
extern INT16U ADS115_readByte(INT8U slaveaddr);
void ADS115_enter_power_down_mode(void);
extern INT32U honeywell_readByte(void);
extern void Honeywell_ready_for_read(void);
#endif

