#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"

#include "i2c.h"

void I2C_uConfiguration(void)
{
	GPIO_InitTypeDef   GPIO_uInitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
	
	GPIO_uInitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_uInitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_uInitStructure.GPIO_OType = GPIO_OType_OD;  
	GPIO_uInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_uInitStructure.GPIO_Speed = GPIO_Speed_10MHz;  
	GPIO_Init(GPIOA,&GPIO_uInitStructure);
	//GPIO_SetBits(GPIOA,GPIO_Pin_9|GPIO_Pin_10); 	
}


void I2C_SDA_OUT(void)
{
	GPIO_InitTypeDef   GPIO_uInitStructure;
	
	GPIO_uInitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_uInitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_uInitStructure.GPIO_OType = GPIO_OType_OD;  
	GPIO_uInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_uInitStructure.GPIO_Speed = GPIO_Speed_Level_3;  // 10M
	GPIO_Init(GPIOA,&GPIO_uInitStructure);
}

void I2C_SDA_IN(void)
{
	GPIO_InitTypeDef   GPIO_uInitStructure;
	
	GPIO_uInitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_uInitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_uInitStructure.GPIO_OType = GPIO_OType_OD;  
	GPIO_uInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_uInitStructure.GPIO_Speed = GPIO_Speed_Level_3;  // 10M
	GPIO_Init(GPIOA,&GPIO_uInitStructure);
}

void I2C_Start(void)
{
	I2C_SDA_OUT();
	
	SDA_H;
	SCL_H;
	Delay_us(5);
	SDA_L;
	Delay_us(5);
	SCL_L;
}

void I2C_Stop(void)
{
	//I2C_SDA_OUT();
	
  SDA_L;
  SCL_H;
  Delay_us(5);
  SDA_H;
  Delay_us(5);
}


void I2C_SendAck(void)
{
  I2C_SDA_OUT();
	
  SDA_L;
  SCL_H;
  Delay_us(5);
  SCL_L;
  Delay_us(5);
}


void I2C_SendNak(void)
{
  I2C_SDA_OUT();
	
  SDA_H;
	//SDA_L;
  SCL_H;
  Delay_us(5);
  SCL_L;
  Delay_us(5);
}


INT8U I2C_RecAck(void)
{
  INT8U RecAck=0;
	
  //I2C_SDA_IN();
	SDA_H;
	Delay_us(5);
  SCL_H;
  Delay_us(5);
	I2C_SDA_IN();
	Delay_us(20);
  RecAck = (INT8U)SDA_READ;
	I2C_SDA_OUT();
	Delay_us(5);
  SCL_L;
  Delay_us(25);
	
  return RecAck;
}





void I2C_SendByte(INT8U dat)
{
  INT8U i;
  //I2C_SDA_OUT();
  for(i=0;i<8;i++)
  {
    if(dat&0x80)
		{
      SDA_H;
		}
    else
		{
      SDA_L;
		}
    dat <<=1;
		Delay_us(5);
    SCL_H;
    Delay_us(5);
    SCL_L;
    Delay_us(5);
  }
	
}


INT8U I2C_RecByte(void)
{
  INT8U i,dat=0;

  I2C_SDA_IN();
	
  for(i=0;i<8;i++)
  {
    SCL_H;
    Delay_us(5);
    dat <<=1;
    dat |= SDA_READ;
    SCL_L;
    Delay_us(5);
  }
  return dat;
}

void ADS115_enter_power_down_mode()
{
	//配置为power-down模式
	I2C_uConfiguration();
  I2C_Start();
  I2C_SendByte(0x00);
	I2C_RecAck();
	I2C_SendByte(0x06);
	
//	//设置PA9和PA10
//	GPIO_InitTypeDef  GPIO_InitStructure;
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOF, ENABLE);
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
}


void Honeywell_ready_for_read()
{
	static INT8U data11;
	//while(1)
	{
		I2C_Start();
		I2C_SendByte(0x31);
		I2C_RecAck();
		data11=I2C_RecByte();
		I2C_SendNak();
		I2C_Stop();
//		if(data11==0x40)
//		{
//			return;
//		}
		Delay_us(5);
	}
}


//void Init_honeywell_sensor(void)
//{
//	I2C_uConfiguration();
//  I2C_Start();
//  I2C_SendByte(0x30);
//	I2C_RecAck();
//  I2C_SendByte(0xAA);
//	I2C_RecAck();
//	I2C_SendByte(0x00);  		
//	I2C_RecAck();
//	I2C_SendByte(0x00);
//	I2C_RecAck();
//  I2C_Stop();
//}

void Init_I2C_MP()
{
	I2C_uConfiguration();
  I2C_Start();
  I2C_SendByte(0x30);
	I2C_RecAck();
  I2C_SendByte(0xAA);
	I2C_RecAck();
	I2C_SendByte(0x00);  		
	I2C_RecAck();
	I2C_SendByte(0x00);
	I2C_RecAck();
  I2C_Stop();
}

INT32U honeywell_readByte()
{
	static INT8U data_staus,data23_16,data15_8,data7_0;
	I2C_Start();
	I2C_SendByte(0x31);
	I2C_RecAck();
	data_staus=I2C_RecByte();
	I2C_SendAck();
	data23_16=I2C_RecByte();
	I2C_SendAck();
	data15_8=I2C_RecByte();
	I2C_SendAck();
	data7_0=I2C_RecByte();
	I2C_SendNak();
	I2C_Stop();
	return data_staus*256*256*256+data23_16*256*256+data15_8*256+data7_0;
}

INT16U ADS115_readByte(INT8U slaveaddr)
{
	static INT8U data1,data2;
	I2C_Start();
	I2C_SendByte(slaveaddr);
	I2C_RecAck();
	I2C_SendByte(0x00);
	I2C_RecAck();
	I2C_Stop();
	I2C_Start();
	I2C_SendByte(slaveaddr+1);
	I2C_RecAck();
	data1=I2C_RecByte();
	I2C_SendAck();
	data2=I2C_RecByte();
	I2C_SendAck();
	I2C_Stop();
	return data1*256+data2;
}

