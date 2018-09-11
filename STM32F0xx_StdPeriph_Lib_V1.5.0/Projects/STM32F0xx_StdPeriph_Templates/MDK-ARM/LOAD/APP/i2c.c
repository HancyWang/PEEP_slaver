#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"

#include "i2c.h"

//honeywell传感器
void I2C_uConfiguration(void)
{
	GPIO_InitTypeDef   GPIO_uInitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
	
	GPIO_uInitStructure.GPIO_Pin = HONNEYWELL_I2C_SCL | HONNEYWELL_I2C_SCL;
	GPIO_uInitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_uInitStructure.GPIO_OType = GPIO_OType_OD;  
	GPIO_uInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_uInitStructure.GPIO_Speed = GPIO_Speed_Level_3;  
	GPIO_Init(HONNEYWELL_IO_PORT,&GPIO_uInitStructure);	
}
void MS5525DSO_I2C_Configuration(void)
{
	GPIO_InitTypeDef   GPIO_uInitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
	
	GPIO_uInitStructure.GPIO_Pin = MS5525DSO_I2C_SCL | MS5525DSO_I2C_SDA;
	GPIO_uInitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_uInitStructure.GPIO_OType = GPIO_OType_OD;  
	GPIO_uInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_uInitStructure.GPIO_Speed = GPIO_Speed_Level_3;  
	GPIO_Init(MS5525DSO_IO_PORT,&GPIO_uInitStructure);	
}


void I2C_SDA_OUT(void)
{
	GPIO_InitTypeDef   GPIO_uInitStructure;
	
	GPIO_uInitStructure.GPIO_Pin = HONNEYWELL_I2C_SDA;
	GPIO_uInitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_uInitStructure.GPIO_OType = GPIO_OType_OD;  
	GPIO_uInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_uInitStructure.GPIO_Speed = GPIO_Speed_Level_3;  // 10M
	GPIO_Init(HONNEYWELL_IO_PORT,&GPIO_uInitStructure);
}
void MS5525DSO_I2C_SDA_OUT(void)
{
	GPIO_InitTypeDef   GPIO_uInitStructure;
	
	GPIO_uInitStructure.GPIO_Pin = MS5525DSO_I2C_SDA;
	GPIO_uInitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_uInitStructure.GPIO_OType = GPIO_OType_OD;  
	GPIO_uInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_uInitStructure.GPIO_Speed = GPIO_Speed_Level_3;  // 10M
	GPIO_Init(MS5525DSO_IO_PORT,&GPIO_uInitStructure);
}


void I2C_SDA_IN(void)
{
	GPIO_InitTypeDef   GPIO_uInitStructure;
	
	GPIO_uInitStructure.GPIO_Pin = HONNEYWELL_I2C_SDA;
	GPIO_uInitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_uInitStructure.GPIO_OType = GPIO_OType_OD;  
	GPIO_uInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_uInitStructure.GPIO_Speed = GPIO_Speed_Level_3;  // 10M
	GPIO_Init(HONNEYWELL_IO_PORT,&GPIO_uInitStructure);
}
void MS5525DSO_I2C_SDA_IN(void)
{
	GPIO_InitTypeDef   GPIO_uInitStructure;
	
	GPIO_uInitStructure.GPIO_Pin = MS5525DSO_I2C_SDA;
	GPIO_uInitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_uInitStructure.GPIO_OType = GPIO_OType_OD;  
	GPIO_uInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_uInitStructure.GPIO_Speed = GPIO_Speed_Level_3;  // 10M
	GPIO_Init(MS5525DSO_IO_PORT,&GPIO_uInitStructure);
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
void MS5525DSO_I2C_Start(void)
{
	MS5525DSO_I2C_SDA_OUT();
	
	MS5525DSO_SDA_H;
	MS5525DSO_SCL_H;
	Delay_us(5);
	MS5525DSO_SDA_L;
	Delay_us(5);
	MS5525DSO_SCL_L;
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
void MS5525DSO_I2C_Stop(void)
{
	MS5525DSO_SDA_L;
  MS5525DSO_SCL_H;
  Delay_us(5);
  MS5525DSO_SDA_H;
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
void MS5525DSO_I2C_SendAck(void)
{
	MS5525DSO_I2C_SDA_OUT();
	
  MS5525DSO_SDA_L;
  MS5525DSO_SCL_H;
  Delay_us(5);
  MS5525DSO_SCL_L;
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
void MS5525DSO_I2C_SendNak(void)
{
	MS5525DSO_I2C_SDA_OUT();
	
  MS5525DSO_SDA_H;
	//SDA_L;
  MS5525DSO_SCL_H;
  Delay_us(5);
  MS5525DSO_SCL_L;
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
INT8U MS5525DSO_I2C_RecAck(void)
{
	INT8U RecAck=0;
	
  //I2C_SDA_IN();
	MS5525DSO_SDA_H;
	Delay_us(5);
  MS5525DSO_SCL_H;
  Delay_us(5);
	MS5525DSO_I2C_SDA_IN();
	Delay_us(20);
  RecAck = (INT8U)MS5525DSO_SDA_READ;
	MS5525DSO_I2C_SDA_OUT();
	Delay_us(5);
  MS5525DSO_SCL_L;
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
void MS5525DSO_I2C_SendByte(INT8U dat)
{
	INT8U i;
  //I2C_SDA_OUT();
  for(i=0;i<8;i++)
  {
    if(dat&0x80)
		{
      MS5525DSO_SDA_H;
		}
    else
		{
      MS5525DSO_SDA_L;
		}
    dat <<=1;
		Delay_us(5);
    MS5525DSO_SCL_H;
    Delay_us(5);
    MS5525DSO_SCL_L;
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
INT8U MS5525DSO_I2C_RecByte(void)
{
	INT8U i,dat=0;

  MS5525DSO_I2C_SDA_IN();
	
  for(i=0;i<8;i++)
  {
    MS5525DSO_SCL_H;
    Delay_us(5);
    dat <<=1;
    dat |= MS5525DSO_SDA_READ;
    MS5525DSO_SCL_L;
    Delay_us(5);
  }
  return dat;
}

//void ADS115_enter_power_down_mode()
//{
//	//配置为power-down模式
//	I2C_uConfiguration();
//  I2C_Start();
//  I2C_SendByte(0x00);
//	I2C_RecAck();
//	I2C_SendByte(0x06);
//	
////	//设置PA9和PA10
////	GPIO_InitTypeDef  GPIO_InitStructure;
////	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOF, ENABLE);
////	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
////	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
////	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
////	GPIO_Init(GPIOA, &GPIO_InitStructure);
////	
////	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
////	GPIO_Init(GPIOA, &GPIO_InitStructure);
//}


INT8U Honeywell_ready_for_read()
{
	 INT8U data11;
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
	return data11;
}


void Init_honeywell_sensor(void)
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

void Init_MS5525DSO_sensor(void)   //复位操作
{
	MS5525DSO_I2C_Configuration();
	MS5525DSO_I2C_Start();
	MS5525DSO_I2C_SendByte(0xEE);     //0x  111 0111 0,前7位是地址0x77,倒数的两位1-CSB,0-W
	MS5525DSO_I2C_RecAck();
	MS5525DSO_I2C_SendByte(0x1E);     
	MS5525DSO_I2C_RecAck();
	MS5525DSO_I2C_Stop();
}


INT16U MS5525DSO_PROM_CX(PROM_CX cx)
{
	INT8U data15_8,data7_0;
	MS5525DSO_I2C_Start();
	MS5525DSO_I2C_SendByte(0xEE);     
	MS5525DSO_I2C_RecAck();
	switch(cx)
	{
		case C1:
			MS5525DSO_I2C_SendByte(0xA2); 
			break;
		case C2:
			MS5525DSO_I2C_SendByte(0xA4); 
			break;
		case C3:
			MS5525DSO_I2C_SendByte(0xA6); 
			break;
		case C4:
			MS5525DSO_I2C_SendByte(0xA8); 
			break;
		case C5:
			MS5525DSO_I2C_SendByte(0xAA); 
			break;
		case C6:
			MS5525DSO_I2C_SendByte(0xAE); 
			break;
		default:
			break;
	} 
	MS5525DSO_I2C_RecAck();
	MS5525DSO_I2C_Stop();
	
	MS5525DSO_I2C_Start();
	MS5525DSO_I2C_SendByte(0xEF);     
	MS5525DSO_I2C_RecAck();
	data15_8=MS5525DSO_I2C_RecByte();
	MS5525DSO_I2C_SendAck();
	data7_0=MS5525DSO_I2C_RecByte();
	MS5525DSO_I2C_SendNak();
	MS5525DSO_I2C_Stop();
	
	return data15_8*256+data7_0;
}

void MS5525DSO_prepare_to_read(DX dx)
{
	//I 2 C Command to initiate a pressure conversion
	MS5525DSO_I2C_Start();
	MS5525DSO_I2C_SendByte(0xEE);
	MS5525DSO_I2C_RecAck();  
	switch(dx)
	{
		case D1:
			MS5525DSO_I2C_SendByte(0x40);  
			break;
		case D2:
			MS5525DSO_I2C_SendByte(0x50);
			break;
		default:
			break;
	}
	MS5525DSO_I2C_RecAck();
	MS5525DSO_I2C_Stop();
}

INT32S MS5525DSO_readByte()
{
	INT8S data23_16,data15_8,data7_0;
		
	MS5525DSO_I2C_Start();
	MS5525DSO_I2C_SendByte(0xEE);
	MS5525DSO_I2C_RecAck();  
	MS5525DSO_I2C_SendByte(0X00);  
	MS5525DSO_I2C_RecAck();
	MS5525DSO_I2C_Stop();
	
	
	MS5525DSO_I2C_Start();
	MS5525DSO_I2C_SendByte(0xEF);  //0x  111 011 0 1,前6位是地址0x76,紧接着0-CSB,1-R
	MS5525DSO_I2C_RecAck();
	data23_16=MS5525DSO_I2C_RecByte();
	MS5525DSO_I2C_SendAck();
	data15_8=MS5525DSO_I2C_RecByte();
	MS5525DSO_I2C_SendAck();
	data7_0=MS5525DSO_I2C_RecByte();
	MS5525DSO_I2C_SendNak();
	MS5525DSO_I2C_Stop();
	
	return data23_16*256*256+data15_8*256+data7_0;
}

#if 0
////需要重写
//void Init_I2C_MP()
//{
////	I2C_uConfiguration();
////  I2C_Start();
////  I2C_SendByte(0x30);
////	I2C_RecAck();
////  I2C_SendByte(0xAA);
////	I2C_RecAck();
////	I2C_SendByte(0x00);  		
////	I2C_RecAck();
////	I2C_SendByte(0x00);
////	I2C_RecAck();
////  I2C_Stop();
//}
#endif

INT32U honeywell_readByte()
{
	INT8U data_staus,data23_16,data15_8,data7_0;
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

//这个sensor不用了，被honeywell取代了
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

