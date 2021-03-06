/**
********************************************************************************
* 版權：
* 模块名称：hardware.c
* 模块功能：
* 创建日期：
* 创 建 者：
* 声    明：
********************************************************************************
**/

/***********************************
* 头文件
***********************************/

#include "hardware.h"
#include "stm32f0xx.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_pwr.h"
//#include "stm32f0xx_rtc.h"
#include "stm32f0xx_dma.h"

#include "delay.h"
#include "os_cfg.h"
#include "datatype.h"

#include "time.h"
#include "i2c.h"
#include "comm_task.h"
#include "protocol_module.h"
#include "key_power_on_task.h"
#include "app.h"
#include "SDP31_sampling_data.h"
/**********************************
*宏定义
***********************************/
#define  SAMPLING_CNT 8
/***********************************
* 全局变量
***********************************/
unsigned short inner_adc_result[SAMPLING_CNT];
 uint16_t RegularConvData_Tab[1]; //ADC1值，PressureSen

int16_t zero_point_of_pressure_sensor;
extern uint8_t parameter_buf[PARAMETER_BUF_LEN];


extern const uint8_t default_parameter_buf[PARAMETER_BUF_LEN];
extern BOOL b_Is_PCB_PowerOn;

//extern uint8_t mode;
/***********************************
* 局部变量
***********************************/
//static //平年的月份日期表
//const uint8_t mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};
/***********************************
* 局部函数
***********************************/


////外部函数
//extern void I2C_SendByte(INT8U dat);
//extern void I2C_Stop(void); 
/**************************************************************
* 初始化OS滴答时钟
//使用TIM3
**************************************************************/
/*
void init_tim(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
  TIM_TimeBaseStructure.TIM_Period = SystemCoreClock/OS_TICKS_PER_SEC;           // 自动重装载寄存器周期的值(计数值) 
  TIM_TimeBaseStructure.TIM_Prescaler = 0;	//时钟预分频数 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;			//向上计数模式
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  TIM_ClearFlag(TIM3, TIM_FLAG_Update);			        // 清除溢出中断标志 
  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
  TIM_Cmd(TIM3, ENABLE);
}*/

/**************************************************************
* 初始化OS滴答时钟
//使用TIM16
**************************************************************/
void init_tim(void)
{
   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16,ENABLE);//外设时钟TIM16

  NVIC_InitStructure.NVIC_IRQChannel = TIM16_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
  TIM_TimeBaseStructure.TIM_Period = SystemCoreClock/OS_TICKS_PER_SEC-1;           //SystemCoreClock/OS_TICKS_PER_SEC     自动重装载寄存器周期的值(计数值) 
  TIM_TimeBaseStructure.TIM_Prescaler = 0;	//时钟预分频数 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;			//向上计数模式
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);
	//TIM_ARRPreloadConfig(TIM16,ENABLE);
	//TIM_PrescalerConfig(TIM16, 1, TIM_PSCReloadMode_Immediate);
//	TIM_BDTRInitTypeDef TIM_BDTRInitStruct;//新增

////	TIM_BDTRInitStruct.TIM_OSSRState=TIM_OSSRState_Enable; 
////	TIM_BDTRInitStruct.TIM_OSSIState=TIM_OSSIState_Enable; 
//	TIM_BDTRInitStruct.TIM_LOCKLevel=TIM_LOCKLevel_1; 
//	TIM_BDTRInitStruct.TIM_DeadTime=0x00; 
//	TIM_BDTRInitStruct.TIM_Break=TIM_Break_Disable; 
//	TIM_BDTRInitStruct.TIM_BreakPolarity=TIM_BreakPolarity_Low; 
//	TIM_BDTRInitStruct.TIM_AutomaticOutput=ENABLE; 
//	TIM_BDTRConfig(TIM16,&TIM_BDTRInitStruct);
	
  TIM_ClearFlag(TIM16, TIM_FLAG_Update);			        // 清除溢出中断标志 
  TIM_ITConfig(TIM16,TIM_IT_Update,ENABLE);	
	
	TIM_Cmd(TIM16, ENABLE);
}


/*******************************************************************************
** 函数名称: Set_Led
** 功能描述: 获取按键所对应的模式
** 输　  入: num-LED编号，ON_OFF-打开或关闭
** 输　  出: 无
** 全局变量: 无
** 调用模块: 无
*******************************************************************************/
void set_led(LED_ID id,BOOL ON_OFF)
{
	switch(id)
	{
		case LED_ID_GREEN:       //power green
			if(ON_OFF==TRUE)
			{
				GPIO_ResetBits(LED_PORT,LED_GREEN_PWR_PIN);
			}
			else 
			{
				GPIO_SetBits(LED_PORT,LED_GREEN_PWR_PIN);
			}
			break;
		case LED_ID_YELLOW:     //power yellow
			if(ON_OFF==TRUE)
			{
				GPIO_ResetBits(LED_PORT,LED_YELLOW_PWR_PIN);
			}
			else
			{
				GPIO_SetBits(LED_PORT,LED_YELLOW_PWR_PIN);
			}
			break;
		default:
			break;
	}
}

/**************************************************************
* 初始LED管脚
**************************************************************/
void Init_LED(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	//推挽输出
	GPIO_InitStructure.GPIO_Pin = LED_GREEN_PWR_PIN|LED_YELLOW_PWR_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LED_PORT, &GPIO_InitStructure);

//	GPIO_ResetBits(LED_PORT,LED_GREEN_PWR_PIN);
//	GPIO_ResetBits(LED_PORT,LED_YELLOW_PWR_PIN);
	GPIO_SetBits(LED_PORT,LED_GREEN_PWR_PIN);
	GPIO_SetBits(LED_PORT,LED_YELLOW_PWR_PIN);
}

void Init_PWRSAVE(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	//电源PWR_SAVE
	GPIO_InitStructure.GPIO_Pin = KEY_PWR_SAVE_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(KEY_PWR_SAVE_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(KEY_PWR_SAVE_PORT,KEY_PWR_SAVE_PIN);
}

void Init_Valve()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11);
}

void Init_Switch_Mode()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Init_Bat_Charge_Stby()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Init_PWR_EN()      //PA15
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_15);
}

void Init_PWR_ON_OFF()      //PB0
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Init_SWITCH_ON_OFF()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}




////校验pressure sensor
//void Calibrate_pressure_sensor(int16_t* p_zeroPoint)
//{
//	//如果是负压怎么办？应该定义成int16?值会不会变成负数？
//	uint16_t arr[10]={0};
//	uint16_t sum=0;
//	delay_ms(50);
////	GPIO_SetBits(GPIOB,GPIO_Pin_10);
////	GPIO_SetBits(GPIOB,GPIO_Pin_11);
////	
////	delay_ms(2000); 
////	GPIO_SetBits(GPIOB,GPIO_Pin_11);
////	GPIO_SetBits(GPIOB,GPIO_Pin_12);
//	
//	for(uint8_t i=0;i<10;i++)
//	{
//		arr[i]=ADS115_readByte(0x90);
//		sum+=arr[i];
//		delay_us(5);
//	}
//	*p_zeroPoint=sum/10;
//}

void Init_USB_OE()                    //PA0
{		
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void Init_Blue_WIFI_ONOFF()  //PA8
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);  //低电平唤醒蓝牙IC
//	GPIO_SetBits(GPIOA,GPIO_Pin_8); 
}

void Init_Blue_EN()        //PB15
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_15);  
}

//初始化USB_ON，PA15
void Init_USB_ON()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_15); 
}

void Init_STDBY_IO()          //PA1
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void Init_CHARGE_IO()    //PA2
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void Init_SENSOR_ONOFF()    //PB6
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_6); 
}


void Init_SDP31_IRQn()    //PB5
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_5); 
}

void Init_WIFI_RST()      //PB1
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_1); 
}

void Init_UART_BLUETOOTH_BRTS()  //PA8  设置高电平
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_8); 
}

void Init_UART_BLUETOOTH_BCTS()  //PB14  设置低电平
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//GPIO_ResetBits(GPIOB,GPIO_Pin_14); 
}

/**************************************************************
* 初始化硬件管脚
**************************************************************/
void init_hardware()
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB |RCC_AHBPeriph_GPIOC, ENABLE);//
	//delay_ms(1);
	//初始化LED灯                            //PA3,PA4
	Init_LED();
	
////	//初始化USB_OE，这里是5V0_AD
////	Init_USB_OE();  													//PA0
////	
////	//初始化USB_ON
////	Init_USB_ON();
//	
//	//初始化 pwr_on_off
//	Init_PWR_ON_OFF();												//PB0
//	
//	//初始化WIFI_RST                         //PB1
//	Init_WIFI_RST();

	//初始化Bluetooth&WIFI ON/PFF
	Init_Blue_WIFI_ONOFF();  									//PA8

//	//初始化WIFI串口RX,TX                     //PB10,PB11
//	Init_UART_WIFI(UART_BAUDRATE);
//	
//	//初始化BLUE_EN
//	Init_Blue_EN();														//PB15
	
	//初始化蓝牙串口RX,TX                     //PA9,PA10 
	Init_UART_BLUETOOTH(UART_BAUDRATE);
	
//	//蓝牙发送模式PA8置1，PB14置0
//	//初始化蓝牙BRTS
//	Init_UART_BLUETOOTH_BRTS();               //PA8
//	//初始化蓝牙BCTS
//	Init_UART_BLUETOOTH_BCTS();               //PB14
//	
//	//初始化STDBY_IO,                         //PA1
//	Init_STDBY_IO();

//	//初始化CHARGE_IO                         //PA2
//	Init_CHARGE_IO();

	Init_SDP31_IRQn();   												//PB5

	//初始化sensor_on/off 										//PB6
	Init_SENSOR_ONOFF();		

	//初始化ADC1的数据采集  									 //PA1,采集压力
	Init_ADC1();
	
//	//初始化ADC_MP  ,honneywell传感器
//	Init_I2C_MP();

//	SDP31_I2C_Configuration();
		
	//初始化ADC_Flow                         
//	Init_ADC_Flow();
}




/**************************************************************
* 板级硬件资源控制
**************************************************************/
//呼吸檢測脚狀態
//BOOL get_exp_status(void)
//{
//	return GPIO_ReadInputDataBit(EXP_DETECT_PORT, EXP_DETECT_PIN);
//}

//按键檢測脚狀態
//TRUE：按下
//FALSE:弹起
BOOL get_key_status(void)
{
	return GPIO_ReadInputDataBit(KEY_DETECT_PORT, KEY_DETECT_PIN);
}






////设置LED指示灯
//void set_led(LED_COLOR color)
//{	
//	switch(color)
//	{
//		case LED_CLOSE:
//			GPIO_SetBits(GREEN_LED_PORT, GREEN_LED_PIN);
//			GPIO_SetBits(RED_LED_PORT, RED_LED_PIN);
////			GPIO_SetBits(BLUE_LED_PORT, BLUE_LED_PIN);
//			break;
//		case LED_BLUE:
//			GPIO_SetBits(GREEN_LED_PORT, GREEN_LED_PIN);
//			GPIO_SetBits(RED_LED_PORT, RED_LED_PIN);
////			GPIO_ResetBits(BLUE_LED_PORT, BLUE_LED_PIN);
//			break;
//		case LED_RED:
//			GPIO_ResetBits(RED_LED_PORT, RED_LED_PIN);
//			GPIO_SetBits(GREEN_LED_PORT, GREEN_LED_PIN);			
////			GPIO_SetBits(BLUE_LED_PORT, BLUE_LED_PIN);
//			break;
//		case LED_GREEN:
//			GPIO_ResetBits(GREEN_LED_PORT, GREEN_LED_PIN);
//			GPIO_SetBits(RED_LED_PORT, RED_LED_PIN);
////			GPIO_SetBits(BLUE_LED_PORT, BLUE_LED_PIN);
//			break;
//		default:
//			break;
//	}
//}
/**************************************************************
* 电池电压检测
**************************************************************/
//uint8_t get_bat_vol_per(void)
//{
//	return 0;

//}


/**************************************************************
* RTC设置
**************************************************************/
//RTC RCC
#if 0
void init_rtc_rcc(void)
{
	//配置时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
}

//rTC配置
void config_rtc(void)
{
	RCC_LSICmd(ENABLE);//
	
	while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);//等待就绪
	
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
	
	RCC_RTCCLKCmd(ENABLE);
	RTC_WaitForSynchro();
}
#endif
#if 0
////RTC初始化
//void init_rtc(void)
//{
//	RTC_InitTypeDef RTC_InitStructure;
//	RTC_DateTypeDef RTC_DateStrcuture;
//	RTC_TimeTypeDef RTC_TimeStrcuture;
//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
//	
//	PWR_BackupAccessCmd(ENABLE);
//	
//	uint32_t dr = RTC_ReadBackupRegister(RTC_BKP_DR1);
//	
//	if(dr != 0x5051)//RTC_BKP_DR1
//	{
//		config_rtc();
//		
//		RTC_InitStructure.RTC_AsynchPrediv = 0x63;
//		RTC_InitStructure.RTC_SynchPrediv = 0x18f;
//		RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
//		
//		if(RTC_Init(&RTC_InitStructure) == ERROR)
//		{
//			//while(1);//初始化失败，许修改
//			//软件复位
//			
//		}
//		
//		RTC_DateStrcuture.RTC_Year = 16;
//		RTC_DateStrcuture.RTC_Month = 12;
//		RTC_DateStrcuture.RTC_Date = 23;
//		RTC_TimeStrcuture.RTC_Hours = 12;
//		RTC_TimeStrcuture.RTC_Minutes = 34;
//		RTC_TimeStrcuture.RTC_Seconds = 56;
//		RTC_SetDate(RTC_Format_BIN, &RTC_DateStrcuture);//RTC_Format_BIN
//		RTC_SetTime(RTC_Format_BIN, &RTC_TimeStrcuture);
//		
//		RTC_WriteBackupRegister(RTC_BKP_DR1, 0x5051);
//	}
//	else
//	{
//		RCC_LSICmd(ENABLE);
//		RTC_WaitForSynchro();
//	}
//}

////判断是否是闰年函数
////月份   1  2  3  4  5  6  7  8  9  10 11 12
////闰年   31 29 31 30 31 30 31 31 30 31 30 31
////非闰年 31 28 31 30 31 30 31 31 30 31 30 31
////year:年份
////返回值:该年份是不是闰年.1,是.0,不是
//uint8_t Is_Leap_Year(uint16_t year)
//{			  
//	if(year%4==0) //必须能被4整除
//	{ 
//		if(year%100==0) 
//		{ 
//			if(year%400==0)return 1;//如果以00结尾,还要能被400整除 	   
//			else return 0;   
//		}else return 1;   
//	}else return 0;	
//}	

////万历年转化
//void convert_rtc(_calendar_obj* calendar, uint32_t rtc)
//{
//	uint32_t temp = 0;
//	uint16_t temp1 = 0;
//	static uint16_t daycnt=0;
//	
//	temp = rtc/86400;   //得到天数(秒钟数对应的)
//	if(daycnt!=temp)//超过一天了
//	{	  
//		daycnt=temp;
//		temp1 = 1970;	//从1970年开始
//		while(temp>=365)
//		{				 
//			if(Is_Leap_Year(temp1))//是闰年
//			{
//				if(temp>=366)temp-=366;//闰年的秒钟数
//				else break;  
//			}
//			else temp-=365;	  //平年 
//			temp1++;  
//		}   
//		calendar->w_year=temp1;//得到年份
//		temp1=0;
//		while(temp>=28)//超过了一个月
//		{
//			if(Is_Leap_Year(calendar->w_year)&&temp1==1)//当年是不是闰年/2月份
//			{
//				if(temp>=29)temp-=29;//闰年的秒钟数
//				else break; 
//			}
//			else 
//			{
//				if(temp>=mon_table[temp1])temp-=mon_table[temp1];//平年
//				else break;
//			}
//			temp1++;  
//		}
//		calendar->w_month=temp1+1;	//得到月份
//		calendar->w_date=temp+1;  	//得到日期 
//	}
//	temp=rtc%86400;     		//得到秒钟数   	   
//	calendar->hour=temp/3600;     	//小时
//	calendar->min=(temp%3600)/60; 	//分钟	
//	calendar->sec=(temp%3600)%60; 	//秒钟
//}

////设置RTC
//void set_rtc(uint32_t rtc)
//{
//	RTC_DateTypeDef RTC_DateStrcuture;
//	RTC_TimeTypeDef RTC_TimeStrcuture;

//	_calendar_obj calendar;
//	
//	convert_rtc(&calendar, rtc);
//	
//	RTC_DateStrcuture.RTC_Year = calendar.w_year;
//	RTC_DateStrcuture.RTC_Month = calendar.w_month;
//	RTC_DateStrcuture.RTC_Date = calendar.w_date;
//	RTC_TimeStrcuture.RTC_Hours = calendar.hour;
//	RTC_TimeStrcuture.RTC_Minutes = calendar.min;
//	RTC_TimeStrcuture.RTC_Seconds = calendar.sec;
//	
//	RTC_SetDate(RTC_Format_BIN, &RTC_DateStrcuture);//RTC_Format_BIN
//	RTC_SetTime(RTC_Format_BIN, &RTC_TimeStrcuture);
//}

//uint32_t get_rtc(void)
//{
//#ifndef RTC_TR_RESERVED_MASK
//#define RTC_TR_RESERVED_MASK    ((uint32_t)0x007F7F7F)	
//#endif
//	
//	return (uint32_t)(RTC->TR & RTC_TR_RESERVED_MASK); 
//}
#endif
/**************************************************************
* FLASH讀寫设置
**************************************************************/
//保存一頁數據
BOOL save_one_page_to_flash(uint32_t Address, uint8_t* buf, uint16_t len)
{
	return TRUE;
}
//讀一頁數據
void read_one_page_from_flash(uint32_t Address, uint8_t* buf, uint16_t len)
{
}
//保存一個半字，兩字節
BOOL save_half_word_to_flash(uint32_t Address, uint16_t data)
{
	return TRUE;
}
//讀一個半字，兩字節
void read_half_word_from_flash(uint32_t Address, uint16_t* pdata)
{
}
/**************************************************************
* /EEPROM讀寫设置
**************************************************************/
//保存一個半字數組
BOOL save_half_word_buf_to_eeprom(uint32_t Address, uint16_t* buf, uint16_t len)
{
	return TRUE;
}
//讀一個半字數組
void read_half_word_buf_from_eeprom(uint32_t Address, uint16_t* buf, uint16_t len)
{
}

void Key_WakeUp_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Init_ADC1(void)
{
	GPIO_InitTypeDef    GPIO_InitStructure;
	DMA_InitTypeDef     DMA_InitStructure;
	ADC_InitTypeDef     ADC_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);  //新增
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;    //PA1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);	
		
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;    //PB0
//	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);		
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);
	
	ADC_DeInit(ADC1);//	复位ADC1
	
  DMA_DeInit(DMA1_Channel1);	//复位ADC1 chanel1
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;  				//取值的外设地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RegularConvData_Tab;					//缓存数组起始地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;														//DMA方向：外设作为数据源
  DMA_InitStructure.DMA_BufferSize = 1;																					//DMA缓存数组大小设置
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;							//外设地址递增禁用
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;												//内存地址递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;		//外设取值大小设置为halfword
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;						//数据大小设置为halfword
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;																//DMA循环模式，即完成后重新覆盖
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;														//DMA设置高优先级
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;																	//内存到内存禁用
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
		  
	DMA_Cmd(DMA1_Channel1, ENABLE);/* DMA1 Channel1 enable */			
	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular); /* Enable ADC_DMA */	
  ADC_DMACmd(ADC1, ENABLE);  
	
		
	ADC_StructInit(&ADC_InitStructure);
	
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12?????�
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //1??�??�?装??1ぷ�??D???�?
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; 
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//陏?Y?????a篁????
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward; //ADC�?楱?璺??�
  ADC_Init(ADC1, &ADC_InitStructure); 
	
  ADC_ChannelConfig(ADC1, ADC_Channel_1 , ADC_SampleTime_239_5Cycles); /* Convert the ADC1 Channel 11 with 239.5 Cycles as sampling time */  
//	ADC_ChannelConfig(ADC1, ADC_Channel_8 , ADC_SampleTime_239_5Cycles);
	
	ADC_GetCalibrationFactor(ADC1); /* ADC Calibration */  
  ADC_Cmd(ADC1, ENABLE);  /* Enable ADCperipheral[PerIdx] */	  
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN)); /* Wait the ADCEN falg */
  ADC_StartOfConversion(ADC1); /* ADC1 regular Software Start Conv */ 
}



//uint16_t Adc_Switch(uint32_t ADC_Channel)
//{
//	
//	//while(ADC_GetFlagStatus(ADC1,ADC_FLAG_ADEN)==RESET); 
//	//配置ADC采用的通道和采样周期
//	ADC_ChannelConfig(ADC1,ADC_Channel,ADC_SampleTime_239_5Cycles);  
//	
//	//Delay_ms(10);
//	//软件启动ADC转换
//  ADC_StartOfConversion(ADC1);  

//	//等待ADC转换完成
//	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC))
//	{
//		//do nothing
//	}
//  return ADC_GetConversionValue(ADC1) ; 
//}

