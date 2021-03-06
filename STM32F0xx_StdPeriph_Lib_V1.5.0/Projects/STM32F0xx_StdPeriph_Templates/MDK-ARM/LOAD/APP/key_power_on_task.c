

/**
********************************************************************************
* 版權：
* 模块名称：key_power_on_task.c
* 模块功能：
* 创建日期：
* 创 建 者：
* 声    明：
********************************************************************************
**/

/***********************************
* 头文件
***********************************/

#include "app.h"
#include "datatype.h"
#include "hardware.h"
#include "fifo.h"
#include "key_power_on_task.h"
#include "protocol_module.h"

#include "i2c.h"
#include "Motor_pwm.h"
#include "stm32f0xx_pwr.h"
#include "stm32f0xx_exti.h"
#include "stm32f0xx_syscfg.h"
#include "stm32f0xx_dma.h"
#include "serial_port.h"
#include "CMD_receive.h"
#include "app.h"
#include "delay.h"
#include "comm_task.h"
#include "iwtdg.h"
#include "rcc_configure.h"
#include "MPXV70_sampling_data.h"
/**********************************
*宏定义
***********************************/

/***********************************
* 全局变量
***********************************/

/***********************************
* 局部变量
***********************************/
extern uint32_t os_ticks;
extern CMD_Receive g_CmdReceive;  // 命令接收控制对象

extern FIFO_TYPE send_fifo;
extern uint8_t send_buf[SEND_BUF_LEN];
extern CHCKMODE_OUTPUT_PWM state;

extern PWM_STATE pwm1_state;
extern PWM_STATE pwm2_state;
extern PWM_STATE pwm3_state;

extern uint8_t mode;
extern BOOL b_switch_mode_changed;
extern BOOL b_palm_checked;
extern uint32_t detectPalm_cnt;
extern uint32_t noPalm_cnt;
extern BOOL b_bat_detected_ok;

extern LED_STATE led_state;
extern BEEP_STATE beep_state;

extern BOOL b_Motor_Ready2Shake;
extern BOOL	 b_Motor_shake;
extern BOOL b_Palm_check_complited;

//extern BOOL led_bink_timing_flag;
//extern BOOL beep_timing_flag;
//extern uint32_t prev_ledBlink_os_tick;
//extern uint32_t prev_beep_os_tick;
extern uint16_t led_bink_cnt;
extern uint16_t beep_cnt;
extern uint16_t delay_cnt;

extern BOOL b_stop_current_works;
extern BOOL b_no_hand_in_place;
extern BOOL b_end_of_treatment;

extern USB_CHARGING_STATE usb_charging_state;
extern BOOL b_release_gas;
extern BOOL led_bink_timing_flag;
extern BOOL	beep_timing_flag;
extern BOOL	usb_charge_timing_flag;
extern BOOL	key_Press_or_Release_timing_flag;
extern BOOL	b_releaseGas_timing_flag;
extern BOOL	b_timing_flag;
//extern BOOL b_detect_palm;
extern uint32_t prev_releaseGas_os_tick;
extern uint32_t prev_ledBlink_os_tick;
extern uint32_t prev_keyPressOrRelease_os_tick;
extern uint32_t prev_usbCharge_os_tick;
extern uint32_t prev_beep_os_tick;
extern uint32_t prev_WaitBeforeStart_os_tick;
extern uint32_t prev_PWM1_os_tick;
extern uint32_t prev_PWM2_os_tick;
extern uint32_t prev_PWM3_os_tick;
extern uint32_t prev_PWM4_os_tick;
extern uint32_t prev_PWM5_os_tick;
extern uint16_t checkPressAgain_cnt;
extern uint8_t wait_cnt;
extern BOOL waitBeforeStart_timing_flag;

extern BOOL	b_self_test;
extern BOOL key_self_test_timing_flag;
extern uint32_t	prev_selfTest_os_tick;
	
extern BOOL	b_usb_push_in;
extern BOOL	b_usb_pull_up;
extern BOOL	b_stop_current_works;
extern uint8_t led_beep_ID;

extern SELF_TEST_STATE self_tet_state;
extern LED_IN_TURN_STATE led_In_Turn_state;
extern BOOL b_LED_ON_in_turn;

extern BOOL b_check_bnt_release;
extern uint8_t selfTest_delay_Cnt;
extern uint8_t nLED_ON_in_turn;
extern uint8_t inflate_cnt;
extern uint8_t hold_cnt;
extern uint8_t deflate_cnt;

extern uint8_t sample_cnt;
extern uint32_t sample_sum;

extern uint8_t deflate_cnt;
extern uint16_t selfTest_inflate_record_1;
extern uint16_t selfTest_inflate_record_2;

extern uint16_t selfTest_hold_record_1;
extern uint16_t selfTest_hold_record_2;
extern uint16_t selfTest_deflate_record_1;
extern uint16_t selfTest_deflate_record_2;
extern uint8_t selfTest_fail_Cnt;
extern uint8_t selfTest_fail_period_H;
extern uint8_t selfTest_fail_period_L;
extern uint8_t selfTest_end_Cnt;

extern BOOL b_detect_hand_before_system_running;
extern uint16_t wait_between_total_cnt;
extern uint8_t value;

extern int16_t MPXV70_zero_point;
extern BOOL b_MPXV70_get_zero_point;
extern MPXV70_STATE MPXV70_state;
//extern BOOL b_start_powerOn_check;
// BOOL b_KeyWkUP_InterrupHappened=FALSE;
// BOOL b_usb_intterruptHappened=FALSE;
//KEY值，这里点按为确认蓝牙连接
typedef enum {
	NO_KEY,
	BLUE_CHECK
}KEY_VAL;


USB_DETECT_STATE usb_detect_state=USB_NOT_DETECT;

MCU_STATE mcu_state=POWER_OFF;
//mcu_state=POWER_OFF;


//SWITCH_MODE prev_switch_mode=SWITCH_MODE1;

//extern uint8_t OUTPUT_FINISH;
BOOL b_Is_PCB_PowerOn=FALSE;
//BOOL b_usb_charge_bat=FALSE;

//volatile KEY_STATE key_state=KEY_STOP_MODE;
KEY_STATE key_state=KEY_UPING;
//KEY_STATE key_state=KEY_DOWNING;

extern uint16_t RegularConvData_Tab[1];
/***********************************
* 局部函数
***********************************/

/*******************************************************************************
** 函数名称: EnterStopMode
** 功能描述: 进入低功耗模式
** 输　  入: 无
** 输　  出: 无
** 全局变量: 无
** 调用模块: 无
*******************************************************************************/
void CfgWFI()
{
	//时钟使能
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB,ENABLE);  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16,ENABLE);
	
	//外部按键,power_key,PB0
	GPIO_InitTypeDef GPIO_InitStructure;  
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;  
	//GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;  
	
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;  
//	GPIO_Init(GPIOA,&GPIO_InitStructure);  
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;  //PB0 power_key
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	//将EXTI0指向PB0
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource0);   

	EXTI_InitTypeDef EXTI_InitStructure;  
	EXTI_InitStructure.EXTI_Line=EXTI_Line0;           
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;  
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;  
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
	EXTI_Init(&EXTI_InitStructure);  

//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource0);    
//	EXTI_InitStructure.EXTI_Line=EXTI_Line0;
//	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising_Falling;   //配置成上升沿和下降沿都可以触发中断
//	EXTI_Init(&EXTI_InitStructure);

//	//EXTI0中断向量配置  
	NVIC_InitTypeDef NVIC_InitStructure;  
////	NVIC_InitStructure.NVIC_IRQChannel=EXTI4_15_IRQn;    
//	NVIC_InitStructure.NVIC_IRQChannelPriority=0x01;  
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;  
//	NVIC_Init(&NVIC_InitStructure);  
	
	NVIC_InitStructure.NVIC_IRQChannelPriority=0x02;
	NVIC_InitStructure.NVIC_IRQChannel=EXTI0_1_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}

#if 0
////PA0,判断USB是插入还是拔出
USB_DETECT_STATE Check_USB_pull_or_push()
{
	while(TRUE)
	{
		//读取PA0的电平
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1)
		{
			Motor_PWM_Freq_Dudy_Set(1,100,0);
			Motor_PWM_Freq_Dudy_Set(2,100,0);
			Motor_PWM_Freq_Dudy_Set(3,100,0);
			Motor_PWM_Freq_Dudy_Set(4,100,0);
			Motor_PWM_Freq_Dudy_Set(5,100,0);
			set_led(LED_ID_MODE1,FALSE);
			set_led(LED_ID_MODE2,FALSE);
			set_led(LED_ID_MODE3,FALSE);
			set_led(LED_ID_GREEN,FALSE);
			//delay_ms(500);  //给500ms的稳定时间
			//set_led(LED_ID_YELLOW,TRUE);  //debug
			
			uint8_t cnt=0;
			//循环5次，如果5次都是高电平，说明已经稳定的插入USB了
			for(uint8_t i=0;i<5;i++)
			{
				delay_ms(5);
				if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1)
				{
					cnt++;
				}
			}
			
			if(cnt==5) 
			{
				cnt=0;
				//set_led(LED_ID_YELLOW,TRUE);  //debug
				return USB_PUSH_IN;
			}
			else
			{
				return USB_NOT_DETECT;
			}
		}
		else if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==0)
		{
			Motor_PWM_Freq_Dudy_Set(1,100,0);
			Motor_PWM_Freq_Dudy_Set(2,100,0);
			Motor_PWM_Freq_Dudy_Set(3,100,0);
			Motor_PWM_Freq_Dudy_Set(4,100,0);
			Motor_PWM_Freq_Dudy_Set(5,100,0);
			set_led(LED_ID_MODE1,FALSE);
			set_led(LED_ID_MODE2,FALSE);
			set_led(LED_ID_MODE3,FALSE);
			set_led(LED_ID_GREEN,FALSE);
			//delay_ms(500);  //给500ms的稳定时间
			
			uint8_t cnt=0;
			for(uint8_t i=0;i<5;i++)
			{
				delay_ms(5);
				if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==0)
				{
					cnt++;
				}
			}
			
			if(cnt==5) 
			{
				cnt=0;
				return USB_PULL_UP;
			}
			else
			{
				return USB_NOT_DETECT;
			}
		}
		else
		{
			return USB_NOT_DETECT;
		}
	}
}
#endif

 
//void EXTI4_15_IRQHandler()
////void EXTI0_1_IRQHandler(void)
//{
//	if(EXTI_GetITStatus(EXTI_Line0)!=RESET)  
//	{ 
////		b_usb_intterruptHappened=TRUE;
//		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1) //高电平表示插入了USB,上拉
//		{
//			//处理USB插入
////			b_usb_push_in=TRUE;
//			usb_detect_state=USB_PUSH_IN;
//		}
//		else if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==0)  //低电平表示拔出了USB，下拉
//		{
//			//处理USB拔出
//			//b_usb_pull_up=TRUE;
//			usb_detect_state=USB_PULL_UP;
//		}
//		else
//		{
//			//do nothing
//		}
//		
//		//新增，明天验证一下, //意思是判断了上拉下拉之后就不再进入这个函数，而是去
//		//usb_charge_battery验证USB插拔是否有效
////		b_usb_intterruptHappened=FALSE;
//	}  
//	EXTI_ClearFlag(EXTI_Line0);
//}

void EXTI0_1_IRQHandler(void)
{  
	if(EXTI_GetITStatus(EXTI_Line0)!=RESET)  
	{ 
		key_state=KEY_DOWNING;
//		set_led(LED_ID_YELLOW,TRUE);
//		set_led(LED_ID_GREEN,TRUE);
//		delay_ms(10);
//		set_led(LED_ID_YELLOW,FALSE);
//		set_led(LED_ID_GREEN,FALSE);
	} 
	EXTI_ClearFlag(EXTI_Line0);
	
} 
//void RCC_Configuration_2(void)
//{
//	RCC_DeInit(); 
////RCC_HSICmd(ENABLE);	//???????
//  RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_12);   //??PLL??????PLL????,?????????????,6?? ??24MHz
//  RCC_PLLCmd(ENABLE);                                    //??????PLL
//  while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)==RESET);      //??PLL???????
//  
//  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);             //???????,??????????????????:HSI,HSE,RCC_SYSCLKSource_PLLCLK?
// 
//  RCC_HCLKConfig(RCC_SYSCLK_Div1);                       //???AHB????,??PLL?AHB???????????AHB?????PLL?????
//  RCC_PCLKConfig(RCC_HCLK_Div1);                         //??APB????
//  RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div4);                //ADC????,?????14MHz
//	 while(RCC_GetSYSCLKSource() != 0x08);                  //???????????, 0x00: HSI used as system clock,0x04: HSE used as system clock ,0x08: PLL used as system clock ?        
//  
//  SystemCoreClockUpdate();               
//}

void init_system_afterWakeUp()
{
	os_ticks = 0;
	//SystemInit();
	RCC_Configuration();
	delay_init();
	os_init();
	init_task();
	
	
//	os_ticks = 0;
//	//os_ticks = 4294967290;
//	delay_init();
//	os_init();
//	SystemInit();
//	init_task();
}


void CfgALLPins4StopMode()
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
	
	//led端口PA3,PA4
	GPIO_InitTypeDef GPIO_InitStructure_LED;
	GPIO_InitStructure_LED.GPIO_Pin = LED_GREEN_PWR_PIN|LED_YELLOW_PWR_PIN;                       
	GPIO_InitStructure_LED.GPIO_Speed = GPIO_Speed_50MHz;     
	GPIO_InitStructure_LED.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure_LED.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_LED.GPIO_PuPd=GPIO_PuPd_UP;
	//GPIO_InitStructure_LED.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(LED_PORT, &GPIO_InitStructure_LED);
	GPIO_SetBits(LED_PORT, LED_GREEN_PWR_PIN|LED_YELLOW_PWR_PIN);

	
	//PA1
	GPIO_InitTypeDef GPIO_InitStructure_PA_1;
	GPIO_InitStructure_PA_1.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure_PA_1.GPIO_Mode = GPIO_Mode_AN;
	//GPIO_InitStructure_PA_1.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure_PA_1.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure_PA_1);	
	
	//关闭ADC
	DMA_Cmd(DMA1_Channel1, DISABLE);/* DMA1 Channel1 enable */			
  ADC_DMACmd(ADC1, DISABLE);
	ADC_Cmd(ADC1, DISABLE);  
	
	//PB0不能配置，这个是中断口
//	//PB0
//	GPIO_InitTypeDef GPIO_InitStructure_PB_0;
//	GPIO_InitStructure_PB_0.GPIO_Pin = GPIO_Pin_0;
//  GPIO_InitStructure_PB_0.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure_PB_0.GPIO_OType=GPIO_OType_PP;
//	//GPIO_InitStructure_PB_0.GPIO_Mode = GPIO_Mode_IN;
//  GPIO_InitStructure_PB_0.GPIO_PuPd = GPIO_PuPd_NOPULL ;
//  GPIO_Init(GPIOB, &GPIO_InitStructure_PB_0);	
//	GPIO_SetBits(GPIOB,GPIO_Pin_0);
	
	//PA8
	GPIO_InitTypeDef GPIO_InitStructure_PA_8;
	GPIO_InitStructure_PA_8.GPIO_Pin = GPIO_Pin_8;                       
	GPIO_InitStructure_PA_8.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PA_8.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure_PA_8.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_PA_8.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PA_8);	
	GPIO_SetBits(GPIOA,GPIO_Pin_8);  //置为高电平，使蓝牙休眠
	
	
	//串口PA9,PA10
	GPIO_InitTypeDef GPIO_InitStructure_UART;
	GPIO_InitStructure_UART.GPIO_Pin = UART_BLUETOOTH_RX_PIN|UART_BLUETOOTH_TX_PIN;                       
	GPIO_InitStructure_UART.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_UART.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure_UART.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_UART.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(UART_BLUETOOTH_IO_PORT, &GPIO_InitStructure_UART);
	GPIO_SetBits(UART_BLUETOOTH_IO_PORT,UART_BLUETOOTH_RX_PIN|UART_BLUETOOTH_TX_PIN); 
	//关闭串口
	DMA_Cmd(UART_DMA_RX_CHANNEL, DISABLE);
	DMA_Cmd(UART_DMA_TX_CHANNEL, DISABLE);
	USART_Cmd(UART, DISABLE);
	
	//PB3,PB4
	GPIO_InitTypeDef GPIO_InitStructure_PB_3_4;
	GPIO_InitStructure_PA_8.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;                       
	GPIO_InitStructure_PA_8.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PA_8.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PA_8.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_PA_8.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure_PB_3_4);	
	
	//PB5
	GPIO_InitTypeDef GPIO_InitStructure_PB_5;
	GPIO_InitStructure_PB_5.GPIO_Pin = GPIO_Pin_5;                       
	GPIO_InitStructure_PB_5.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PB_5.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PB_5.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_PB_5.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure_PB_5);
	
	//PB6
	GPIO_InitTypeDef GPIO_InitStructure_PB_6;
	GPIO_InitStructure_PB_6.GPIO_Pin = GPIO_Pin_6;                       
	GPIO_InitStructure_PB_6.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PB_6.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PB_6.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_PB_6.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure_PB_6);
//	GPIO_InitStructure_PB_6.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure_PB_6.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStructure_PB_6.GPIO_PuPd=GPIO_PuPd_UP;
//	GPIO_Init(GPIOB, &GPIO_InitStructure_PB_6);
//	GPIO_SetBits(GPIOB,GPIO_Pin_6);

	//PA0,PA2,PA5,PA6,PA7,PA11,PA12,PA15
	GPIO_InitTypeDef GPIO_InitStructure_PA_0_2_5_6_7_11_12_15;
	GPIO_InitStructure_PA_0_2_5_6_7_11_12_15.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_15;                       
	GPIO_InitStructure_PA_0_2_5_6_7_11_12_15.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PA_0_2_5_6_7_11_12_15.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PA_0_2_5_6_7_11_12_15.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_PA_0_2_5_6_7_11_12_15.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PA_0_2_5_6_7_11_12_15);
	
	//PB1,PB7,PB8,PB9,PB10,PB11,PB12,PB13,PB14,PB15
	GPIO_InitTypeDef GPIO_InitStructure_PB_1_7_8_9_10_11_12_13_14_15;
	GPIO_InitStructure_PB_1_7_8_9_10_11_12_13_14_15.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;                       
	GPIO_InitStructure_PB_1_7_8_9_10_11_12_13_14_15.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PB_1_7_8_9_10_11_12_13_14_15.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PB_1_7_8_9_10_11_12_13_14_15.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_PB_1_7_8_9_10_11_12_13_14_15.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB, &GPIO_InitStructure_PB_1_7_8_9_10_11_12_13_14_15);
	
	//PC13,14,15
	GPIO_InitTypeDef GPIO_InitStructure_PC_13_14_15;
	GPIO_InitStructure_PC_13_14_15.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;                       
	GPIO_InitStructure_PC_13_14_15.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PC_13_14_15.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PC_13_14_15.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_PC_13_14_15.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC, &GPIO_InitStructure_PC_13_14_15);

#if 0	


////	
//	
//	//PWR save，PA12
//	GPIO_InitTypeDef GPIO_InitStructure_PA12;
//	GPIO_InitStructure_PA12.GPIO_Pin = GPIO_Pin_12;                       
//	GPIO_InitStructure_PA12.GPIO_Speed = GPIO_Speed_50MHz;       
//	GPIO_InitStructure_PA12.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure_PA12.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStructure_PA12.GPIO_PuPd=GPIO_PuPd_UP;
////	GPIO_InitStructure_PA12.GPIO_PuPd=GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOA, &GPIO_InitStructure_PA12);
//	GPIO_SetBits(GPIOA,GPIO_Pin_12);   //输出高电平，关管子，省电

////	GPIO_InitTypeDef GPIO_InitStructure_PA12;
////	GPIO_InitStructure_PA12.GPIO_Pin = GPIO_Pin_12;                       
////	GPIO_InitStructure_PA12.GPIO_Speed = GPIO_Speed_50MHz;       
////	GPIO_InitStructure_PA12.GPIO_Mode = GPIO_Mode_IN;
////	GPIO_InitStructure_PA12.GPIO_PuPd=GPIO_PuPd_NOPULL;
////	GPIO_Init(GPIOA, &GPIO_InitStructure_PA12);

//	
//	//PWR EN，PA15
//	GPIO_InitTypeDef GPIO_InitStructure_PA15;
//	GPIO_InitStructure_PA15.GPIO_Pin = GPIO_Pin_15;                       
//	GPIO_InitStructure_PA15.GPIO_Speed = GPIO_Speed_50MHz;       
//	GPIO_InitStructure_PA15.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure_PA15.GPIO_OType=GPIO_OType_PP;
//	//GPIO_InitStructure_PA15.GPIO_OType=GPIO_OType_OD;
//	GPIO_InitStructure_PA15.GPIO_PuPd=GPIO_PuPd_DOWN;
//	GPIO_Init(GPIOA, &GPIO_InitStructure_PA15);
//	GPIO_ResetBits(GPIOA,GPIO_Pin_15);   //输出低电平，关管子，省电

////	GPIO_InitTypeDef GPIO_InitStructure_PA15;
////	GPIO_InitStructure_PA15.GPIO_Pin = GPIO_Pin_15;                       
////	GPIO_InitStructure_PA15.GPIO_Speed = GPIO_Speed_50MHz;       
////	GPIO_InitStructure_PA15.GPIO_Mode = GPIO_Mode_IN;
////	GPIO_InitStructure_PA15.GPIO_PuPd=GPIO_PuPd_NOPULL;
////	GPIO_Init(GPIOA, &GPIO_InitStructure_PA15);

//	
//	//PWM1(PA6),PWM2(PA7)  inflate_pwm2(PA11)
//	GPIO_InitTypeDef GPIO_InitStructure_PA_6_7_11;
//	GPIO_InitStructure_PA_6_7_11.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_11;                       
//	GPIO_InitStructure_PA_6_7_11.GPIO_Speed = GPIO_Speed_50MHz;       
//	GPIO_InitStructure_PA_6_7_11.GPIO_Mode = GPIO_Mode_OUT;
//	//GPIO_InitStructure_PA_6_7_11.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PA_6_7_11.GPIO_OType=GPIO_OType_PP;
//	//GPIO_InitStructure_PA_6_7_11.GPIO_PuPd=GPIO_PuPd_NOPULL;
//	GPIO_InitStructure_PA_6_7_11.GPIO_PuPd=GPIO_PuPd_DOWN;
//	GPIO_Init(GPIOA, &GPIO_InitStructure_PA_6_7_11);
//	GPIO_ResetBits(GPIOA,GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_11);
//	
//	//PWM3(PB1) BEEP_PWM(PB14)
//	GPIO_InitTypeDef GPIO_InitStructure_PB_1_14;
//	GPIO_InitStructure_PB_1_14.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_14;                       
//	GPIO_InitStructure_PB_1_14.GPIO_Speed = GPIO_Speed_50MHz;       
////	GPIO_InitStructure_PB_1_14.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure_PB_1_14.GPIO_Mode = GPIO_Mode_IN;
////	GPIO_InitStructure_PB_1_14.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStructure_PB_1_14.GPIO_PuPd=GPIO_PuPd_DOWN;
//	GPIO_Init(GPIOB, &GPIO_InitStructure_PB_1_14);
//	GPIO_ResetBits(GPIOB,GPIO_Pin_1|GPIO_Pin_14);
//	
////	GPIO_InitTypeDef GPIO_InitStructure_PB_1;
////	GPIO_InitStructure_PB_1.GPIO_Pin = GPIO_Pin_1;                       
////	GPIO_InitStructure_PB_1.GPIO_Speed = GPIO_Speed_50MHz;       
////	GPIO_InitStructure_PB_1.GPIO_Mode = GPIO_Mode_OUT;
////	//GPIO_InitStructure_PB_1.GPIO_Mode = GPIO_Mode_IN;
////	GPIO_InitStructure_PB_1.GPIO_OType=GPIO_OType_PP;
////	//GPIO_InitStructure_PB_1.GPIO_PuPd=GPIO_PuPd_NOPULL;
////	GPIO_InitStructure_PB_1.GPIO_PuPd=GPIO_PuPd_DOWN;
////	GPIO_Init(GPIOB, &GPIO_InitStructure_PB_1);
////	GPIO_ResetBits(GPIOB,GPIO_Pin_1);

////	GPIO_InitTypeDef GPIO_InitStructure_PB_14;
////	GPIO_InitStructure_PB_14.GPIO_Pin = GPIO_Pin_14;                       
////	GPIO_InitStructure_PB_14.GPIO_Speed = GPIO_Speed_50MHz;       
////	GPIO_InitStructure_PB_14.GPIO_Mode = GPIO_Mode_OUT;
////	//GPIO_InitStructure_PB_1.GPIO_Mode = GPIO_Mode_IN;
////	GPIO_InitStructure_PB_14.GPIO_OType=GPIO_OType_PP;
////	//GPIO_InitStructure_PB_1.GPIO_PuPd=GPIO_PuPd_NOPULL;
////	GPIO_InitStructure_PB_14.GPIO_PuPd=GPIO_PuPd_DOWN;
////	GPIO_Init(GPIOB, &GPIO_InitStructure_PB_14);
////	GPIO_ResetBits(GPIOB,GPIO_Pin_14);

//	
//	//BAT_CHARGE,BAT_STDBY   PA4,PA5
//	GPIO_InitTypeDef GPIO_InitStructure_PA_4_5;
//	GPIO_InitStructure_PA_4_5.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;                       
//	GPIO_InitStructure_PA_4_5.GPIO_Speed = GPIO_Speed_50MHz;       
//	GPIO_InitStructure_PA_4_5.GPIO_Mode = GPIO_Mode_IN;
////	GPIO_InitStructure_PA_4_5.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStructure_PA_4_5.GPIO_PuPd=GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOA, &GPIO_InitStructure_PA_4_5);
//	
////	GPIO_InitTypeDef GPIO_InitStructure_PA_4_5;
////	GPIO_InitStructure_PA_4_5.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;                       
////	GPIO_InitStructure_PA_4_5.GPIO_Speed = GPIO_Speed_50MHz;       
////	GPIO_InitStructure_PA_4_5.GPIO_Mode = GPIO_Mode_OUT;
////	GPIO_InitStructure_PA_4_5.GPIO_OType=GPIO_OType_PP;
////	GPIO_InitStructure_PA_4_5.GPIO_PuPd=GPIO_PuPd_UP;
////	GPIO_Init(GPIOA, &GPIO_InitStructure_PA_4_5);
////	GPIO_SetBits(GPIOB,GPIO_Pin_4|GPIO_Pin_5);
//	
//	//USB_OE PA0
//	GPIO_InitTypeDef GPIO_InitStructure_PA0;
//	GPIO_InitStructure_PA0.GPIO_Pin = GPIO_Pin_0;                       
//	GPIO_InitStructure_PA0.GPIO_Speed = GPIO_Speed_50MHz;       
//	//GPIO_InitStructure_PA0.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure_PA0.GPIO_Mode = GPIO_Mode_IN;
////	GPIO_InitStructure_PA_4_5.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStructure_PA_4_5.GPIO_PuPd=GPIO_PuPd_DOWN;
//	//GPIO_InitStructure_PA_4_5.GPIO_PuPd=GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOA, &GPIO_InitStructure_PA0);
//	
//	//VAVLE PB10,PB11
//	GPIO_InitTypeDef GPIO_InitStructure_PB_10_11;
//	GPIO_InitStructure_PB_10_11.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;                       
//	GPIO_InitStructure_PB_10_11.GPIO_Speed = GPIO_Speed_50MHz;       
//	GPIO_InitStructure_PB_10_11.GPIO_Mode = GPIO_Mode_IN;
////	GPIO_InitStructure_PB_10_11.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStructure_PB_10_11.GPIO_PuPd=GPIO_PuPd_DOWN;
//	GPIO_Init(GPIOB, &GPIO_InitStructure_PB_10_11);
//	
//	//POWER ON/OFF (PA8)
////	GPIO_InitTypeDef GPIO_InitStructure_PA_8;
////	GPIO_InitStructure_PA_8.GPIO_Pin = GPIO_Pin_8;                       
////	GPIO_InitStructure_PA_8.GPIO_Speed = GPIO_Speed_50MHz;       
////	GPIO_InitStructure_PA_8.GPIO_Mode = GPIO_Mode_IN;
////	//GPIO_InitStructure_PA_8.GPIO_OType=GPIO_OType_PP;
////	//GPIO_InitStructure_PA_8.GPIO_PuPd=GPIO_PuPd_DOWN;
////	GPIO_Init(GPIOA, &GPIO_InitStructure_PA_8);
////	//GPIO_SetBits(GPIOA,GPIO_Pin_8);

//	GPIO_InitTypeDef GPIO_InitStructure_PA_8;
//	GPIO_InitStructure_PA_8.GPIO_Pin = GPIO_Pin_8;                       
//	GPIO_InitStructure_PA_8.GPIO_Speed = GPIO_Speed_50MHz;       
//	GPIO_InitStructure_PA_8.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure_PA_8.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStructure_PA_8.GPIO_PuPd=GPIO_PuPd_UP;
//	GPIO_Init(GPIOA, &GPIO_InitStructure_PA_8);
//	GPIO_SetBits(GPIOA,GPIO_Pin_8);

//	//SWITCH ON/OFF (PB13)   ,SWITCH MODE(PB15)
//	GPIO_InitTypeDef GPIO_InitStructure_PB_13_15;
//	GPIO_InitStructure_PB_13_15.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_15;                       
//	GPIO_InitStructure_PB_13_15.GPIO_Speed = GPIO_Speed_50MHz;       
//	GPIO_InitStructure_PB_13_15.GPIO_Mode = GPIO_Mode_IN;
//	//GPIO_InitStructure_PB_13_15.GPIO_OType=GPIO_OType_PP;
//	//GPIO_InitStructure_PB_13_15.GPIO_PuPd=GPIO_PuPd_UP;
//	GPIO_InitStructure_PB_13_15.GPIO_PuPd=GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOB, &GPIO_InitStructure_PB_13_15);
//	
////		GPIO_InitTypeDef GPIO_InitStructure_PB_15;
////		GPIO_InitStructure_PB_15.GPIO_Pin = GPIO_Pin_15;                       
////		GPIO_InitStructure_PB_15.GPIO_Speed = GPIO_Speed_50MHz;       
////		GPIO_InitStructure_PB_15.GPIO_Mode = GPIO_Mode_OUT;
////		GPIO_InitStructure_PB_15.GPIO_OType=GPIO_OType_PP;
////		GPIO_InitStructure_PB_15.GPIO_PuPd=GPIO_PuPd_UP;
////		GPIO_Init(GPIOB, &GPIO_InitStructure_PB_15);
////		GPIO_SetBits(GPIOB,GPIO_Pin_15);
//	
//	//PB7,PB8,PB9,PB12
//	GPIO_InitTypeDef GPIO_InitStructure_PB_7_8_9_12;
//	GPIO_InitStructure_PB_7_8_9_12.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_12;                       
//	GPIO_InitStructure_PB_7_8_9_12.GPIO_Speed = GPIO_Speed_50MHz;       
//	GPIO_InitStructure_PB_7_8_9_12.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PB_7_8_9_12.GPIO_PuPd = GPIO_PuPd_DOWN;
//	GPIO_Init(GPIOB, &GPIO_InitStructure_PB_7_8_9_12);
//	
//	//PC13,PC14,PC15
//	GPIO_InitTypeDef GPIO_InitStructure_PC_13_14_15;
//	GPIO_InitStructure_PC_13_14_15.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;                       
//	GPIO_InitStructure_PC_13_14_15.GPIO_Speed = GPIO_Speed_50MHz;       
//	GPIO_InitStructure_PC_13_14_15.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PC_13_14_15.GPIO_PuPd = GPIO_PuPd_DOWN;
//	GPIO_Init(GPIOC, &GPIO_InitStructure_PC_13_14_15);
//	
//	//PF0,PF1
//	GPIO_InitTypeDef GPIO_InitStructure_PF_0_1;
//	GPIO_InitStructure_PF_0_1.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;                       
//	GPIO_InitStructure_PF_0_1.GPIO_Speed = GPIO_Speed_50MHz;       
//	GPIO_InitStructure_PF_0_1.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PF_0_1.GPIO_PuPd = GPIO_PuPd_DOWN;
//	GPIO_Init(GPIOF, &GPIO_InitStructure_PF_0_1);
#endif
}

void Init_gloab_viriable()
{
	key_state=KEY_UPING;
	mcu_state=POWER_OFF;
	b_MPXV70_get_zero_point=FALSE;
	MPXV70_zero_point=0;
	MPXV70_state=MPXV70_SAMPLE_DATA;
}

//进入stop模式，采用中断唤醒
void EnterStopMode()
{
	Init_gloab_viriable();
	
	//配置中断
	CfgWFI();
	//I2C芯片ADS115进入power-down模式
	//ADS115_enter_power_down_mode();

	CfgALLPins4StopMode();   
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);  
	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
}

static uint8_t tmp=0;
void test(void)
{
	if(tmp==50)
	{
		tmp=0;
		set_led(LED_ID_GREEN,TRUE);
		delay_ms(1000);
		set_led(LED_ID_GREEN,FALSE);
	}
	else
	{
		tmp++;
		//set_led(LED_ID_GREEN,FALSE);
	}
	os_delay_ms(TASK_TEST_ID, 20);
}

/*******************************************************************************
** 函数名称: key_power_on_task
** 功能描述: 检查按键是否启动，必须连续按1s以上才能启动系统
** 输　  入: 无
** 输　  出: 无
** 全局变量: 无
** 调用模块: 无
*******************************************************************************/
void key_power_on_task(void)
{
	static uint8_t wakeup_Cnt;
	if(key_state==KEY_DOWNING)
	{
		//set_led(LED_ID_YELLOW,TRUE);
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)==0)
		{
			if(wakeup_Cnt==5)
			{
				wakeup_Cnt=0;	
				b_Is_PCB_PowerOn=!b_Is_PCB_PowerOn;
				
				if(b_Is_PCB_PowerOn)
				{
					mcu_state=POWER_ON;	
					key_state=KEY_WAKE_UP;						
				}
				else
				{
					mcu_state=POWER_OFF;	
					key_state=KEY_STOP_MODE;
				}
			}
			else
			{
				wakeup_Cnt++;
			}
		}
		else
		{
			wakeup_Cnt=0;
			if(!b_Is_PCB_PowerOn)  //b_Is_PCB_PowerOn为FALSE是才进行判断，按键时间过短，不允许启动
			{
				key_state=KEY_FAIL_WAKEUP;
			}
		}
	}
//	
	if(key_state==KEY_FAIL_WAKEUP)
	{
			EnterStopMode();
			init_system_afterWakeUp();
	}
	
	if(key_state==KEY_WAKE_UP)
	{
		//if(b_bat_detected_ok)
		{
			//开机
			set_led(LED_ID_GREEN,TRUE);
//			set_led(LED_ID_YELLOW,TRUE);
//			GPIO_ResetBits(LED_PORT,LED_GREEN_PWR_PIN);
//			GPIO_ResetBits(LED_PORT,LED_YELLOW_PWR_PIN);
//	
			key_state=KEY_UPING;
		}
	}
	
	if(key_state==KEY_STOP_MODE)
	{
		EnterStopMode();
		init_system_afterWakeUp();
	}
	os_delay_10us(KEY_LED_TASK_ID, KEY_LED_PERIOD);  //20ms来一次
//	os_delay_ms(KEY_LED_TASK_ID, KEY_LED_PERIOD);
}

