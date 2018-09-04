/*******************************************************************************
* 版权所有 :  
* 版本号   :  1.0
* 文件名   :  serial_port.h
* 生成日期 : 
* 作者     :  
* 功能说明 :  命令处理任务

*******************************************************************************/
#ifndef  __SERIAL_PORT_H__
#define  __SERIAL_PORT_H__
/*******************************************************************************
*                                 头文件包含
*******************************************************************************/
#include "stm32f0xx.h"
#include "device_type.h"
/**********************************
*宏定义
***********************************/
//#define STM32F030F4P6

//串口蓝牙的串口配置 UART1 ，PA9-TX,PA10-RX
#ifdef STM32F030F4P6             //这个不会执行，不用这个芯片了
#define UART    USART1
#define UART_CLKSRC   RCC_APB2Periph_USART1
#define UART_DMA_RX_CHANNEL   DMA1_Channel3
#define UART_DMA_TX_CHANNEL   DMA1_Channel2
#else
#define UART    USART2          //目前使用的是这个芯片，STM32F070CB              
#define UART_CLKSRC RCC_APB1Periph_USART2
#define UART_DMA_RX_CHANNEL   DMA1_Channel5
#define UART_DMA_TX_CHANNEL   DMA1_Channel4
#endif

#define UART_IO_PORT			GPIOA
#define	UART_IO_CLKSRC		RCC_AHBPeriph_GPIOA   
#define UART_RX_PIN				GPIO_Pin_10
#define UART_TX_PIN				GPIO_Pin_9
#define UART_RX_AF_PIN_SOURCE GPIO_PinSource10
#define UART_TX_AF_PIN_SOURCE GPIO_PinSource9

//串口wifi的串口配置  UART3 PB10-TX  PB11-RX
#define UART_WIFI USART3
#define UART_WIFI_CLKSRC RCC_APB1Periph_USART3

#define UART_WIFI_IO_PORT GPIOB
#define UART_WIFI_IO_CLKSRC RCC_AHBPeriph_GPIOB
#define UART_WIFI_RX_PIN  GPIO_Pin_11
#define UART_WIFI_TX_PIN  GPIO_Pin_10
#define UART_WIFI_RX_AF_PIN_SOURCE GPIO_PinSource11
#define UART_WIFI_TX_AF_PIN_SOURCE GPIO_Pinsource10


//串口蓝牙的串口配置  UART1 PB10-TX  PB11-RX
#define UART_BLUETOOTH            						USART1
#define UART_BLUETOOTH_CLKSRC      						RCC_APB2Periph_USART1

#define UART_BLUETOOTH_IO_PORT			          GPIOA
#define	UART_BLUETOOTH_IO_CLKSRC		          RCC_AHBPeriph_GPIOA   
#define UART_BLUETOOTH_RX_PIN									GPIO_Pin_10
#define UART_BLUETOOTH_TX_PIN									GPIO_Pin_9
#define UART_BLUETOOTH_RX_AF_PIN_SOURCE				GPIO_PinSource10
#define UART_BLUETOOTH_TX_AF_PIN_SOURCE 			GPIO_PinSource9



#define UART_BAUDRATE   115200 

#define DMA_CLKSRC RCC_AHBPeriph_DMA1
//#define UART_DMA_RX_CHANNEL   DMA1_Channel3
//#define UART_DMA_TX_CHANNEL   DMA1_Channel2
////#define UART_DMA_RX_CHANNEL   DMA1_Channel5
////#define UART_DMA_TX_CHANNEL   DMA1_Channel4

/***********************************
* 全局变量
***********************************/

/***********************************
* 類型定義
***********************************/

/***********************************
* 外部函数
***********************************/
void UARTInit(uint8_t* p_rec_buf, uint32_t rec_num);
void UartSendNBytes (uint8_t *p_buf, uint32_t num);
int32_t GetUartReceiverResidualCnt(void);
void Init_UART_WIFI(uint32_t BaudRate);
void Init_UART_BLUETOOTH(uint32_t BaudRate);
void USART_BLUETOOTH_SendBuf(uint8_t *pBuf, uint32_t u32Len);
uint8_t USART_BLUETOOTH_ReciverBuf(void);
void USART_WIFI_SendBuf(uint8_t *pBuf, uint32_t u32Len);
uint8_t USART_WIFI_ReciverBuf(void);
#endif
