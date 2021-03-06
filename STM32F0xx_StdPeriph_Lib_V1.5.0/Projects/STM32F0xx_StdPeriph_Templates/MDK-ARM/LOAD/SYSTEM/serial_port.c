/*******************************************************************************
* 项目编号 :  
* 版本号   :  1.0
* 文件名   :  serial_port.c
* 生成日期 :  
* 作者     :  
* 功能说明 :  命令处理任务
*******************************************************************************/
/*******************************************************************************
*                                 头文件包含
*******************************************************************************/
#include "serial_port.h"
#include "hardware.h"
#include "stm32f0xx_usart.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_dma.h"
#include "datatype.h"

/*******************************************************************************
*                                 局部變量
*******************************************************************************/

/*******************************************************************************
*                                 全局變量
*******************************************************************************/

/*******************************************************************************
*                                 函數定義
*******************************************************************************/
/*******************************************************************************
* 函数名称 : UARTInit
* 功能描述 : 初始化串口
* 输入参数 : 无
* 输出参数 : 无
* 函数返回 : 无
*******************************************************************************/
void UARTInit(uint8_t* p_rec_buf, uint32_t rec_num)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHBPeriphClockCmd(UART_IO_CLKSRC, ENABLE);  //使能GPIOA的时钟
	RCC_AHBPeriphClockCmd(DMA_CLKSRC, ENABLE);
	
	#ifdef STM32F030F4P6
	RCC_APB2PeriphClockCmd(UART_CLKSRC, ENABLE);//使能USART的时钟
	#else
	RCC_APB1PeriphClockCmd(UART_CLKSRC, ENABLE);//使能USART的时钟
	#endif
	
//	RCC_APB2PeriphClockCmd(UART_CLKSRC, ENABLE);//使能USART的时钟
//	//RCC_APB1PeriphClockCmd(UART_CLKSRC, ENABLE);//使能USART的时钟
	
	/* USART1的端口配置 */
	GPIO_PinAFConfig(UART_IO_PORT, UART_RX_AF_PIN_SOURCE, GPIO_AF_1);//配置PA9成第二功能引脚	TX
	GPIO_PinAFConfig(UART_IO_PORT, UART_TX_AF_PIN_SOURCE, GPIO_AF_1);//配置PA10成第二功能引脚  RX	

	GPIO_InitStructure.GPIO_Pin = UART_RX_PIN | UART_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(UART_IO_PORT, &GPIO_InitStructure);

	/* USART1的基本配置 */
	USART_InitStructure.USART_BaudRate = UART_BAUDRATE;              //波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART, &USART_InitStructure);	
	#if 1	
	//DMA 發送配置
	DMA_DeInit(UART_DMA_TX_CHANNEL);	/* DMA1 Channel1 Config */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&UART->TDR);//外设地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;//内存地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//外设作为数据传输的来源
  DMA_InitStructure.DMA_BufferSize = (uint32_t)0;//
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址寄存器不变
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址寄存器不变
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//数据宽度为8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//数据宽度为8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA_Priority设定DMA通道x的软件优先级
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//DMA通道x没有设置为内存到内存传输
  DMA_Init(UART_DMA_TX_CHANNEL, &DMA_InitStructure);

		//DMA 接收配置
	DMA_DeInit(UART_DMA_RX_CHANNEL);	/* DMA1 Channel1 Config */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&UART->RDR);//外设地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)p_rec_buf;//内存地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//外设作为数据传输的来源
  DMA_InitStructure.DMA_BufferSize = rec_num;//
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址寄存器不变
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址寄存器不变
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//数据宽度为8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//数据宽度为8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA_Priority设定DMA通道x的软件优先级
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//DMA通道x没有设置为内存到内存传输
  DMA_Init(UART_DMA_RX_CHANNEL, &DMA_InitStructure);
	
	USART_DMACmd(UART, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
	
	DMA_Cmd(UART_DMA_RX_CHANNEL, ENABLE);
	DMA_Cmd(UART_DMA_TX_CHANNEL, ENABLE);
	#endif	
	USART_Cmd(UART, ENABLE);

}

/*******************************************************************************
* 函数名称 : UART_WIFI_Init
* 功能描述 : 初始化串口wifi
* 输入参数 : 波特率
* 输出参数 : 无
* 函数返回 : 无
*******************************************************************************/
void Init_UART_WIFI(uint32_t BaudRate)
{
		USART_InitTypeDef USART_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);  
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
    
		//PB10-TX,PB11-RX
    GPIO_PinAFConfig(UART_WIFI_IO_PORT,UART_WIFI_RX_PIN,GPIO_AF_4);
    GPIO_PinAFConfig(UART_WIFI_IO_PORT,UART_WIFI_TX_PIN,GPIO_AF_4);  

    GPIO_InitStruct.GPIO_Pin=UART_WIFI_RX_PIN|UART_WIFI_TX_PIN;
    GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_Init(UART_WIFI_IO_PORT,&GPIO_InitStruct);

    /*USART基本配置*/
    USART_InitStruct.USART_BaudRate=UART_BAUDRATE;
    USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
    USART_InitStruct.USART_Parity=USART_Parity_No;
    USART_InitStruct.USART_StopBits=USART_StopBits_1;
    USART_InitStruct.USART_WordLength=USART_WordLength_8b;
    USART_Init(UART_WIFI,&USART_InitStruct);

//    /*??????*/
//    NVIC_Config(USART1_IRQn);
//    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
    USART_Cmd(UART_WIFI,ENABLE);
}


//发送串口wifi数据
void USART_WIFI_SendBuf(uint8_t *pBuf, uint32_t u32Len)
{
    while(u32Len--)
    {
        /*判断发送缓冲区是否为空*/
        while(!USART_GetFlagStatus(UART_WIFI,USART_FLAG_TXE));
        USART_SendData(UART_WIFI,*pBuf++);
    }
}

//接收串口wifi数据
uint8_t USART_WIFI_ReciverBuf(void)
{
	//判断接收缓冲区是否为空
	while(!USART_GetFlagStatus(UART_WIFI,USART_FLAG_RXNE)){};
  return USART_ReceiveData(UART_WIFI);
}


//发送串口蓝牙数据
//void USART_BLUETOOTH_SendBuf(uint8_t *pBuf, uint32_t u32Len)
//{
//	while(u32Len--)
//    {
//        /*判断发送缓冲区是否为空*/
//        while(!USART_GetFlagStatus(UART_BLUETOOTH,USART_FLAG_TXE));
//        USART_SendData(UART_BLUETOOTH,*pBuf++);
//    }
//}


//void USART_BLUETOOTH_SendBuf(USART_TypeDef * USARTx,uint8_t *s)
//{
//	while(*s!='\0')
//	{ 
//		while(USART_GetFlagStatus(USARTx,USART_FLAG_TC )==RESET);	
//		USART_SendData(USARTx,*s);
//		s++;
//	}
//}

void USART_BLUETOOTH_SendBuf(uint8_t *s)
{
	while(*s!='\0')
	{ 
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC )==RESET);	
		USART_SendData(USART1,*s);
		s++;
	}
}

//接收串口蓝牙数据
uint8_t USART_BLUETOOTH_ReciverBuf(void)
{
	//判断接收缓冲区是否为空
	while(!USART_GetFlagStatus(UART_BLUETOOTH,USART_FLAG_RXNE)){};
  return USART_ReceiveData(UART_BLUETOOTH);
}


/*******************************************************************************
* 函数名称 : UART_BLUETOOTH_Init
* 功能描述 : 初始化串口蓝牙
* 输入参数 : 波特率
* 输出参数 : 无
* 函数返回 : 无
*******************************************************************************/
void Init_UART_BLUETOOTH(uint32_t BaudRate)
{
	USART_DeInit(UART_BLUETOOTH);
	USART_InitTypeDef USART_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_APB2PeriphClockCmd(UART_BLUETOOTH_CLKSRC,ENABLE);  
	RCC_AHBPeriphClockCmd(UART_BLUETOOTH_IO_CLKSRC,ENABLE);
	
	//PA9,PA10
	GPIO_PinAFConfig(UART_BLUETOOTH_IO_PORT,UART_BLUETOOTH_TX_AF_PIN_SOURCE,GPIO_AF_1);
	GPIO_PinAFConfig(UART_BLUETOOTH_IO_PORT,UART_BLUETOOTH_TX_AF_PIN_SOURCE,GPIO_AF_1);  

	GPIO_InitStruct.GPIO_Pin=UART_BLUETOOTH_RX_PIN|UART_BLUETOOTH_TX_PIN;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStruct);


	/*USART基本配置*/
	USART_InitStruct.USART_BaudRate=UART_BAUDRATE;
	USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStruct.USART_Parity=USART_Parity_No;
	USART_InitStruct.USART_StopBits=USART_StopBits_1;
	USART_InitStruct.USART_WordLength=USART_WordLength_8b;
	USART_Init(UART_BLUETOOTH,&USART_InitStruct);
		

//		NVIC_InitTypeDef NVIC_InitStructure;
//		
//		/* USART1 IRQ Channel configuration */
//		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//		NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//		NVIC_Init(&NVIC_InitStructure);
//		
//		USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	USART_Cmd(UART_BLUETOOTH,ENABLE);
}


/*******************************************************************************
* 函数名称 : DMA_ReWorkCfg
* 功能描述 : 对于一个已经初始化的DMA通道, 重新配置其工作
* 输入参数 : dma_Channel_x  指向DMA通道的指针
*            mem_address    重新配置的内存地址
*            num            重新传输的数目
*******************************************************************************/
static void DMA_ReWorkCfg (DMA_Channel_TypeDef* dma_Channel_x, uint32_t mem_address, uint32_t num)
{
	dma_Channel_x->CCR &= 0xFFFFFFFE;                /* 先禁止DMAx通道 */

	dma_Channel_x->CMAR = mem_address;               /* 重新配置内存地址 */

	dma_Channel_x->CNDTR = num;                      /* 重新传输数目 */

	dma_Channel_x->CCR |= 0x00000001;                /* 使能DMAx通道 */
}

/*******************************************************************************
* 函数名称 : UartSendNBytes
* 功能描述 : 通过DMA机制从串口发送数据
* 输入参数 : 无
* 输出参数 : 无
* 函数返回 : 无
*******************************************************************************/
void UartSendNBytes (uint8_t *p_buf, uint32_t num)
{
	DMA_ReWorkCfg(UART_DMA_TX_CHANNEL, (uint32_t)p_buf, num);
}

/*******************************************************************************
* 函数名称 : GetUartReceiverResidualCnt
* 功能描述 : 取串口pdc缓冲区剩余计数
* 输入参数 : 无
* 输出参数 : 无
* 函数返回 :串口pdc缓冲区剩余计数 
*******************************************************************************/
int32_t GetUartReceiverResidualCnt(void)
{
	return(DMA_GetCurrDataCounter(UART_DMA_RX_CHANNEL));
}



