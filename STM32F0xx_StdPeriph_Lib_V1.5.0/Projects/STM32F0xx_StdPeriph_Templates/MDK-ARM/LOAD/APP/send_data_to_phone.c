#include "send_data_to_phone.h"
#include "serial_port.h"
#include "key_power_on_task.h"

extern MCU_STATE mcu_state;
void send_data_to_phone_task()
{
	
	//if(mcu_state==POWER_ON)
	{
		//1.采样flow sensor和pressure sensor的ADC值
	
		//2.通过串口wifi，发送数据到手机
//		uint8_t buffer[20]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x20};
//		USART_BLUETOOTH_SendBuf(buffer,20);
		
//		static uint8_t recv_buf[100];
//		for(int i=0;i<100;i++)
//		{
//			recv_buf[i]=	USART_BLUETOOTH_ReciverBuf();
//		}
		uint16_t data=0x1234;
		USART_SendData(USART1,data);
		os_delay_ms(KEY_SEND_DATA_TO_PHONE_TASK_ID, SEND_DATA_TO_PHONE_PERIOD);
	}
	
}
