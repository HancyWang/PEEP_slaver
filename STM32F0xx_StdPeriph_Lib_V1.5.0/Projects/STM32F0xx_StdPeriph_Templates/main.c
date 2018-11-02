/**
  ******************************************************************************
  * @file    Project/STM32F0xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    05-December-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F0xx_StdPeriph_Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
	
#include "stm32f0xx.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_rcc.h"

#include "delay.h"
#include "os_cfg.h"
#include "hardware.h"
#include "app.h"

#include "comm_task.h"
#include "key_power_on_task.h"
#include "hardware.h"
#include "i2c.h"
#include "Motor_pwm.h"
//#include "device_type.h"
#include "stm32f0xx_dma.h"
#include "iwtdg.h"	
#include "rcc_configure.h"	
#include "app.h"
 const uint8_t default_parameter_buf[PARAMETER_BUF_LEN] = {
#if 1
200,2,

//MODE1
0x11,1,100,50,6,1,1,4,
0x12,1,100,50,6,1,1,4,
0x13,1,100,50,6,1,1,4,
0x14,1,100,60,6,1,1,4,
0x15,1,100,60,6,1,1,4,
0x16,1,100,60,6,1,1,4,

0x21,1,100,50,6,1,1,4,
0x22,1,100,50,6,1,1,4,
0x23,1,100,50,6,1,1,4,
0x24,1,100,60,6,1,1,4,
0x25,1,100,60,6,1,1,4,
0x26,1,100,60,6,1,1,4,

0x31,40,1,100,40,10,10,1,0,2,
0x32,60,1,100,45,10,10,1,0,2,
0x33,80,1,100,50,10,10,1,0,2,
0x34,100,1,100,55,10,10,1,0,2,
0x35,140,1,100,60,10,10,1,0,2,
0x36,160,1,100,65,10,10,1,0,2,


//MODE2
0x11,1,100,70,10,1,1,2,
0x12,1,100,70,10,1,1,2,
0x13,1,100,70,10,1,1,2,
0x14,1,100,80,10,1,1,2,
0x15,1,100,80,10,1,1,2,
0x16,1,100,80,10,1,1,2,

0x21,1,100,70,10,1,1,2,
0x22,1,100,70,10,1,1,2,
0x23,1,100,70,10,1,1,2,
0x24,1,100,80,10,1,1,2,
0x25,1,100,80,10,1,1,2,
0x26,1,100,80,10,1,1,2,

0x31,50,1,100,45,10,5,1,1,2,
0x32,65,1,100,50,10,5,1,1,2,
0x33,85,1,100,55,10,5,1,1,2,
0x34,105,1,100,60,10,5,1,1,2,
0x35,145,1,100,65,10,5,1,1,2,
0x36,165,1,100,65,10,5,1,1,2,


//MODE3
0x11,1,100,80,10,1,1,3,
0x12,1,100,80,10,1,1,3,
0x13,1,100,80,10,1,1,3,
0x14,1,100,90,10,1,1,3,
0x15,1,100,90,10,1,1,3,
0x16,1,100,90,10,1,1,3,

0x21,1,100,80,10,1,1,3,
0x22,1,100,80,10,1,1,3,
0x23,1,100,80,10,1,1,3,
0x24,1,100,90,10,1,1,3,
0x25,1,100,90,10,1,1,3,
0x26,1,100,90,10,1,1,3,

0x31,50,1,100,80,10,4,1,1,3,
0x32,68,1,100,80,10,4,1,1,3,
0x33,88,1,100,80,10,4,1,1,3,
0x34,110,1,100,90,10,4,1,1,3,
0x35,150,1,100,90,10,4,1,1,3,
0x36,170,1,100,90,10,4,1,1,3,

//Checksum
0x36,0xB7
		#endif
	};


int main(void)
{
  RCC_Configuration();
 	//self_tet_state=SELF_TEST_INFLATE;
	//Init_iWtdg(4,1250);  //4*2^4=64分频，1250(大概是1250*1.6ms=2s)
  delay_init();
	os_init();
	
#ifdef _DEBUG
#else	
	//进入stop模式
	EnterStopMode();
	//唤醒之后先初始化系统
	init_system_afterWakeUp();
#endif
	
	os_create_task(init_task, OS_TRUE, INIT_TASK_ID);
	os_start();
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
