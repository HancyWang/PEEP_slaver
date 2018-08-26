#include "os_core.h"
#include "os_cfg.h"
#include "stm32f0xx.h"
#include OS_INIT_HEAD_FILE


/******************************
* OS��ȫ�ֱ�������
*******************************/
/*�΄տ��u�K*/
TCB_DATA task_array[OS_MAX_TASK];
/*����ʱ�Ӽ���*/
uint32_t os_ticks = 0;

 uint16_t prev_pwm_tick_cnt;
 uint16_t pwm_tick_cnt;

/*
* ������Ⱥ���
*/
void os_start(void)
{
  while(1)
  {
    static uint32_t index;
    for(index = 0; index < OS_MAX_TASK; index++)
    {
      if(task_array[index].run_en == OS_TRUE)//���б�־�Ƿ���Ч
      {
          if(task_array[index].delay_start + task_array[index].delay_period \
                  <= os_ticks)//�ӕr�r�g��
          {
              task_array[index].delay_start = 0;

              task_array[index].delay_period = 0;

              (*(task_array[index].ptask))();//��������
          }
      }
    }
  }
}

/*
* ��������
*��������
* void (*ptask)(void);
  uint8_t id;
  uint8_t run_en;
  uint32_t run_cnt;
*/
void os_create_task(void(*c_ptask)(void), uint8_t c_run_en, uint8_t c_id)
{
  uint8_t index = 0;
  index = c_id;
  task_array[index].run_en  = c_run_en;
  task_array[index].id      = c_id;
  task_array[index].ptask   = c_ptask;
  task_array[index].delay_period = 0;
  task_array[index].delay_start = 0;
}

/*
* ʹ������
*/
void os_resume_task(uint8_t r_id)
{
  if(r_id > OS_MAX_TASK-1)
    return;
  else
  {
    uint8_t index = 0;
    index = r_id;
    task_array[index].run_en = OS_TRUE;
  }
}

/*
* ��������
*/
void os_pend_task(uint8_t r_id)
{
  if(r_id > OS_MAX_TASK-1)
    return;
  else
  {
    uint8_t index = 0;
    index = r_id;
    task_array[index].run_en = OS_FALSE;
  }
}

/*
* �ӕr�΄գ��΄��ГQ
*/
void os_delay_ms(uint8_t r_id, uint32_t ms)
{
	if(r_id > OS_MAX_TASK-1)
    return;
	else
	{
		uint8_t index = 0;
    index = r_id;
		
		task_array[index].delay_start = os_ticks;
    task_array[index].delay_period = ms*OS_TICKS_PER_SEC/1000;
	}   
}

/*
* ϵͳ�δ�ʱ���ж�
*/
void os_isr_ticks(void)
{
  os_ticks ++;//ʱ�ӽ��ļ�����һ
}

/*
* ϵͳ��ʼ��
*/
void os_init(void)
{
	//os_ticks=4294967000;
	OS_INIT();
}

void Init_sysTick(uint8_t SYSCLK)
{
//	 uint16_t fac_us,fac_ms;
//   SysTick->CTRL&=0xfffffffb;    //bit2??,??????  HCLK/8
//   fac_us=SYSCLK/1;      
//   fac_ms=(uint16_t)fac_us*1000;
//   SysTick->LOAD=(uint32_t)1*fac_ms;    //????(SysTick->LOAD?24bit)
//	//SysTick->LOAD=48000;
//   SysTick->VAL =0x00;                    //?????
//   SysTick->CTRL |=(1<<0 |1<<1);    //????    
//    /* Function successful */
}

