#ifndef OS_CORE_H
#define OS_CORE_H

#include "stm32f0xx.h"

/*����������ƿ�ṹ��*/
typedef struct SCB_DATA
{
  void(*ptask)(void);
  uint8_t id;
  uint8_t run_en;
  uint16_t delay_period;
  uint32_t delay_start;
}TCB_DATA;

/*�ⲿ�������x*/
void os_init(void);
//void Init_sysTick(uint8_t SYSCLK);

void os_start(void);
void os_create_task(void(*c_ptask)(void), uint8_t c_run_en, uint8_t c_id);
void os_resume_task(uint8_t r_id);
void os_pend_task(uint8_t r_id);
void os_isr_ticks(void);
void os_delay_ms(uint8_t r_id, uint32_t ms);

#endif
