#ifndef OS_CORE_H
#define OS_CORE_H

#include "stm32f0xx.h"

/*定义任务控制块结构体*/
typedef struct SCB_DATA
{
  void(*ptask)(void);
  uint8_t id;
  uint8_t run_en;
  uint32_t delay_period;
  uint32_t delay_start;
}TCB_DATA;

/*外部函數定義*/
void os_init(void);
//void Init_sysTick(uint8_t SYSCLK);

void os_start(void);
void os_create_task(void(*c_ptask)(void), uint8_t c_run_en, uint8_t c_id);
void os_resume_task(uint8_t r_id);
void os_pend_task(uint8_t r_id);
void os_isr_ticks(void);
void os_delay_ms(uint8_t r_id, uint32_t ms);
void os_delay_100us(uint8_t r_id, uint32_t x100us);
void os_delay_10us(uint8_t r_id, uint32_t x10us);

#endif
