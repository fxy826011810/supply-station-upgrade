#ifndef __TIM_H
#define __TIM_H
#include "stm32f10x.h"
extern uint16_t _485Time,_485Count;
void Bsp_Tim_Init(void);
uint32_t Get_Time_Micros(void);
//void Pwm_Enable(void);
void Timer2_Init(u16 Period);


#endif

