#ifndef __CONTROL_H
#define __CONTROL_H
#include "stm32f10x.h"

#define init_time_ms 6000//��ʼ��б��
#define moterOperat_ms 1000//��ʼ��б��

typedef enum System_mode_t
{
	prepare=2,
  normal=1,
	stop=3,
}System_mode_t;//����ģʽ

typedef enum supp1y_status_t
{
	off=0,
	start=1,
	wait=2,
	end=3,
	
}supp1y_status_t;//����վ״̬
extern System_mode_t System_mode;
void Control_Loop(void);
void GMPitch_ControlLoop(void);
void LoopMode(uint8_t mode);
void Supply_ControlLoop1(void);
void Supply_ControlLoop2(void);
void CM1_Mode(uint8_t mode);
void CM21_Mode(uint8_t mode);


#endif
