#include "main.h"
void Bsp_NVIC_Init(void)
{
	NVIC_InitTypeDef											nvic;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	//can1
	nvic.NVIC_IRQChannel									= USB_LP_CAN1_RX0_IRQn;
	nvic.NVIC_IRQChannelCmd								= ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority					= 0;
	nvic.NVIC_IRQChannelSubPriority						= 0;
	NVIC_Init(&nvic);

	//tim6
	nvic.NVIC_IRQChannel						= TIM2_IRQn;
	nvic.NVIC_IRQChannelCmd						= ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority		= 2;
	nvic.NVIC_IRQChannelSubPriority				= 2;
	NVIC_Init(&nvic);



}
