#ifndef __GPIO_H
#define __GPIO_H
#include "stm32f10x.h"
#define	LED_HEAT() GPIOA->ODR^=GPIO_Pin_7|GPIO_Pin_8

typedef struct gpio_init_t
{
	uint32_t rcc;//Ê±ÖÓ
	GPIO_TypeDef* gpio;//GPIO
	uint16_t pin;//Òý½Å
	GPIOSpeed_TypeDef speed;
  GPIOMode_TypeDef mode;
}gpio_init_t;
typedef struct key_t
{
	uint8_t time;
	uint8_t status;
}key_t;
extern key_t key_1,key_2,key_3,key_4;
void Key_Scan(void);
void Bsp_GPIO_Init(void);
void whole_Scan(void);

#endif
