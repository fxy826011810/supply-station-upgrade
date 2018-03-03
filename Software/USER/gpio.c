#include "main.h"
#define KEY_1 GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0)
#define KEY_2 GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_1)
#define KEY_3 GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)
#define KEY_4 GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)
key_t key_1={0,0},key_2={0,0},key_3={0,0},key_4={0,0};
#define key_set 0
static void gpio_init(gpio_init_t *gpio)
{
	GPIO_InitTypeDef						GPIO;
	if(gpio->mode==GPIO_Mode_AF_PP)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(gpio->rcc, ENABLE);
  GPIO.GPIO_Mode=gpio->mode;
	GPIO.GPIO_Speed=gpio->speed;
	GPIO.GPIO_Pin=gpio->pin;//按板子定
	GPIO_Init(gpio->gpio,&GPIO);
}

void Key_Scan(void)
{
	if(KEY_1==key_set)
		{
			if(key_1.time>4)
			{
				key_1.status=1;
			}
			else
			{
				key_1.time+=1;
			}
			
		}
	else
		{
			key_1.time=0;
			key_1.status=0;
		}
	if(KEY_2==key_set)
		{
			if(key_2.time>4)
			{
				key_2.status=1;
			}
			else
			{
				key_2.time+=1;
			}
		}
	else
		{
			key_2.time=0;
			key_2.status=0;
		}
	if(KEY_3==key_set)
		{
			if(key_3.time>4)
			{
				key_3.status=1;
			}
			else
			{
				key_3.time+=1;
			}
		}
	else
		{
			key_3.time=0;
			key_3.status=0;
		}
	if(KEY_4==key_set)
		{
			if(key_4.time>4)
			{
				key_4.status=1;
			}
			else
			{
				key_4.time+=1;
			}
		}
	else
		{
			key_4.time=0;
			key_4.status=0;
		}
}






gpio_init_t \
gpio_can1r ={RCC_APB2Periph_GPIOB,	GPIOB,GPIO_Pin_8,	GPIO_Speed_50MHz,	GPIO_Mode_IPU},\
gpio_can1t=	{RCC_APB2Periph_GPIOB,	GPIOB,GPIO_Pin_9,	GPIO_Speed_50MHz,	GPIO_Mode_AF_PP},\
//光电开关
gpio_key1={RCC_APB2Periph_GPIOE,	GPIOE,GPIO_Pin_0,	GPIO_Speed_50MHz,	GPIO_Mode_IPU},\
gpio_key2={RCC_APB2Periph_GPIOE,	GPIOE,GPIO_Pin_1,	GPIO_Speed_50MHz,	GPIO_Mode_IPU},\

gpio_key3={RCC_APB2Periph_GPIOE,	GPIOE,GPIO_Pin_2,	GPIO_Speed_50MHz,	GPIO_Mode_IPU},\
gpio_key4={RCC_APB2Periph_GPIOE,	GPIOE,GPIO_Pin_3,	GPIO_Speed_50MHz,	GPIO_Mode_IPU},\

gpio_led2={RCC_APB2Periph_GPIOA,	GPIOA,GPIO_Pin_10,	GPIO_Speed_50MHz,	GPIO_Mode_Out_PP};


void Bsp_GPIO_Init(void)
{

//	//can总线gpio初始化
	gpio_init(&gpio_can1r);
	gpio_init(&gpio_can1t);

	//外设按键手动控制
	gpio_init(&gpio_key1);
	gpio_init(&gpio_key2);
	gpio_init(&gpio_key3);
	gpio_init(&gpio_key4);

	GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);
}
