#include "main.h"

void system_init(void)
{

  Bsp_NVIC_Init();	//中断初始化
	Bsp_Pid_Init();		//pid初始化
	Bsp_GPIO_Init();	//GPIO初始化
	Bsp_Can_Init();		//can初始化
	Bsp_Tim_Init();		//定时器初始化(tim6)
}


int main(void)
{
  system_init();//系统初始化
	while (1)
	{					


  }
  
}
