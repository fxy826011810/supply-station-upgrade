#include "main.h"



//仅用于给媳妇参考学习使用
//华北理工大学	 范翔宇
//QQ					:826011810
//邮箱				:826011810@qq.com

void system_init(void)
{
  Bsp_NVIC_Init();
	Bsp_Pid_Init();
	Bsp_GPIO_Init();
	Bsp_Can_Init();
//	CM1Encoder.ecd_bias=0x1C00;
//	CM2Encoder.ecd_bias=0x1F4C;
	delay_ms(1000);
	delay_ms(1000);
	Bsp_Tim_Init();

}


int main(void)
{
	
     system_init();//系统初始化

	while (1)
	{					

  }
  
}
