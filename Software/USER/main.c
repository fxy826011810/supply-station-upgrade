#include "main.h"

void system_init(void)
{

  Bsp_NVIC_Init();	//�жϳ�ʼ��
	Bsp_Pid_Init();		//pid��ʼ��
	Bsp_GPIO_Init();	//GPIO��ʼ��
	Bsp_Can_Init();		//can��ʼ��
	Bsp_Tim_Init();		//��ʱ����ʼ��(tim6)
}


int main(void)
{
  system_init();//ϵͳ��ʼ��
	while (1)
	{					


  }
  
}
