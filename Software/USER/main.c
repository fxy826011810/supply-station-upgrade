#include "main.h"



//�����ڸ�ϱ���ο�ѧϰʹ��
//��������ѧ	 ������
//QQ					:826011810
//����				:826011810@qq.com

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
	
     system_init();//ϵͳ��ʼ��

	while (1)
	{					

  }
  
}
