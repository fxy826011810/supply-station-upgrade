#include "main.h"


//定义了两个结构体变量 
//名字和名字代表的意义没有关系，只是为了分别存放两个电机解算的数
Encoder _12V_RM6025Encoder = {0,0,0,0,0,0,0,0,0};//存放1电机
Encoder _24V_RM6025Encoder = {0,0,0,0,0,0,0,0,0};//存放二电机


 void Bsp_Can_Init(void)
{
			CAN_InitTypeDef                       can;
			CAN_FilterInitTypeDef                 can_filter;
			 

			RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

			//can1
			CAN_DeInit(CAN1);
			CAN_StructInit(&can);
			can.CAN_ABOM									= DISABLE;
			can.CAN_AWUM									= DISABLE;
			can.CAN_NART									= DISABLE;
			can.CAN_RFLM									= DISABLE;
			can.CAN_TTCM									= DISABLE;
			can.CAN_TXFP									= ENABLE;
			can.CAN_SJW										= CAN_SJW_1tq;
			can.CAN_BS1										= CAN_BS1_3tq;
			can.CAN_BS2										= CAN_BS2_5tq;
			can.CAN_Prescaler							= 4;
			can.CAN_Mode									= CAN_Mode_Normal;
			CAN_Init(CAN1, &can);
			
			
			can_filter.CAN_FilterActivation					= ENABLE;
			can_filter.CAN_FilterFIFOAssignment			= CAN_Filter_FIFO0;
			can_filter.CAN_FilterIdHigh							= 0x0000;
			can_filter.CAN_FilterIdLow							= 0x0000;
			can_filter.CAN_FilterMaskIdHigh					= 0x0000;
			can_filter.CAN_FilterMaskIdLow					= 0x0000;
			can_filter.CAN_FilterMode								= CAN_FilterMode_IdMask;
			can_filter.CAN_FilterNumber							= 0;
			can_filter.CAN_FilterScale							= CAN_FilterScale_32bit;
			can_filter.CAN_FilterNumber							= 0;
			CAN_FilterInit(&can_filter);
			
			CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);

}

void EncoderProcess(volatile Encoder *v, CanRxMsg * msg)//电机编码器值解算函数
{
			int i=0;
			int32_t temp_sum = 0;    
			v->last_raw_value = v->raw_value;
			v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
			v->diff = v->raw_value - v->last_raw_value;
			if(v->diff < -7500)    //两次编码器的反馈值差别太大，表示圈数发生了改变
			{
				v->round_cnt++;
				v->ecd_raw_rate = v->diff + 8192;
			}
			else if(v-> diff>7500)
			{
				v->round_cnt--;
				v->ecd_raw_rate = v->diff- 8192;
			}		
			else
			{
				v->ecd_raw_rate = v->diff;
			}
			//计算得到连续的编码器输出值
			v->ecd_value = v->raw_value- v->ecd_bias + v->round_cnt * 8192;
			//计算得到角度值，范围正负无穷大
			v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
			v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
			if(v->buf_count == RATE_BUF_SIZE)
			{
				v->buf_count = 0;
			}
			//计算速度平均值
			for(i = 0;i < RATE_BUF_SIZE; i++)
			{
				temp_sum += v->rate_buf[i];
			}
			v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);					
}





void cm_senddata(CAN_TypeDef* CANx, int16_t id, int16_t num)//电机发送  id=200发给24v id=0x1FF发给12v
{
		CanTxMsg  sendmessage;
		sendmessage.DLC									= 0x08;
		sendmessage.IDE									= CAN_ID_STD;
		sendmessage.RTR									= CAN_RTR_DATA;
	
		
	if(id==0x1FF)//24v时执行if内
	{
		sendmessage.StdId								= 0x1FF;
		sendmessage.Data[0] = 0;
		sendmessage.Data[1] = 0;
		sendmessage.Data[2] = ((num) >> 8);
		sendmessage.Data[3] = (num);
	}
	else if(id==0x200)//12v执行if内
	{
		sendmessage.StdId								= 0x200;
		sendmessage.Data[0]								= ((num) >> 8);
		sendmessage.Data[1]								= (num);
		sendmessage.Data[2]								= 0;
		sendmessage.Data[3]								= 0;
	}	
		sendmessage.Data[4]								= 0;
		sendmessage.Data[5]								= 0;
		sendmessage.Data[6]								= 0;
		sendmessage.Data[7]								= 0;
		CAN_Transmit(CANx, &sendmessage);
}

void ec60_senddata(CAN_TypeDef* CANx, int16_t id, int16_t num1, int16_t num2, int16_t num3, int16_t num4)//电机发送  id=200发给24v id=0x1FF发给12v
	{
		CanTxMsg  sendmessage;
		sendmessage.DLC										= 0x08;
		sendmessage.IDE										= CAN_ID_STD;
		sendmessage.RTR										= CAN_RTR_DATA;
	
		

		sendmessage.StdId									= id;
		sendmessage.Data[0]								= ((num1) >> 8);
		sendmessage.Data[1]								= (num1);
		sendmessage.Data[2]								= ((num2) >> 8);
		sendmessage.Data[3]								= (num2);
		sendmessage.Data[4]								= ((num3) >> 8);
		sendmessage.Data[5]								= (num3);
		sendmessage.Data[6]								= ((num4) >> 8);
		sendmessage.Data[7]								= (num4);
		CAN_Transmit(CANx, &sendmessage);
}



 void Can_RecviveData(CanRxMsg * rec)//返回值解算
{
	static long int time1=0,time2=0,time3=0,time4=0;
			switch (rec->StdId)
			{
				case CAN_RM6025_12v_REC_ID:
				{
					time1++;
					if(time1%10==0)
					{
						if(time1==10000)
							time1=0;
						
					EncoderProcess(&_12V_RM6025Encoder,rec);
					}
				}
					break;
				case CAN_RM6025_24v_REC_ID:
				{
					time2++;
					if(time2%10==0)
					{
						if(time2==10000)
							time2=0;
					EncoderProcess(&_24V_RM6025Encoder,rec);
					}
				}
					break;
				case 0x203:
				{
					time3++;
					if(time3%10==0)
					{
						if(time3==10000)
							time3=0;
					EncoderProcess(&_12V_RM6025Encoder,rec);
					}
				}
					break;
				case 0x204:
				{
					time4++;
					if(time4%10==0)
					{
						if(time4==10000)
							time4=0;
					EncoderProcess(&_24V_RM6025Encoder,rec);
					}
				}
					break;
			}
			
			
}



void USB_LP_CAN1_RX0_IRQHandler(void)//CAN1中断
{
			CanRxMsg		receivemessage;
			if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!=0)
			{
				CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
				CAN_Receive(CAN1, CAN_FIFO0, &receivemessage);//写在这里是为了能够正常的退出中断
        Can_RecviveData(&receivemessage);
			}
}




