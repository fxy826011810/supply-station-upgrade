#include "main.h"

////�������� ecd_bias��ֵ
////*iΪ����ֵ��ָ��
////dirΪ����  ��dir=1ʱΪ�ۼ� dir=0ʱ�ۼ� maxminΪ�ۼ��ۼ�����ֵ
//void numSmooth(int32_t *i,int32_t maxmin,uint8_t dir)
//{
//	if(dir==1)
//	{
//		if(*i<maxmin)
//		{
//			*i++;
//		}
//	}
//	else if(dir==0)
//	{
//		if(*i>maxmin)
//		{
//			*i--;
//		}
//	}
//	
//}
void smoothFilter(float i,float *o,float rate)
{
	*o=(*o)*(1-rate)+(i)*rate;
	//*o=(*o)+(i)*rate;
}


void GMPitch_ControlLoop(void)//aΪid 1 ��ֵ��ʱ�룬��ֵʱ��  bΪid 2 ��ֵ˳ʱ�룬��ֵ��ʱ��
{

////	
	CM1PositionPID.realdata[0] =CM1Encoder.ecd_angle;
  CM1PositionPID.setdata=0;
  CM1PositionPID.test(&CM1PositionPID);
        
  CM1SpeedPID.setdata= CM1PositionPID.output[0];
  CM1SpeedPID.realdata[NOW]=CM1Encoder.filter_rate;
  CM1SpeedPID.test(&CM1SpeedPID);
////
	CM2PositionPID.realdata[0] =CM2Encoder.ecd_angle;
  CM2PositionPID.setdata=0;
  CM2PositionPID.test(&CM2PositionPID);
        
  CM2SpeedPID.setdata= CM2PositionPID.output[0];
  CM2SpeedPID.realdata[NOW]=CM2Encoder.filter_rate;
  CM2SpeedPID.test(&CM2SpeedPID);



	
}
//���������pid����
void PID_ControlLoop(void)
{
   GMPitch_ControlLoop();
   ec60_senddata(CAN1, 0x200,(int16_t)CM1SpeedPID.output[0],0,(int16_t)CM2SpeedPID.output[0],0);//�������  id=200����24v id=0x1FF����12v
}	



#define HALF_ANGLE 180 //�����ת�Ƕ�
#define WAIT_TIME 1000 //�ȴ��趨ʱ��



#define GDKG_ON 	1  //��翪�ؿ�
#define GDKG_OFF	0	//��翪�ع�
typedef enum
{
	lock=0,
	unlock=1,
}supplyLockStatus_t;//����վ��ö��


typedef struct
{
	uint16_t 							waitTime; 					//�ȴ�ʱ��							����
	float 							supplyOffset;					//����ϵ��ҵĳ�ʼֵ
	float 								supplySetAngle;			//���ڴ�����Ժ�� 		Encoder.ecd_bias
	supplyLockStatus_t 		supplyLockStatus;		//����վ��ö�� 				����
	supp1y_status_t 			operateStatus;			//����վ����״̬ö��		����
	key_t 								*key1;							//��翪��1						ָ��
	key_t 								*key2;							//��翪��1						ָ��
	Encoder 							*encoder;						//���������	����ֵ			ָ��
}supply_t;

//���һ
//{
//�ȴ�ʱ��						0
//���Ժ��ecd_bias 	��ֵΪ0�����������ӵ�ֵ
//����վ�� 					unlock
//����վ����״̬			off
//��翪��1       		ָ��key_1
//��翪��2       		ָ��key_2
//���������					ָ��CM1Encoder
//}
supply_t supply1=\
{\
0,\
0.0f,\
0.0f,\
unlock,\
off,\
&key_1,\
&key_2,\
&CM1Encoder,\
};

//�����
//{
//�ȴ�ʱ��						0
//���Ժ��ecd_bias 	��ֵΪ0�����������ӵ�ֵ
//����վ�� 					unlock
//����վ����״̬			off
//��翪��1       		ָ��key_3
//��翪��2       		ָ��key_4
//���������					ָ��CM2Encoder
//}
supply_t supply2=\
{\
0,\
0.0f,\
0.0f,\
unlock,\
off,\
&key_3,\
&key_4,\
&CM2Encoder,\
};



void Supply_ControlLoop(supply_t *supply)//����վ��״̬����
{
		//����������翪��ȫ�ǹص����
		// 1.��;�˳�ʱ
		// 2.û�мӵ�ʱ
		if(((supply->key1->status==GDKG_OFF)&&(supply->key2->status==GDKG_OFF)))
	{
		{
			supply->supplySetAngle=0.0f;
			supply->encoder->ecd_bias=(int)supply->supplyOffset+(int)supply->supplySetAngle;
		}
		supply->supplyLockStatus=unlock;//����վ���� unlock�����������ö��
		//��;�˳�ʱ
		if((supply->operateStatus==start||supply->operateStatus==wait))//��;������վ���
		{
			supply->waitTime=1000;
		}
	}
	//4		//endʱ������ϻ�����ת
	if(supply->operateStatus==end)
	{
		{
			smoothFilter(0.0f,&supply->supplySetAngle,0.01);
			supply->encoder->ecd_bias=(int)supply->supplyOffset+(int)supply->supplySetAngle;
		}
		if(supply->supplySetAngle<0xF)
		{
			//ת����ʱ״̬��Ϊoff����һ�μӵ�����
			supply->operateStatus=off;
		}
	}
	//3		//waitʱ������µȴ�
	if(supply->operateStatus==wait)
	{
		{
			supply->supplySetAngle=0x1000;
			supply->encoder->ecd_bias=(int)supply->supplyOffset+(int)supply->supplySetAngle;
		}
		supply->waitTime++;
		if(supply->waitTime>WAIT_TIME)
		{
			//�ȴ�ʱ�䵽��״̬��Ϊend���������¼ʱ��ı��������ȴ���һ��
			supply->waitTime=0;
			supply->operateStatus=end;
		}
	}
	//2		//startʱ������»�����ת
	if(supply->operateStatus==start)
	{
		{//////////////////////////////////////////////////////////////////////////////////
			smoothFilter((float)0x1000,&supply->supplySetAngle,0.01);
			supply->encoder->ecd_bias=(int)supply->supplyOffset+(int)supply->supplySetAngle;
		}//////////////////////////////////////////////////////////////////////////////////
		if(supply->supplySetAngle>0xFF0)
		{
			//ת����ʱ״̬��Ϊwait
			supply->operateStatus=wait;
		}
	}	
	
				//��翪��1�͹�翪��1ͬʱ����
				//����վ״̬Ϊoff
	//1		//����վ����Ϊδ������״̬
				//��������ȫ������ʱ������վ״̬��Ϊ��ʼ�ӵ�
	if(supply->key1->status==GDKG_ON && supply->key2->status==GDKG_ON && supply->operateStatus==off && supply->supplyLockStatus==unlock)
	{
		supply->operateStatus=start;
		supply->supplyLockStatus=lock;//������վ����������;�ı�״̬
	}	
	
}


uint32_t heart=0;
//ϵͳ״̬			��ʼ��׼��״̬
System_mode_t System_mode=prepare;
void Control_Loop(void)//ģʽ����
{
	heart++;
	//��翪��ɨ��
	Key_Scan();
	
	if(System_mode==normal)
	{
		//����վ1
		Supply_ControlLoop(&supply1);
		//����վ2
		Supply_ControlLoop(&supply2);
	}
	if(System_mode==prepare)
	{
		
		smoothFilter((float)0x1C00,&supply1.supplyOffset,0.005);
		smoothFilter((float)0x1F4C,&supply2.supplyOffset,0.005);
		
		supply1.encoder->ecd_bias=(int)supply1.supplyOffset;
		supply2.encoder->ecd_bias=(int)supply2.supplyOffset;
		
		if((int)supply1.supplyOffset>0x1BF1&&(int)supply1.supplyOffset>0x1F3C)
		{
			supply1.supplyOffset=(float)0x1C00;
			supply2.supplyOffset=(float)0x1F4C;
			
			supply1.encoder->ecd_bias=(int)supply1.supplyOffset;
			supply2.encoder->ecd_bias=(int)supply2.supplyOffset;
			
			System_mode=normal;
		}
		
	}
	if(heart%10==0)
	{
	//���PID����
		PID_ControlLoop();
	}
	
	if(heart==10000)
		heart =0;
	
}	


