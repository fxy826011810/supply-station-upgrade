#include "main.h"




//�˺���Ϊ�ٶ�PID+λ��PID 
//������������ֱ�Ϊ λ��PIDָ�� ���ٶ�PIDָ�� �͵��������ָ��
void RM6025_ControlLoop(PID_TypeDef *PositionPID , PID_TypeDef *SpeedPID , Encoder *encoder)
{
	PositionPID->setdata	=0;											//λ��PID�趨ֵ
	PositionPID->realdata =encoder->ecd_angle;		//λ��PIDʵ��ֵ
  PositionPID->calc(PositionPID);								//λ��PID����
        
  SpeedPID->setdata			= PositionPID->output;	//�ٶ�PID�趨ֵ
  SpeedPID->realdata		=encoder->filter_rate;	//�ٶ�PIDʵ��ֵ
  SpeedPID->calc(SpeedPID);											//�ٶ�PID����
}



//������һ�κ궨��û�κ�����ֻ��Ϊ�˸��õ����ֺ���
//��ͬ������ĺ���
#define _12v_RM6025_ControlLoop RM6025_ControlLoop
#define _24v_RM6025_ControlLoop RM6025_ControlLoop

#define _12v_RM6025_Senddata 		cm_senddata
#define _24v_RM6025_Senddata 		cm_senddata
#define EC60_SendData						ec60_senddata 

//���������pid����
void PID_ControlLoop(void)
{
	{	//=============��������Ϊ���������pid����===========//
		{
		_12v_RM6025_ControlLoop(&_12v_RM6025_PositionPID,&_12v_RM6025_SpeedPID,&_12V_RM6025Encoder);//12v�ٶȼ�λ�� ��������ͬ��RM6025_ControlLoop
		_24v_RM6025_ControlLoop(&_24v_RM6025_PositionPID,&_24v_RM6025_SpeedPID,&_24V_RM6025Encoder);//24v�ٶȼ�λ�� ��������ͬ��RM6025_ControlLoop
		}
		//=============����Ϊ������ͬ����id������===========//
		{
//		_12v_RM6025_Senddata(CAN1,CAN_RM6025_12v_ID,_12v_RM6025_SpeedPID.output);//12v���ͺ���
//		_24v_RM6025_Senddata(CAN1,CAN_RM6025_24v_ID,_24v_RM6025_SpeedPID.output);//24v���ͺ���
		}
		//=============����Ϊ������ͬ����id������ ���ݵ������id������д�ں�4��������===========//
		{
		EC60_SendData(CAN1,0x200,_12v_RM6025_SpeedPID.output,_24v_RM6025_SpeedPID.output,0,0);
		}
	}

}	



//����վ��״̬����
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
	float 							supplyOffset;				//����ϵ��ҵĳ�ʼֵ
	float 								supplySetAngle;			//���ڴ�����Ժ�� 		Encoder.ecd_bias
	supplyLockStatus_t 		supplyLockStatus;		//����վ��ö�� 				����
	supply_status_t 			operateStatus;			//����վ����״̬ö��		����
	key_t 								*key1;							//��翪��1						ָ��
	key_t 								*key2;							//��翪��1						ָ��
	ramp_t 								*Ramp;							//ramp								ָ��
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
//ramp					  	ָ��GMPRamp
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
&GMPRamp,\
&_12V_RM6025Encoder,\
};

//�����
//{
//�ȴ�ʱ��						0
//���Ժ��ecd_bias 	��ֵΪ0�����������ӵ�ֵ
//����վ�� 					unlock
//����վ����״̬			off
//��翪��1       		ָ��key_3
//��翪��2       		ָ��key_4
//ramp					  	ָ��GYRamp
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
&GMYRamp,\
&_24V_RM6025Encoder,\
};

void smoothFilter(float i,float *o,float rate)
{
	*o=(*o)*(1-rate)+(i)*rate;
	//*o=(*o)+(i)*rate;
}

void Supply_ControlLoop(supply_t *supply)//����վ��״̬����
{
		//����������翪��ȫ�ǹص����
		// 1.��;�˳�ʱ
		// 2.û�мӵ�ʱ
		if(((supply->key1->status==GDKG_OFF)&&(supply->key2->status==GDKG_OFF)))
	{
		{
			//////////////////////////////////////////////////////////////////////////////////
			supply->supplySetAngle=0.0f;
			supply->encoder->ecd_bias=(int)(supply->supplyOffset+supply->supplySetAngle);
			//////////////////////////////////////////////////////////////////////////////////
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
			//��ʱsupply->supplySetAngleΪ0x1000
			//ͨ��һ�������ļ�Сʹsupply->supplySetAngle����0x0
			//supply->supplyOffsetΪ��ʼֵ
			//////////////////////////////////////////////////////////////////////////////////
			smoothFilter(0.0f,&supply->supplySetAngle,0.01);
			supply->encoder->ecd_bias=(int)(supply->supplyOffset+supply->supplySetAngle);
			//////////////////////////////////////////////////////////////////////////////////
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
			//��������û������ν
			//////////////////////////////////////////////////////////////////////////////////
			supply->supplySetAngle=0x1000;
			supply->encoder->ecd_bias=(int)(supply->supplyOffset+supply->supplySetAngle);
			//////////////////////////////////////////////////////////////////////////////////
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
		{
			//��ʱsupply->supplySetAngleΪ0
			//ͨ��һ������������ʹsupply->supplySetAngle����0x1000
			//supply->supplyOffsetΪ��ʼֵ
			//////////////////////////////////////////////////////////////////////////////////
			smoothFilter((float)0x1000,&supply->supplySetAngle,0.01);
			supply->encoder->ecd_bias=(int)(supply->supplyOffset+supply->supplySetAngle);
			//////////////////////////////////////////////////////////////////////////////////
		}
		
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

//void setFilterEncoderAngle()
//{
//		smoothFilter((float)0x1C00,&supply1.supplyOffset,0.005);
//		smoothFilter((float)0x1F4C,&supply2.supplyOffset,0.005);
//		
//		supply1.encoder->ecd_bias=(int)supply1.supplyOffset;
//		supply2.encoder->ecd_bias=(int)supply2.supplyOffset;
//}

//void setEncoderAngle(float a,float b,)
//{
//		supply1.supplyOffset=(float)0x1C00;
//		supply2.supplyOffset=(float)0x1F4C;
//			
//		supply1.encoder->ecd_bias=(int)supply1.supplyOffset;
//		supply2.encoder->ecd_bias=(int)supply2.supplyOffset;
//}

System_mode_t System_mode=prepare;
uint32_t heart=0;
void ControlLoop(void)//ģʽ����
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
	//ϵͳ�ϵ紦��prepare��׼��״̬supply.supplyOffsetΪ0
	//ͨ��һ������������ʹsupply1.supplyOffset���ڶ��˵ĳ�ʼֵ
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

