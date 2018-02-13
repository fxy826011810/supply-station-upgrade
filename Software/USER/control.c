#include "main.h"

////用于线性 ecd_bias的值
////*i为输入值的指针
////dir为方向  即dir=1时为累加 dir=0时累减 maxmin为累加累减的阈值
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


void GMPitch_ControlLoop(void)//a为id 1 正值逆时针，负值时针  b为id 2 正值顺时针，负值逆时针
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
//两个电机的pid控制
void PID_ControlLoop(void)
{
   GMPitch_ControlLoop();
   ec60_senddata(CAN1, 0x200,(int16_t)CM1SpeedPID.output[0],0,(int16_t)CM2SpeedPID.output[0],0);//电机发送  id=200发给24v id=0x1FF发给12v
}	



#define HALF_ANGLE 180 //电机旋转角度
#define WAIT_TIME 1000 //等待设定时间



#define GDKG_ON 	1  //光电开关开
#define GDKG_OFF	0	//光电开关关
typedef enum
{
	lock=0,
	unlock=1,
}supplyLockStatus_t;//补给站锁枚举


typedef struct
{
	uint16_t 							waitTime; 					//等待时间							变量
	float 							supplyOffset;					//电机上电找的初始值
	float 								supplySetAngle;			//用于存放线性后的 		Encoder.ecd_bias
	supplyLockStatus_t 		supplyLockStatus;		//补给站锁枚举 				变量
	supp1y_status_t 			operateStatus;			//补给站运行状态枚举		变量
	key_t 								*key1;							//光电开关1						指针
	key_t 								*key2;							//光电开关1						指针
	Encoder 							*encoder;						//电机编码器	解算值			指针
}supply_t;

//电机一
//{
//等待时间						0
//线性后的ecd_bias 	初值为0，即缓慢增加的值
//补给站锁 					unlock
//补给站运行状态			off
//光电开关1       		指向key_1
//光电开关2       		指向key_2
//电机编码器					指向CM1Encoder
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

//电机二
//{
//等待时间						0
//线性后的ecd_bias 	初值为0，即缓慢增加的值
//补给站锁 					unlock
//补给站运行状态			off
//光电开关1       		指向key_3
//光电开关2       		指向key_4
//电机编码器					指向CM2Encoder
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



void Supply_ControlLoop(supply_t *supply)//补给站的状态控制
{
		//满足两个光电开关全是关的情况
		// 1.中途退出时
		// 2.没有加弹时
		if(((supply->key1->status==GDKG_OFF)&&(supply->key2->status==GDKG_OFF)))
	{
		{
			supply->supplySetAngle=0.0f;
			supply->encoder->ecd_bias=(int)supply->supplyOffset+(int)supply->supplySetAngle;
		}
		supply->supplyLockStatus=unlock;//补给站解锁 unlock见函数上面的枚举
		//中途退出时
		if((supply->operateStatus==start||supply->operateStatus==wait))//中途出补给站情况
		{
			supply->waitTime=1000;
		}
	}
	//4		//end时电机向上缓慢旋转
	if(supply->operateStatus==end)
	{
		{
			smoothFilter(0.0f,&supply->supplySetAngle,0.01);
			supply->encoder->ecd_bias=(int)supply->supplyOffset+(int)supply->supplySetAngle;
		}
		if(supply->supplySetAngle<0xF)
		{
			//转到顶时状态变为off结束一次加弹过程
			supply->operateStatus=off;
		}
	}
	//3		//wait时电机向下等待
	if(supply->operateStatus==wait)
	{
		{
			supply->supplySetAngle=0x1000;
			supply->encoder->ecd_bias=(int)supply->supplyOffset+(int)supply->supplySetAngle;
		}
		supply->waitTime++;
		if(supply->waitTime>WAIT_TIME)
		{
			//等待时间到了状态变为end，并归零记录时间的变量，并等待下一次
			supply->waitTime=0;
			supply->operateStatus=end;
		}
	}
	//2		//start时电机向下缓慢旋转
	if(supply->operateStatus==start)
	{
		{//////////////////////////////////////////////////////////////////////////////////
			smoothFilter((float)0x1000,&supply->supplySetAngle,0.01);
			supply->encoder->ecd_bias=(int)supply->supplyOffset+(int)supply->supplySetAngle;
		}//////////////////////////////////////////////////////////////////////////////////
		if(supply->supplySetAngle>0xFF0)
		{
			//转到底时状态变为wait
			supply->operateStatus=wait;
		}
	}	
	
				//光电开关1和光电开关1同时开启
				//补给站状态为off
	//1		//补给站处于为未加锁的状态
				//以上条件全部满足时将补给站状态变为开始加蛋
	if(supply->key1->status==GDKG_ON && supply->key2->status==GDKG_ON && supply->operateStatus==off && supply->supplyLockStatus==unlock)
	{
		supply->operateStatus=start;
		supply->supplyLockStatus=lock;//给补给站加锁不能中途改变状态
	}	
	
}


uint32_t heart=0;
//系统状态			初始化准备状态
System_mode_t System_mode=prepare;
void Control_Loop(void)//模式控制
{
	heart++;
	//光电开关扫描
	Key_Scan();
	
	if(System_mode==normal)
	{
		//补给站1
		Supply_ControlLoop(&supply1);
		//补给站2
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
	//电机PID控制
		PID_ControlLoop();
	}
	
	if(heart==10000)
		heart =0;
	
}	


