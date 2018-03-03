#include "main.h"




//此函数为速度PID+位置PID 
//两个输入参数分别为 位置PID指针 和速度PID指针 和电机编码器指针
void RM6025_ControlLoop(PID_TypeDef *PositionPID , PID_TypeDef *SpeedPID , Encoder *encoder)
{
	PositionPID->setdata	=0;											//位置PID设定值
	PositionPID->realdata =encoder->ecd_angle;		//位置PID实际值
  PositionPID->calc(PositionPID);								//位置PID计算
        
  SpeedPID->setdata			= PositionPID->output;	//速度PID设定值
  SpeedPID->realdata		=encoder->filter_rate;	//速度PID实际值
  SpeedPID->calc(SpeedPID);											//速度PID计算
}



//进行了一次宏定义没任何意义只是为了更好的区分函数
//等同于上面的函数
#define _12v_RM6025_ControlLoop RM6025_ControlLoop
#define _24v_RM6025_ControlLoop RM6025_ControlLoop

#define _12v_RM6025_Senddata 		cm_senddata
#define _24v_RM6025_Senddata 		cm_senddata
#define EC60_SendData						ec60_senddata 

//两个电机的pid控制
void PID_ControlLoop(void)
{
	{	//=============大括号内为两个电机的pid运算===========//
		{
		_12v_RM6025_ControlLoop(&_12v_RM6025_PositionPID,&_12v_RM6025_SpeedPID,&_12V_RM6025Encoder);//12v速度加位置 函数名等同于RM6025_ControlLoop
		_24v_RM6025_ControlLoop(&_24v_RM6025_PositionPID,&_24v_RM6025_SpeedPID,&_24V_RM6025Encoder);//24v速度加位置 函数名等同于RM6025_ControlLoop
		}
		//=============以下为两个不同发送id的做法===========//
		{
//		_12v_RM6025_Senddata(CAN1,CAN_RM6025_12v_ID,_12v_RM6025_SpeedPID.output);//12v发送函数
//		_24v_RM6025_Senddata(CAN1,CAN_RM6025_24v_ID,_24v_RM6025_SpeedPID.output);//24v发送函数
		}
		//=============以下为两个相同发送id的做法 根据电机返回id将数据写在后4个数上面===========//
		{
		EC60_SendData(CAN1,0x200,_12v_RM6025_SpeedPID.output,_24v_RM6025_SpeedPID.output,0,0);
		}
	}

}	



//补给站的状态控制
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
	float 							supplyOffset;				//电机上电找的初始值
	float 								supplySetAngle;			//用于存放线性后的 		Encoder.ecd_bias
	supplyLockStatus_t 		supplyLockStatus;		//补给站锁枚举 				变量
	supply_status_t 			operateStatus;			//补给站运行状态枚举		变量
	key_t 								*key1;							//光电开关1						指针
	key_t 								*key2;							//光电开关1						指针
	ramp_t 								*Ramp;							//ramp								指针
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
//ramp					  	指向GMPRamp
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
&GMPRamp,\
&_12V_RM6025Encoder,\
};

//电机二
//{
//等待时间						0
//线性后的ecd_bias 	初值为0，即缓慢增加的值
//补给站锁 					unlock
//补给站运行状态			off
//光电开关1       		指向key_3
//光电开关2       		指向key_4
//ramp					  	指向GYRamp
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
&GMYRamp,\
&_24V_RM6025Encoder,\
};

void smoothFilter(float i,float *o,float rate)
{
	*o=(*o)*(1-rate)+(i)*rate;
	//*o=(*o)+(i)*rate;
}

void Supply_ControlLoop(supply_t *supply)//补给站的状态控制
{
		//满足两个光电开关全是关的情况
		// 1.中途退出时
		// 2.没有加弹时
		if(((supply->key1->status==GDKG_OFF)&&(supply->key2->status==GDKG_OFF)))
	{
		{
			//////////////////////////////////////////////////////////////////////////////////
			supply->supplySetAngle=0.0f;
			supply->encoder->ecd_bias=(int)(supply->supplyOffset+supply->supplySetAngle);
			//////////////////////////////////////////////////////////////////////////////////
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
			//此时supply->supplySetAngle为0x1000
			//通过一个缓慢的减小使supply->supplySetAngle等于0x0
			//supply->supplyOffset为初始值
			//////////////////////////////////////////////////////////////////////////////////
			smoothFilter(0.0f,&supply->supplySetAngle,0.01);
			supply->encoder->ecd_bias=(int)(supply->supplyOffset+supply->supplySetAngle);
			//////////////////////////////////////////////////////////////////////////////////
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
			//这两句有没有无所谓
			//////////////////////////////////////////////////////////////////////////////////
			supply->supplySetAngle=0x1000;
			supply->encoder->ecd_bias=(int)(supply->supplyOffset+supply->supplySetAngle);
			//////////////////////////////////////////////////////////////////////////////////
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
		{
			//此时supply->supplySetAngle为0
			//通过一个缓慢的增加使supply->supplySetAngle等于0x1000
			//supply->supplyOffset为初始值
			//////////////////////////////////////////////////////////////////////////////////
			smoothFilter((float)0x1000,&supply->supplySetAngle,0.01);
			supply->encoder->ecd_bias=(int)(supply->supplyOffset+supply->supplySetAngle);
			//////////////////////////////////////////////////////////////////////////////////
		}
		
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
void ControlLoop(void)//模式控制
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
	//系统上电处于prepare即准备状态supply.supplyOffset为0
	//通过一个缓慢的增加使supply1.supplyOffset等于顶端的初始值
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

