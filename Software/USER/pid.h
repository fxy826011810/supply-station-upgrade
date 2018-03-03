#ifndef __PID_H
#define __PID_H


typedef struct PID_TypeDef
{	float Kp;//比例系数
	float Ki;//积分系数
	float Kd;//微分系数
        
	float setdata;//设定值
	float setimax;//设定积分最大值
  float setomax;//输出最大值
	 
	float Pout;//比例输出
	float Iout;//积分输出
	float Dout;//微分输出
	float error[2];//偏差值
	float realdata;//实际速度值
	float output;//PID输出
	void(*calc)(struct PID_TypeDef *pid);
	void(*reset)(struct PID_TypeDef *pid);

}PID_TypeDef;


void Bsp_Pid_Init(void);
void Pid_Test(PID_TypeDef* pid);
void  Pid_Reset(PID_TypeDef* pid);



#define CMPositionPID_default \
{\
0.0f,\
0.0f,\
0.0f,\
0,\
2000,\
4900,\
0,\
0,\
0,\
{0,0},\
0,\
0,\
&Pid_Test,\
&Pid_Reset,\
}

#define CMSpeedPID_default \
{\
0.0f,\
0.0f,\
0.0f,\
0,\
2000,\
4900,\
0,\
0,\
0,\
{0,0},\
0,\
0,\
&Pid_Test,\
&Pid_Reset,\
}



extern PID_TypeDef _12v_RM6025_PositionPID ;
extern PID_TypeDef _12v_RM6025_SpeedPID ;
extern PID_TypeDef _24v_RM6025_PositionPID ;
extern PID_TypeDef _24v_RM6025_SpeedPID ;

#endif
