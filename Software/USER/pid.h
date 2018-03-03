#ifndef __PID_H
#define __PID_H


typedef struct PID_TypeDef
{	float Kp;//����ϵ��
	float Ki;//����ϵ��
	float Kd;//΢��ϵ��
        
	float setdata;//�趨ֵ
	float setimax;//�趨�������ֵ
  float setomax;//������ֵ
	 
	float Pout;//�������
	float Iout;//�������
	float Dout;//΢�����
	float error[2];//ƫ��ֵ
	float realdata;//ʵ���ٶ�ֵ
	float output;//PID���
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
