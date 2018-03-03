#include "main.h"


PID_TypeDef _12v_RM6025_PositionPID = CMPositionPID_default;
PID_TypeDef _12v_RM6025_SpeedPID = CMSpeedPID_default;

PID_TypeDef _24v_RM6025_PositionPID = CMPositionPID_default;
PID_TypeDef _24v_RM6025_SpeedPID = CMSpeedPID_default;


void abs_limit(float *a, float ABS_MAX)
{
	if (*a > ABS_MAX)
		*a = ABS_MAX;
	if (*a < -ABS_MAX)
		*a = -ABS_MAX;
}

void  Pid_Reset(PID_TypeDef* pid)
{
	pid->Pout = 0;
	pid->Iout = 0;
	pid->Dout = 0;
	pid->output = 0;
}

void Bsp_Pid_Init(void)
	{
	_12v_RM6025_PositionPID.reset(&_12v_RM6025_PositionPID);
	_12v_RM6025_SpeedPID.reset(&_12v_RM6025_SpeedPID);
	_24v_RM6025_PositionPID.reset(&_24v_RM6025_PositionPID);
	_24v_RM6025_SpeedPID.reset(&_24v_RM6025_SpeedPID);

	}
	
enum
{
	NOW = 0,
	LAST = 1,
};

void Pid_Test(PID_TypeDef* pid)
{
		pid->error[NOW] = pid->setdata - pid->realdata;
		pid->Pout = pid->Kp*pid->error[NOW];
		pid->Iout += pid->Ki*pid->error[NOW];
		pid->Dout = pid->Kd*(pid->error[NOW] - pid->error[LAST]);
		abs_limit(&(pid->Iout),pid->setimax);
		pid->output = pid->Pout + pid->Iout + pid->Dout;
		abs_limit(&(pid->output),pid->setomax);
		pid->error[LAST] = pid->error[NOW];
}
