#include "ti_msp_dl_config.h"
#include "PID.h"
#include "encoder.h"

PID_TypeDef motorA;
PID_TypeDef motorB;
PID_TypeDef move;
PID_TypeDef angle;

void pid_init(void)
{
	motorA.ref = 300 * 0.987;		motorB.ref = 300;				move.ref = 0;			angle.ref = 0;				
	motorA.p = 6.5;					motorB.p = 6.5;					move.p = 0;       angle.p = 0;					
	motorA.i = 0.18;				motorB.i = 0.18;				move.i = 0;        angle.i = 0;				
	motorA.d = 0;					motorB.d = 0;				move.d = 0;        angle.d = 0;				
	motorA.max_out = 800;			motorB.max_out = 800;		move.max_out = 32; angle.max_out = 32;	
	motorA.max_iout = 500;			motorB.max_iout = 500;		move.max_iout = 0; angle.max_iout = 0;
}

void PID_ResetState(PID_TypeDef *pid)
{
	pid->pos_out = 0.0f;
	pid->error = 0.0f;
	pid->lasterror = 0.0f;
	pid->preverror = 0.0f;
	pid->pout = 0.0f;
	pid->iout = 0.0f;
	pid->dout = 0.0f;
}

void PID_SetParam(PID_TypeDef *pid, float kp, float ki, float kd)
{
	pid->p = kp;
	pid->i = ki;
	pid->d = kd;
	PID_ResetState(pid);
}

float PID_ctrl(PID_TypeDef *pid, float realspeed, float ref)
{	
	pid->error=ref-realspeed;
	
	pid->pout=pid->p*pid->error;
	pid->iout+=pid->i * pid->error;
	pid->dout=pid->d*(pid->error-pid->lasterror);
	pid->lasterror = pid->error;
	abs_limit(&pid->iout,pid->max_iout);
	
	pid->pos_out=pid->pout+pid->iout+pid->dout;
	abs_limit(&pid->pos_out,pid->max_out);
	return(int32_t)pid->pos_out;
}
float PID_angle(PID_TypeDef *pid, float realspeed, float ref)
{	
	pid->error=ref-realspeed;
	
	pid->pout=pid->p*pid->error;
	pid->iout+=pid->i * pid->error;
	pid->dout=pid->d*(pid->error-pid->lasterror);
	pid->lasterror = pid->error;
	abs_limit(&pid->iout,pid->max_iout);
	
	pid->pos_out=pid->pout+pid->iout+pid->dout;
	abs_limit(&pid->pos_out,pid->max_out);
	return(int32_t)pid->pos_out;
}

void abs_limit(float *out, float maxout)
{
	if(*out > maxout)
	{
		*out = maxout;
	}
	else if(*out < -maxout)
	{
		*out = -maxout;
	}
}

// void pid_test(void)
// {
// 	printf("%.2f,%.2f,%.2f,%.2f\r\n",
// 		motorA.ref, (float) g_leftSpeed,
// 		motorB.ref, (float) g_rightSpeed);
// 	delay_ms(VOFA_PRINT_PERIOD_MS);
// }


