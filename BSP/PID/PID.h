#ifndef __PID_H__
#define __PID_H__

typedef struct
{
	float pos_out;		 //�����
	float error;		//���
	float lasterror;	//�ϴε����
	float preverror;	//���ϴε����
	float ref;			//Ŀ��ֵ
	
	float p;
	float i;
	float d;
	float pout;
	float iout;
	float dout;
	
	float max_out;
	float max_iout;
	
}PID_TypeDef;
void pid_init(void);
void PID_ResetState(PID_TypeDef *pid);
void PID_SetParam(PID_TypeDef *pid, float kp, float ki, float kd);
float PID_ctrl(PID_TypeDef *pid, float realspeed, float ref);
float PID_angle(PID_TypeDef *pid, float realspeed, float ref);
void abs_limit(float *out, float maxout);
extern PID_TypeDef motorA;
extern PID_TypeDef motorB;
extern PID_TypeDef angle;
extern PID_TypeDef move;

#endif
