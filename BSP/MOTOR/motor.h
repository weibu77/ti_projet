#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "ti_msp_dl_config.h"

#define MOTOR_PWM_PERIOD   (1600)
#define MOTOR_PWM_LOAD     (MOTOR_PWM_PERIOD - 1)

void Motor_Init(void);
void Motor_SetSpeed(int Motor1, int Motor2);
void Motor_Stop(void);
void SpeedLoop_Start(void);
void SpeedLoop_StartStraight(uint32_t durationMs);
void SpeedLoop_SetRampStep(float rampStepMmS);
void SpeedLoop_SetStraightMode(uint8_t enable);
void SpeedLoop_SetStraightTargetYaw(float yawDeg);
void SpeedLoop_Stop(void);
void SpeedLoop_TimerIRQHandler(void);
void SpeedLoop_FormatVofaLine(char *buffer, uint16_t size);

#endif
