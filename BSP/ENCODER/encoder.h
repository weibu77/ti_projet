#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "ti_msp_dl_config.h"

#define ENCODER_PPR                         (500)
#define ENCODER_GEAR_RATIO                  (28)
#define ENCODER_COUNTS_PER_MOTOR_REV        (ENCODER_PPR)
#define ENCODER_COUNTS_PER_OUTPUT_REV       (ENCODER_COUNTS_PER_MOTOR_REV * ENCODER_GEAR_RATIO)
#define ENCODER_SPEED_SAMPLE_MS             (10)
#define ENCODER_WHEEL_DIAMETER_MM           (65)
#define ENCODER_PI_X100                     (314)
#define ENCODER_SPEED_SCALE                 (1000 / ENCODER_SPEED_SAMPLE_MS)
#define ENCODER_LINEAR_SPEED_DEN            (ENCODER_COUNTS_PER_OUTPUT_REV * 100)
#define ENCODER_LINEAR_SPEED_NUM            (ENCODER_SPEED_SCALE * ENCODER_WHEEL_DIAMETER_MM * ENCODER_PI_X100)

void Encoder_Init(void);
void Encoder_SpeedTimerInit(void);
void Encoder_SpeedTimerIRQHandler(void);
int32_t Encoder_GetLeft(void);
int32_t Encoder_GetRight(void);
int32_t Encoder_GetLeftSpeed(void);
int32_t Encoder_GetRightSpeed(void);
int32_t Encoder_GetLeftLinearSpeed(void);
int32_t Encoder_GetRightLinearSpeed(void);
int32_t Encoder_GetLeftRPM(void);
int32_t Encoder_GetRightRPM(void);
void Encoder_Reset(void);

extern volatile int32_t EncoderLeftCnt;
extern volatile int32_t EncoderRightCnt;
extern volatile int32_t EncoderLeftSpeed;
extern volatile int32_t EncoderRightSpeed;
extern volatile int32_t EncoderLeftLinearSpeed;
extern volatile int32_t EncoderRightLinearSpeed;

#define EncoderCnt EncoderLeftCnt

#endif
