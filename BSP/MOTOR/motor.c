#include "motor.h"
#include "encoder.h"
#include "PID.h"
#include "ICM42688.h"
#include <stdio.h>

#define SPEED_FILTER_ALPHA 0.35f
#define SPEED_RAMP_STEP_DEFAULT_MM_S (10.0f)
#define STRAIGHT_YAW_KP 4.0f
#define STRAIGHT_YAW_CORRECTION_LIMIT_MM_S 80.0f

static volatile int32_t s_leftSpeed = 0;
static volatile int32_t s_rightSpeed = 0;
static volatile int32_t s_leftPwm = 0;
static volatile int32_t s_rightPwm = 0;
static volatile float s_leftRampRef = 0.0f;
static volatile float s_rightRampRef = 0.0f;
static volatile float s_rampStepMmS = SPEED_RAMP_STEP_DEFAULT_MM_S;
static volatile uint16_t s_runTicksRemaining = 0U;
static volatile uint8_t s_straightMode = 0U;
static volatile float s_straightTargetYawDeg = 0.0f;
static float s_leftSpeedFiltered = 0.0f;
static float s_rightSpeedFiltered = 0.0f;
static uint16_t s_speedLoopTick = 0U;

static void SpeedLoop_StopUnlocked(void)
{
    s_speedLoopTick = 0U;
    s_straightMode = 0U;
    s_straightTargetYawDeg = 0.0f;
    s_runTicksRemaining = 0U;
    motorA.ref = 0.0f;
    motorB.ref = 0.0f;
    s_leftRampRef = 0.0f;
    s_rightRampRef = 0.0f;
    s_leftSpeedFiltered = 0.0f;
    s_rightSpeedFiltered = 0.0f;
    s_leftSpeed = 0;
    s_rightSpeed = 0;
    PID_ResetState(&motorA);
    PID_ResetState(&motorB);
    Motor_SetSpeed(0, 0);
}

static uint32_t Motor_SpeedToCompare(int speed)
{
    if (speed < 0) {
        speed = -speed;
    }

    if (speed > MOTOR_PWM_PERIOD) {
        speed = MOTOR_PWM_PERIOD;
    }

    if (speed == 0) {
        return MOTOR_PWM_LOAD;
    }

    return (uint32_t) (MOTOR_PWM_PERIOD - speed);
}

static float SpeedLoop_RampToTarget(float current, float target)
{
    if (current < target) {
        current += s_rampStepMmS;
        if (current > target) {
            current = target;
        }
    } else if (current > target) {
        current -= s_rampStepMmS;
        if (current < target) {
            current = target;
        }
    }

    return current;
}

void Motor_Init(void)
{
    Motor_Stop();
    DL_TimerA_startCounter(PWM_Motor_INST);
}

void Motor_SetSpeed(int Motor1, int Motor2)
{
    if (Motor1 >= 0) {
        DL_GPIO_clearPins(Motor_PORT, Motor_AIN2_PIN);
        DL_GPIO_setPins(Motor_PORT, Motor_AIN1_PIN);
    } else {
        DL_GPIO_clearPins(Motor_PORT, Motor_AIN1_PIN);
        DL_GPIO_setPins(Motor_PORT, Motor_AIN2_PIN);
    }
    DL_TimerA_setCaptureCompareValue(PWM_Motor_INST,
        Motor_SpeedToCompare(Motor1), GPIO_PWM_Motor_C0_IDX);

    if (Motor2 >= 0) {
        DL_GPIO_clearPins(Motor_PORT, Motor_BIN1_PIN);
        DL_GPIO_setPins(Motor_PORT, Motor_BIN2_PIN);
    } else {
        DL_GPIO_clearPins(Motor_PORT, Motor_BIN2_PIN);
        DL_GPIO_setPins(Motor_PORT, Motor_BIN1_PIN);
    }
    DL_TimerA_setCaptureCompareValue(PWM_Motor_INST,
        Motor_SpeedToCompare(Motor2), GPIO_PWM_Motor_C1_IDX);
}

void Motor_Stop(void)
{
    DL_TimerA_setCaptureCompareValue(PWM_Motor_INST, MOTOR_PWM_LOAD, GPIO_PWM_Motor_C0_IDX);
    DL_TimerA_setCaptureCompareValue(PWM_Motor_INST, MOTOR_PWM_LOAD, GPIO_PWM_Motor_C1_IDX);

    DL_GPIO_clearPins(Motor_PORT, Motor_AIN1_PIN |
                                  Motor_AIN2_PIN |
                                  Motor_BIN1_PIN |
                                  Motor_BIN2_PIN);
}

void SpeedLoop_Start(void)
{
    __disable_irq();
    s_speedLoopTick = 1U;
    s_straightMode = 0U;
    s_straightTargetYawDeg = 0.0f;
    s_runTicksRemaining = 0U;
    motorA.ref = 0.0f;
    motorB.ref = 0.0f;
    s_leftRampRef = 0.0f;
    s_rightRampRef = 0.0f;
    s_leftSpeedFiltered = 0.0f;
    s_rightSpeedFiltered = 0.0f;
    s_leftSpeed = 0;
    s_rightSpeed = 0;
    PID_ResetState(&motorA);
    PID_ResetState(&motorB);
    __enable_irq();
    Encoder_SpeedTimerInit();
}

void SpeedLoop_StartStraight(uint32_t durationMs)
{
    __disable_irq();
    s_speedLoopTick = 1U;
    s_straightMode = 1U;
    s_straightTargetYawDeg = 0.0f;
    s_runTicksRemaining = (uint16_t) ((durationMs + ENCODER_SPEED_SAMPLE_MS - 1U) /
                                     ENCODER_SPEED_SAMPLE_MS);
    s_leftRampRef = 0.0f;
    s_rightRampRef = 0.0f;
    s_leftSpeedFiltered = 0.0f;
    s_rightSpeedFiltered = 0.0f;
    s_leftSpeed = 0;
    s_rightSpeed = 0;
    PID_ResetState(&motorA);
    PID_ResetState(&motorB);
    __enable_irq();
    Encoder_SpeedTimerInit();
}

void SpeedLoop_SetStraightMode(uint8_t enable)
{
    s_straightMode = (enable != 0U) ? 1U : 0U;
}

void SpeedLoop_SetRampStep(float rampStepMmS)
{
    if (rampStepMmS <= 0.0f) {
        rampStepMmS = SPEED_RAMP_STEP_DEFAULT_MM_S;
    }
    s_rampStepMmS = rampStepMmS;
}

void SpeedLoop_SetStraightTargetYaw(float yawDeg)
{
    s_straightTargetYawDeg = yawDeg;
}

void SpeedLoop_Stop(void)
{
    __disable_irq();
    SpeedLoop_StopUnlocked();
    s_rampStepMmS = SPEED_RAMP_STEP_DEFAULT_MM_S;
    __enable_irq();
}

void SpeedLoop_TimerIRQHandler(void)
{
    int32_t leftRawSpeed;
    int32_t rightRawSpeed;
    float leftTarget;
    float rightTarget;
    float yawCorrection;

    if (s_speedLoopTick == 0U) {
        return;
    }

    if (s_runTicksRemaining != 0U) {
        s_runTicksRemaining--;
        if (s_runTicksRemaining == 0U) {
            SpeedLoop_StopUnlocked();
            return;
        }
    }

    Encoder_SpeedTimerIRQHandler();
    leftRawSpeed = Encoder_GetLeftLinearSpeed();
    rightRawSpeed = Encoder_GetRightLinearSpeed();

    s_leftSpeedFiltered += SPEED_FILTER_ALPHA * ((float) leftRawSpeed - s_leftSpeedFiltered);
    s_rightSpeedFiltered += SPEED_FILTER_ALPHA * ((float) rightRawSpeed - s_rightSpeedFiltered);
    s_leftSpeed = (int32_t) s_leftSpeedFiltered;
    s_rightSpeed = (int32_t) s_rightSpeedFiltered;

    leftTarget = motorA.ref;
    rightTarget = motorB.ref;
    if (s_straightMode != 0U) {
        yawCorrection = STRAIGHT_YAW_KP *
            (ICM42688_YawGetDeg() - s_straightTargetYawDeg);
        if (yawCorrection > STRAIGHT_YAW_CORRECTION_LIMIT_MM_S) {
            yawCorrection = STRAIGHT_YAW_CORRECTION_LIMIT_MM_S;
        } else if (yawCorrection < -STRAIGHT_YAW_CORRECTION_LIMIT_MM_S) {
            yawCorrection = -STRAIGHT_YAW_CORRECTION_LIMIT_MM_S;
        }

        leftTarget = motorA.ref + yawCorrection;
        rightTarget = motorB.ref - yawCorrection;
    }

    s_leftRampRef = SpeedLoop_RampToTarget(s_leftRampRef, leftTarget);
    s_rightRampRef = SpeedLoop_RampToTarget(s_rightRampRef, rightTarget);

    s_leftPwm = (int32_t) PID_ctrl(&motorA, (float) s_leftSpeed, s_leftRampRef);
    s_rightPwm = (int32_t) PID_ctrl(&motorB, (float) s_rightSpeed, s_rightRampRef);
    Motor_SetSpeed(s_leftPwm, s_rightPwm);
}

void SpeedLoop_FormatVofaLine(char *buffer, uint16_t size)
{
    if ((buffer == NULL) || (size == 0U)) {
        return;
    }

    snprintf(buffer, size, "%.2f,%ld,%.2f,%ld\r\n",
        (double) s_leftRampRef, (long) s_leftSpeed,
        (double) s_rightRampRef, (long) s_rightSpeed);
}
