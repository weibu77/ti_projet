#include "encoder.h"

volatile int32_t EncoderLeftCnt = 0;
volatile int32_t EncoderRightCnt = 0;
volatile int32_t EncoderLeftSpeed = 0;
volatile int32_t EncoderRightSpeed = 0;
volatile int32_t EncoderLeftLinearSpeed = 0;
volatile int32_t EncoderRightLinearSpeed = 0;

static int32_t s_leftLastCnt = 0;
static int32_t s_rightLastCnt = 0;

static int32_t Encoder_CountsToLinearSpeed(int32_t speed)
{
    return (int32_t)(((int64_t) speed * ENCODER_LINEAR_SPEED_NUM) /
                     ENCODER_LINEAR_SPEED_DEN);
}

void Encoder_Init(void)
{
    const uint32_t encoderPins = Encoder_Pin_Left_A_PIN |
                                 Encoder_Pin_Right_A_PIN;

    DL_GPIO_clearInterruptStatus(Encoder_PORT, encoderPins);
    DL_GPIO_enableInterrupt(Encoder_PORT, encoderPins);

    NVIC_EnableIRQ(Encoder_INT_IRQN);
}

void Encoder_SpeedTimerInit(void)
{
    s_leftLastCnt = EncoderLeftCnt;
    s_rightLastCnt = EncoderRightCnt;

    DL_TimerG_clearInterruptStatus(TIMER_G0_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    NVIC_EnableIRQ(TIMER_G0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_G0_INST);
}

int32_t Encoder_GetLeft(void)
{
    return EncoderLeftCnt;
}

int32_t Encoder_GetRight(void)
{
    return EncoderRightCnt;
}

int32_t Encoder_GetLeftSpeed(void)
{
    return EncoderLeftSpeed;
}

int32_t Encoder_GetRightSpeed(void)
{
    return EncoderRightSpeed;
}

int32_t Encoder_GetLeftLinearSpeed(void)
{
    return EncoderLeftLinearSpeed;
}

int32_t Encoder_GetRightLinearSpeed(void)
{
    return EncoderRightLinearSpeed;
}

int32_t Encoder_GetLeftRPM(void)
{
    return Encoder_GetLeftLinearSpeed();
}

int32_t Encoder_GetRightRPM(void)
{
    return Encoder_GetRightLinearSpeed();
}

void Encoder_Reset(void)
{
    __disable_irq();
    EncoderLeftCnt = 0;
    EncoderRightCnt = 0;
    EncoderLeftSpeed = 0;
    EncoderRightSpeed = 0;
    EncoderLeftLinearSpeed = 0;
    EncoderRightLinearSpeed = 0;
    s_leftLastCnt = 0;
    s_rightLastCnt = 0;
    __enable_irq();
}

void Encoder_SpeedTimerIRQHandler(void)
{
    int32_t leftCnt;
    int32_t rightCnt;

    leftCnt = EncoderLeftCnt;
    rightCnt = EncoderRightCnt;

    EncoderLeftSpeed = leftCnt - s_leftLastCnt;
    EncoderRightSpeed = rightCnt - s_rightLastCnt;
    s_leftLastCnt = leftCnt;
    s_rightLastCnt = rightCnt;

    EncoderLeftLinearSpeed = Encoder_CountsToLinearSpeed(EncoderLeftSpeed);
    EncoderRightLinearSpeed = Encoder_CountsToLinearSpeed(EncoderRightSpeed);
}

void GROUP1_IRQHandler(void)
{
    const uint32_t encoderPins = Encoder_Pin_Left_A_PIN | Encoder_Pin_Right_A_PIN;
    uint32_t gpioA = DL_GPIO_getEnabledInterruptStatus(Encoder_PORT,
        encoderPins);
    uint32_t pins = DL_GPIO_readPins(Encoder_PORT,
        Encoder_Pin_Left_B_PIN | Encoder_Pin_Right_B_PIN);

    if ((gpioA & Encoder_Pin_Left_A_PIN) != 0U) {
        if ((pins & Encoder_Pin_Left_B_PIN) != 0U) {
            EncoderLeftCnt--;
        } else {
            EncoderLeftCnt++;
        }
    }

    if ((gpioA & Encoder_Pin_Right_A_PIN) != 0U) {
        if ((pins & Encoder_Pin_Right_B_PIN) != 0U) {
            EncoderRightCnt++;
        } else {
            EncoderRightCnt--;
        }
    }

    DL_GPIO_clearInterruptStatus(Encoder_PORT, gpioA);
}
