#include "ti_msp_dl_config.h"
#include "control.h"
#include "encoder.h"
#include "motor.h"
#include "PID.h"
#include "uart.h"
#include "gray_sensor.h"
#include "ICM42688.h"
#include "oled.h"
#include "delay.h"

int main(void)
{
    SYSCFG_DL_init();

    Encoder_Init();
    Motor_Init();
    pid_init();
    UART_Init();
    OLED_Init();
    Control_Init();
    (void) ICM42688_YawInit(ICM42688_YAW_DEFAULT_BIAS_DPS);

    Motor_SetSpeed(0, 0);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
    GraySensor_InitDefault();
    OLED_ShowString(0U, 0U, (uint8_t *) "GYRO CAL", 16U, 1U);
    OLED_Refresh();

    ICM42688_YawQuickCalibrate();
    Encoder_SpeedTimerInit();
    switch (Control_WaitForStartSelection(CONTROL_PRINT_PERIOD_MS,
        CONTROL_KEY_DEBOUNCE_MS)) {
    case CONTROL_START_TASK3_LOW_SPEED:
        Control_StartTask3LowSpeedOneLap();
        break;
    case CONTROL_START_TASK2_LOW_SPEED:
        Control_StartTask2LowSpeed();
        break;
    case CONTROL_START_TASK3:
        Control_StartTask3(0U);
        break;
    case CONTROL_START_TASK2:
        Control_StartTask2();
        break;
    case CONTROL_START_TASK1:
    default:
        Control_StartTask1();
        break;
    }

    while (1) {
        char vofaLine[CONTROL_VOFA_BUFFER_SIZE];
        static uint16_t printMs = 0U;
        static uint16_t oledMs = 0U;

        UART_Task();
        ICM42688_YawTask();
        Control_Task10ms();

        printMs = (uint16_t) (printMs + CONTROL_TASK_PERIOD_MS);
        oledMs = (uint16_t) (oledMs + CONTROL_TASK_PERIOD_MS);
        if (printMs >= CONTROL_PRINT_PERIOD_MS) {
            printMs = 0U;
            SpeedLoop_FormatVofaLine(vofaLine, sizeof(vofaLine));
            UART_SendString(vofaLine);
            ICM42688_YawSendLine();
        }
        if (oledMs >= CONTROL_OLED_PERIOD_MS) {
            oledMs = 0U;
            Control_OLEDTask();
        }

        delay_ms(CONTROL_TASK_PERIOD_MS);
    }
}

void TIMER_G0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_G0_INST)) {
    case DL_TIMER_IIDX_ZERO:
        SpeedLoop_TimerIRQHandler();
        ICM42688_YawOnTimer(ENCODER_SPEED_SAMPLE_MS);
        break;
    default:
        break;
    }
}

void ADC12_0_INST_IRQHandler(void)
{
    GraySensor_ADCIRQHandler();
}

void UART0_INST_IRQHandler(void)
{
    UART_IRQHandler();
}
