#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <stdint.h>

#define CONTROL_TASK_PERIOD_MS      (10U)
#define CONTROL_PRINT_PERIOD_MS     (50U)
#define CONTROL_OLED_PERIOD_MS      (100U)
#define CONTROL_KEY_DEBOUNCE_MS     (20U)
#define CONTROL_VOFA_BUFFER_SIZE    (64U)

typedef enum {
    CONTROL_START_NONE = 0,
    CONTROL_START_TASK1,
    CONTROL_START_TASK2,
    CONTROL_START_TASK3,
    CONTROL_START_TASK2_LOW_SPEED,
    CONTROL_START_TASK3_LOW_SPEED
} ControlStartSelection;

void Control_Init(void);
void Control_WaitForStartKey(uint32_t printPeriodMs, uint32_t debounceMs);
ControlStartSelection Control_WaitForStartSelection(uint32_t printPeriodMs,
    uint32_t debounceMs);
void Control_StartTask1(void);
void Control_StartTask2(void);
void Control_StartTask2LowSpeed(void);
void Control_StartTask3(uint8_t lapCount);
void Control_StartTask3LowSpeedOneLap(void);
void Control_Task10ms(void);
void Control_OLEDTask(void);
void Control_ServiceDebug(uint32_t printPeriodMs, uint32_t *printMs,
    uint32_t *oledMs);

#endif
