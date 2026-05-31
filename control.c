#include "control.h"
#include "ti_msp_dl_config.h"
#include "encoder.h"
#include "motor.h"
#include "PID.h"
#include "delay.h"
#include "uart.h"
#include "gray_sensor.h"
#include "ICM42688.h"
#include "key.h"
#include "oled.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define CONTROL_DEFAULT_BASE_SPEED_MM_S    (220.0f)
#define CONTROL_LOW_BASE_SPEED_MM_S        (150.0f)
#define CONTROL_DEFAULT_RAMP_STEP_MM_S     (10.0f)
#define CONTROL_LOW_RAMP_STEP_MM_S         (5.0f)
#define CONTROL_TASK2_STRAIGHT_TARGET_YAW  (-180.0f)
#define CONTROL_TASK3_AC_FIRST_YAW         (-44.0f)
#define CONTROL_TASK3_AC_SECOND_YAW        (-44.0f)
#define CONTROL_TASK3_AC_THIRD_YAW         (-44.0f)
#define CONTROL_TASK3_AC_ENTRY_YAW         (1.5f)
#define CONTROL_TASK3_BD_FIRST_YAW         (225.0f)
#define CONTROL_TASK3_BD_SECOND_YAW        (225.0f)
#define CONTROL_TASK3_BD_THIRD_YAW         (225.0f)
#define CONTROL_TASK3_BD_ENTRY_YAW         (178.0f)
#define CONTROL_TASK3_AC_DISTANCE_MM       (1050U)
#define CONTROL_TASK3_BD_DISTANCE_MM       (1100U)
#define CONTROL_TASK3_SEEK_SPEED_MM_S      (120.0f)
#define CONTROL_TASK3_TURN_KP              (8.0f)
#define CONTROL_TASK3_TURN_MIN_SPEED_MM_S  (25.0f)
#define CONTROL_TASK3_TURN_SPEED_LIMIT     (40.0f)
#define CONTROL_TASK3_YAW_DONE_DEG         (1.5f)
#define CONTROL_TASK3_YAW_STABLE_TICKS     (8U)
#define CONTROL_TASK3_MIN_LAPS             (1U)
#define CONTROL_TASK3_MAX_LAPS             (4U)
#define CONTROL_LEFT_SPEED_SCALE           (0.987f)
#define CONTROL_STOP_B_MS                  (3000U)
#define CONTROL_STOP_C_ALERT_MS            (1000U)
#define CONTROL_POINT_ALERT_MS             (300U)
#define CONTROL_TRACK_KP                   (0.45f)
#define CONTROL_TRACK_KD                   (0.10f)
#define CONTROL_TRACK_CORRECTION_LIMIT     (120.0f)
#define CONTROL_TRACK_BLACK_SUM_MIN        (260U)
#define CONTROL_TRACK_CORRECTION_DECAY     (0.55f)
#define CONTROL_BLACK_STABLE_TICKS         (3U)
#define CONTROL_WHITE_STABLE_TICKS         (10U)
#define CONTROL_CENTER_LEFT_BIT            (3U)
#define CONTROL_CENTER_RIGHT_BIT           (4U)
#define CONTROL_CENTER_STABLE_TICKS        (3U)
#define CONTROL_CENTER_TURN_KP             (0.12f)
#define CONTROL_CENTER_TURN_MIN_SPEED_MM_S (18.0f)
#define CONTROL_CENTER_TURN_SPEED_LIMIT    (35.0f)

typedef enum {
    CONTROL_STATE_IDLE = 0,
    CONTROL_STATE_TASK1_AB,
    CONTROL_STATE_TASK1_STOP_B,
    CONTROL_STATE_TASK1_TRACK_TO_C,
    CONTROL_STATE_TASK1_STOP_C,
    CONTROL_STATE_TASK2_AB,
    CONTROL_STATE_TASK2_CENTER_TO_C,
    CONTROL_STATE_TASK2_TRACK_TO_C,
    CONTROL_STATE_TASK2_STRAIGHT_TO_D,
    CONTROL_STATE_TASK2_CENTER_TO_A,
    CONTROL_STATE_TASK2_TRACK_TO_A,
    CONTROL_STATE_TASK2_STOP_A,
    CONTROL_STATE_TASK3_TURN_TO_C,
    CONTROL_STATE_TASK3_DRIVE_TO_C,
    CONTROL_STATE_TASK3_ALIGN_C,
    CONTROL_STATE_TASK3_SEEK_C,
    CONTROL_STATE_TASK3_CENTER_TO_B,
    CONTROL_STATE_TASK3_TRACK_TO_B,
    CONTROL_STATE_TASK3_TURN_TO_D,
    CONTROL_STATE_TASK3_DRIVE_TO_D,
    CONTROL_STATE_TASK3_ALIGN_D,
    CONTROL_STATE_TASK3_SEEK_D,
    CONTROL_STATE_TASK3_CENTER_TO_A,
    CONTROL_STATE_TASK3_TRACK_TO_A,
    CONTROL_STATE_TASK3_STOP_A,
    CONTROL_STATE_DONE
} ControlState;

typedef struct {
    float error;
    float lastError;
    float correction;
} TrackState;

typedef enum {
    CONTROL_GRAY_CAL_IDLE = 0,
    CONTROL_GRAY_CAL_WAIT_WHITE,
    CONTROL_GRAY_CAL_WAIT_BLACK,
    CONTROL_GRAY_CAL_READY_SAVE,
    CONTROL_GRAY_CAL_SAVE_ERROR,
    CONTROL_GRAY_CAL_DONE
} ControlGrayCalibrationState;

static ControlState g_controlState = CONTROL_STATE_IDLE;
static TrackState g_trackState;
static float g_baseSpeedMmS = CONTROL_DEFAULT_BASE_SPEED_MM_S;
static uint32_t g_stateElapsedMs = 0U;
static uint8_t g_blackStableTicks = 0U;
static uint8_t g_whiteStableTicks = 0U;
static float g_cPointYawDeg = 0.0f;
static uint8_t g_cPointYawValid = 0U;
static uint32_t g_alertRemainingMs = 0U;
static uint8_t g_lowSpeedTaskActive = 0U;
static uint8_t g_centerStableTicks = 0U;
static int8_t g_centerLastTurnSign = 0;
static uint8_t g_task3LapTarget = CONTROL_TASK3_MIN_LAPS;
static uint8_t g_task3LapCurrent = 0U;
static uint8_t g_task3YawStableTicks = 0U;
static ControlGrayCalibrationState g_grayCalState =
    CONTROL_GRAY_CAL_IDLE;
static uint16_t g_grayCalWhite[GRAY_SENSOR_CHANNELS];
static uint16_t g_grayCalBlack[GRAY_SENSOR_CHANNELS];

static const char *Control_GrayCalStatusText(void)
{
    switch (g_grayCalState) {
    case CONTROL_GRAY_CAL_IDLE:
        return "C:-";
    case CONTROL_GRAY_CAL_WAIT_WHITE:
        return "C:W";
    case CONTROL_GRAY_CAL_WAIT_BLACK:
        return "C:B";
    case CONTROL_GRAY_CAL_READY_SAVE:
        return "C:S";
    case CONTROL_GRAY_CAL_SAVE_ERROR:
        return "C:E";
    case CONTROL_GRAY_CAL_DONE:
    default:
        return "C:D";
    }
}

static void Control_AlertSet(uint8_t on)
{
#if defined(Alert_PORT) && defined(Alert_LED_PIN) && defined(Alert_BUZZER_PIN)
    if (on != 0U) {
        DL_GPIO_clearPins(Alert_PORT, Alert_LED_PIN);
        DL_GPIO_setPins(Alert_PORT, Alert_BUZZER_PIN);
    } else {
        DL_GPIO_setPins(Alert_PORT, Alert_LED_PIN);
        DL_GPIO_clearPins(Alert_PORT, Alert_BUZZER_PIN);
    }
#else
    (void) on;
#endif
}

static void Control_AlertPulse(uint32_t durationMs)
{
    g_alertRemainingMs = durationMs;
    Control_AlertSet(1U);
}

static void Control_AlertTask(void)
{
    if (g_alertRemainingMs == 0U) {
        return;
    }

    if (g_alertRemainingMs <= CONTROL_TASK_PERIOD_MS) {
        g_alertRemainingMs = 0U;
        Control_AlertSet(0U);
    } else {
        g_alertRemainingMs -= CONTROL_TASK_PERIOD_MS;
    }
}

static void Control_ShowSignedFixed3(uint8_t x, uint8_t y, float value)
{
    int32_t scaled;
    uint32_t absValue;

    if (value >= 0.0f) {
        scaled = (int32_t) ((value * 1000.0f) + 0.5f);
        OLED_ShowString(x, y, (uint8_t *) "+", 8U, 1U);
    } else {
        scaled = (int32_t) ((value * 1000.0f) - 0.5f);
        OLED_ShowString(x, y, (uint8_t *) "-", 8U, 1U);
    }

    absValue = (scaled < 0) ? (uint32_t) (-scaled) : (uint32_t) scaled;
    OLED_ShowNum((uint8_t) (x + 6U), y, absValue / 1000U, 4U, 8U, 1U);
    OLED_ShowString((uint8_t) (x + 30U), y, (uint8_t *) ".", 8U, 1U);
    OLED_ShowNum((uint8_t) (x + 36U), y, absValue % 1000U, 3U, 8U, 1U);
}

static void Control_OLEDShowGrayCalibration(void)
{
    OLED_Clear();
    OLED_ShowString(0U, 0U, (uint8_t *) "GRAY CAL", 16U, 1U);

    switch (g_grayCalState) {
    case CONTROL_GRAY_CAL_WAIT_WHITE:
        OLED_ShowString(0U, 20U, (uint8_t *) "MODE:ON", 8U, 1U);
        OLED_ShowString(0U, 32U, (uint8_t *) "KEY4 WHITE", 8U, 1U);
        OLED_ShowString(0U, 44U, (uint8_t *) "W:-- B:--", 8U, 1U);
        break;
    case CONTROL_GRAY_CAL_WAIT_BLACK:
        OLED_ShowString(0U, 20U, (uint8_t *) "WHITE OK", 8U, 1U);
        OLED_ShowString(0U, 32U, (uint8_t *) "KEY4 BLACK", 8U, 1U);
        OLED_ShowString(0U, 44U, (uint8_t *) "W:OK B:--", 8U, 1U);
        break;
    case CONTROL_GRAY_CAL_READY_SAVE:
        OLED_ShowString(0U, 20U, (uint8_t *) "BLACK OK", 8U, 1U);
        OLED_ShowString(0U, 32U, (uint8_t *) "KEY4 SAVE", 8U, 1U);
        OLED_ShowString(0U, 44U, (uint8_t *) "W:OK B:OK", 8U, 1U);
        break;
    case CONTROL_GRAY_CAL_SAVE_ERROR:
        OLED_ShowString(0U, 20U, (uint8_t *) "SAVE ERR", 8U, 1U);
        OLED_ShowString(0U, 32U, (uint8_t *) "KEY4 RETRY", 8U, 1U);
        OLED_ShowString(0U, 44U, (uint8_t *) "CHECK FLASH", 8U, 1U);
        break;
    case CONTROL_GRAY_CAL_DONE:
        OLED_ShowString(0U, 20U, (uint8_t *) "SAVE OK", 8U, 1U);
        OLED_ShowString(0U, 32U, (uint8_t *) "THR UPDATED", 8U, 1U);
        OLED_ShowString(0U, 44U, (uint8_t *) "SELECT TASK", 8U, 1U);
        break;
    case CONTROL_GRAY_CAL_IDLE:
    default:
        OLED_ShowString(0U, 20U, (uint8_t *) "READY", 8U, 1U);
        break;
    }

    OLED_Refresh();
}

static void Control_OLEDShowDebug(void)
{
    uint8_t digital = GraySensor_GetDigital(&g_gray_sensor);

    if ((g_controlState == CONTROL_STATE_IDLE) &&
        (g_grayCalState != CONTROL_GRAY_CAL_IDLE)) {
        Control_OLEDShowGrayCalibration();
        return;
    }

    OLED_Clear();
    OLED_ShowString(0U, 0U, (uint8_t *) "Y:", 8U, 1U);
    Control_ShowSignedFixed3(12U, 0U, ICM42688_YawGetDeg());

    OLED_ShowString(0U, 12U, (uint8_t *) "CY:", 8U, 1U);
    if (g_cPointYawValid != 0U) {
        Control_ShowSignedFixed3(18U, 12U, g_cPointYawDeg);
    } else {
        OLED_ShowString(18U, 12U, (uint8_t *) "--.---", 8U, 1U);
    }

    OLED_ShowString(0U, 24U, (uint8_t *) "S:", 8U, 1U);
    OLED_ShowNum(12U, 24U, (uint32_t) g_controlState, 2U, 8U, 1U);
    OLED_ShowString(36U, 24U,
        (uint8_t *) (GraySensor_GetLastError() == 0U ? "G:OK" : "G:ER"),
        8U, 1U);
    OLED_ShowString(72U, 24U, (uint8_t *) "N:", 8U, 1U);
    OLED_ShowNum(84U, 24U, (uint32_t) g_task3LapTarget, 1U, 8U, 1U);
    OLED_ShowString(102U, 24U,
        (uint8_t *) Control_GrayCalStatusText(), 8U, 1U);

    OLED_ShowString(0U, 36U, (uint8_t *) "D:", 8U, 1U);
    for (uint8_t i = 0U; i < GRAY_SENSOR_CHANNELS; i++) {
        OLED_ShowNum((uint8_t) (12U + i * 7U), 36U,
            (digital >> i) & 0x01U, 1U, 8U, 1U);
    }

    OLED_Refresh();
}

static void Control_UpdateGrayOled(void)
{
    (void) GraySensor_Task(&g_gray_sensor);
    Control_OLEDShowDebug();
}

void Control_OLEDTask(void)
{
    Control_UpdateGrayOled();
}

static void Control_SetStraightSpeed(float speed)
{
    motorA.ref = speed * CONTROL_LEFT_SPEED_SCALE;
    motorB.ref = speed;
}

static void Control_SetTaskBaseSpeed(float baseSpeedMmS)
{
    g_baseSpeedMmS = baseSpeedMmS;
}

static void Control_ApplyTaskRampStep(void)
{
    if (g_lowSpeedTaskActive != 0U) {
        SpeedLoop_SetRampStep(CONTROL_LOW_RAMP_STEP_MM_S);
    } else {
        SpeedLoop_SetRampStep(CONTROL_DEFAULT_RAMP_STEP_MM_S);
    }
}

static void Control_StartStraight(float targetYawDeg)
{
    Control_ApplyTaskRampStep();
    SpeedLoop_Start();
    SpeedLoop_SetStraightTargetYaw(targetYawDeg);
    SpeedLoop_SetStraightMode(1U);
    Control_SetStraightSpeed(g_baseSpeedMmS);
}

static int32_t Control_Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static float Control_Task3AcTargetYaw(void)
{
    if (g_task3LapCurrent == 0U) {
        return CONTROL_TASK3_AC_FIRST_YAW;
    }
    if (g_task3LapCurrent == 1U) {
        return CONTROL_TASK3_AC_SECOND_YAW;
    }
    return CONTROL_TASK3_AC_THIRD_YAW;
}

static float Control_Task3BdTargetYaw(void)
{
    if (g_task3LapCurrent == 0U) {
        return CONTROL_TASK3_BD_FIRST_YAW;
    }
    if (g_task3LapCurrent == 1U) {
        return CONTROL_TASK3_BD_SECOND_YAW;
    }
    return CONTROL_TASK3_BD_THIRD_YAW;
}

static uint32_t Control_Task3DistanceTargetCounts(uint32_t distanceMm)
{
    return (uint32_t) (((uint64_t) distanceMm *
        (uint64_t) ENCODER_LINEAR_SPEED_DEN) /
        ((uint64_t) ENCODER_WHEEL_DIAMETER_MM *
        (uint64_t) ENCODER_PI_X100));
}

static uint32_t Control_AverageEncoderAbsCounts(void)
{
    return (uint32_t) ((Control_Abs32(Encoder_GetLeft()) +
        Control_Abs32(Encoder_GetRight())) / 2);
}

static float Control_LimitFloat(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static void Control_TrackReset(void)
{
    g_trackState.error = 0.0f;
    g_trackState.lastError = 0.0f;
    g_trackState.correction = 0.0f;
}

static uint8_t Control_ReadGrayLine(uint8_t *digital, uint32_t *blackSum,
    int32_t *weightedSum)
{
    static const int16_t weights[GRAY_SENSOR_CHANNELS] = {
        -300, -200, -150, -30, 30, 150, 200, 300
    };

    *blackSum = 0U;
    *weightedSum = 0;

    if (GraySensor_Task(&g_gray_sensor) == 0U) {
        *digital = GraySensor_GetDigital(&g_gray_sensor);
        return 0U;
    }

    *digital = GraySensor_GetDigital(&g_gray_sensor);
    for (uint8_t i = 0U; i < GRAY_SENSOR_CHANNELS; i++) {
        uint16_t blackness = (g_gray_sensor.normalized[i] < g_gray_sensor.adc_max) ?
            (uint16_t) (g_gray_sensor.adc_max - g_gray_sensor.normalized[i]) : 0U;

        *blackSum += blackness;
        *weightedSum += (int32_t) blackness * weights[i];
    }

    return (uint8_t) (GraySensor_GetLastError() == 0U);
}

static uint8_t Control_IsBlackDetected(uint8_t digital, uint32_t blackSum,
    uint8_t grayOk)
{
    return (uint8_t) ((grayOk != 0U) && (digital != 0xFFU) &&
        (blackSum >= CONTROL_TRACK_BLACK_SUM_MIN));
}

static uint8_t Control_IsFullWhite(uint8_t digital, uint32_t blackSum,
    uint8_t grayOk)
{
    (void) blackSum;
    return (uint8_t) ((grayOk != 0U) && (digital == 0xFFU));
}

static void Control_UpdateLineTrack(uint8_t digital, uint32_t blackSum,
    int32_t weightedSum, uint8_t grayOk)
{
    float correction;
    float baseSpeed = g_baseSpeedMmS;

    if (Control_IsBlackDetected(digital, blackSum, grayOk) == 0U) {
        g_trackState.error = 0.0f;
        g_trackState.lastError = 0.0f;
        correction = g_trackState.correction * CONTROL_TRACK_CORRECTION_DECAY;
        if ((correction < 3.0f) && (correction > -3.0f)) {
            correction = 0.0f;
        }
    } else {
        g_trackState.error = (float) weightedSum / (float) blackSum;
        correction = CONTROL_TRACK_KP * g_trackState.error +
            CONTROL_TRACK_KD * (g_trackState.error - g_trackState.lastError);
        correction = Control_LimitFloat(correction,
            CONTROL_TRACK_CORRECTION_LIMIT);
        g_trackState.lastError = g_trackState.error;
    }

    g_trackState.correction = correction;
    motorA.ref = baseSpeed * CONTROL_LEFT_SPEED_SCALE + correction;
    motorB.ref = baseSpeed - correction;
}

static void Control_EnterState(ControlState state)
{
    g_controlState = state;
    g_stateElapsedMs = 0U;
}

static void Control_ResetStableCounters(void)
{
    g_blackStableTicks = 0U;
    g_whiteStableTicks = 0U;
    g_task3YawStableTicks = 0U;
    g_centerStableTicks = 0U;
    g_centerLastTurnSign = 0;
}

static uint8_t Control_CenterBitsOnBlack(uint8_t digital, uint8_t grayOk)
{
    uint8_t centerMask = (uint8_t) ((1U << CONTROL_CENTER_LEFT_BIT) |
        (1U << CONTROL_CENTER_RIGHT_BIT));

    return (uint8_t) ((grayOk != 0U) && ((digital & centerMask) == 0U));
}

static uint8_t Control_UpdateLineCenter(uint8_t digital, uint32_t blackSum,
    int32_t weightedSum, uint8_t grayOk)
{
    float turnSpeed = 0.0f;

    SpeedLoop_SetStraightMode(0U);

    if (Control_CenterBitsOnBlack(digital, grayOk) != 0U) {
        Control_SetStraightSpeed(0.0f);
        if (g_centerStableTicks < CONTROL_CENTER_STABLE_TICKS) {
            g_centerStableTicks++;
        }
        return (uint8_t) (g_centerStableTicks >= CONTROL_CENTER_STABLE_TICKS);
    }

    g_centerStableTicks = 0U;
    if (Control_IsBlackDetected(digital, blackSum, grayOk) != 0U) {
        float error = (float) weightedSum / (float) blackSum;
        turnSpeed = Control_LimitFloat(error * CONTROL_CENTER_TURN_KP,
            CONTROL_CENTER_TURN_SPEED_LIMIT);
        if ((turnSpeed > 0.0f) &&
            (turnSpeed < CONTROL_CENTER_TURN_MIN_SPEED_MM_S)) {
            turnSpeed = CONTROL_CENTER_TURN_MIN_SPEED_MM_S;
        } else if ((turnSpeed < 0.0f) &&
            (turnSpeed > -CONTROL_CENTER_TURN_MIN_SPEED_MM_S)) {
            turnSpeed = -CONTROL_CENTER_TURN_MIN_SPEED_MM_S;
        }
        g_centerLastTurnSign = (turnSpeed >= 0.0f) ? 1 : -1;
    } else if (g_centerLastTurnSign > 0) {
        turnSpeed = CONTROL_CENTER_TURN_MIN_SPEED_MM_S;
    } else if (g_centerLastTurnSign < 0) {
        turnSpeed = -CONTROL_CENTER_TURN_MIN_SPEED_MM_S;
    }

    motorA.ref = turnSpeed;
    motorB.ref = -turnSpeed;
    return 0U;
}

static uint8_t Control_UpdateYawTurn(float targetYawDeg)
{
    float error = ICM42688_YawGetDeg() - targetYawDeg;
    float turnSpeed;

    if ((error <= CONTROL_TASK3_YAW_DONE_DEG) &&
        (error >= -CONTROL_TASK3_YAW_DONE_DEG)) {
        Control_SetStraightSpeed(0.0f);
        if (g_task3YawStableTicks < CONTROL_TASK3_YAW_STABLE_TICKS) {
            g_task3YawStableTicks++;
        }
        return (uint8_t)
            (g_task3YawStableTicks >= CONTROL_TASK3_YAW_STABLE_TICKS);
    }

    g_task3YawStableTicks = 0U;
    turnSpeed = Control_LimitFloat(error * CONTROL_TASK3_TURN_KP,
        CONTROL_TASK3_TURN_SPEED_LIMIT);
    if ((turnSpeed > 0.0f) &&
        (turnSpeed < CONTROL_TASK3_TURN_MIN_SPEED_MM_S)) {
        turnSpeed = CONTROL_TASK3_TURN_MIN_SPEED_MM_S;
    } else if ((turnSpeed < 0.0f) &&
        (turnSpeed > -CONTROL_TASK3_TURN_MIN_SPEED_MM_S)) {
        turnSpeed = -CONTROL_TASK3_TURN_MIN_SPEED_MM_S;
    }

    motorA.ref = turnSpeed;
    motorB.ref = -turnSpeed;
    return 0U;
}

static void Control_StartTask3Drive(float targetYawDeg)
{
    Encoder_Reset();
    Control_ApplyTaskRampStep();
    SpeedLoop_Start();
    SpeedLoop_SetStraightTargetYaw(targetYawDeg);
    SpeedLoop_SetStraightMode(1U);
    Control_SetStraightSpeed(g_baseSpeedMmS);
    Control_ResetStableCounters();
}

static void Control_StartTask3Turn(float targetYawDeg)
{
    (void) targetYawDeg;
    Encoder_Reset();
    Control_ApplyTaskRampStep();
    SpeedLoop_Start();
    SpeedLoop_SetStraightMode(0U);
    Control_SetStraightSpeed(0.0f);
    Control_ResetStableCounters();
}

void Control_Init(void)
{
    Key_EnablePullUps();
    g_controlState = CONTROL_STATE_IDLE;
    g_baseSpeedMmS = CONTROL_DEFAULT_BASE_SPEED_MM_S;
    g_stateElapsedMs = 0U;
    g_blackStableTicks = 0U;
    g_whiteStableTicks = 0U;
    g_cPointYawDeg = 0.0f;
    g_cPointYawValid = 0U;
    g_alertRemainingMs = 0U;
    g_lowSpeedTaskActive = 0U;
    g_centerStableTicks = 0U;
    g_centerLastTurnSign = 0;
    g_task3LapTarget = CONTROL_TASK3_MIN_LAPS;
    g_task3LapCurrent = 0U;
    g_task3YawStableTicks = 0U;
    g_grayCalState = CONTROL_GRAY_CAL_IDLE;
    memset(g_grayCalWhite, 0, sizeof(g_grayCalWhite));
    memset(g_grayCalBlack, 0, sizeof(g_grayCalBlack));
    Control_TrackReset();
    Control_AlertSet(0U);
}

void Control_ServiceDebug(uint32_t printPeriodMs, uint32_t *printMs,
    uint32_t *oledMs)
{
    UART_Task();
    ICM42688_YawTask();

    if (*printMs >= printPeriodMs) {
        *printMs = 0U;
        ICM42688_YawSendLine();
    }

    if (*oledMs >= CONTROL_OLED_PERIOD_MS) {
        *oledMs = 0U;
        Control_UpdateGrayOled();
    }
}

static uint8_t Control_OnlyGrayCalKeyPressed(void)
{
    return (uint8_t) ((Key_Key4IsPressed() != 0U) &&
        (Key_UserIsPressed() == 0U) &&
        (Key_Key1IsPressed() == 0U) &&
        (Key_Key2IsPressed() == 0U) &&
        (Key_Key3IsPressed() == 0U) &&
        (Key_Key5IsPressed() == 0U));
}

static uint8_t Control_OnlyLowSpeedModeKeyPressed(void)
{
    return (uint8_t) ((Key_Key5IsPressed() != 0U) &&
        (Key_UserIsPressed() == 0U) &&
        (Key_Key1IsPressed() == 0U) &&
        (Key_Key2IsPressed() == 0U) &&
        (Key_Key3IsPressed() == 0U) &&
        (Key_Key4IsPressed() == 0U));
}

static uint8_t Control_CaptureGrayCalibration(uint16_t *dest)
{
    if (GraySensor_Task(&g_gray_sensor) == 0U) {
        return 0U;
    }

    memcpy(dest, g_gray_sensor.analog, sizeof(g_gray_sensor.analog));
    return 1U;
}

static void Control_SendGrayCalibrationLine(const char *name,
    const uint16_t *value)
{
    char line[128];

    snprintf(line, sizeof(line),
        "GRAY_CAL,%s,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
        name,
        (unsigned int) value[0], (unsigned int) value[1],
        (unsigned int) value[2], (unsigned int) value[3],
        (unsigned int) value[4], (unsigned int) value[5],
        (unsigned int) value[6], (unsigned int) value[7]);
    UART_SendString(line);
}

static void Control_WaitForAllKeysReleased(uint32_t printPeriodMs,
    uint32_t debounceMs, uint32_t *printMs, uint32_t *oledMs)
{
    uint32_t stableMs = 0U;

    while (stableMs < debounceMs) {
        Control_ServiceDebug(printPeriodMs, printMs, oledMs);

        if (Key_AllReleased() != 0U) {
            stableMs++;
        } else {
            stableMs = 0U;
        }

        delay_ms(1U);
        (*printMs)++;
        (*oledMs)++;
    }
}

static uint8_t Control_HandleGrayCalibrationKey(uint32_t printPeriodMs,
    uint32_t debounceMs, uint32_t *printMs, uint32_t *oledMs)
{
    uint32_t stableMs = 0U;
    uint8_t actionOk = 1U;

    if ((g_grayCalState == CONTROL_GRAY_CAL_DONE) ||
        (Control_OnlyGrayCalKeyPressed() == 0U)) {
        return 0U;
    }

    while (stableMs < debounceMs) {
        Control_ServiceDebug(printPeriodMs, printMs, oledMs);

        if (Control_OnlyGrayCalKeyPressed() != 0U) {
            stableMs++;
        } else {
            return 0U;
        }

        delay_ms(1U);
        (*printMs)++;
        (*oledMs)++;
    }

    switch (g_grayCalState) {
    case CONTROL_GRAY_CAL_IDLE:
        g_grayCalState = CONTROL_GRAY_CAL_WAIT_WHITE;
        UART_SendString("GRAY_CAL,MODE\r\n");
        UART_SendString("GRAY_CAL,NEXT,WHITE\r\n");
        break;
    case CONTROL_GRAY_CAL_WAIT_WHITE:
        actionOk = Control_CaptureGrayCalibration(g_grayCalWhite);
        if (actionOk != 0U) {
            g_grayCalState = CONTROL_GRAY_CAL_WAIT_BLACK;
            Control_SendGrayCalibrationLine("WHITE", g_grayCalWhite);
            UART_SendString("GRAY_CAL,WHITE_OK\r\n");
        }
        break;
    case CONTROL_GRAY_CAL_WAIT_BLACK:
        actionOk = Control_CaptureGrayCalibration(g_grayCalBlack);
        if (actionOk != 0U) {
            Control_SendGrayCalibrationLine("BLACK", g_grayCalBlack);
            g_grayCalState = CONTROL_GRAY_CAL_READY_SAVE;
            UART_SendString("GRAY_CAL,BLACK_OK\r\n");
        }
        break;
    case CONTROL_GRAY_CAL_READY_SAVE:
    case CONTROL_GRAY_CAL_SAVE_ERROR:
        actionOk = GraySensor_SaveCalibration(g_grayCalWhite, g_grayCalBlack);
        if (actionOk != 0U) {
            GraySensor_Init(&g_gray_sensor, g_grayCalWhite, g_grayCalBlack);
            g_grayCalState = CONTROL_GRAY_CAL_DONE;
            UART_SendString("GRAY_CAL,SAVED\r\n");
            UART_SendString("GRAY_CAL,DONE\r\n");
        } else {
            g_grayCalState = CONTROL_GRAY_CAL_SAVE_ERROR;
            UART_SendError("GRAY_CAL_SAVE");
        }
        break;
    case CONTROL_GRAY_CAL_DONE:
    default:
        break;
    }

    if ((actionOk == 0U) &&
        ((g_grayCalState == CONTROL_GRAY_CAL_WAIT_WHITE) ||
        (g_grayCalState == CONTROL_GRAY_CAL_WAIT_BLACK))) {
        UART_SendError("GRAY_CAL_ADC");
    }

    Control_UpdateGrayOled();
    Control_WaitForAllKeysReleased(printPeriodMs, debounceMs, printMs, oledMs);

    return 1U;
}

static uint8_t Control_GrayCalibrationBlocksStart(void)
{
    return (uint8_t) ((g_grayCalState != CONTROL_GRAY_CAL_IDLE) &&
        (g_grayCalState != CONTROL_GRAY_CAL_DONE));
}

static ControlStartSelection Control_WaitForLowSpeedSelection(
    uint32_t printPeriodMs, uint32_t debounceMs, uint32_t *printMs,
    uint32_t *oledMs)
{
    uint32_t stableMs = 0U;
    ControlStartSelection selection = CONTROL_START_NONE;

    UART_SendString("LOW_SPEED,MODE\r\n");
    Control_WaitForAllKeysReleased(printPeriodMs, debounceMs, printMs,
        oledMs);

    while (stableMs < debounceMs) {
        Control_ServiceDebug(printPeriodMs, printMs, oledMs);

        if (Control_HandleGrayCalibrationKey(printPeriodMs, debounceMs,
                printMs, oledMs) != 0U) {
            selection = CONTROL_START_NONE;
            stableMs = 0U;
            continue;
        }

        if (Control_GrayCalibrationBlocksStart() != 0U) {
            selection = CONTROL_START_NONE;
            stableMs = 0U;
        } else if ((Key_UserIsPressed() != 0U) &&
            (Key_Key1IsPressed() == 0U) &&
            (Key_Key2IsPressed() == 0U) &&
            (Key_Key3IsPressed() == 0U) &&
            (Key_Key4IsPressed() == 0U) &&
            (Key_Key5IsPressed() == 0U)) {
            selection = CONTROL_START_TASK2_LOW_SPEED;
            stableMs++;
        } else if ((Key_Key1IsPressed() != 0U) &&
            (Key_UserIsPressed() == 0U) &&
            (Key_Key2IsPressed() == 0U) &&
            (Key_Key3IsPressed() == 0U) &&
            (Key_Key4IsPressed() == 0U) &&
            (Key_Key5IsPressed() == 0U)) {
            selection = CONTROL_START_TASK3_LOW_SPEED;
            stableMs++;
        } else {
            selection = CONTROL_START_NONE;
            stableMs = 0U;
        }

        delay_ms(1U);
        (*printMs)++;
        (*oledMs)++;
    }

    return selection;
}

void Control_WaitForStartKey(uint32_t printPeriodMs, uint32_t debounceMs)
{
    (void) Control_WaitForStartSelection(printPeriodMs, debounceMs);
}

ControlStartSelection Control_WaitForStartSelection(uint32_t printPeriodMs,
    uint32_t debounceMs)
{
    uint32_t printMs = printPeriodMs;
    uint32_t oledMs = CONTROL_OLED_PERIOD_MS;
    uint32_t stableMs = 0U;
    uint8_t task3SettingMode = 0U;
    uint8_t key1WasPressed = 0U;
    uint8_t key2WasPressed = 0U;
    uint8_t lowSpeedModeSelected = 0U;
    ControlStartSelection selection = CONTROL_START_NONE;

    while (stableMs < debounceMs) {
        Control_ServiceDebug(printPeriodMs, &printMs, &oledMs);

        if (Key_AllReleased() != 0U) {
            stableMs++;
        } else {
            stableMs = 0U;
        }

        delay_ms(1U);
        printMs++;
        oledMs++;
    }

    stableMs = 0U;
    while (stableMs < debounceMs) {
        Control_ServiceDebug(printPeriodMs, &printMs, &oledMs);

        if (Control_HandleGrayCalibrationKey(printPeriodMs, debounceMs,
                &printMs, &oledMs) != 0U) {
            selection = CONTROL_START_NONE;
            stableMs = 0U;
            continue;
        }

        if (Control_GrayCalibrationBlocksStart() != 0U) {
            selection = CONTROL_START_NONE;
            stableMs = 0U;
        } else if ((Key_UserIsPressed() != 0U) && (Key_Key1IsPressed() == 0U) &&
            (Key_Key2IsPressed() == 0U) &&
            (Key_Key3IsPressed() == 0U) &&
            (Key_Key4IsPressed() == 0U) &&
            (Key_Key5IsPressed() == 0U)) {
            selection = CONTROL_START_TASK1;
            stableMs++;
        } else if ((Key_Key1IsPressed() != 0U) && (Key_UserIsPressed() == 0U) &&
            (Key_Key2IsPressed() == 0U) &&
            (Key_Key3IsPressed() == 0U) &&
            (Key_Key4IsPressed() == 0U) &&
            (Key_Key5IsPressed() == 0U)) {
            selection = CONTROL_START_TASK2;
            stableMs++;
        } else if ((Key_Key2IsPressed() != 0U) && (Key_UserIsPressed() == 0U) &&
            (Key_Key1IsPressed() == 0U) &&
            (Key_Key3IsPressed() == 0U) &&
            (Key_Key4IsPressed() == 0U) &&
            (Key_Key5IsPressed() == 0U)) {
            selection = CONTROL_START_TASK3;
            stableMs++;
        } else if (Control_OnlyLowSpeedModeKeyPressed() != 0U) {
            selection = CONTROL_START_NONE;
            lowSpeedModeSelected = 1U;
            stableMs++;
        } else {
            selection = CONTROL_START_NONE;
            lowSpeedModeSelected = 0U;
            stableMs = 0U;
        }

        delay_ms(1U);
        printMs++;
        oledMs++;
    }

    if (lowSpeedModeSelected != 0U) {
        return Control_WaitForLowSpeedSelection(printPeriodMs, debounceMs,
            &printMs, &oledMs);
    }

    if (selection != CONTROL_START_TASK3) {
        return selection;
    }

    task3SettingMode = 1U;
    stableMs = 0U;
    while (stableMs < debounceMs) {
        Control_ServiceDebug(printPeriodMs, &printMs, &oledMs);

        if (Key_AllReleased() != 0U) {
            stableMs++;
        } else {
            stableMs = 0U;
        }

        delay_ms(1U);
        printMs++;
        oledMs++;
    }

    while (task3SettingMode != 0U) {
        Control_ServiceDebug(printPeriodMs, &printMs, &oledMs);

        if (Control_HandleGrayCalibrationKey(printPeriodMs, debounceMs,
                &printMs, &oledMs) != 0U) {
            key1WasPressed = 0U;
            key2WasPressed = 0U;
            continue;
        }

        if (Control_GrayCalibrationBlocksStart() != 0U) {
            key1WasPressed = 0U;
            key2WasPressed = 0U;
            delay_ms(1U);
            printMs++;
            oledMs++;
            continue;
        }

        if (Key_Key2IsPressed() == 0U) {
            key2WasPressed = 0U;
        }

        if ((Key_Key1IsPressed() != 0U) && (key1WasPressed == 0U)) {
            delay_ms(debounceMs);
            printMs = (uint32_t) (printMs + debounceMs);
            oledMs = (uint32_t) (oledMs + debounceMs);
            if ((Key_Key1IsPressed() != 0U) &&
                (Key_Key2IsPressed() == 0U) &&
                (Key_UserIsPressed() == 0U) &&
                (Key_Key3IsPressed() == 0U) &&
                (Key_Key4IsPressed() == 0U) &&
                (Key_Key5IsPressed() == 0U)) {
                g_task3LapTarget++;
                if (g_task3LapTarget > CONTROL_TASK3_MAX_LAPS) {
                    g_task3LapTarget = CONTROL_TASK3_MIN_LAPS;
                }
                Control_UpdateGrayOled();
                key1WasPressed = 1U;
            }
        } else if (Key_Key1IsPressed() == 0U) {
            key1WasPressed = 0U;
        }

        if ((Key_Key2IsPressed() != 0U) && (key2WasPressed == 0U) &&
            (Key_Key1IsPressed() == 0U) &&
            (Key_UserIsPressed() == 0U) &&
            (Key_Key3IsPressed() == 0U) &&
            (Key_Key4IsPressed() == 0U) &&
            (Key_Key5IsPressed() == 0U)) {
            stableMs = 0U;
            while (stableMs < debounceMs) {
                Control_ServiceDebug(printPeriodMs, &printMs, &oledMs);
                if ((Key_Key2IsPressed() != 0U) &&
                    (Key_Key1IsPressed() == 0U) &&
                    (Key_UserIsPressed() == 0U) &&
                    (Key_Key3IsPressed() == 0U) &&
                    (Key_Key4IsPressed() == 0U) &&
                    (Key_Key5IsPressed() == 0U)) {
                    stableMs++;
                } else {
                    stableMs = 0U;
                }

                delay_ms(1U);
                printMs++;
                oledMs++;
            }
            key2WasPressed = 1U;
            task3SettingMode = 0U;
        }

        delay_ms(1U);
        printMs++;
        oledMs++;
    }

    return selection;
}

void Control_StartTask1(void)
{
    g_lowSpeedTaskActive = 0U;
    SpeedLoop_SetRampStep(CONTROL_DEFAULT_RAMP_STEP_MM_S);
    Control_SetTaskBaseSpeed(CONTROL_DEFAULT_BASE_SPEED_MM_S);
    ICM42688_YawReset(0.0f);
    Encoder_Reset();
    Control_StartStraight(0.0f);
    Control_TrackReset();
    Control_ResetStableCounters();
    g_cPointYawDeg = 0.0f;
    g_cPointYawValid = 0U;
    g_alertRemainingMs = 0U;
    Control_AlertSet(0U);
    Control_EnterState(CONTROL_STATE_TASK1_AB);
}

static void Control_StartTask2WithBaseSpeed(float baseSpeedMmS)
{
    if (baseSpeedMmS == CONTROL_LOW_BASE_SPEED_MM_S) {
        g_lowSpeedTaskActive = 1U;
        SpeedLoop_SetRampStep(CONTROL_LOW_RAMP_STEP_MM_S);
    } else {
        g_lowSpeedTaskActive = 0U;
        SpeedLoop_SetRampStep(CONTROL_DEFAULT_RAMP_STEP_MM_S);
    }
    Control_SetTaskBaseSpeed(baseSpeedMmS);
    ICM42688_YawReset(0.0f);
    Encoder_Reset();
    Control_StartStraight(0.0f);
    Control_TrackReset();
    Control_ResetStableCounters();
    g_cPointYawDeg = 0.0f;
    g_cPointYawValid = 0U;
    g_alertRemainingMs = 0U;
    Control_AlertSet(0U);
    Control_EnterState(CONTROL_STATE_TASK2_AB);
}

void Control_StartTask2(void)
{
    Control_StartTask2WithBaseSpeed(CONTROL_DEFAULT_BASE_SPEED_MM_S);
}

void Control_StartTask2LowSpeed(void)
{
    Control_StartTask2WithBaseSpeed(CONTROL_LOW_BASE_SPEED_MM_S);
}

static void Control_StartTask3WithBaseSpeed(uint8_t lapCount,
    float baseSpeedMmS)
{
    if (baseSpeedMmS == CONTROL_LOW_BASE_SPEED_MM_S) {
        g_lowSpeedTaskActive = 1U;
        SpeedLoop_SetRampStep(CONTROL_LOW_RAMP_STEP_MM_S);
    } else {
        g_lowSpeedTaskActive = 0U;
        SpeedLoop_SetRampStep(CONTROL_DEFAULT_RAMP_STEP_MM_S);
    }
    Control_SetTaskBaseSpeed(baseSpeedMmS);

    if (lapCount == 0U) {
        lapCount = g_task3LapTarget;
    } else if (lapCount < CONTROL_TASK3_MIN_LAPS) {
        lapCount = CONTROL_TASK3_MIN_LAPS;
    } else if (lapCount > CONTROL_TASK3_MAX_LAPS) {
        lapCount = CONTROL_TASK3_MAX_LAPS;
    }

    g_task3LapTarget = lapCount;
    g_task3LapCurrent = 0U;
    ICM42688_YawReset(0.0f);
    Encoder_Reset();
    Control_StartTask3Turn(Control_Task3AcTargetYaw());
    Control_TrackReset();
    Control_ResetStableCounters();
    g_cPointYawDeg = 0.0f;
    g_cPointYawValid = 0U;
    g_alertRemainingMs = 0U;
    Control_AlertSet(0U);
    Control_EnterState(CONTROL_STATE_TASK3_TURN_TO_C);
}

void Control_StartTask3(uint8_t lapCount)
{
    Control_StartTask3WithBaseSpeed(lapCount, CONTROL_DEFAULT_BASE_SPEED_MM_S);
}

void Control_StartTask3LowSpeedOneLap(void)
{
    Control_StartTask3WithBaseSpeed(1U, CONTROL_LOW_BASE_SPEED_MM_S);
}

void Control_Task10ms(void)
{
    uint8_t digital = 0xFFU;
    uint32_t blackSum = 0U;
    int32_t weightedSum = 0;
    uint8_t grayOk = 0U;

    g_stateElapsedMs += CONTROL_TASK_PERIOD_MS;
    Control_AlertTask();

    switch (g_controlState) {
    case CONTROL_STATE_TASK1_AB:
        grayOk = Control_ReadGrayLine(&digital, &blackSum, &weightedSum);
        Control_SetStraightSpeed(g_baseSpeedMmS);

        if (Control_IsBlackDetected(digital, blackSum, grayOk) != 0U) {
            if (g_blackStableTicks < CONTROL_BLACK_STABLE_TICKS) {
                g_blackStableTicks++;
            }
        } else {
            g_blackStableTicks = 0U;
        }

        if (g_blackStableTicks >= CONTROL_BLACK_STABLE_TICKS) {
            SpeedLoop_Stop();
            g_alertRemainingMs = 0U;
            Control_AlertSet(1U);
            Control_EnterState(CONTROL_STATE_TASK1_STOP_B);
        }
        break;

    case CONTROL_STATE_TASK1_STOP_B:
        if (g_stateElapsedMs >= CONTROL_STOP_B_MS) {
            Control_AlertSet(0U);
            Encoder_Reset();
            SpeedLoop_Start();
            SpeedLoop_SetStraightMode(0U);
            Control_TrackReset();
            Control_ResetStableCounters();
            Control_EnterState(CONTROL_STATE_TASK1_TRACK_TO_C);
        }
        break;

    case CONTROL_STATE_TASK1_TRACK_TO_C:
        grayOk = Control_ReadGrayLine(&digital, &blackSum, &weightedSum);
        Control_UpdateLineTrack(digital, blackSum, weightedSum, grayOk);

        if (Control_IsFullWhite(digital, blackSum, grayOk) != 0U) {
            if (g_whiteStableTicks < CONTROL_WHITE_STABLE_TICKS) {
                g_whiteStableTicks++;
            }
        } else {
            g_whiteStableTicks = 0U;
        }

        if (g_whiteStableTicks >= CONTROL_WHITE_STABLE_TICKS) {
            g_cPointYawDeg = ICM42688_YawGetDeg();
            g_cPointYawValid = 1U;
            SpeedLoop_Stop();
            g_alertRemainingMs = 0U;
            Control_AlertSet(1U);
            Control_EnterState(CONTROL_STATE_TASK1_STOP_C);
        }
        break;

    case CONTROL_STATE_TASK1_STOP_C:
        if (g_stateElapsedMs >= CONTROL_STOP_C_ALERT_MS) {
            Control_AlertSet(0U);
            Control_EnterState(CONTROL_STATE_DONE);
        }
        break;

    case CONTROL_STATE_TASK2_AB:
        grayOk = Control_ReadGrayLine(&digital, &blackSum, &weightedSum);
        Control_SetStraightSpeed(g_baseSpeedMmS);

        if (Control_IsBlackDetected(digital, blackSum, grayOk) != 0U) {
            if (g_blackStableTicks < CONTROL_BLACK_STABLE_TICKS) {
                g_blackStableTicks++;
            }
        } else {
            g_blackStableTicks = 0U;
        }

        if (g_blackStableTicks >= CONTROL_BLACK_STABLE_TICKS) {
            Control_AlertPulse(CONTROL_POINT_ALERT_MS);
            SpeedLoop_SetStraightMode(0U);
            Control_TrackReset();
            Control_ResetStableCounters();
            if (g_lowSpeedTaskActive != 0U) {
                Control_EnterState(CONTROL_STATE_TASK2_CENTER_TO_C);
            } else {
                Control_EnterState(CONTROL_STATE_TASK2_TRACK_TO_C);
            }
        }
        break;

    case CONTROL_STATE_TASK2_CENTER_TO_C:
        grayOk = Control_ReadGrayLine(&digital, &blackSum, &weightedSum);
        if (Control_UpdateLineCenter(digital, blackSum, weightedSum,
                grayOk) != 0U) {
            Control_TrackReset();
            Control_ResetStableCounters();
            Control_EnterState(CONTROL_STATE_TASK2_TRACK_TO_C);
        }
        break;

    case CONTROL_STATE_TASK2_TRACK_TO_C:
        grayOk = Control_ReadGrayLine(&digital, &blackSum, &weightedSum);
        Control_UpdateLineTrack(digital, blackSum, weightedSum, grayOk);

        if (Control_IsFullWhite(digital, blackSum, grayOk) != 0U) {
            if (g_whiteStableTicks < CONTROL_WHITE_STABLE_TICKS) {
                g_whiteStableTicks++;
            }
        } else {
            g_whiteStableTicks = 0U;
        }

        if (g_whiteStableTicks >= CONTROL_WHITE_STABLE_TICKS) {
            g_cPointYawDeg = ICM42688_YawGetDeg();
            g_cPointYawValid = 1U;
            Control_AlertPulse(CONTROL_POINT_ALERT_MS);
            SpeedLoop_SetStraightTargetYaw(CONTROL_TASK2_STRAIGHT_TARGET_YAW);
            SpeedLoop_SetStraightMode(1U);
            Control_SetStraightSpeed(g_baseSpeedMmS);
            Control_ResetStableCounters();
            Control_EnterState(CONTROL_STATE_TASK2_STRAIGHT_TO_D);
        }
        break;

    case CONTROL_STATE_TASK2_STRAIGHT_TO_D:
        grayOk = Control_ReadGrayLine(&digital, &blackSum, &weightedSum);
        Control_SetStraightSpeed(g_baseSpeedMmS);

        if (Control_IsBlackDetected(digital, blackSum, grayOk) != 0U) {
            if (g_blackStableTicks < CONTROL_BLACK_STABLE_TICKS) {
                g_blackStableTicks++;
            }
        } else {
            g_blackStableTicks = 0U;
        }

        if (g_blackStableTicks >= CONTROL_BLACK_STABLE_TICKS) {
            Control_AlertPulse(CONTROL_POINT_ALERT_MS);
            SpeedLoop_SetStraightMode(0U);
            Control_TrackReset();
            Control_ResetStableCounters();
            if (g_lowSpeedTaskActive != 0U) {
                Control_EnterState(CONTROL_STATE_TASK2_CENTER_TO_A);
            } else {
                Control_EnterState(CONTROL_STATE_TASK2_TRACK_TO_A);
            }
        }
        break;

    case CONTROL_STATE_TASK2_CENTER_TO_A:
        grayOk = Control_ReadGrayLine(&digital, &blackSum, &weightedSum);
        if (Control_UpdateLineCenter(digital, blackSum, weightedSum,
                grayOk) != 0U) {
            Control_TrackReset();
            Control_ResetStableCounters();
            Control_EnterState(CONTROL_STATE_TASK2_TRACK_TO_A);
        }
        break;

    case CONTROL_STATE_TASK2_TRACK_TO_A:
        grayOk = Control_ReadGrayLine(&digital, &blackSum, &weightedSum);
        Control_UpdateLineTrack(digital, blackSum, weightedSum, grayOk);

        if (Control_IsFullWhite(digital, blackSum, grayOk) != 0U) {
            if (g_whiteStableTicks < CONTROL_WHITE_STABLE_TICKS) {
                g_whiteStableTicks++;
            }
        } else {
            g_whiteStableTicks = 0U;
        }

        if (g_whiteStableTicks >= CONTROL_WHITE_STABLE_TICKS) {
            SpeedLoop_Stop();
            g_alertRemainingMs = 0U;
            Control_AlertSet(1U);
            Control_EnterState(CONTROL_STATE_TASK2_STOP_A);
        }
        break;

    case CONTROL_STATE_TASK2_STOP_A:
        if (g_stateElapsedMs >= CONTROL_STOP_C_ALERT_MS) {
            Control_AlertSet(0U);
            Control_EnterState(CONTROL_STATE_DONE);
        }
        break;

    case CONTROL_STATE_TASK3_TURN_TO_C:
        if (Control_UpdateYawTurn(Control_Task3AcTargetYaw()) != 0U) {
            SpeedLoop_Stop();
            Control_StartTask3Drive(Control_Task3AcTargetYaw());
            Control_EnterState(CONTROL_STATE_TASK3_DRIVE_TO_C);
        }
        break;

    case CONTROL_STATE_TASK3_DRIVE_TO_C:
        Control_SetStraightSpeed(g_baseSpeedMmS);

        if (Control_AverageEncoderAbsCounts() >=
            Control_Task3DistanceTargetCounts(CONTROL_TASK3_AC_DISTANCE_MM)) {
            SpeedLoop_Stop();
            Control_StartTask3Turn(CONTROL_TASK3_AC_ENTRY_YAW);
            Control_EnterState(CONTROL_STATE_TASK3_ALIGN_C);
        }
        break;

    case CONTROL_STATE_TASK3_ALIGN_C:
        if (Control_UpdateYawTurn(CONTROL_TASK3_AC_ENTRY_YAW) != 0U) {
            SpeedLoop_Stop();
            SpeedLoop_Start();
            SpeedLoop_SetStraightTargetYaw(CONTROL_TASK3_AC_ENTRY_YAW);
            SpeedLoop_SetStraightMode(1U);
            Control_SetStraightSpeed(CONTROL_TASK3_SEEK_SPEED_MM_S);
            Control_ResetStableCounters();
            Control_EnterState(CONTROL_STATE_TASK3_SEEK_C);
        }
        break;

    case CONTROL_STATE_TASK3_SEEK_C:
        grayOk = Control_ReadGrayLine(&digital, &blackSum, &weightedSum);
        Control_SetStraightSpeed(CONTROL_TASK3_SEEK_SPEED_MM_S);

        if (Control_IsBlackDetected(digital, blackSum, grayOk) != 0U) {
            if (g_blackStableTicks < CONTROL_BLACK_STABLE_TICKS) {
                g_blackStableTicks++;
            }
        } else {
            g_blackStableTicks = 0U;
        }

        if (g_blackStableTicks >= CONTROL_BLACK_STABLE_TICKS) {
            g_cPointYawDeg = ICM42688_YawGetDeg();
            g_cPointYawValid = 1U;
            Control_AlertPulse(CONTROL_POINT_ALERT_MS);
            SpeedLoop_SetStraightMode(0U);
            Control_TrackReset();
            Control_ResetStableCounters();
            if (g_lowSpeedTaskActive != 0U) {
                Control_EnterState(CONTROL_STATE_TASK3_CENTER_TO_B);
            } else {
                Control_EnterState(CONTROL_STATE_TASK3_TRACK_TO_B);
            }
        }
        break;

    case CONTROL_STATE_TASK3_CENTER_TO_B:
        grayOk = Control_ReadGrayLine(&digital, &blackSum, &weightedSum);
        if (Control_UpdateLineCenter(digital, blackSum, weightedSum,
                grayOk) != 0U) {
            Control_TrackReset();
            Control_ResetStableCounters();
            Control_EnterState(CONTROL_STATE_TASK3_TRACK_TO_B);
        }
        break;

    case CONTROL_STATE_TASK3_TRACK_TO_B:
        grayOk = Control_ReadGrayLine(&digital, &blackSum, &weightedSum);
        Control_UpdateLineTrack(digital, blackSum, weightedSum, grayOk);

        if (Control_IsFullWhite(digital, blackSum, grayOk) != 0U) {
            if (g_whiteStableTicks < CONTROL_WHITE_STABLE_TICKS) {
                g_whiteStableTicks++;
            }
        } else {
            g_whiteStableTicks = 0U;
        }

        if (g_whiteStableTicks >= CONTROL_WHITE_STABLE_TICKS) {
            Control_AlertPulse(CONTROL_POINT_ALERT_MS);
            SpeedLoop_Stop();
            Control_StartTask3Turn(Control_Task3BdTargetYaw());
            Control_ResetStableCounters();
            Control_EnterState(CONTROL_STATE_TASK3_TURN_TO_D);
        }
        break;

    case CONTROL_STATE_TASK3_TURN_TO_D:
        if (Control_UpdateYawTurn(Control_Task3BdTargetYaw()) != 0U) {
            SpeedLoop_Stop();
            Control_StartTask3Drive(Control_Task3BdTargetYaw());
            Control_EnterState(CONTROL_STATE_TASK3_DRIVE_TO_D);
        }
        break;

    case CONTROL_STATE_TASK3_DRIVE_TO_D:
        Control_SetStraightSpeed(g_baseSpeedMmS);

        if (Control_AverageEncoderAbsCounts() >=
            Control_Task3DistanceTargetCounts(CONTROL_TASK3_BD_DISTANCE_MM)) {
            SpeedLoop_Stop();
            Control_StartTask3Turn(CONTROL_TASK3_BD_ENTRY_YAW);
            Control_EnterState(CONTROL_STATE_TASK3_ALIGN_D);
        }
        break;

    case CONTROL_STATE_TASK3_ALIGN_D:
        if (Control_UpdateYawTurn(CONTROL_TASK3_BD_ENTRY_YAW) != 0U) {
            SpeedLoop_Stop();
            SpeedLoop_Start();
            SpeedLoop_SetStraightTargetYaw(CONTROL_TASK3_BD_ENTRY_YAW);
            SpeedLoop_SetStraightMode(1U);
            Control_SetStraightSpeed(CONTROL_TASK3_SEEK_SPEED_MM_S);
            Control_ResetStableCounters();
            Control_EnterState(CONTROL_STATE_TASK3_SEEK_D);
        }
        break;

    case CONTROL_STATE_TASK3_SEEK_D:
        grayOk = Control_ReadGrayLine(&digital, &blackSum, &weightedSum);
        Control_SetStraightSpeed(CONTROL_TASK3_SEEK_SPEED_MM_S);

        if (Control_IsBlackDetected(digital, blackSum, grayOk) != 0U) {
            if (g_blackStableTicks < CONTROL_BLACK_STABLE_TICKS) {
                g_blackStableTicks++;
            }
        } else {
            g_blackStableTicks = 0U;
        }

        if (g_blackStableTicks >= CONTROL_BLACK_STABLE_TICKS) {
            Control_AlertPulse(CONTROL_POINT_ALERT_MS);
            SpeedLoop_SetStraightMode(0U);
            Control_TrackReset();
            Control_ResetStableCounters();
            if (g_lowSpeedTaskActive != 0U) {
                Control_EnterState(CONTROL_STATE_TASK3_CENTER_TO_A);
            } else {
                Control_EnterState(CONTROL_STATE_TASK3_TRACK_TO_A);
            }
        }
        break;

    case CONTROL_STATE_TASK3_CENTER_TO_A:
        grayOk = Control_ReadGrayLine(&digital, &blackSum, &weightedSum);
        if (Control_UpdateLineCenter(digital, blackSum, weightedSum,
                grayOk) != 0U) {
            Control_TrackReset();
            Control_ResetStableCounters();
            Control_EnterState(CONTROL_STATE_TASK3_TRACK_TO_A);
        }
        break;

    case CONTROL_STATE_TASK3_TRACK_TO_A:
        grayOk = Control_ReadGrayLine(&digital, &blackSum, &weightedSum);
        Control_UpdateLineTrack(digital, blackSum, weightedSum, grayOk);

        if (Control_IsFullWhite(digital, blackSum, grayOk) != 0U) {
            if (g_whiteStableTicks < CONTROL_WHITE_STABLE_TICKS) {
                g_whiteStableTicks++;
            }
        } else {
            g_whiteStableTicks = 0U;
        }

        if (g_whiteStableTicks >= CONTROL_WHITE_STABLE_TICKS) {
            Control_AlertPulse(CONTROL_POINT_ALERT_MS);
            g_task3LapCurrent++;
            if (g_task3LapCurrent >= g_task3LapTarget) {
                SpeedLoop_Stop();
                g_alertRemainingMs = 0U;
                Control_AlertSet(1U);
                Control_EnterState(CONTROL_STATE_TASK3_STOP_A);
            } else {
                SpeedLoop_Stop();
                Control_StartTask3Turn(Control_Task3AcTargetYaw());
                Control_ResetStableCounters();
                Control_EnterState(CONTROL_STATE_TASK3_TURN_TO_C);
            }
        }
        break;

    case CONTROL_STATE_TASK3_STOP_A:
        if (g_stateElapsedMs >= CONTROL_STOP_C_ALERT_MS) {
            Control_AlertSet(0U);
            Control_EnterState(CONTROL_STATE_DONE);
        }
        break;

    case CONTROL_STATE_IDLE:
    case CONTROL_STATE_DONE:
    default:
        break;
    }
}
