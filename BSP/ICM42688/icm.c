#include "icm.h"
#include <math.h>

#ifndef PI
#define PI 3.141592654f
#endif

#define ICM_SAMPLE_HZ_DEFAULT          100.0f
#define ICM_SAMPLE_DT_DEFAULT_S        (1.0f / ICM_SAMPLE_HZ_DEFAULT)
#define ICM_SAMPLE_DT_MIN_S            0.001f
#define ICM_SAMPLE_DT_MAX_S            0.050f
#define ICM_YAW_FILTER_FS_HZ           100
#define ICM_YAW_FILTER_FC_HZ           20.0f
#define ICM_YAW_FILTER_Q               0.5773f
#define ICM_YAW_DEADZONE_DPS           0.05f
#define ICM_YAW_BIAS_LEARN_ALPHA       0.002f
#define ICM_YAW_BIAS_LIMIT_DPS         2.0f

float Pitch_a, Roll_a, Yaw_a;
float Pitch_g, Roll_g, Yaw_g;
float Pitch_g_F, Roll_g_F, Yaw_g_F;
float Yaw_GyroAngle;
float Yaw_TotalAngle;
float Yaw_AngleLast;
int Yaw_RoundCount;
biquad_state adc_error;

static biquad_state g_yaw_biquad;
static uint8_t g_filter_initialized;
static uint8_t g_runtime_bias_learning;
static float g_sample_dt_s = ICM_SAMPLE_DT_DEFAULT_S;
static float g_sample_hz = ICM_SAMPLE_HZ_DEFAULT;
static float g_yaw_runtime_bias;
static float g_yaw_rate_used;
static ICM_YawMode g_yaw_mode = ICM_YAW_MODE_DEADZONE;

static float ICM_AbsF(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float ICM_ApplyDeadzone(float value, float deadzone)
{
    if (ICM_AbsF(value) < deadzone) {
        return 0.0f;
    }

    return value;
}

static float ICM_ClampSampleDt(float dt_s)
{
    if ((dt_s != dt_s) || (dt_s < ICM_SAMPLE_DT_MIN_S) ||
        (dt_s > ICM_SAMPLE_DT_MAX_S)) {
        return ICM_SAMPLE_DT_DEFAULT_S;
    }

    return dt_s;
}

static void ICM_ResetBiquadState(biquad_state *state)
{
    if (state == 0) {
        return;
    }

    state->x1 = 0.0f;
    state->x2 = 0.0f;
    state->y1 = 0.0f;
    state->y2 = 0.0f;
}

static void ICM_UpdateYawRuntimeBias(float gyro_z_calibrated)
{
    if (g_runtime_bias_learning == 0U) {
        return;
    }

    if (ICM_AbsF(gyro_z_calibrated - g_yaw_runtime_bias) >
        ICM_YAW_BIAS_LIMIT_DPS) {
        return;
    }

    g_yaw_runtime_bias +=
        ICM_YAW_BIAS_LEARN_ALPHA * (gyro_z_calibrated - g_yaw_runtime_bias);
}

static void ICM_EnsureFilterInit(void)
{
    if (g_filter_initialized == 0U) {
        Filter_Init();
    }
}

void Filter_Init(void)
{
    biquad_filter_init(&g_yaw_biquad, BIQUAD_LOWPASS, ICM_YAW_FILTER_FS_HZ,
        ICM_YAW_FILTER_FC_HZ, ICM_YAW_FILTER_Q);
    biquad_filter_init(&adc_error, BIQUAD_LOWPASS, ICM_YAW_FILTER_FS_HZ,
        10.0f, ICM_YAW_FILTER_Q);
    g_filter_initialized = 1U;
}

void ICM_ResetAttitude(void)
{
    Pitch_a = 0.0f;
    Roll_a = 0.0f;
    Yaw_a = 0.0f;
    Pitch_g = 0.0f;
    Roll_g = 0.0f;
    Yaw_g = 0.0f;
    Pitch_g_F = 0.0f;
    Roll_g_F = 0.0f;
    Yaw_g_F = 0.0f;
    Yaw_GyroAngle = 0.0f;
    Yaw_TotalAngle = 0.0f;
    Yaw_AngleLast = 0.0f;
    Yaw_RoundCount = 0;
    g_sample_dt_s = ICM_SAMPLE_DT_DEFAULT_S;
    g_sample_hz = ICM_SAMPLE_HZ_DEFAULT;
    g_yaw_runtime_bias = 0.0f;
    g_yaw_rate_used = 0.0f;
    g_runtime_bias_learning = 0U;
    g_yaw_mode = ICM_YAW_MODE_DEADZONE;
    ICM_EnsureFilterInit();
    ICM_ResetBiquadState(&g_yaw_biquad);
    ICM_ResetBiquadState(&adc_error);
}

void ICM_YawFilterReset(float yawDeg)
{
    ICM_EnsureFilterInit();
    Yaw_GyroAngle = yawDeg;
    Yaw_TotalAngle = yawDeg;
    Yaw_a = yawDeg;
    Yaw_AngleLast = yawDeg;
    Yaw_RoundCount = 0;
    Yaw_g = 0.0f;
    Yaw_g_F = 0.0f;
    g_yaw_rate_used = 0.0f;
    ICM_ResetBiquadState(&g_yaw_biquad);
}

float ICM_YawFilterUpdate(float correctedGzDps, float dt_s)
{
    float rateUsed;

    ICM_EnsureFilterInit();

    dt_s = ICM_ClampSampleDt(dt_s);
    g_sample_dt_s = dt_s;
    g_sample_hz = 1.0f / dt_s;

    Yaw_g = correctedGzDps;

    if (g_yaw_mode == ICM_YAW_MODE_DEADZONE_LPF_BIAS) {
        ICM_UpdateYawRuntimeBias(Yaw_g);
        rateUsed = Yaw_g - g_yaw_runtime_bias;
    } else {
        rateUsed = Yaw_g;
    }

    if ((g_yaw_mode == ICM_YAW_MODE_DEADZONE) ||
        (g_yaw_mode == ICM_YAW_MODE_DEADZONE_LPF) ||
        (g_yaw_mode == ICM_YAW_MODE_DEADZONE_LPF_BIAS)) {
        rateUsed = ICM_ApplyDeadzone(rateUsed, ICM_YAW_DEADZONE_DPS);
    }

    g_yaw_rate_used = rateUsed;
    if ((g_yaw_mode == ICM_YAW_MODE_DEADZONE_LPF) ||
        (g_yaw_mode == ICM_YAW_MODE_DEADZONE_LPF_BIAS)) {
        Yaw_g_F = biquad(&g_yaw_biquad, g_yaw_rate_used);
    } else {
        Yaw_g_F = g_yaw_rate_used;
    }

    Yaw_GyroAngle += Yaw_g_F * dt_s;
    Yaw_TotalAngle = Yaw_GyroAngle;
    Yaw_a = Yaw_TotalAngle;
    Yaw_AngleLast = Yaw_a;

    return Yaw_TotalAngle;
}

float ICM_YawFilterGetYawDeg(void)
{
    return Yaw_TotalAngle;
}

float ICM_YawFilterGetInputRateDps(void)
{
    return Yaw_g;
}

float ICM_YawFilterGetFilteredRateDps(void)
{
    return Yaw_g_F;
}

void get_ICM_data(float dt_s)
{
    (void) ICM_YawFilterUpdate(Yaw_g, dt_s);
}

float ICM_GetSampleDtMs(void)
{
    return g_sample_dt_s * 1000.0f;
}

float ICM_GetSampleHz(void)
{
    return g_sample_hz;
}

void ICM_SetRuntimeBiasLearning(uint8_t enable)
{
    g_runtime_bias_learning = (enable != 0U) ? 1U : 0U;
}

float ICM_GetYawRuntimeBias(void)
{
    return g_yaw_runtime_bias;
}

float ICM_GetYawRateUsed(void)
{
    return g_yaw_rate_used;
}

void ICM_SetYawMode(ICM_YawMode mode)
{
    if (mode > ICM_YAW_MODE_DEADZONE_LPF_BIAS) {
        mode = ICM_YAW_MODE_DEADZONE;
    }

    g_yaw_mode = mode;
    g_runtime_bias_learning =
        (mode == ICM_YAW_MODE_DEADZONE_LPF_BIAS) ? 1U : 0U;
    g_yaw_runtime_bias = 0.0f;
    g_yaw_rate_used = 0.0f;
    ICM_EnsureFilterInit();
    ICM_ResetBiquadState(&g_yaw_biquad);
}

ICM_YawMode ICM_GetYawMode(void)
{
    return g_yaw_mode;
}

const char *ICM_GetYawModeName(void)
{
    switch (g_yaw_mode) {
    case ICM_YAW_MODE_RAW:
        return "RAW";
    case ICM_YAW_MODE_DEADZONE:
        return "DEADZONE";
    case ICM_YAW_MODE_DEADZONE_LPF:
        return "DEADZONE_LPF";
    case ICM_YAW_MODE_DEADZONE_LPF_BIAS:
        return "DEADZONE_LPF_BIAS";
    default:
        return "UNKNOWN";
    }
}

void biquad_filter_init(biquad_state *state, biquad_type type, int fs,
    float fc, float q_value)
{
    float w0, sin_w0, cos_w0, alpha;
    float b0 = 1.0f;
    float b1 = 0.0f;
    float b2 = 0.0f;
    float a0 = 1.0f;
    float a1 = 0.0f;
    float a2 = 0.0f;

    if ((state == 0) || (fs <= 0) || (fc <= 0.0f) || (q_value <= 0.0f)) {
        return;
    }

    w0 = 2.0f * PI * fc / (float) fs;
    sin_w0 = sinf(w0);
    cos_w0 = cosf(w0);
    alpha = sin_w0 / (2.0f * q_value);

    switch (type) {
    case BIQUAD_LOWPASS:
        b0 = (1.0f - cos_w0) * 0.5f;
        b1 = 1.0f - cos_w0;
        b2 = b0;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cos_w0;
        a2 = 1.0f - alpha;
        break;
    case BIQUAD_HIGHPASS:
        b0 = (1.0f + cos_w0) * 0.5f;
        b1 = -(1.0f + cos_w0);
        b2 = b0;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cos_w0;
        a2 = 1.0f - alpha;
        break;
    case BIQUAD_BANDPASS_PEAK:
        b0 = alpha;
        b1 = 0.0f;
        b2 = -alpha;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cos_w0;
        a2 = 1.0f - alpha;
        break;
    case BIQUAD_BANDSTOP_NOTCH:
        b0 = 1.0f;
        b1 = -2.0f * cos_w0;
        b2 = 1.0f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cos_w0;
        a2 = 1.0f - alpha;
        break;
    default:
        break;
    }

    state->a0 = b0 / a0;
    state->a1 = b1 / a0;
    state->a2 = b2 / a0;
    state->a3 = a1 / a0;
    state->a4 = a2 / a0;
    ICM_ResetBiquadState(state);
}

float biquad(biquad_state *state, float data)
{
    float result;

    if (state == 0) {
        return data;
    }

    result = state->a0 * data + state->a1 * state->x1 +
        state->a2 * state->x2 - state->a3 * state->y1 -
        state->a4 * state->y2;
    state->x2 = state->x1;
    state->x1 = data;
    state->y2 = state->y1;
    state->y1 = result;

    return result;
}

float invSqrt(float x)
{
    if (x <= 0.0f) {
        return 0.0f;
    }

    return 1.0f / sqrtf(x);
}
