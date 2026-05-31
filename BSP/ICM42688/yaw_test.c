#include <stdint.h>
#include "yaw_test.h"
#include "delay.h"
#include "oled.h"
#include "ICM42688.h"

#define YAW_CALIB_SAMPLES       (200U)
#define YAW_CALIB_DELAY_MS      (5U)
#define YAW_UPDATE_DELAY_MS     (20U)

static float g_gz_bias;
static float g_yaw_deg;
static uint8_t g_init_error;
static volatile uint32_t g_time_ms;

static void OLED_ShowFixedTenths(uint8_t x, uint8_t y, int32_t value,
    uint8_t int_len, uint8_t size, uint8_t mode)
{
    uint32_t abs_value;
    uint32_t integer;
    uint32_t decimal;
    uint8_t step = size / 2U;

    if (value < 0) {
        OLED_ShowChar(x, y, '-', size, mode);
        abs_value = (uint32_t) (-value);
    } else {
        OLED_ShowChar(x, y, '+', size, mode);
        abs_value = (uint32_t) value;
    }

    integer = abs_value / 10U;
    decimal = abs_value % 10U;
    OLED_ShowNum(x + step, y, integer, int_len, size, mode);
    OLED_ShowChar(x + step * (int_len + 1U), y, '.', size, mode);
    OLED_ShowNum(x + step * (int_len + 2U), y, decimal, 1, size, mode);
}

static float YawTest_CalibrateGyroZ(void)
{
    short ax, ay, az, gx, gy, gz;
    int32_t sum = 0;
    uint16_t valid = 0;

    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"YAW MODE", 16, 1);
    OLED_ShowString(0, 24, (uint8_t *)"KEEP STILL", 16, 1);
    OLED_ShowString(0, 48, (uint8_t *)"CAL GZ", 16, 1);
    OLED_Refresh();

    for (uint16_t i = 0U; i < YAW_CALIB_SAMPLES; i++) {
        if (ICM_Get_Raw_data(&ax, &ay, &az, &gx, &gy, &gz) == 0U) {
            sum += gz;
            valid++;
        }
        delay_ms(YAW_CALIB_DELAY_MS);
    }

    if (valid == 0U) {
        return 0.0f;
    }

    return (float) sum / (float) valid;
}

void YawTest_Init(void)
{
    uint8_t icm_id = SPI_Gryo_Init();

    g_init_error = ICM42688_GetInitError();

    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"ICM ID", 16, 1);
    OLED_ShowNum(64, 0, icm_id, 3, 16, 1);
    OLED_ShowString(0, 16, (uint8_t *)"ERR", 16, 1);
    OLED_ShowNum(40, 16, g_init_error, 3, 16, 1);
    OLED_Refresh();
    delay_ms(1000);

    g_gz_bias = YawTest_CalibrateGyroZ();
    g_yaw_deg = 0.0f;
}

void YawTest_OnTimer10ms(void)
{
    g_time_ms += 10U;
}

void YawTest_Run(void)
{
    short ax, ay, az, gx, gy, gz;
    uint8_t raw_ret;
    uint8_t pwr;
    uint8_t whoami;
    uint32_t now_ms;
    uint32_t last_ms = g_time_ms;
    float dt_s;
    int32_t yaw_tenths;

    while (1) {
        raw_ret = ICM_Get_Raw_data(&ax, &ay, &az, &gx, &gy, &gz);
        whoami = ICM42688_ReadWhoAmI();
        pwr = ICM_Read_Byte(ICM42688_REG_PWR_MGMT0);
        now_ms = g_time_ms;
        dt_s = (float) (now_ms - last_ms) / 1000.0f;
        last_ms = now_ms;

        if ((pwr & (ICM42688_PWR_ACCEL_LN_MODE | ICM42688_PWR_GYRO_LN_MODE)) !=
            (ICM42688_PWR_ACCEL_LN_MODE | ICM42688_PWR_GYRO_LN_MODE)) {
            OLED_Clear();
            OLED_ShowString(0, 0, (uint8_t *)"ID", 16, 1);
            OLED_ShowNum(20, 0, whoami, 3, 16, 1);
            OLED_ShowString(52, 0, (uint8_t *)"R", 16, 1);
            OLED_ShowNum(68, 0, raw_ret, 1, 16, 1);
            OLED_ShowString(86, 0, (uint8_t *)"P", 16, 1);
            OLED_ShowNum(102, 0, pwr, 3, 16, 1);
            OLED_ShowString(0, 48, (uint8_t *)"E", 16, 1);
            OLED_ShowNum(16, 48, g_init_error, 3, 16, 1);
            OLED_Refresh();
            delay_ms(100);
            continue;
        }

        if ((raw_ret == 0U) && (dt_s > 0.0f)) {
            g_yaw_deg += (((float) gz - g_gz_bias) / 32.8f) * dt_s;
            if (g_yaw_deg > 180.0f) {
                g_yaw_deg -= 360.0f;
            } else if (g_yaw_deg < -180.0f) {
                g_yaw_deg += 360.0f;
            }
        }

        yaw_tenths = (int32_t) ((g_yaw_deg * 10.0f) +
            ((g_yaw_deg >= 0.0f) ? 0.5f : -0.5f));

        OLED_Clear();
        OLED_ShowString(0, 0, (uint8_t *)"YAW", 24, 1);
        OLED_ShowFixedTenths(20, 28, yaw_tenths, 3, 24, 1);
        OLED_ShowString(0, 56, (uint8_t *)"ID", 8, 1);
        OLED_ShowNum(12, 56, whoami, 3, 8, 1);
        OLED_ShowString(42, 56, (uint8_t *)"R", 8, 1);
        OLED_ShowNum(48, 56, raw_ret, 1, 8, 1);
        OLED_ShowString(66, 56, (uint8_t *)"P", 8, 1);
        OLED_ShowNum(72, 56, pwr, 3, 8, 1);
        OLED_Refresh();
        delay_ms(YAW_UPDATE_DELAY_MS);
    }
}
