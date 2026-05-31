#include "gray_test.h"

#include <stdint.h>
#include "delay.h"
#include "gray_sensor.h"
#include "oled.h"

static const uint16_t s_gray_white[GRAY_SENSOR_CHANNELS] = {
    600U, 600U, 600U, 600U, 600U, 600U, 600U, 600U
};

static const uint16_t s_gray_black[GRAY_SENSOR_CHANNELS] = {
    100U, 100U, 100U, 100U, 100U, 100U, 100U, 100U
};

static void GrayTest_ShowDigital(uint8_t digital)
{
    for (uint8_t i = 0U; i < GRAY_SENSOR_CHANNELS; i++) {
        OLED_ShowNum((uint8_t) (i * 8U), 16U,
            (digital >> i) & 0x01U, 1U, 16U, 1U);
    }
}

static void GrayTest_ShowAnalog(const GraySensor *sensor)
{
    for (uint8_t i = 0U; i < 4U; i++) {
        OLED_ShowNum((uint8_t) (i * 32U), 34U, sensor->analog[i], 4U, 8U, 1U);
    }

    for (uint8_t i = 0U; i < 4U; i++) {
        OLED_ShowNum((uint8_t) (i * 32U), 46U, sensor->analog[i + 4U], 4U, 8U, 1U);
    }
}

void GrayTest_Run(void)
{
    GraySensor_Init(&g_gray_sensor, s_gray_white, s_gray_black);

    while (1) {
        uint8_t ok = GraySensor_Task(&g_gray_sensor);
        uint8_t digital = GraySensor_GetDigital(&g_gray_sensor);

        OLED_Clear();
        OLED_ShowString(0U, 0U, (uint8_t *)"GRAY", 16U, 1U);
        OLED_ShowString(50U, 0U, (uint8_t *)(ok ? "OK" : "ERR"), 16U, 1U);
        OLED_ShowNum(96U, 0U, GraySensor_GetLastError(), 1U, 16U, 1U);
        GrayTest_ShowDigital(digital);
        GrayTest_ShowAnalog(&g_gray_sensor);
        OLED_Refresh();

        delay_ms(50);
    }
}
