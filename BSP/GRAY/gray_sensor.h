#ifndef __GRAY_SENSOR_H__
#define __GRAY_SENSOR_H__

#include <stdint.h>
#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GRAY_SENSOR_CHANNELS        (8U)
#define GRAY_ADC_SAMPLE_COUNT       (40U)
#define GRAY_SENSOR_CAL_FLASH_ADDRESS   (0x0001FC00UL)
#define GRAY_SENSOR_CAL_FLASH_SIZE      (DL_FLASHCTL_SECTOR_SIZE)

typedef struct {
    uint16_t analog[GRAY_SENSOR_CHANNELS];
    uint16_t normalized[GRAY_SENSOR_CHANNELS];
    uint16_t calibrated_white[GRAY_SENSOR_CHANNELS];
    uint16_t calibrated_black[GRAY_SENSOR_CHANNELS];
    uint16_t gray_white[GRAY_SENSOR_CHANNELS];
    uint16_t gray_black[GRAY_SENSOR_CHANNELS];
    float normal_factor[GRAY_SENSOR_CHANNELS];
    uint16_t adc_max;
    uint8_t digital;
    uint8_t ready;
    uint8_t last_error;
} GraySensor;

extern volatile uint16_t Gray_ADCValue[GRAY_ADC_SAMPLE_COUNT];
extern GraySensor g_gray_sensor;

void GraySensor_Init(GraySensor *sensor, const uint16_t *white,
    const uint16_t *black);
void GraySensor_InitDefault(void);
uint8_t GraySensor_LoadSavedCalibration(uint16_t *white, uint16_t *black);
uint8_t GraySensor_SaveCalibration(const uint16_t *white,
    const uint16_t *black);
void GraySensor_ADCStart(void);
uint8_t GraySensor_Task(GraySensor *sensor);
uint8_t GraySensor_GetDigital(const GraySensor *sensor);
uint8_t GraySensor_GetAnalog(const GraySensor *sensor, uint16_t *result);
uint8_t GraySensor_GetNormalized(const GraySensor *sensor, uint16_t *result);
uint16_t GraySensor_ReadADCMean(uint8_t count);
uint8_t GraySensor_GetLastError(void);
void GraySensor_ADCIRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif
