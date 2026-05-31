#ifndef __ICM42688_H__
#define __ICM42688_H__

#include <stdint.h>
#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef u8
#define u8 uint8_t
#endif

#ifndef u16
#define u16 uint16_t
#endif

#ifndef u32
#define u32 uint32_t
#endif

#ifndef SPI_ICM42688_INST
#define SPI_ICM42688_INST                         SPI0
#define GPIO_SPI_ICM42688_SCLK_PORT               GPIOA
#define GPIO_SPI_ICM42688_SCLK_PIN                DL_GPIO_PIN_12
#define GPIO_SPI_ICM42688_IOMUX_SCLK              IOMUX_PINCM34
#define GPIO_SPI_ICM42688_IOMUX_SCLK_FUNC         IOMUX_PINCM34_PF_SPI0_SCLK
#define GPIO_SPI_ICM42688_PICO_PORT               GPIOA
#define GPIO_SPI_ICM42688_PICO_PIN                DL_GPIO_PIN_14
#define GPIO_SPI_ICM42688_IOMUX_PICO              IOMUX_PINCM36
#define GPIO_SPI_ICM42688_IOMUX_PICO_FUNC         IOMUX_PINCM36_PF_SPI0_PICO
#define GPIO_SPI_ICM42688_POCI_PORT               GPIOA
#define GPIO_SPI_ICM42688_POCI_PIN                DL_GPIO_PIN_13
#define GPIO_SPI_ICM42688_IOMUX_POCI              IOMUX_PINCM35
#define GPIO_SPI_ICM42688_IOMUX_POCI_FUNC         IOMUX_PINCM35_PF_SPI0_POCI
#endif

#ifndef ICM42688_PORT
#define ICM42688_PORT                             GPIOA
#endif

#ifndef ICM42688_CS_PIN
#ifdef ICM_ICM_CS_PIN
#define ICM42688_CS_PIN                           ICM_ICM_CS_PIN
#define ICM42688_CS_IOMUX                         ICM_ICM_CS_IOMUX
#elif defined(ICM42688_ICM_CS_PIN)
#define ICM42688_CS_PIN                           ICM42688_ICM_CS_PIN
#define ICM42688_CS_IOMUX                         ICM42688_ICM_CS_IOMUX
#else
#define ICM42688_CS_PIN                           DL_GPIO_PIN_15
#define ICM42688_CS_IOMUX                         IOMUX_PINCM37
#endif
#endif

#define ICM42688_CS_LOW()                         DL_GPIO_clearPins(ICM42688_PORT, ICM42688_CS_PIN)
#define ICM42688_CS_HIGH()                        DL_GPIO_setPins(ICM42688_PORT, ICM42688_CS_PIN)

#define ICM42688_WHO_AM_I_VALUE                   (0x47U)
#define ICM42688_INIT_OK                          (0x00U)
#define ICM42688_INIT_WHOAMI_FAILED               (0xE0U)
#define ICM42688_INIT_PWR_WRITE_FAILED            (0xE1U)

#define ICM42688_REG_DEVICE_CONFIG                (0x11U)
#define ICM42688_REG_DRIVE_CONFIG                 (0x13U)
#define ICM42688_REG_INT_CONFIG                   (0x14U)
#define ICM42688_REG_FIFO_CONFIG                  (0x16U)
#define ICM42688_REG_TEMP_DATA1                   (0x1DU)
#define ICM42688_REG_TEMP_DATA0                   (0x1EU)
#define ICM42688_REG_ACCEL_DATA_X1                (0x1FU)
#define ICM42688_REG_ACCEL_DATA_X0                (0x20U)
#define ICM42688_REG_ACCEL_DATA_Y1                (0x21U)
#define ICM42688_REG_ACCEL_DATA_Y0                (0x22U)
#define ICM42688_REG_ACCEL_DATA_Z1                (0x23U)
#define ICM42688_REG_ACCEL_DATA_Z0                (0x24U)
#define ICM42688_REG_GYRO_DATA_X1                 (0x25U)
#define ICM42688_REG_GYRO_DATA_X0                 (0x26U)
#define ICM42688_REG_GYRO_DATA_Y1                 (0x27U)
#define ICM42688_REG_GYRO_DATA_Y0                 (0x28U)
#define ICM42688_REG_GYRO_DATA_Z1                 (0x29U)
#define ICM42688_REG_GYRO_DATA_Z0                 (0x2AU)
#define ICM42688_REG_INT_STATUS                   (0x2DU)
#define ICM42688_REG_INTF_CONFIG0                 (0x4CU)
#define ICM42688_REG_INTF_CONFIG1                 (0x4DU)
#define ICM42688_REG_PWR_MGMT0                    (0x4EU)
#define ICM42688_REG_GYRO_CONFIG0                 (0x4FU)
#define ICM42688_REG_ACCEL_CONFIG0                (0x50U)
#define ICM42688_REG_GYRO_ACCEL_CONFIG0           (0x52U)
#define ICM42688_REG_WHO_AM_I                     (0x75U)
#define ICM42688_REG_REG_BANK_SEL                 (0x76U)
#define ICM42688_REG_INTF_CONFIG4                 (0x7AU)

#define ICM42688_DEVICE_CONFIG_SOFT_RESET         (0x01U)
#define ICM42688_INTF_CONFIG4_SPI4W               (0x02U)
#define ICM42688_PWR_ACCEL_LN_MODE                (0x03U)
#define ICM42688_PWR_GYRO_LN_MODE                 (0x0CU)

#define ICM42688_ODR_50HZ                         (0x09U)
#define ICM42688_ODR_100HZ                        (0x08U)
#define ICM42688_ACCEL_FS_8G                      (0x20U)
#define ICM42688_GYRO_FS_1000DPS                  (0x20U)

#define ICM42688_YAW_DEFAULT_BIAS_DPS             (0.87547255f)
#define ICM42688_YAW_QUICK_BIAS_OFFSET_DPS        (-0.00514078f)
#define ICM42688_YAW_WARMUP_MS                    (1500U)
#define ICM42688_YAW_QUICK_CALIB_MS               (2000U)
#define ICM42688_YAW_CALIB_DELAY_MS               (5U)

#define WHO_AM_I                                  ICM42688_REG_WHO_AM_I
#define ICM_ACCEL_DATA_X1                         ICM42688_REG_ACCEL_DATA_X1
#define ICM_TEMP_OUTH_REG                         ICM42688_REG_TEMP_DATA1
#define ICM_GYRO_XOUTH_REG                        ICM42688_REG_GYRO_DATA_X1
#define ICM_ACCEL_XOUTH_REG                       ICM42688_REG_ACCEL_DATA_X1

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} ICM42688_RawData;

typedef struct {
    float gz_dps;
    float gzc_dps;
    float yaw_deg;
    int16_t gz_raw;
    uint8_t read_ok;
} ICM42688_YawData;

void ICM42688_BusInit(void);
uint8_t ICM42688_Init(void);
uint8_t ICM42688_GetInitError(void);
uint8_t ICM42688_ReadWhoAmI(void);
uint8_t ICM42688_ReadRaw(ICM42688_RawData *raw);
float ICM42688_AccelRawToG(int16_t raw);
float ICM42688_GyroRawToDps(int16_t raw);
uint8_t ICM42688_YawInit(float gyroZBiasDps);
uint8_t ICM42688_YawUpdate(uint32_t dt_ms);
void ICM42688_YawReset(float yawDeg);
void ICM42688_YawSetBias(float gyroZBiasDps);
float ICM42688_YawGetBias(void);
float ICM42688_YawGetDeg(void);
float ICM42688_YawGetGzDps(void);
float ICM42688_YawGetCorrectedGzDps(void);
int16_t ICM42688_YawGetGzRaw(void);
uint8_t ICM42688_YawGetData(ICM42688_YawData *data);
void ICM42688_YawFormatLine(char *buffer, uint16_t size);
void ICM42688_YawOnTimer(uint32_t dt_ms);
void ICM42688_YawTask(void);
void ICM42688_YawSendLine(void);
void ICM42688_YawUpdateAndSendLine(uint32_t dt_ms);
void ICM42688_YawQuickCalibrate(void);
void ICM42688_YawWaitForStartKey(uint32_t periodMs, uint32_t debounceMs);

void Test_SPI_Gyro(void);
unsigned char SPI_Gryo_Init(void);
unsigned char ICM_Set_Gyro_Fsr(unsigned char fsr);
unsigned char ICM_Set_Accel_Fsr(unsigned char fsr);
unsigned char ICM_Set_LPF(unsigned short lpf);
unsigned char ICM_Set_Rate(unsigned short rate);
short ICM_Get_Temperature(void);
unsigned char ICM_Get_Gyroscope(short *gx, short *gy, short *gz);
unsigned char ICM_Get_Accelerometer(short *ax, short *ay, short *az);
unsigned char ICM_Get_Raw_data(short *ax, short *ay, short *az,
    short *gx, short *gy, short *gz);
unsigned char ICM_Read_Len(unsigned char reg, unsigned char len, unsigned char *buf);
unsigned char ICM_Write_Byte(unsigned char reg, unsigned char value);
unsigned char ICM_Read_Byte(unsigned char reg);

#ifdef __cplusplus
}
#endif

#endif
