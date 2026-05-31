#include "ICM42688.h"
#include "delay.h"
#include "icm.h"
#include "key.h"
#include "uart.h"
#include <stdio.h>

#define ICM42688_READ_BIT              (0x80U)
#define ICM42688_SPI_TIMEOUT           (100000UL)
#define ICM42688_YAW_LINE_BUFFER_SIZE  (96U)

static uint8_t g_icm42688_init_error;
static uint8_t g_icm42688_last_whoami;
static DL_SPI_FRAME_FORMAT g_icm42688_frame_format =
    DL_SPI_FRAME_FORMAT_MOTO3_POL1_PHA0;
static float g_icm42688_yaw_deg;
static float g_icm42688_gz_dps;
static float g_icm42688_gzc_dps;
static float g_icm42688_gz_bias_dps;
static int16_t g_icm42688_gz_raw;
static uint8_t g_icm42688_yaw_read_ok;
static volatile uint32_t g_icm42688_yaw_pending_ms;

static uint8_t icm42688_spi_transfer(uint8_t tx)
{
    uint32_t timeout = ICM42688_SPI_TIMEOUT;

    while (DL_SPI_isTXFIFOFull(SPI_ICM42688_INST) && timeout--) {
    }

    DL_SPI_transmitData8(SPI_ICM42688_INST, tx);

    timeout = ICM42688_SPI_TIMEOUT;
    while (DL_SPI_isRXFIFOEmpty(SPI_ICM42688_INST) && timeout--) {
    }

    return DL_SPI_receiveData8(SPI_ICM42688_INST);
}

static void icm42688_drain_rx(void)
{
    uint8_t dummy;

    while (DL_SPI_receiveDataCheck8(SPI_ICM42688_INST, &dummy)) {
        (void) dummy;
    }
}

static uint8_t icm42688_write_verify(uint8_t reg, uint8_t value)
{
    for (uint8_t i = 0; i < 5; i++) {
        ICM_Write_Byte(reg, value);
        delay_ms(2);
        if (ICM_Read_Byte(reg) == value) {
            return 0;
        }
    }

    return ICM_Read_Byte(reg);
}

void ICM42688_BusInit(void)
{
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_SPI_ICM42688_IOMUX_SCLK, GPIO_SPI_ICM42688_IOMUX_SCLK_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_SPI_ICM42688_IOMUX_PICO, GPIO_SPI_ICM42688_IOMUX_PICO_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_SPI_ICM42688_IOMUX_POCI, GPIO_SPI_ICM42688_IOMUX_POCI_FUNC);

    DL_GPIO_initDigitalOutput(ICM42688_CS_IOMUX);
    DL_GPIO_setPins(ICM42688_PORT, ICM42688_CS_PIN);
    DL_GPIO_enableOutput(ICM42688_PORT, ICM42688_CS_PIN);

    DL_SPI_reset(SPI_ICM42688_INST);
    DL_SPI_enablePower(SPI_ICM42688_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    const DL_SPI_ClockConfig clockConfig = {
        .clockSel = DL_SPI_CLOCK_BUSCLK,
        .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1,
    };

    const DL_SPI_Config spiConfig = {
        .mode = DL_SPI_MODE_CONTROLLER,
        .frameFormat = g_icm42688_frame_format,
        .parity = DL_SPI_PARITY_NONE,
        .dataSize = DL_SPI_DATA_SIZE_8,
        .bitOrder = DL_SPI_BIT_ORDER_MSB_FIRST,
        .chipSelectPin = DL_SPI_CHIP_SELECT_NONE,
    };

    DL_SPI_setClockConfig(SPI_ICM42688_INST, (DL_SPI_ClockConfig *) &clockConfig);
    DL_SPI_init(SPI_ICM42688_INST, (DL_SPI_Config *) &spiConfig);
    DL_SPI_setBitRateSerialClockDivider(SPI_ICM42688_INST, 3);
    DL_SPI_setFIFOThreshold(SPI_ICM42688_INST,
        DL_SPI_RX_FIFO_LEVEL_ONE_FRAME, DL_SPI_TX_FIFO_LEVEL_ONE_FRAME);
    DL_SPI_enable(SPI_ICM42688_INST);

    icm42688_drain_rx();
}

static void icm42688_set_frame_format(DL_SPI_FRAME_FORMAT frameFormat)
{
    g_icm42688_frame_format = frameFormat;
    ICM42688_BusInit();
    delay_ms(2);
}

uint8_t ICM42688_ReadWhoAmI(void)
{
    return ICM_Read_Byte(ICM42688_REG_WHO_AM_I);
}

uint8_t ICM42688_GetInitError(void)
{
    return g_icm42688_init_error;
}

uint8_t ICM42688_Init(void)
{
    static const DL_SPI_FRAME_FORMAT frameFormats[] = {
        DL_SPI_FRAME_FORMAT_MOTO3_POL1_PHA0,
        DL_SPI_FRAME_FORMAT_MOTO3_POL1_PHA1,
        DL_SPI_FRAME_FORMAT_MOTO3_POL0_PHA0,
        DL_SPI_FRAME_FORMAT_MOTO3_POL0_PHA1,
    };
    uint8_t whoami;
    uint8_t pwr;

    g_icm42688_init_error = ICM42688_INIT_OK;
    g_icm42688_last_whoami = 0U;

    for (uint8_t i = 0U; i < sizeof(frameFormats) / sizeof(frameFormats[0]); i++) {
        icm42688_set_frame_format(frameFormats[i]);
        delay_ms(10);

        whoami = ICM42688_ReadWhoAmI();
        g_icm42688_last_whoami = whoami;
        if (whoami == ICM42688_WHO_AM_I_VALUE) {
            break;
        }
    }

    if (g_icm42688_last_whoami != ICM42688_WHO_AM_I_VALUE) {
        g_icm42688_init_error = ICM42688_INIT_WHOAMI_FAILED;
        return g_icm42688_init_error;
    }

    ICM_Write_Byte(ICM42688_REG_REG_BANK_SEL, 0x00);
    ICM_Write_Byte(ICM42688_REG_DEVICE_CONFIG, ICM42688_DEVICE_CONFIG_SOFT_RESET);
    delay_ms(100);

    ICM_Write_Byte(ICM42688_REG_REG_BANK_SEL, 0x01);
    ICM_Write_Byte(ICM42688_REG_INTF_CONFIG4, ICM42688_INTF_CONFIG4_SPI4W);
    ICM_Write_Byte(ICM42688_REG_REG_BANK_SEL, 0x00);
    delay_ms(2);

    whoami = ICM42688_ReadWhoAmI();
    if (whoami != ICM42688_WHO_AM_I_VALUE) {
        g_icm42688_last_whoami = whoami;
        g_icm42688_init_error = ICM42688_INIT_WHOAMI_FAILED;
        return g_icm42688_init_error;
    }

    ICM_Write_Byte(ICM42688_REG_REG_BANK_SEL, 0x00);
    delay_ms(2);
    ICM_Write_Byte(ICM42688_REG_FIFO_CONFIG, 0x00);
    delay_ms(2);
    ICM_Write_Byte(ICM42688_REG_ACCEL_CONFIG0,
        ICM42688_ACCEL_FS_8G | ICM42688_ODR_50HZ);
    delay_ms(2);
    ICM_Write_Byte(ICM42688_REG_GYRO_CONFIG0,
        ICM42688_GYRO_FS_1000DPS | ICM42688_ODR_100HZ);
    delay_ms(2);
    ICM_Write_Byte(ICM42688_REG_GYRO_ACCEL_CONFIG0, 0x44);
    delay_ms(2);

    pwr = ICM_Read_Byte(ICM42688_REG_PWR_MGMT0);
    pwr = (pwr & ~(ICM42688_PWR_ACCEL_LN_MODE | ICM42688_PWR_GYRO_LN_MODE)) |
        ICM42688_PWR_ACCEL_LN_MODE;
    ICM_Write_Byte(ICM42688_REG_PWR_MGMT0, pwr);
    delay_ms(2);

    if (icm42688_write_verify(ICM42688_REG_PWR_MGMT0,
            pwr | ICM42688_PWR_GYRO_LN_MODE) != 0) {
        g_icm42688_init_error = ICM42688_INIT_PWR_WRITE_FAILED;
        return g_icm42688_init_error;
    }

    delay_ms(100);

    return 0;
}

uint8_t ICM42688_ReadRaw(ICM42688_RawData *raw)
{
    uint8_t data[12];
    uint8_t ret;

    if (raw == 0) {
        return 1;
    }

    ICM42688_CS_LOW();
    icm42688_drain_rx();
    (void) icm42688_spi_transfer(ICM42688_REG_ACCEL_DATA_X1 | ICM42688_READ_BIT);
    for (uint8_t i = 0; i < sizeof(data); i++) {
        data[i] = icm42688_spi_transfer(0xFF);
    }
    while (DL_SPI_isBusy(SPI_ICM42688_INST)) {
    }
    ICM42688_CS_HIGH();

    raw->ax = (int16_t) (((uint16_t) data[0] << 8) | data[1]);
    raw->ay = (int16_t) (((uint16_t) data[2] << 8) | data[3]);
    raw->az = (int16_t) (((uint16_t) data[4] << 8) | data[5]);
    raw->gx = (int16_t) (((uint16_t) data[6] << 8) | data[7]);
    raw->gy = (int16_t) (((uint16_t) data[8] << 8) | data[9]);
    raw->gz = (int16_t) (((uint16_t) data[10] << 8) | data[11]);

    ret = 0;
    return ret;
}

float ICM42688_AccelRawToG(int16_t raw)
{
    return (float) raw / 4096.0f;
}

float ICM42688_GyroRawToDps(int16_t raw)
{
    return (float) raw / 32.8f;
}

uint8_t ICM42688_YawInit(float gyroZBiasDps)
{
    uint8_t ret = ICM42688_Init();

    g_icm42688_gz_bias_dps = gyroZBiasDps;
    g_icm42688_yaw_deg = 0.0f;
    g_icm42688_gz_dps = 0.0f;
    g_icm42688_gzc_dps = 0.0f;
    g_icm42688_gz_raw = 0;
    g_icm42688_yaw_read_ok = 0U;
    g_icm42688_yaw_pending_ms = 0U;
    Filter_Init();
    ICM_YawFilterReset(0.0f);
    ICM_SetYawMode(ICM_YAW_MODE_DEADZONE);

    return ret;
}

uint8_t ICM42688_YawUpdate(uint32_t dt_ms)
{
    ICM42688_RawData raw;
    uint8_t ret = ICM42688_ReadRaw(&raw);
    float correctedGz;

    if (ret != 0U) {
        g_icm42688_yaw_read_ok = 0U;
        return ret;
    }

    g_icm42688_gz_raw = raw.gz;
    g_icm42688_gz_dps = ICM42688_GyroRawToDps(raw.gz);
    correctedGz = g_icm42688_gz_dps - g_icm42688_gz_bias_dps;
    g_icm42688_yaw_deg =
        ICM_YawFilterUpdate(correctedGz, ((float) dt_ms / 1000.0f));
    g_icm42688_gzc_dps = ICM_YawFilterGetFilteredRateDps();

    g_icm42688_yaw_read_ok = 1U;
    return 0U;
}

void ICM42688_YawReset(float yawDeg)
{
    g_icm42688_yaw_deg = yawDeg;
    g_icm42688_gzc_dps = 0.0f;
    ICM_YawFilterReset(yawDeg);
}

void ICM42688_YawSetBias(float gyroZBiasDps)
{
    g_icm42688_gz_bias_dps = gyroZBiasDps;
}

float ICM42688_YawGetBias(void)
{
    return g_icm42688_gz_bias_dps;
}

float ICM42688_YawGetDeg(void)
{
    return g_icm42688_yaw_deg;
}

float ICM42688_YawGetGzDps(void)
{
    return g_icm42688_gz_dps;
}

float ICM42688_YawGetCorrectedGzDps(void)
{
    return g_icm42688_gzc_dps;
}

int16_t ICM42688_YawGetGzRaw(void)
{
    return g_icm42688_gz_raw;
}

uint8_t ICM42688_YawGetData(ICM42688_YawData *data)
{
    if (data == 0) {
        return 1U;
    }

    data->gz_dps = g_icm42688_gz_dps;
    data->gzc_dps = g_icm42688_gzc_dps;
    data->yaw_deg = g_icm42688_yaw_deg;
    data->gz_raw = g_icm42688_gz_raw;
    data->read_ok = g_icm42688_yaw_read_ok;

    return 0U;
}

void ICM42688_YawFormatLine(char *buffer, uint16_t size)
{
    ICM42688_YawData data;

    if ((buffer == 0) || (size == 0U)) {
        return;
    }

    (void) ICM42688_YawGetData(&data);
    snprintf(buffer, size,
        "GYRO,gz=%.6f,gzc=%.6f,yaw=%.5f,raw=%d,ok=%u\r\n",
        (double) data.gz_dps, (double) data.gzc_dps, (double) data.yaw_deg,
        (int) data.gz_raw, (unsigned int) data.read_ok);
}

void ICM42688_YawOnTimer(uint32_t dt_ms)
{
    g_icm42688_yaw_pending_ms += dt_ms;
}

void ICM42688_YawTask(void)
{
    uint32_t pendingMs;

    __disable_irq();
    pendingMs = g_icm42688_yaw_pending_ms;
    g_icm42688_yaw_pending_ms = 0U;
    __enable_irq();

    if (pendingMs != 0U) {
        (void) ICM42688_YawUpdate(pendingMs);
    }
}

void ICM42688_YawSendLine(void)
{
    char gyroLine[ICM42688_YAW_LINE_BUFFER_SIZE];

    ICM42688_YawFormatLine(gyroLine, sizeof(gyroLine));
    UART_SendString(gyroLine);
}

void ICM42688_YawUpdateAndSendLine(uint32_t dt_ms)
{
    (void) ICM42688_YawUpdate(dt_ms);
    ICM42688_YawSendLine();
}

static void ICM42688_YawSendCalibStatus(const char *stage,
    uint16_t samples, float biasDps)
{
    char line[80];

    snprintf(line, sizeof(line), "GYRO_CAL,%s,samples=%u,bias=%.6f\r\n",
        stage, (unsigned int) samples, (double) biasDps);
    UART_SendString(line);
}

static float ICM42688_YawCalibrateGyroZ(uint16_t sampleCount,
    uint16_t delayMs, const char *stage)
{
    ICM42688_RawData raw;
    int32_t sum = 0;
    uint16_t valid = 0U;
    uint16_t reportStep = sampleCount / 4U;
    float biasDps;

    if (reportStep == 0U) {
        reportStep = 1U;
    }

    for (uint16_t i = 0U; i < sampleCount; i++) {
        UART_Task();
        if (ICM42688_ReadRaw(&raw) == 0U) {
            sum += raw.gz;
            valid++;
        }

        if (((i + 1U) % reportStep) == 0U) {
            biasDps = (valid == 0U) ? 0.0f :
                ((float) sum / (float) valid) / 32.8f;
            ICM42688_YawSendCalibStatus(stage, valid, biasDps);
        }

        delay_ms(delayMs);
    }

    if (valid == 0U) {
        return 0.0f;
    }

    return ((float) sum / (float) valid) / 32.8f;
}

static void ICM42688_YawWarmup(uint16_t warmupMs, uint16_t delayMs)
{
    ICM42688_RawData raw;
    uint16_t samples;

    if (delayMs == 0U) {
        delayMs = 1U;
    }

    samples = (uint16_t) (warmupMs / delayMs);
    for (uint16_t i = 0U; i < samples; i++) {
        UART_Task();
        (void) ICM42688_ReadRaw(&raw);
        delay_ms(delayMs);
    }
}

void ICM42688_YawQuickCalibrate(void)
{
    const uint16_t quickSamples =
        (uint16_t) (ICM42688_YAW_QUICK_CALIB_MS /
            ICM42688_YAW_CALIB_DELAY_MS);
    float biasDps;
    float finalBiasDps;

    UART_SendString("GYRO_CAL,KEEP_STILL\r\n");
    UART_SendString("GYRO_CAL,WARMUP\r\n");
    ICM42688_YawWarmup(ICM42688_YAW_WARMUP_MS,
        ICM42688_YAW_CALIB_DELAY_MS);

    biasDps = ICM42688_YawCalibrateGyroZ(quickSamples,
        ICM42688_YAW_CALIB_DELAY_MS, "QUICK");
    finalBiasDps = biasDps + ICM42688_YAW_QUICK_BIAS_OFFSET_DPS;
    ICM42688_YawSetBias(finalBiasDps);
    ICM42688_YawReset(0.0f);
    ICM42688_YawSendCalibStatus("QUICK_DONE", quickSamples, finalBiasDps);
}

void ICM42688_YawWaitForStartKey(uint32_t periodMs, uint32_t debounceMs)
{
    uint32_t printMs = periodMs;
    uint32_t stableMs = 0U;

    while (stableMs < debounceMs) {
        UART_Task();
        ICM42688_YawTask();

        if (printMs >= periodMs) {
            printMs = 0U;
            ICM42688_YawSendLine();
        }

        if (Key_IsReleased() != 0U) {
            stableMs++;
        } else {
            stableMs = 0U;
        }

        delay_ms(1U);
        printMs++;
    }

    stableMs = 0U;
    while (stableMs < debounceMs) {
        UART_Task();
        ICM42688_YawTask();

        if (printMs >= periodMs) {
            printMs = 0U;
            ICM42688_YawSendLine();
        }

        if (Key_IsPressed() != 0U) {
            stableMs++;
        } else {
            stableMs = 0U;
        }

        delay_ms(1U);
        printMs++;
    }
}

unsigned char SPI_Gryo_Init(void)
{
    uint8_t ret = ICM42688_Init();

    if (ret == ICM42688_INIT_OK) {
        return ICM42688_WHO_AM_I_VALUE;
    }

    return ret;
}

unsigned char ICM_Set_Gyro_Fsr(unsigned char fsr)
{
    static const uint8_t fsrMap[] = {
        0x60, 0x40, 0x20, 0x00
    };
    uint8_t odr;

    if (fsr > 3) {
        fsr = 3;
    }

    odr = ICM_Read_Byte(ICM42688_REG_GYRO_CONFIG0) & 0x0F;
    return ICM_Write_Byte(ICM42688_REG_GYRO_CONFIG0, fsrMap[fsr] | odr);
}

unsigned char ICM_Set_Accel_Fsr(unsigned char fsr)
{
    static const uint8_t fsrMap[] = {
        0x60, 0x40, 0x20, 0x00
    };
    uint8_t odr;

    if (fsr > 3) {
        fsr = 3;
    }

    odr = ICM_Read_Byte(ICM42688_REG_ACCEL_CONFIG0) & 0x0F;
    return ICM_Write_Byte(ICM42688_REG_ACCEL_CONFIG0, fsrMap[fsr] | odr);
}

unsigned char ICM_Set_LPF(unsigned short lpf)
{
    (void) lpf;
    return 0;
}

unsigned char ICM_Set_Rate(unsigned short rate)
{
    uint8_t odr;

    if (rate >= 1000) {
        odr = 0x06;
    } else if (rate >= 200) {
        odr = 0x07;
    } else if (rate >= 100) {
        odr = 0x08;
    } else if (rate >= 50) {
        odr = 0x09;
    } else if (rate >= 25) {
        odr = 0x0A;
    } else {
        odr = 0x0B;
    }

    ICM_Write_Byte(ICM42688_REG_ACCEL_CONFIG0,
        (ICM_Read_Byte(ICM42688_REG_ACCEL_CONFIG0) & 0xF0) | odr);
    ICM_Write_Byte(ICM42688_REG_GYRO_CONFIG0,
        (ICM_Read_Byte(ICM42688_REG_GYRO_CONFIG0) & 0xF0) | odr);

    return 0;
}

short ICM_Get_Temperature(void)
{
    uint8_t buf[3];
    int16_t raw;
    float temp;

    ICM_Read_Len(ICM42688_REG_TEMP_DATA1, 2, buf);
    raw = (int16_t) (((uint16_t) buf[1] << 8) | buf[2]);
    temp = ((float) raw / 132.48f) + 25.0f;

    return (short) (temp * 100.0f);
}

unsigned char ICM_Get_Gyroscope(short *gx, short *gy, short *gz)
{
    uint8_t buf[7];
    uint8_t ret;

    if ((gx == 0) || (gy == 0) || (gz == 0)) {
        return 1;
    }

    ret = ICM_Read_Len(ICM42688_REG_GYRO_DATA_X1, 6, buf);
    *gx = (short) (((uint16_t) buf[1] << 8) | buf[2]);
    *gy = (short) (((uint16_t) buf[3] << 8) | buf[4]);
    *gz = (short) (((uint16_t) buf[5] << 8) | buf[6]);

    return ret;
}

unsigned char ICM_Get_Accelerometer(short *ax, short *ay, short *az)
{
    uint8_t buf[7];
    uint8_t ret;

    if ((ax == 0) || (ay == 0) || (az == 0)) {
        return 1;
    }

    ret = ICM_Read_Len(ICM42688_REG_ACCEL_DATA_X1, 6, buf);
    *ax = (short) (((uint16_t) buf[1] << 8) | buf[2]);
    *ay = (short) (((uint16_t) buf[3] << 8) | buf[4]);
    *az = (short) (((uint16_t) buf[5] << 8) | buf[6]);

    return ret;
}

unsigned char ICM_Get_Raw_data(short *ax, short *ay, short *az,
    short *gx, short *gy, short *gz)
{
    ICM42688_RawData raw;
    uint8_t ret;

    if ((ax == 0) || (ay == 0) || (az == 0) ||
        (gx == 0) || (gy == 0) || (gz == 0)) {
        return 1;
    }

    ret = ICM42688_ReadRaw(&raw);
    *ax = raw.ax;
    *ay = raw.ay;
    *az = raw.az;
    *gx = raw.gx;
    *gy = raw.gy;
    *gz = raw.gz;

    return ret;
}

unsigned char ICM_Read_Len(unsigned char reg, unsigned char len, unsigned char *buf)
{
    if ((buf == 0) || (len == 0)) {
        return 1;
    }

    ICM42688_CS_LOW();
    icm42688_drain_rx();
    buf[0] = reg | ICM42688_READ_BIT;
    (void) icm42688_spi_transfer(buf[0]);
    for (uint8_t i = 1; i <= len; i++) {
        buf[i] = icm42688_spi_transfer(0xFF);
    }
    while (DL_SPI_isBusy(SPI_ICM42688_INST)) {
    }
    ICM42688_CS_HIGH();

    return 0;
}

unsigned char ICM_Write_Byte(unsigned char reg, unsigned char value)
{
    ICM42688_CS_LOW();
    icm42688_drain_rx();
    (void) icm42688_spi_transfer(reg & 0x7F);
    (void) icm42688_spi_transfer(value);
    while (DL_SPI_isBusy(SPI_ICM42688_INST)) {
    }
    ICM42688_CS_HIGH();

    return 0;
}

unsigned char ICM_Read_Byte(unsigned char reg)
{
    uint8_t value;

    ICM42688_CS_LOW();
    icm42688_drain_rx();
    (void) icm42688_spi_transfer(reg | ICM42688_READ_BIT);
    value = icm42688_spi_transfer(0xFF);
    while (DL_SPI_isBusy(SPI_ICM42688_INST)) {
    }
    ICM42688_CS_HIGH();

    return value;
}

void Test_SPI_Gyro(void)
{
    (void) SPI_Gryo_Init();
}
