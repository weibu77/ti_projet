#include "gray_sensor.h"

#include <string.h>
#include "delay.h"

#define GRAY_SENSOR_DIRECTION_REVERSE   (1U)
#define GRAY_ADC_WAIT_TIMEOUT           (100000UL)
#define GRAY_SENSOR_CAL_MAGIC           (0x47524159UL)
#define GRAY_SENSOR_CAL_VERSION         (1U)
#define GRAY_SENSOR_CAL_CRC_INIT        (2166136261UL)
#define GRAY_SENSOR_CAL_CRC_PRIME       (16777619UL)

volatile uint16_t Gray_ADCValue[GRAY_ADC_SAMPLE_COUNT];
GraySensor g_gray_sensor;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t channels;
    uint16_t white[GRAY_SENSOR_CHANNELS];
    uint16_t black[GRAY_SENSOR_CHANNELS];
    uint32_t crc;
    uint32_t reserved;
} GraySensorCalibrationRecord;

typedef char GraySensorCalibrationRecordSizeCheck[
    (sizeof(GraySensorCalibrationRecord) % 8U) == 0U ? 1 : -1];
typedef char GraySensorCalibrationSectorSizeCheck[
    (GRAY_SENSOR_CAL_FLASH_SIZE >= sizeof(GraySensorCalibrationRecord)) ?
        1 : -1];

static const uint16_t s_gray_default_white[GRAY_SENSOR_CHANNELS] = {
    1059U, 1196U, 1235U, 1245U, 1205U, 1084U, 1273U, 1177U
};

static const uint16_t s_gray_default_black[GRAY_SENSOR_CHANNELS] = {
    44U, 53U, 52U, 72U, 56U, 67U, 109U, 53U
};

static uint8_t g_gray_last_error;
static volatile uint8_t g_gray_dma_done;

static uint32_t GraySensor_CrcUpdateByte(uint32_t crc, uint8_t value)
{
    crc ^= value;
    return crc * GRAY_SENSOR_CAL_CRC_PRIME;
}

static uint32_t GraySensor_CrcUpdate16(uint32_t crc, uint16_t value)
{
    crc = GraySensor_CrcUpdateByte(crc, (uint8_t) (value & 0xFFU));
    return GraySensor_CrcUpdateByte(crc, (uint8_t) (value >> 8U));
}

static uint32_t GraySensor_CrcUpdate32(uint32_t crc, uint32_t value)
{
    crc = GraySensor_CrcUpdateByte(crc, (uint8_t) (value & 0xFFUL));
    crc = GraySensor_CrcUpdateByte(crc, (uint8_t) ((value >> 8U) & 0xFFUL));
    crc = GraySensor_CrcUpdateByte(crc, (uint8_t) ((value >> 16U) & 0xFFUL));
    return GraySensor_CrcUpdateByte(crc,
        (uint8_t) ((value >> 24U) & 0xFFUL));
}

static uint32_t GraySensor_CalcCalibrationCrc(
    const GraySensorCalibrationRecord *record)
{
    uint32_t crc = GRAY_SENSOR_CAL_CRC_INIT;

    crc = GraySensor_CrcUpdate32(crc, record->magic);
    crc = GraySensor_CrcUpdate16(crc, record->version);
    crc = GraySensor_CrcUpdate16(crc, record->channels);
    for (uint8_t i = 0U; i < GRAY_SENSOR_CHANNELS; i++) {
        crc = GraySensor_CrcUpdate16(crc, record->white[i]);
    }
    for (uint8_t i = 0U; i < GRAY_SENSOR_CHANNELS; i++) {
        crc = GraySensor_CrcUpdate16(crc, record->black[i]);
    }

    return crc;
}

static void GraySensor_BuildCalibrationRecord(
    GraySensorCalibrationRecord *record, const uint16_t *white,
    const uint16_t *black)
{
    memset(record, 0xFF, sizeof(*record));
    record->magic = GRAY_SENSOR_CAL_MAGIC;
    record->version = GRAY_SENSOR_CAL_VERSION;
    record->channels = GRAY_SENSOR_CHANNELS;
    memcpy(record->white, white, sizeof(record->white));
    memcpy(record->black, black, sizeof(record->black));
    record->crc = GraySensor_CalcCalibrationCrc(record);
}

static uint8_t GraySensor_CalibrationRecordIsValid(
    const GraySensorCalibrationRecord *record)
{
    if ((record->magic != GRAY_SENSOR_CAL_MAGIC) ||
        (record->version != GRAY_SENSOR_CAL_VERSION) ||
        (record->channels != GRAY_SENSOR_CHANNELS)) {
        return 0U;
    }

    return (uint8_t) (record->crc == GraySensor_CalcCalibrationCrc(record));
}

static void GraySensor_AddressWrite(uint8_t channel)
{
    if (channel & 0x01U) {
        DL_GPIO_clearPins(Gray_Address_PORT, Gray_Address_PIN_0_PIN);
    } else {
        DL_GPIO_setPins(Gray_Address_PORT, Gray_Address_PIN_0_PIN);
    }

    if (channel & 0x02U) {
        DL_GPIO_clearPins(Gray_Address_PORT, Gray_Address_PIN_1_PIN);
    } else {
        DL_GPIO_setPins(Gray_Address_PORT, Gray_Address_PIN_1_PIN);
    }

    if (channel & 0x04U) {
        DL_GPIO_clearPins(Gray_Address_PORT, Gray_Address_PIN_2_PIN);
    } else {
        DL_GPIO_setPins(Gray_Address_PORT, Gray_Address_PIN_2_PIN);
    }
}

void GraySensor_ADCStart(void)
{
    DL_ADC12_stopConversion(ADC12_0_INST);
    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);
    DL_ADC12_disableDMA(ADC12_0_INST);
    DL_ADC12_clearInterruptStatus(ADC12_0_INST,
        DL_ADC12_INTERRUPT_DMA_DONE | DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);

    memset((void *) Gray_ADCValue, 0, sizeof(Gray_ADCValue));
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID,
        DL_ADC12_getMemResultAddress(ADC12_0_INST, DL_ADC12_MEM_IDX_0));
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID,
        (uint32_t) &Gray_ADCValue[0]);
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, GRAY_ADC_SAMPLE_COUNT);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);

    g_gray_dma_done = 0U;
    DL_ADC12_enableInterrupt(ADC12_0_INST, DL_ADC12_INTERRUPT_DMA_DONE);
    DL_ADC12_enableDMA(ADC12_0_INST);
    DL_ADC12_startConversion(ADC12_0_INST);
}

uint16_t GraySensor_ReadADCMean(uint8_t count)
{
    uint32_t sum = 0;
    uint32_t timeout = GRAY_ADC_WAIT_TIMEOUT;

    if ((count == 0U) || (count > GRAY_ADC_SAMPLE_COUNT)) {
        count = GRAY_ADC_SAMPLE_COUNT;
    }

    GraySensor_ADCStart();

    while ((g_gray_dma_done == 0U) &&
        ((DL_ADC12_getRawInterruptStatus(ADC12_0_INST,
            DL_ADC12_INTERRUPT_DMA_DONE) & DL_ADC12_INTERRUPT_DMA_DONE) == 0U) &&
        (timeout > 0U)) {
        timeout--;
    }

    DL_ADC12_stopConversion(ADC12_0_INST);
    DL_ADC12_disableDMA(ADC12_0_INST);
    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);

    if (timeout == 0U) {
        g_gray_last_error = 1U;
        return 0U;
    }

    DL_ADC12_clearInterruptStatus(ADC12_0_INST,
        DL_ADC12_INTERRUPT_DMA_DONE | DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);

    for (uint8_t i = 0U; i < count; i++) {
        sum += Gray_ADCValue[i];
    }

    g_gray_last_error = 0U;
    return (uint16_t) (sum / count);
}

void GraySensor_Init(GraySensor *sensor, const uint16_t *white,
    const uint16_t *black)
{
    memset(sensor, 0, sizeof(*sensor));
    sensor->adc_max = 4095U;

    for (uint8_t i = 0U; i < GRAY_SENSOR_CHANNELS; i++) {
        uint16_t w = white[i];
        uint16_t b = black[i];

        if (b > w) {
            uint16_t temp = w;
            w = b;
            b = temp;
        }

        sensor->calibrated_white[i] = w;
        sensor->calibrated_black[i] = b;
        sensor->gray_white[i] = (uint16_t) (((uint32_t) w * 2U + b) / 3U);
        sensor->gray_black[i] = (uint16_t) (((uint32_t) w + (uint32_t) b * 2U) / 3U);

        if (w != b) {
            sensor->normal_factor[i] = (float) sensor->adc_max / (float) (w - b);
        }
    }

    sensor->ready = 1U;
    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);
    DL_ADC12_disableDMA(ADC12_0_INST);
}

void GraySensor_InitDefault(void)
{
    uint16_t white[GRAY_SENSOR_CHANNELS];
    uint16_t black[GRAY_SENSOR_CHANNELS];

    if (GraySensor_LoadSavedCalibration(white, black) != 0U) {
        GraySensor_Init(&g_gray_sensor, white, black);
    } else {
        GraySensor_Init(&g_gray_sensor, s_gray_default_white,
            s_gray_default_black);
    }
}

uint8_t GraySensor_LoadSavedCalibration(uint16_t *white, uint16_t *black)
{
    const GraySensorCalibrationRecord *record =
        (const GraySensorCalibrationRecord *) GRAY_SENSOR_CAL_FLASH_ADDRESS;

    if ((white == NULL) || (black == NULL) ||
        (GraySensor_CalibrationRecordIsValid(record) == 0U)) {
        return 0U;
    }

    memcpy(white, record->white, sizeof(record->white));
    memcpy(black, record->black, sizeof(record->black));
    return 1U;
}

uint8_t GraySensor_SaveCalibration(const uint16_t *white,
    const uint16_t *black)
{
    GraySensorCalibrationRecord record;
    uint32_t *recordWords = (uint32_t *) &record;
    DL_FLASHCTL_COMMAND_STATUS status;
    uint32_t primask;
    uint32_t address = GRAY_SENSOR_CAL_FLASH_ADDRESS;
    uint8_t ok = 1U;

    if ((white == NULL) || (black == NULL)) {
        return 0U;
    }

    GraySensor_BuildCalibrationRecord(&record, white, black);

    primask = __get_PRIMASK();
    __disable_irq();

    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(FLASHCTL, GRAY_SENSOR_CAL_FLASH_ADDRESS,
        DL_FLASHCTL_REGION_SELECT_MAIN);
    status = DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL,
        GRAY_SENSOR_CAL_FLASH_ADDRESS, DL_FLASHCTL_COMMAND_SIZE_SECTOR);

    if (status == DL_FLASHCTL_COMMAND_STATUS_FAILED) {
        ok = 0U;
    }

    for (uint32_t offset = 0U;
         (ok != 0U) && (offset < sizeof(GraySensorCalibrationRecord));
         offset += 8U) {
        DL_FlashCTL_executeClearStatus(FLASHCTL);
        DL_FlashCTL_unprotectSector(FLASHCTL, address,
            DL_FLASHCTL_REGION_SELECT_MAIN);
#ifdef __MSPM0_HAS_ECC__
        status = DL_FlashCTL_programMemoryFromRAM64WithECCGenerated(
            FLASHCTL, address, &recordWords[offset / 4U]);
#else
        status = DL_FlashCTL_programMemoryFromRAM64(
            FLASHCTL, address, &recordWords[offset / 4U]);
#endif
        if (status == DL_FLASHCTL_COMMAND_STATUS_FAILED) {
            ok = 0U;
        }
        address += 8U;
    }

    DL_FlashCTL_protectSector(FLASHCTL, GRAY_SENSOR_CAL_FLASH_ADDRESS,
        DL_FLASHCTL_REGION_SELECT_MAIN);

    if (primask == 0U) {
        __enable_irq();
    }

    if ((ok == 0U) || (GraySensor_LoadSavedCalibration(
            (uint16_t *) record.white, (uint16_t *) record.black) == 0U)) {
        return 0U;
    }

    return 1U;
}

uint8_t GraySensor_Task(GraySensor *sensor)
{
    uint8_t digital = sensor->digital;

    for (uint8_t i = 0U; i < GRAY_SENSOR_CHANNELS; i++) {
        uint8_t index;
        uint16_t value;

        GraySensor_AddressWrite(i);
        delay_us(10);

        value = GraySensor_ReadADCMean(GRAY_ADC_SAMPLE_COUNT);
        index = GRAY_SENSOR_DIRECTION_REVERSE ? (uint8_t) (7U - i) : i;
        sensor->analog[index] = value;

        if (value > sensor->gray_white[index]) {
            digital |= (uint8_t) (1U << index);
        } else if (value < sensor->gray_black[index]) {
            digital &= (uint8_t) ~(1U << index);
        }

        if (sensor->ready && (sensor->normal_factor[index] > 0.0f) &&
            (value > sensor->calibrated_black[index])) {
            uint32_t normalized = (uint32_t)
                (((float) (value - sensor->calibrated_black[index])) *
                sensor->normal_factor[index]);

            if (normalized > sensor->adc_max) {
                normalized = sensor->adc_max;
            }
            sensor->normalized[index] = (uint16_t) normalized;
        } else {
            sensor->normalized[index] = 0U;
        }
    }

    sensor->digital = digital;
    sensor->last_error = g_gray_last_error;
    return (sensor->last_error == 0U);
}

uint8_t GraySensor_GetDigital(const GraySensor *sensor)
{
    return sensor->digital;
}

uint8_t GraySensor_GetAnalog(const GraySensor *sensor, uint16_t *result)
{
    memcpy(result, sensor->analog, sizeof(sensor->analog));
    return sensor->ready;
}

uint8_t GraySensor_GetNormalized(const GraySensor *sensor, uint16_t *result)
{
    memcpy(result, sensor->normalized, sizeof(sensor->normalized));
    return sensor->ready;
}

uint8_t GraySensor_GetLastError(void)
{
    return g_gray_last_error;
}

void GraySensor_ADCIRQHandler(void)
{
    switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST)) {
    case DL_ADC12_IIDX_DMA_DONE:
        g_gray_dma_done = 1U;
        break;
    default:
        break;
    }
}
