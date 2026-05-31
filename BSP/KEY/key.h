#ifndef __KEY_H__
#define __KEY_H__

#include "ti_msp_dl_config.h"
#include "delay.h"
#include <stdint.h>

static inline uint8_t Key_IsPressed(void)
{
#if defined(Key_Key1_PIN)
    return (uint8_t) ((DL_GPIO_readPins(Key_PORT, Key_Key1_PIN) & Key_Key1_PIN) == 0U);
#else
    return (uint8_t) ((DL_GPIO_readPins(Key_PORT, Key_User_PIN) & Key_User_PIN) == 0U);
#endif
}

static inline uint8_t Key_UserIsPressed(void)
{
#if defined(Key_User_PIN)
    return (uint8_t) ((DL_GPIO_readPins(Key_PORT, Key_User_PIN) & Key_User_PIN) == 0U);
#else
    return 0U;
#endif
}

static inline uint8_t Key_Key1IsPressed(void)
{
#if defined(Key_Key1_PIN)
    return (uint8_t) ((DL_GPIO_readPins(Key_PORT, Key_Key1_PIN) & Key_Key1_PIN) == 0U);
#else
    return Key_IsPressed();
#endif
}

static inline uint8_t Key_Key2IsPressed(void)
{
#if defined(Key_Key2_PIN)
    return (uint8_t) ((DL_GPIO_readPins(Key_PORT, Key_Key2_PIN) & Key_Key2_PIN) == 0U);
#else
    return 0U;
#endif
}

static inline uint8_t Key_Key3IsPressed(void)
{
#if defined(Key_Key3_PIN)
    return (uint8_t) ((DL_GPIO_readPins(Key_PORT, Key_Key3_PIN) & Key_Key3_PIN) == 0U);
#else
    return 0U;
#endif
}

static inline uint8_t Key_Key4IsPressed(void)
{
#if defined(Key_Key4_PIN)
    return (uint8_t) ((DL_GPIO_readPins(Key_PORT, Key_Key4_PIN) & Key_Key4_PIN) == 0U);
#else
    return 0U;
#endif
}

static inline uint8_t Key_Key5IsPressed(void)
{
#if defined(Key_Key5_PIN)
    return (uint8_t) ((DL_GPIO_readPins(Key_PORT, Key_Key5_PIN) & Key_Key5_PIN) == 0U);
#else
    return 0U;
#endif
}

static inline uint8_t Key_IsReleased(void)
{
    return (uint8_t) (Key_IsPressed() == 0U);
}

static inline uint8_t Key_AllReleased(void)
{
    return (uint8_t) ((Key_UserIsPressed() == 0U) &&
        (Key_Key1IsPressed() == 0U) &&
        (Key_Key2IsPressed() == 0U) &&
        (Key_Key3IsPressed() == 0U) &&
        (Key_Key4IsPressed() == 0U) &&
        (Key_Key5IsPressed() == 0U));
}

static inline void Key_EnablePullUps(void)
{
#if defined(Key_User_IOMUX)
    DL_GPIO_setDigitalInternalResistor(Key_User_IOMUX,
        DL_GPIO_RESISTOR_PULL_UP);
#endif
#if defined(Key_Key1_IOMUX)
    DL_GPIO_setDigitalInternalResistor(Key_Key1_IOMUX,
        DL_GPIO_RESISTOR_PULL_UP);
#endif
#if defined(Key_Key2_IOMUX)
    DL_GPIO_setDigitalInternalResistor(Key_Key2_IOMUX,
        DL_GPIO_RESISTOR_PULL_UP);
#endif
#if defined(Key_Key3_IOMUX)
    DL_GPIO_setDigitalInternalResistor(Key_Key3_IOMUX,
        DL_GPIO_RESISTOR_PULL_UP);
#endif
#if defined(Key_Key4_IOMUX)
    DL_GPIO_setDigitalInternalResistor(Key_Key4_IOMUX,
        DL_GPIO_RESISTOR_PULL_UP);
#endif
#if defined(Key_Key5_IOMUX)
    DL_GPIO_setDigitalInternalResistor(Key_Key5_IOMUX,
        DL_GPIO_RESISTOR_PULL_UP);
#endif
}

static inline void Key_WaitForPressed(uint32_t debounceMs)
{
    while (1) {
        if (Key_IsPressed() != 0U) {
            delay_ms(debounceMs);
            if (Key_IsPressed() != 0U) {
                break;
            }
        }
        delay_ms(1U);
    }
}

#endif
