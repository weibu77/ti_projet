#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>
#include "ti_msp_dl_config.h"

void UART_Init(void);
void UART_SendByte(uint8_t data);
void UART_SendString(const char *str);
uint8_t UART_ReadByte(uint8_t *data);
uint8_t UART_GetRxOverflow(void);
void UART_ProcessInput(void);
void UART_Task(void);
void UART_SendAck(const char *text);
void UART_SendError(const char *text);
void UART_IRQHandler(void);

#endif
