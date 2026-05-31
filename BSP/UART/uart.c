#include "uart.h"
#include "motor.h"
#include "PID.h"
#include "ICM42688.h"
#include "icm.h"

#include <stdio.h>

#define UART_RX_BUFFER_SIZE 128U
#define UART_CMD_BUFFER_SIZE 64U

static volatile uint8_t s_uartRxBuffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t s_uartRxHead = 0U;
static volatile uint16_t s_uartRxTail = 0U;
static volatile uint8_t s_uartRxOverflow = 0U;

static void UART_HandleCommand(char *cmd);
static char *SkipSpaces(char *s);
static char *NextToken(char **cursor);
static uint8_t ParseFloatToken(const char *text, float *value);
static uint8_t StrEqual(const char *a, const char *b);
static char ToUpperChar(char ch);
static void SendPidStatus(void);
static void SendYawStatus(void);

void UART_Init(void)
{
    DL_UART_Main_clearInterruptStatus(UART0_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART0_INST_INT_IRQN);
}

void UART_SendByte(uint8_t data)
{
    DL_UART_Main_transmitDataBlocking(UART0_INST, data);
}

void UART_SendString(const char *str)
{
    while ((str != NULL) && (*str != '\0')) {
        UART_SendByte((uint8_t) *str);
        str++;
    }
}

uint8_t UART_ReadByte(uint8_t *data)
{
    if (data == NULL) {
        return 0U;
    }

    __disable_irq();
    if (s_uartRxHead == s_uartRxTail) {
        __enable_irq();
        return 0U;
    }

    *data = s_uartRxBuffer[s_uartRxTail];
    s_uartRxTail = (uint16_t) ((s_uartRxTail + 1U) % UART_RX_BUFFER_SIZE);
    __enable_irq();

    return 1U;
}

uint8_t UART_GetRxOverflow(void)
{
    uint8_t overflow;

    __disable_irq();
    overflow = s_uartRxOverflow;
    s_uartRxOverflow = 0U;
    __enable_irq();

    return overflow;
}

void UART_ProcessInput(void)
{
    static char cmdBuffer[UART_CMD_BUFFER_SIZE];
    static uint16_t cmdIndex = 0U;
    uint8_t data;

    while (UART_ReadByte(&data) != 0U) {
        if ((data == '\r') || (data == '\n')) {
            if (cmdIndex > 0U) {
                cmdBuffer[cmdIndex] = '\0';
                UART_HandleCommand(cmdBuffer);
                cmdIndex = 0U;
            }
        } else if (cmdIndex < (UART_CMD_BUFFER_SIZE - 1U)) {
            cmdBuffer[cmdIndex] = (char) data;
            cmdIndex++;
        } else {
            cmdIndex = 0U;
            UART_SendError("CMD_TOO_LONG");
        }
    }
}

void UART_Task(void)
{
    UART_ProcessInput();
    if (UART_GetRxOverflow() != 0U) {
        UART_SendError("RX_OVERFLOW");
    }
}

void UART_SendAck(const char *text)
{
    char line[32];

    snprintf(line, sizeof(line), "OK,%s\r\n", text);
    UART_SendString(line);
}

void UART_SendError(const char *text)
{
    char line[40];

    snprintf(line, sizeof(line), "ERR,%s\r\n", text);
    UART_SendString(line);
}

void UART_IRQHandler(void)
{
    uint8_t data;
    uint16_t nextHead;

    switch (DL_UART_Main_getPendingInterrupt(UART0_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        data = DL_UART_Main_receiveData(UART0_INST);
        nextHead = (uint16_t) ((s_uartRxHead + 1U) % UART_RX_BUFFER_SIZE);
        if (nextHead != s_uartRxTail) {
            s_uartRxBuffer[s_uartRxHead] = data;
            s_uartRxHead = nextHead;
        } else {
            s_uartRxOverflow = 1U;
        }
        break;
    default:
        break;
    }
}

int fputc(int ch, FILE *stream)
{
    (void) stream;

    if (ch == '\n') {
        UART_SendByte('\r');
    }
    UART_SendByte((uint8_t) ch);

    return ch;
}

static void UART_HandleCommand(char *cmd)
{
    char *cursor = cmd;
    char *op = NextToken(&cursor);
    char *arg1;
    char *arg2;
    char *arg3;
    float v1;
    float v2;
    float v3;

    if (op == NULL) {
        return;
    }

    if (StrEqual(op, "P") != 0U) {
        arg1 = NextToken(&cursor);
        arg2 = NextToken(&cursor);
        arg3 = NextToken(&cursor);
        if ((ParseFloatToken(arg1, &v1) == 0U) ||
            (ParseFloatToken(arg2, &v2) == 0U) ||
            (ParseFloatToken(arg3, &v3) == 0U)) {
            UART_SendError("P_FORMAT");
            return;
        }
        __disable_irq();
        PID_SetParam(&motorA, v1, v2, v3);
        PID_SetParam(&motorB, v1, v2, v3);
        __enable_irq();
        UART_SendAck("P");
    } else if (StrEqual(op, "PL") != 0U) {
        arg1 = NextToken(&cursor);
        arg2 = NextToken(&cursor);
        arg3 = NextToken(&cursor);
        if ((ParseFloatToken(arg1, &v1) == 0U) ||
            (ParseFloatToken(arg2, &v2) == 0U) ||
            (ParseFloatToken(arg3, &v3) == 0U)) {
            UART_SendError("PL_FORMAT");
            return;
        }
        __disable_irq();
        PID_SetParam(&motorA, v1, v2, v3);
        __enable_irq();
        UART_SendAck("PL");
    } else if (StrEqual(op, "PR") != 0U) {
        arg1 = NextToken(&cursor);
        arg2 = NextToken(&cursor);
        arg3 = NextToken(&cursor);
        if ((ParseFloatToken(arg1, &v1) == 0U) ||
            (ParseFloatToken(arg2, &v2) == 0U) ||
            (ParseFloatToken(arg3, &v3) == 0U)) {
            UART_SendError("PR_FORMAT");
            return;
        }
        __disable_irq();
        PID_SetParam(&motorB, v1, v2, v3);
        __enable_irq();
        UART_SendAck("PR");
    } else if (StrEqual(op, "R") != 0U) {
        arg1 = NextToken(&cursor);
        arg2 = NextToken(&cursor);
        if ((ParseFloatToken(arg1, &v1) == 0U) ||
            (ParseFloatToken(arg2, &v2) == 0U)) {
            UART_SendError("R_FORMAT");
            return;
        }
        __disable_irq();
        motorA.ref = v1;
        motorB.ref = v2;
        PID_ResetState(&motorA);
        PID_ResetState(&motorB);
        __enable_irq();
        UART_SendAck("R");
    } else if (StrEqual(op, "S") != 0U) {
        SpeedLoop_Stop();
        UART_SendAck("S");
    } else if (StrEqual(op, "G") != 0U) {
        arg1 = NextToken(&cursor);
        if (ParseFloatToken(arg1, &v1) == 0U) {
            UART_SendError("G_FORMAT");
            return;
        }
        ICM42688_YawSetBias(v1);
        ICM42688_YawReset(0.0f);
        UART_SendAck("G");
    } else if (StrEqual(op, "GA") != 0U) {
        arg1 = NextToken(&cursor);
        if (ParseFloatToken(arg1, &v1) == 0U) {
            UART_SendError("GA_FORMAT");
            return;
        }
        ICM42688_YawSetBias(ICM42688_YawGetBias() + v1);
        ICM42688_YawReset(0.0f);
        UART_SendAck("GA");
    } else if (StrEqual(op, "YM") != 0U) {
        arg1 = NextToken(&cursor);
        if (ParseFloatToken(arg1, &v1) == 0U) {
            UART_SendError("YM_FORMAT");
            return;
        }
        if ((v1 < 0.0f) || (v1 > 3.0f)) {
            UART_SendError("YM_RANGE");
            return;
        }
        ICM_SetYawMode((ICM_YawMode) ((uint8_t) v1));
        ICM42688_YawReset(0.0f);
        SendYawStatus();
    } else if (StrEqual(op, "YQ") != 0U) {
        SendYawStatus();
    } else if (StrEqual(op, "Q") != 0U) {
        SendPidStatus();
    } else {
        UART_SendError("UNKNOWN_CMD");
    }
}

static char *SkipSpaces(char *s)
{
    while ((s != NULL) && ((*s == ' ') || (*s == '\t'))) {
        s++;
    }

    return s;
}

static char *NextToken(char **cursor)
{
    char *start;

    if (cursor == NULL) {
        return NULL;
    }

    start = SkipSpaces(*cursor);
    if ((start == NULL) || (*start == '\0')) {
        *cursor = start;
        return NULL;
    }

    *cursor = start;
    while ((**cursor != '\0') && (**cursor != ' ') && (**cursor != '\t')) {
        **cursor = ToUpperChar(**cursor);
        (*cursor)++;
    }

    if (**cursor != '\0') {
        **cursor = '\0';
        (*cursor)++;
    }

    return start;
}

static uint8_t ParseFloatToken(const char *text, float *value)
{
    float sign = 1.0f;
    float result = 0.0f;
    float frac = 0.1f;
    uint8_t hasDigit = 0U;

    if ((text == NULL) || (value == NULL)) {
        return 0U;
    }

    if (*text == '-') {
        sign = -1.0f;
        text++;
    } else if (*text == '+') {
        text++;
    }

    while ((*text >= '0') && (*text <= '9')) {
        result = (result * 10.0f) + (float) (*text - '0');
        hasDigit = 1U;
        text++;
    }

    if (*text == '.') {
        text++;
        while ((*text >= '0') && (*text <= '9')) {
            result += (float) (*text - '0') * frac;
            frac *= 0.1f;
            hasDigit = 1U;
            text++;
        }
    }

    if ((*text != '\0') || (hasDigit == 0U)) {
        return 0U;
    }

    *value = sign * result;
    return 1U;
}

static uint8_t StrEqual(const char *a, const char *b)
{
    if ((a == NULL) || (b == NULL)) {
        return 0U;
    }

    while ((*a != '\0') && (*b != '\0')) {
        if (ToUpperChar(*a) != ToUpperChar(*b)) {
            return 0U;
        }
        a++;
        b++;
    }

    return (uint8_t) ((*a == '\0') && (*b == '\0'));
}

static char ToUpperChar(char ch)
{
    if ((ch >= 'a') && (ch <= 'z')) {
        return (char) (ch - ('a' - 'A'));
    }

    return ch;
}

static void SendPidStatus(void)
{
    char line[128];

    snprintf(line, sizeof(line),
        "CFG,LP=%.3f,LI=%.3f,LD=%.3f,LR=%.2f,RP=%.3f,RI=%.3f,RD=%.3f,RR=%.2f,GB=%.6f\r\n",
        (double) motorA.p, (double) motorA.i, (double) motorA.d, (double) motorA.ref,
        (double) motorB.p, (double) motorB.i, (double) motorB.d, (double) motorB.ref,
        (double) ICM42688_YawGetBias());
    UART_SendString(line);
}

static void SendYawStatus(void)
{
    char line[128];

    snprintf(line, sizeof(line),
        "YAW,mode=%u,name=%s,bias=%.6f,rbias=%.6f,rate=%.6f,yaw=%.5f\r\n",
        (unsigned int) ICM_GetYawMode(), ICM_GetYawModeName(),
        (double) ICM42688_YawGetBias(),
        (double) ICM_GetYawRuntimeBias(),
        (double) ICM42688_YawGetCorrectedGzDps(),
        (double) ICM42688_YawGetDeg());
    UART_SendString(line);
}
