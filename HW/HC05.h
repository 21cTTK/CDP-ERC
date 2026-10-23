#ifndef __HC05_H
#define __HC05_H

#include <stdint.h>

#define HC05_UART_BAUDRATE       9600U
#define HC05_UART_PACKET_HEAD    0x23U	//'#'
#define HC05_UART_PACKET_TAIL    0x23U	//'#'
#define HC05_UART_PACKET_MAXLEN  114U

void HC05_Init(void);
void HC05_SendString(char *String);
void HC05_SendStringFormatted(const char *str_f, ...);
void HC05_GetRxPacket(char *buffer);
float   HC05_ParseMsgFloat(const char *float_str);
uint8_t HC05_GetRxFlag(void);
uint8_t HC05_PidMsgTimeReached(uint32_t nowMs);
void HC05_ISR(void);

#endif//HC05_H
