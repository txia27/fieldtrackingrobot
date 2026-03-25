#ifndef UART_H
#define UART_H

#include <stdint.h>

void UART2_Init(void);

void UART2_WriteChar(char c);

void UART2_WriteString(char *s);

void UART2_WriteInt(uint16_t val);

#endif