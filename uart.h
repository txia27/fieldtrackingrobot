#ifndef UART2_H
#define UART2_H

void UART2_Init(int baud);

int  UART2_Write(int count, unsigned char *buf);
int  UART2_Read(int max, unsigned char *buf);

void UART2_Putc(char c);
void UART2_Puts(char *s);

#endif