#ifndef PID_CONFIG_H
#define PID_CONFIG_H

#include <stdint.h>

#define BUFFER_SIZE 64

void DMA_init(void);
uint8_t uart_pid(void);

#endif
