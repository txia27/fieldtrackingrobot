#ifndef ADC_H
#define ADC_H //just to tell preprocessor this file is already processed

#include <stdint.h>
#include <stdbool.h>

void ADC_Init(void);
uint16_t ADC_Read_Channel(uint8_t ch);

#endif 	