#ifndef ADC_H
#define ADC_H //just to tell preprocessor this file is already processed

#include <stdint.h>
#include <stdbool.h>

void ADC_Init(void);
//uint16_t ADC_Read_Channel(uint8_t ch);
void ADC_Read_All(uint16_t *ch4, uint16_t *ch5, uint16_t *ch6);

#endif 	