#include <stdint.h>
#include <stdbool.h>
#include "Common/Include/stm32l051xx.h"

void ADC_Init(void)
{
    //Enable clocks
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    //Set PA4, PA5, PA6 to analog mode
    GPIOA->MODER |= (3 << 8) | (3 << 10) | (3 << 12);
    GPIOA->PUPDR &= ~((3 << 8) | (3 << 10) | (3 << 12));

    //Disable ADC if already enabled
    if (ADC1->CR & ADC_CR_ADEN)
    {
        ADC1->CR |= ADC_CR_ADDIS;
        while (ADC1->CR & ADC_CR_ADEN);
    }

    //Enable ADC voltage regulator
    ADC1->CR |= ADC_CR_ADVREGEN;
    for (volatile int i = 0; i < 1000; i++); // delay

    //Sampling time (VERY IMPORTANT for your circuit)
    ADC1->SMPR |= 4;  // max sample time

    //Enable ADC
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY));
}
/*
uint16_t ADC_Read_Channel(uint8_t ch)
{
    ADC1->CHSELR = (1 << ch);

    ADC1->CR |= ADC_CR_ADSTART;

    uint32_t timeout = 10000;

    while (!(ADC1->ISR & ADC_ISR_EOC))
    {
        if (--timeout == 0)
        {
            return 61; // fail safe
        }
    }

    return ADC1->DR;
}
*/
void ADC_Read_All(uint16_t *ch4, uint16_t *ch5, uint16_t *ch6){
    ADC1->CHSELR = (1<<4)|(1<<5)|(1<<6);
    ADC1->CR |= ADC_CR_ADSTART;

    while(!(ADC1->ISR & ADC_ISR_EOC));
    *ch4 = ADC1->DR;
    
    while(!(ADC1->ISR & ADC_ISR_EOC));
    *ch5 = ADC1->DR;

    while(!(ADC1->ISR & ADC_ISR_EOC));
    *ch6 = ADC1->DR;
}