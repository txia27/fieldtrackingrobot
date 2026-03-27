//  pinout
//		     ---------
//	VDD   -	|1		32|  -  VSS
//	PC14  -	|2		31|  -  BOOT0
//	PC15  -	|3		30|  -  PB7
//	NRST  -	|4		29|  -  PB6
//	VDDA  -	|5		28|  -  PB5
//	PA0   -	|6		27|  -  PB4
//	PA1   -	|7		26|  -  PB3
//	PA2   -	|8		25|  -  PA15
//	PA3   -	|9		24|  -  PA14
//	PA4   -	|10	 	23|  -  PA13
//	PA5   -	|11		22|  -  PA12
//	PA6   -	|12		21|  -  PA11
//	PA7   -	|13		20|  -  PA10
//	PB0   -	|14		19|  -  PA9
//	PB1   -	|15		18|  -  PA8
//	VSS   -	|16		17|  -  VDD
//		     ---------


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "decoder.h"
#include "Common/Include/stm32l051xx.h"


//need to configure pin PA7 (pin 13)
//configure timer 2 to measure the period of the signal at that pin, and use that to determine which command was sent by the encoder

volatile int signal_start = 0;
volatile int capture = 0;
volatile int command_signal = 0;
volatile int pulse_width = 0;
volatile int signal_flag = 0; // flag to indicate a new signal has been captured

const int commands[12] = {1800, 2900, 4000, 5000, 6100, 7100, 8200, 9300, 10400, 11400, 12500, 13600};
// change these numbers depending on how the encoder is configured (eg. 0.001s, 0.002s)
// currently in microseconds


void initialize_decoder(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN; // peripheral clock enable for port A
    
    GPIOA->MODER &= ~(BIT14 | BIT15); // clear bits for pin PA7
    GPIOA->MODER |= BIT15; // Make pin PA7 Alternate Function

    GPIOA->AFR[0] &= ~(0xF << 28); // clear bits for pin PA7
    GPIOA->AFR[0] |= (5 << 28); // set alternate function for pin PA7 to AF5 (TIM22_CH2)

    // Activate pull up for pin PA7:
    GPIOA->PUPDR |= BIT14; 
    GPIOA->PUPDR &= ~(BIT15); 

}


void initialize_timer22(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM22EN; // enable clock for timer 22
    TIM22->PSC = 32 - 1; // set prescaler to 32 (assuming 32 MHz clock, this gives us a timer frequency of 1 MHz, or a resolution of 1 microsecond)
    TIM22->ARR = 0xFFFF; // set auto-reload register to maximum

    TIM22->CCMR1 &= ~(0b11 << 8); // clear bits for channel 2
    TIM22->CCMR1 |=  (0b01 << 8); // set channel

    TIM22->CCER |= TIM_CCER_CC2P | TIM_CCER_CC2NP; // set channel 2 to capture on both rising and falling edges
    TIM22->CCER |=  TIM_CCER_CC2E; // enable capture for channel

    TIM22->DIER |= TIM_DIER_CC2IE; // enable capture/compare interrupt for channel 2
    NVIC_EnableIRQ(TIM22_IRQn); // enable interrupt in NVIC

    TIM22->CR1 |= TIM_CR1_CEN; // start timer

}


void TIM22_Handler(void)
{
    if (TIM22->SR & TIM_SR_CC2IF) // check if capture/compare interrupt flag is set for channel 2
    {
        capture = TIM22->CCR2; // read captured value (time of signal edge)

        if ((GPIOA->IDR & BIT7) == 0) // check if signal is currently low (falling edge)
        {
            signal_start = capture; // record start time of signal
        }

        else // signal is currently high (rising edge)
        {
            if (capture >= signal_start) // check for timer overflow
            {
                pulse_width = capture - signal_start; // calculate pulse width
            }

            else
            {
                pulse_width = (0xFFFF - signal_start) + capture; // handle timer overflow
            }

            signal_flag = 1; // set flag to indicate a new signal has been captured
        }

        TIM22->SR &= ~TIM_SR_CC2IF; // clear interrupt flag
    }
}


int decode (int signal_length) 
{
    // decoder lookup table to determine which command was sent
    if      (signal_length < commands[0] - ERROR) command_signal = 0; // default state
    else if (signal_length < commands[0] + ERROR) command_signal = 1; // stop moving            (0.5 - 1.5)
    else if (signal_length < commands[1] + ERROR) command_signal = 2; // turn left              (1.5 - 2.5)
    else if (signal_length < commands[2] + ERROR) command_signal = 3; // turn right             (2.5 - 3.5)
    else if (signal_length < commands[3] + ERROR) command_signal = 4; // move forward           (3.5 - 4.5)
    else if (signal_length < commands[4] + ERROR) command_signal = 5; // move reverse           (4.5 - 5.5)
    else if (signal_length < commands[5] + ERROR) command_signal = 6; // turn around            (5.5 - 6.5)
    else if (signal_length < commands[6] + ERROR) command_signal = 7; // mode                   (6.5 - 7.5)
    else if (signal_length < commands[7] + ERROR) command_signal = 8; // start                  (7.5 - 8.5)
    else if (signal_length < commands[8] + ERROR) command_signal = 9; // fwd-right              (8.5 - 9.5)
    else if (signal_length < commands[9] + ERROR) command_signal = 10; // fwd-left              (9.5 - 10.5)
    else if (signal_length < commands[10] + ERROR) command_signal = 11; // back-right           (10.5 - 11.5)
    else if (signal_length < commands[11] + ERROR) command_signal = 12; // back-left            (11.5 - 12.5)
    else command_signal = 99; // error state, send error message (beep?)

    return command_signal;
}
