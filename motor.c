#include <stdint.h>
#include <stdbool.h>
#include "motor.h"
#include "../Common/Include/stm32l051xx.h"

void Motor_Init(void){
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2
    RCC->IOPENR |= BIT0;

    // Set pins as alternate function for PWM
    GPIOA->MODER = (GPIOA->MODER & ~(BIT1|BIT0)) | BIT1; // PA0
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xF << 0)) | (0x2 << 0); // Configure alternate function as timer
	GPIOA->OTYPER &= ~BIT0; // Push-pull

    GPIOA->MODER = (GPIOA->MODER & ~(BIT2|BIT3)) | BIT3; // PA1
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xF << 4)) | (0x2 << 4); // Configure alternate function as timer
    GPIOA->OTYPER &= ~BIT1; // Push-pull

    GPIOA->MODER = (GPIOA->MODER & ~(BIT4|BIT5)) | BIT5; // PA2
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xF << 8)) | (0x2 << 8); // Configure alternate function as timer
    GPIOA->OTYPER &= ~BIT2; // Push-pull,

    GPIOA->MODER = (GPIOA->MODER & ~(BIT6|BIT7)) | BIT7; // PA3
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xF << 12)) | (0x2 << 12); // Configure alternate function as timer
	GPIOA->OTYPER &= ~BIT3; // Push-pull

    //==Configure TIM2==//

    // 1. Prescaler to scale down TIM2 from SysTick(32MHz) to 20 kHz
    TIM2->PSC = 1;

    // 2. Set TIM2 Auto Reload Register (This is the period of the PWM)
    TIM2->ARR = PWM_MAX - 1;

    // 3. Capture and Compare Registers (CCR1-4). This controls the duty cycle. If the counter is below CCR, the pin is high.
    // Start them at 0
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;

    // 4. Output Compare Mode. (Tell each pin to operate in PWM mode 1 and enable the output)
    TIM2->CCMR1 = (6 << 4)  |  // CH1 PWM mode 1
                  (1 << 3)  |  // CH1 preload enable
                  (6 << 12) |  // CH2 PWM mode 1
                  (1 << 11);   // CH2 preload enable

    TIM2->CCMR2 = (6 << 4)  |  // CH3 PWM mode 1
                  (1 << 3)  |  // CH3 preload enable
                  (6 << 12) |  // CH4 PWM mode 1
                  (1 << 11);   // CH4 preload enable

    TIM2->CCER = BIT0 | BIT4 | BIT8 | BIT12;

    // Enable TIM2
    TIM2->CR1 |= BIT0;
}

void Motor_SetPWM(int left_pwm, int right_pwm){
    TIM2->CCR1 = left_pwm;               // PA0: Motor A IN1
    TIM2->CCR2 = PWM_MAX - left_pwm;     // PA1: Motor A IN2
    TIM2->CCR3 = right_pwm;              // PA2: Motor B IN1
    TIM2->CCR4 = PWM_MAX - right_pwm;    // PA3: Motor B IN2
}

