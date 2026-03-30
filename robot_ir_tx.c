#include "Common/Include/stm32l051xx.h"
#include "robot_ir_tx.h"

#define PWM_FREQ 38000

static void delay_us(uint16_t us)
{
    for (volatile int i = 0; i < us * 8; i++);
}

void ir_tx_init(void)
{
    // Enable GPIOA + TIM1
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // PA8 → AF2 (TIM1_CH1)
    GPIOA->MODER &= ~(3 << (8 * 2));
    GPIOA->MODER |=  (2 << (8 * 2));   // AF mode

    GPIOA->AFR[1] &= ~(0xF << ((8 - 8) * 4));
    GPIOA->AFR[1] |=  (2 << ((8 - 8) * 4));  // AF2

    // --- TIM1 setup ---
    TIM1->PSC = 0;
    TIM1->ARR = (F_CPU / PWM_FREQ) - 1;
    TIM1->CCR1 = TIM1->ARR / 2;

    // PWM mode 1
    TIM1->CCMR1 &= ~(7 << 4);
    TIM1->CCMR1 |=  (6 << 4);

    // Enable main output (IMPORTANT for TIM1)
    TIM1->BDTR |= TIM_BDTR_MOE;

    // Start OFF
    TIM1->CCER &= ~TIM_CCER_CC1E;

    // Enable timer
    TIM1->CR1 |= TIM_CR1_CEN;
}

static inline void IR_on(void)
{
    TIM1->CCER |= TIM_CCER_CC1E;
}

static inline void IR_off(void)
{
    TIM1->CCER &= ~TIM_CCER_CC1E;
}

void ir_tx_send(uint16_t pulse_us)
{
    IR_on();
    delay_us(pulse_us);
    IR_off();
    delay_us(3000);
}