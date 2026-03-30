#include "Common/Include/stm32l051xx.h"
#include "robot_ir_tx.h"

#define PWM_FREQ 38000

static void delay_us(uint16_t us)
{
    for (volatile int i = 0; i < us * 8; i++);
}

void ir_tx_init(void)
{
    // Enable GPIOA + TIM21
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;

    // --- PA15 → AF mode ---
    GPIOA->MODER &= ~(3 << (15 * 2));
    GPIOA->MODER |=  (2 << (15 * 2));   // AF mode

    // AF0 = TIM21_CH1 (on L0)
    GPIOA->AFR[1] &= ~(0xF << ((15 - 8) * 4));
    GPIOA->AFR[1] |=  (0 << ((15 - 8) * 4));

    // --- Timer setup ---
    TIM21->PSC = 0;
    TIM21->ARR = (32000000L / PWM_FREQ) - 1;
    TIM21->CCR1 = TIM21->ARR / 2;

    // PWM mode 1
    TIM21->CCMR1 &= ~(7 << 4);
    TIM21->CCMR1 |=  (6 << 4);

    // Start OFF
    TIM21->CCER &= ~TIM_CCER_CC1E;

    // Enable timer
    TIM21->CR1 |= TIM_CR1_CEN;
}

static inline void IR_on(void)
{
    TIM21->CCER |= TIM_CCER_CC1E;
}

static inline void IR_off(void)
{
    TIM21->CCER &= ~TIM_CCER_CC1E;
}

void ir_tx_send(uint16_t pulse_us)
{
    IR_on();
    delay_us(pulse_us);
    IR_off();
    delay_us(3000);
}