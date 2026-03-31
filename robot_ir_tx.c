#include "Common/Include/stm32l051xx.h"
#include "robot_ir_tx.h"

// PB4 will be used as output
#define IR_PIN (1 << 4)

void ir_tx_init(void)
{
    // Enable GPIOB
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    // PB4 → output mode
    GPIOB->MODER &= ~(3 << (4 * 2));
    GPIOB->MODER |=  (1 << (4 * 2)); // general purpose output

    // Make sure it's push-pull
    GPIOB->OTYPER &= ~IR_PIN;
}

static inline void IR_high(void)
{
    GPIOB->ODR |= IR_PIN;
}

static inline void IR_low(void)
{
    GPIOB->ODR &= ~IR_PIN;
}

// Use TIM22 (already running at 1 MHz) for delay
static void delay_us_tim22(uint16_t us)
{
    uint16_t start = TIM22->CNT;
    while ((uint16_t)(TIM22->CNT - start) < us);
}

// Generate 38 kHz carrier
static void IR_carrier(uint16_t duration_us)
{
    for (uint16_t i = 0; i < duration_us / 26; i++) // 26 µs period ≈ 38 kHz
    {
        IR_high();
        delay_us_tim22(13);

        IR_low();
        delay_us_tim22(13);
    }
}

// Send pulse (same protocol as before)
void ir_tx_send(uint16_t pulse_us)
{
    IR_carrier(pulse_us);
    delay_us_tim22(3000); // spacing
}