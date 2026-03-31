#include "Common/Include/stm32l051xx.h"
#include "robot_ir_tx.h"

#define IR_PIN (1 << 4)   // PB4

// ---------- INIT ----------
void ir_tx_init(void)
{
    // Enable GPIOB
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    // PB4 → output mode
    GPIOB->MODER &= ~(3 << (4 * 2));
    GPIOB->MODER |=  (1 << (4 * 2));

    // Push-pull
    GPIOB->OTYPER &= ~IR_PIN;
}

// ---------- TIM22 DELAY (1 MHz timer assumed) ----------
static void delay_us_tim22(uint16_t us)
{
    uint16_t start = TIM22->CNT;
    while ((uint16_t)(TIM22->CNT - start) < us);
}

// ---------- BASIC GPIO ----------
static inline void IR_high(void)
{
    GPIOB->ODR |= IR_PIN;
}

static inline void IR_low(void)
{
    GPIOB->ODR &= ~IR_PIN;
}

// ---------- TEST 1: LED ON ----------
void ir_tx_test_led_on(void)
{
    IR_high();          // solid ON
}

// ---------- 38 kHz carrier ----------
static void IR_carrier(uint16_t duration_us)
{
    for (uint16_t i = 0; i < duration_us / 26; i++)
    {
        IR_high();
        delay_us_tim22(13);

        IR_low();
        delay_us_tim22(13);
    }
}

// ---------- TEST 2: PWM burst ----------
void ir_tx_test_pwm(uint16_t pulse_us)
{
    IR_carrier(pulse_us);
    delay_us_tim22(3000); // spacing
}

// Send pulse (same protocol as before)
void ir_tx_send(uint16_t pulse_us)
{
    IR_carrier(pulse_us);
    delay_us_tim22(3000); // spacing
}