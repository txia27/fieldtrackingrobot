#include "Common/Include/stm32l051xx.h"
#include <stdint.h>
#include <stdbool.h>

void UART2_Init(void)
{
    // 1. Enable clocks
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // 2. Set PA2 (TX) and PA3 (RX) to alternate function
    GPIOA->MODER &= ~((3 << 4) | (3 << 6));   // clear PA2, PA3
    GPIOA->MODER |=  ((2 << 4) | (2 << 6));   // AF mode

    // 3. Set AF4 (USART2)
    GPIOA->AFR[0] &= ~((0xF << 8) | (0xF << 12));
    GPIOA->AFR[0] |=  ((4 << 8) | (4 << 12));

    // 4. Configure baud rate
    // Assuming 32 MHz clock
    USART2->BRR = 32000000 / 115200;  // ≈ 278

    // 5. Enable transmitter and receiver
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;

    // 6. Enable USART
    USART2->CR1 |= USART_CR1_UE;
}

void UART2_WriteChar(char c)
{
    while (!(USART2->ISR & USART_ISR_TXE)); // wait until ready
    USART2->TDR = c;
}

void UART2_WriteString(char *s)
{
    while (*s)
    {
        UART2_WriteChar(*s++);
    }
}

void UART2_WriteInt(uint16_t val)
{
    char buf[10];
    int i = 0;

    if (val == 0)
    {
        UART2_WriteChar('0');
        return;
    }

    while (val > 0)
    {
        buf[i++] = (val % 10) + '0';
        val /= 10;
    }

    while (i--)
    {
        UART2_WriteChar(buf[i]);
    }
}