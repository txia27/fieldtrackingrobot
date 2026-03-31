#include "Common/Include/stm32l051xx.h"
#include "uart.h"

#define MAXBUFFER 64

typedef struct {
    unsigned char Buffer[MAXBUFFER];
    unsigned Head, Tail, Count;
} ComBuffer;

static ComBuffer RXBuf, TXBuf;

static unsigned ComOpen = 0;
static unsigned ComError = 0;

// ---------- Buffer helpers ----------
static int PutBuf(ComBuffer *Buf, unsigned char Data)
{
    if ((Buf->Head == Buf->Tail) && (Buf->Count != 0)) return 1;

    __disable_irq();
    Buf->Buffer[Buf->Head++] = Data;
    Buf->Count++;
    if (Buf->Head == MAXBUFFER) Buf->Head = 0;
    __enable_irq();

    return 0;
}

static unsigned char GetBuf(ComBuffer *Buf)
{
    unsigned char Data = 0;

    if (Buf->Count == 0) return 0;

    __disable_irq();
    Data = Buf->Buffer[Buf->Tail++];
    if (Buf->Tail == MAXBUFFER) Buf->Tail = 0;
    Buf->Count--;
    __enable_irq();

    return Data;
}

static unsigned GetBufCount(ComBuffer *Buf)
{
    return Buf->Count;
}

// ---------- TX / RX handlers ----------
static void usart2_tx(void)
{
    if (GetBufCount(&TXBuf))
    {
        USART2->TDR = GetBuf(&TXBuf);
    }
    else
    {
        USART2->CR1 &= ~BIT3; // disable transmitter interrupt
        if (USART2->ISR & BIT6) USART2->ICR |= BIT6;
        if (USART2->ISR & BIT7) USART2->RQR |= BIT4;
    }
}

static void usart2_rx(void)
{
    if (PutBuf(&RXBuf, USART2->RDR))
        ComError = 1;
}

// ---------- IRQ ----------
void USART2_Handler(void)
{
    if (USART2->ISR & BIT7) usart2_tx();   // TXE
    if (USART2->ISR & BIT5) usart2_rx();   // RXNE
}

// ---------- Init ----------
void UART2_Init(int baud)
{
    int div = 32000000 / baud;

    __disable_irq();

    RXBuf.Head = RXBuf.Tail = RXBuf.Count = 0;
    TXBuf.Head = TXBuf.Tail = TXBuf.Count = 0;

    ComOpen = 1;
    ComError = 0;

    // GPIOA clock
    RCC->IOPENR |= BIT0;

    // USART2 clock
    RCC->APB1ENR |= BIT17;

    // --- PA14 (TX) ---
    GPIOA->MODER &= ~(3 << (14 * 2));
    GPIOA->MODER |=  (2 << (14 * 2));   // AF
    GPIOA->AFR[1] &= ~(0xF << ((14 - 8) * 4));
    GPIOA->AFR[1] |= (4 << ((14 - 8) * 4)); // AF4
    GPIOA->OTYPER &= ~(1 << 14);
    GPIOA->OSPEEDR |= (1 << (14 * 2));

    // --- PA15 (RX) ---
    GPIOA->MODER &= ~(3 << (15 * 2));
    GPIOA->MODER |=  (2 << (15 * 2));   // AF
    GPIOA->AFR[1] &= ~(0xF << ((15 - 8) * 4));
    GPIOA->AFR[1] |= (4 << ((15 - 8) * 4)); // AF4

    // USART config
    USART2->BRR = div;
    USART2->CR1 |= (BIT2 | BIT3 | BIT5 | BIT6); // RX, TX, interrupts
    USART2->CR2 = 0;
    USART2->CR3 = 0;
    USART2->CR1 |= BIT0; // enable

    // Enable IRQ
    NVIC->ISER[0] |= BIT28; // USART2 interrupt

    __enable_irq();
}

// ---------- Write ----------
int UART2_Write(int count, unsigned char *buf)
{
    if (!ComOpen) return -1;

    if (count >= MAXBUFFER) return -2;

    while ((MAXBUFFER - GetBufCount(&TXBuf)) < count);

    for (int i = 0; i < count; i++)
        PutBuf(&TXBuf, buf[i]);

    if ((USART2->CR1 & BIT3) == 0)
    {
        USART2->CR1 |= BIT3;
        USART2->TDR = GetBuf(&TXBuf);
    }

    return 0;
}

// ---------- Read ----------
int UART2_Read(int max, unsigned char *buf)
{
    if (!ComOpen) return -1;

    int i = 0;

    while ((i < max - 1) && GetBufCount(&RXBuf))
        buf[i++] = GetBuf(&RXBuf);

    if (i > 0)
    {
        buf[i] = 0;
        return i;
    }

    return 0;
}

// 
void UART2_Putc(char c)
{
    UART2_Write(1, (unsigned char *)&c);
}

void UART2_Puts(char *s)
{
    while (*s) UART2_Putc(*s++);
}