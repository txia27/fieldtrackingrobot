#include <stdint.h>
#include <stdbool.h>
#include "Common/Include/stm32l051xx.h"

#define BUFFER_SIZE 64
volatile uint8_t packetBuffer[BUFFER_SIZE];
uint16_t readIndex = 0;
int stateFlag = 0;
int syncDelay = 0;


void DMA_init(){
    USART1->CR1 &= ~BIT5;  // Disable RXNE interrupt

    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    USART1->CR3 |= USART_CR3_DMAR;

    DMA1_CSELR->CSELR &= ~DMA_CSELR_C3S;
    DMA1_CSELR->CSELR |= (3 << 8);

    DMA1_Channel3->CPAR = (uint32_t)&(USART1->RDR);
    DMA1_Channel3->CMAR = (uint32_t)packetBuffer;
    DMA1_Channel3->CNDTR = BUFFER_SIZE;

    DMA1_Channel3->CCR |= DMA_CCR_MINC | DMA_CCR_CIRC;
    DMA1_Channel3->CCR |= DMA_CCR_EN;
}

uint8_t uart_pid(){
    //read iteratively from uart - main will handle this
    //Pass uart to this function, 

    uint8_t current = packetBuffer[(readIndex%BUFFER_SIZE)];
    uint16_t writeIndex = BUFFER_SIZE - DMA1_Channel3->CNDTR;

    if( writeIndex == readIndex ){
        return 0x00;
    }
    else{
        if((current == 0xAF) && stateFlag==0 ){
            stateFlag = 1;
        }
        else if((current == 0xC1) && stateFlag==1 ){
            stateFlag = 2;
        }
        else if((current != 0xC1) && stateFlag == 1 ){
            stateFlag = 0;
        }
        else if((current == 0x21) && stateFlag==2 ){
            stateFlag = 3;
        }
        else if((current != 0x21) && stateFlag==2 ){
            stateFlag = 0;
        }
        else if(stateFlag == 3){
            if(syncDelay == 3){
                syncDelay = 0;
                stateFlag = 0;
                readIndex++;
                
                return current;
            }
            else{
                syncDelay++;
            }
        }
        
        readIndex++;
        return 0x00;
    }
}