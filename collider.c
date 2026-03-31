#include "collider.h"
#include "../Common/Include/stm32l051xx.h"
#include "../Common/Include/serial.h"
#include "stdio.h"
#include "vl53l0x.h"

unsigned char success;
unsigned short range;
uint16_t packet;

void I2C_init (void)
{
	RCC->IOPENR  |= BIT1;// peripheral clock enable for port B (I2C pins are in port B)
	RCC->APB1ENR  |= BIT21; // peripheral clock enable for I2C1 (page 177 of RM0451 Reference manual)
	
	//Configure PB6 for I2C1_SCL, pin 29 in LQFP32 package (page 44 of datasheet en.DM00108219.pdf)
	GPIOB->MODER = (GPIOB->MODER & ~(BIT12|BIT13)) | BIT13; // PB6 AF-Mode (page 200 of RM0451 Reference manual)
	GPIOB->AFR[0] |= BIT24; // AF1 selected (page 204 of RM0451 Reference manual)
	
	//Configure PB7 for I2C1_SDA, pin 30 in LQFP32 package (page 44 of datasheet en.DM00108219.pdf)
	GPIOB->MODER = (GPIOB->MODER & ~(BIT14|BIT15)) | BIT15; // PB7 AF-Mode (page 200 of RM0451 Reference manual)
	GPIOB->AFR[0] |= BIT28; // AF1 selected (page 204 of RM0451 Reference manual)
	
	GPIOB->OTYPER   |= BIT6 | BIT7; // I2C pins (PB6 and PB7)  must be open drain
	GPIOB->OSPEEDR  |= BIT12 | BIT14; // Medium speed for PB6 and PB7

	// This configures the I2C clock (Check page 564 of RM0451).  SCLK must be 100kHz or less.
	I2C1->TIMINGR = (uint32_t)0x70420f13;
	I2C1->CR1 = I2C_CR1_PE; // Enable I2C1 peripheral
}

void I2C_Send_2byte(uint8_t address, uint16_t data)
{
    I2C1->CR1 = I2C_CR1_PE;
    I2C1->CR2 = ((uint32_t)(address << 1)) | (2 << 16) | I2C_CR2_START; // atomic, same pattern as I2C_Send

    while(!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = (data >> 8) & 0xFF; // high byte first

    while(!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = data & 0xFF; // low byte

    while(!(I2C1->ISR & I2C_ISR_TC));
    I2C1->CR2 |= I2C_CR2_STOP;
}

void check_success_vl53(void){
	success = vl53l0x_init();
	if(success)
	{
		printf("VL53L0x initialization succeeded.\r\n");
		fflush(stdout);
	}
	else
	{
		printf("VL53L0x initialization failed.\r\n");
		fflush(stdout);
	}
}

//Sends packets of vl53 to slave device. Packet format: 0x1 (12 bits of range data)
void poll_vl53_I2C(void){
	
	success = vl53l0x_read_range_single(&range);
		if(success)
		{
			printf("D: %4u (mm)   \r\n", range);
	        
	        // The implementation of printf() in newlib doesn't transmit until a '\n'
	        // is found or the buffer is full, so flush(stdout); to transmit now!!!
	        fflush(stdout); 

			
		}

}

