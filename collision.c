#include <stdio.h>
#include "Common/Include/stm32l051xx.h"
#include "Common/Include/serial.h"
#include "vl53l0x.h"

// LQFP32 pinout
//              ----------
//        VDD -|1       32|- VSS
//       PC14 -|2       31|- BOOT0
//       PC15 -|3       30|- PB7 (I2C1_SDA)
//       NRST -|4       29|- PB6 (I2C1_SCL)
//       VDDA -|5       28|- PB5
//        PA0 -|6       27|- PB4
//        PA1 -|7       26|- PB3
//        PA2 -|8       25|- PA15
//        PA3 -|9       24|- PA14
//        PA4 -|10      23|- PA13
//        PA5 -|11      22|- PA12
//        PA6 -|12      21|- PA11
//        PA7 -|13      20|- PA10 (Reserved for RXD)
//        PB0 -|14      19|- PA9  (Reserved for TXD)
//        PB1 -|15      18|- PA8
//        VSS -|16      17|- VDD
//              ----------

#define F_CPU 32000000L
#define NUNCHUK_ADDRESS 0x52

extern void wait_1ms(void);
extern void waitms(int len);

unsigned char i2c_read_addr8_data8(unsigned char address, unsigned char * value)
{
	
	// First we send the address we want to read from:
	I2C1->CR1 = I2C_CR1_PE;
	I2C1->CR2 = I2C_CR2_AUTOEND | (1 << 16) | (0x52);
	I2C1->CR2 |= I2C_CR2_START; // Go
	while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Wait for address to go

	I2C1->TXDR = address; // send data
	while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Check Tx empty

	waitms(1);
	
	// Second: we gatter the data sent by the slave device
	I2C1->CR1 = I2C_CR1_PE | I2C_CR1_RXIE;
	I2C1->CR2 = I2C_CR2_AUTOEND | (1 << 16) | I2C_CR2_RD_WRN | (0x52); // Read one byte from slave 0x52
	I2C1->CR2 |= I2C_CR2_START; // Go
    
	while ((I2C1->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE) {} // Wait for data to arrive
	*value=I2C1->RXDR; // Reading 'receive' register clears RXNE flag

	waitms(1);

	return 1;
}

unsigned char i2c_read_addr8_data16(unsigned char address, unsigned short * value)
{
	// First we send the address we want to read from:
	I2C1->CR1 = I2C_CR1_PE;
	I2C1->CR2 = I2C_CR2_AUTOEND | (1 << 16) | (0x52);
	I2C1->CR2 |= I2C_CR2_START; // Go
	while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Wait for address to go

	I2C1->TXDR = address; // send data
	while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Check Tx empty

	waitms(1);
	
	// Second: we gatter the data sent by the slave device
	I2C1->CR1 = I2C_CR1_PE | I2C_CR1_RXIE;
	I2C1->CR2 = I2C_CR2_AUTOEND | (2<<16) | I2C_CR2_RD_WRN | (0x52); // Read two bytes from slave 0x52
	I2C1->CR2 |= I2C_CR2_START; // Go
    
	while ((I2C1->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE) {} // Wait for data to arrive
	*value=I2C1->RXDR*256; // Reading 'receive' register clears RXNE flag
   
	while ((I2C1->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE) {} // Wait for data to arrive
	*value+=I2C1->RXDR; // Reading 'receive' register clears RXNE flag
    
	waitms(1);
	
	return 1;
}

unsigned char i2c_write_addr8_data8(unsigned char address, unsigned char value)
{
	I2C1->CR1 = I2C_CR1_PE;
	I2C1->CR2 = I2C_CR2_AUTOEND | (2 << 16) | (0x52);
	I2C1->CR2 |= I2C_CR2_START;
	while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Wait for address to go

	I2C1->TXDR = address; // send register address
	while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Check Tx empty

	I2C1->TXDR = value; // send data
	while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Check Tx empty

	waitms(1);

	return 1;
}

// From the VL53L0X datasheet:
//
// The registers shown in the table below can be used to validate the user I2C interface.
// Address (After fresh reset, without API loaded)
//    0xC0 0xEE
//    0xC1 0xAA
//    0xC2 0x10
//    0x51 0x0099 (0x0096 after initialization)
//    0x61 0x0000
//
// Not needed, but it was useful to debug the I2C interface, so left here.
void validate_I2C_interface (void)
{
	unsigned char val8 = 0;
	unsigned short val16 = 0;
	
    printf("\n");   
    
    i2c_read_addr8_data8(0xc0, &val8);
    printf("Reg(0xc0): 0x%02x\r\n", val8);

    i2c_read_addr8_data8(0xc1, &val8);
    printf("Reg(0xc1): 0x%02x\r\n", val8);

    i2c_read_addr8_data8(0xc2, &val8);
    printf("Reg(0xc2): 0x%02x\r\n", val8);
    
    i2c_read_addr8_data16(0x51, &val16);
    printf("Reg(0x51): 0x%04x\r\n", val16);

    i2c_read_addr8_data16(0x61, &val16);
    printf("Reg(0x61): 0x%04x\r\n", val16);
    
    printf("\r\n");
}

void coll_loop(unsigned short* range){
	unsigned char success;

	success = vl53l0x_read_range_single(range);
	if(success)
	{
		printf("D: %4u (mm)\r", *range);    // The implementation of printf() in newlib doesn't transmit until a '\n'
	     // is found or the buffer is full, so flush(stdout); to transmit now!!!
	    fflush(stdout); 
	    waitms(100);
		}
}