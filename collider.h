#ifndef COLLIDER_H
#define COLLIDER_H

#define NUNCHUK_ADDRESS 0x52
#define SLAVE_ADDRESS 0x42

#include <stdint.h>

extern unsigned char success;
extern unsigned short range;
extern uint16_t packet;

void I2C_Send_2byte(uint8_t address, uint16_t data);
void I2C_init (void);
void check_success_vl53(void);
void poll_vl53_I2C(void);


#endif