#ifndef COLLISION_H
#define COLLISION_H

void wait_1ms(void);
void coll_loop(unsigned short*);
void waitms(int len);
unsigned char i2c_read_addr8_data8(unsigned char address, unsigned char * value);
unsigned char i2c_read_addr8_data16(unsigned char address, unsigned short * value);
unsigned char i2c_write_addr8_data8(unsigned char address, unsigned char value);
void validate_I2C_interface(void);

#endif
