#ifndef ROBOT_IR_TX_H
#define ROBOT_IR_TX_H

#define SIG_MODE0 16000
#define SIG_MODE1 17000
#define SIG_MODE2 18000
#define SIG_MANUAL 19000
#define SIG_AUTO  20000
#define SIG_INTERSECTION 21000
#define SIG_COLLISION 22000
#define SIG_FINISH 23000
#define SIG_START 24000

#include <stdint.h>

void ir_tx_init(void);
void ir_tx_test_led_on(void);
void ir_tx_test_pwm(uint16_t pulse_us);
void ir_tx_send(uint16_t pulse_us);
void transmit_pulse_us(uint16_t duration_us);

#endif