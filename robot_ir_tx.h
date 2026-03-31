#ifndef ROBOT_IR_TX_H
#define ROBOT_IR_TX_H

#include <stdint.h>

void ir_tx_init(void);
void ir_tx_test_led_on(void);
void ir_tx_test_pwm(uint16_t pulse_us);
void ir_tx_send(uint16_t pulse_us);

#endif