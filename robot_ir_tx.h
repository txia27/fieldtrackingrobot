#ifndef IR_TX_H
#define IR_TX_H

#include <stdint.h>

void ir_tx_init(void);
void ir_tx_send(uint16_t pulse_us);

#endif