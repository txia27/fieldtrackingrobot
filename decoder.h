#ifndef DECODER_H
#define DECODER_H
#define ERROR 500

void initialize_decoder(void);
void initialize_timer22(void);
int decode(int signal_length);

extern volatile int command_signal;
extern volatile int pulse_width;
extern volatile int signal_flag;
extern volatile int signal_length;
extern volatile int signal_start;
extern volatile int capture;

#endif