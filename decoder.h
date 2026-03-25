#define ERROR 150

void initialize_decoder(void);
void initialize_timer22(void);
int decode(int signal_length);

extern volatile int command_signal;
