#define ERROR 150

void initialize_decoder(void);
void initialize_timer22(void);
int decode(int signal_length);

extern volatile int command_signal;
extern volatile int pulse_width;
extern volatile int signal_flag;
extern volatile int signal_length = 0;
extern volatile int signal_start = 0;
extern volatile int capture = 0;
