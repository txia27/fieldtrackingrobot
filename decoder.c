//  pinout
//		     ---------
//	VDD   -	|1		32|  -  VSS
//	PC14  -	|2		31|  -  BOOT0
//	PC15  -	|3		30|  -  PB7
//	NRST  -	|4		29|  -  PB6
//	VDDA  -	|5		28|  -  PB5
//	PA0   -	|6		27|  -  PB4
//	PA1   -	|7		26|  -  PB3
//	PA2   -	|8		25|  -  PA15
//	PA3   -	|9		24|  -  PA14
//	PA4   -	|10	 	23|  -  PA13
//	PA5   -	|11		22|  -  PA12
//	PA6   -	|12		21|  -  PA11
//	PA7   -	|13		20|  -  PA10
//	PB0   -	|14		19|  -  PA9
//	PB1   -	|15		18|  -  PA8
//	VSS   -	|16		17|  -  VDD
//		     ---------

#include <stdio.h>
#include <math.h>
#include "decoder.h"


//need to configure pin (likely PA1 or PA6) as input from receiver wiring
//configure timer 2 to measure the period of the signal at that pin, and use that to determine which command was sent by the encoder

volatile int command_signal = 0;
volatile int signal_length = 0;
const float commands[10] = {1,2,3,4,5,6,7,8,9,10};
// change these numbers depending on how the encoder is configured (eg. 0.001s, 0.002s)

int decode (signal_length) {

    if      (signal_length < commands[0] - ERROR) command_signal = 0; // default state
    else if (signal_length < commands[1] - ERROR) command_signal = 1; // stop moving (0.5 - 1.5)
    else if (signal_length < commands[2] - ERROR) command_signal = 2; // turn right (1.5 - 2.5)
    else if (signal_length < commands[3] - ERROR) command_signal = 3; // turn left (2.5 - 3.5)
    else if (signal_length < commands[4] - ERROR) command_signal = 4; // move forward (3.5 - 4.5)
    else if (signal_length < commands[5] - ERROR) command_signal = 5; // move reverse (4.5 - 5.5)
    else if (signal_length < commands[6] - ERROR) command_signal = 6; // turn around (5.5 - 6.5)
    else if (signal_length < commands[7] - ERROR) command_signal = 7; // predetermined path 1 (6.5 - 7.5)
    else if (signal_length < commands[8] - ERROR) command_signal = 8; // predetermined path 2 (7.5 - 8.5)
    else if (signal_length < commands[9] - ERROR) command_signal = 9; // predetermined path 3 (8.5 - 9.5)

    else command_signal = 67; // error state, send error message (beep?)
    // decoder lookup table to determine which command was sent

    return command_signal;
}


int main(void)
{
    
    while(1)
    {

        // if falling edge detected (signal received), do this

        signal_length = measure_signal_length(n);
        // measures the length of the signal received, and stores it in signal_length

        command_signal = decode(signal_length);
        // decodes the signal_length into a command_signal (0-7), and stores it in command_signal
        
        //send command_signal to the rest of the system

        command_signal = 0; 
        // reset command signal for next loop
    }
}


float measure_signal_length(void)
{
    
}
