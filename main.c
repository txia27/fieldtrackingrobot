#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Common/Include/stm32l051xx.h"
#include "motor.h"
#include "ADC.h"
#include "collision.h"
#include "decoder.h"
#include "uart.h"
#include "Common/Include/serial.h"

#define F_CPU 32000000L

void delay(int dly)
{
	while( dly--);
}

void wait_1ms(void)
{
	SysTick->LOAD = (F_CPU/1000L) - 1;  // set reload register, counter rolls over from zero, hence -1
	SysTick->VAL = 0; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
	// For SysTick info check the STM32L0xxx Cortex-M0 programmiE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while((SysTick->CTRL & BIT16)==0); // Bit 16 is the COUNTFLAG.  True when counter rolls over from zero.
	SysTick->CTRL = 0x00; // Disable Systick counter
}

void waitms(int len)
{
	while(len--) wait_1ms();
}

// LQFP32 pinout
//             ----------
//       VDD -|1       32|- VSS
//      PC14 -|2       31|- BOOT0
//      PC15 -|3       30|- PB7
//      NRST -|4       29|- PB6
//      VDDA -|5       28|- PB5
//       PA0 -|6       27|- PB4
//       PA1 -|7       26|- PB3
//       PA2 -|8       25|- PA15
//       PA3 -|9       24|- PA14
//       PA4 -|10      23|- PA13
//       PA5 -|11      22|- PA12
//       PA6 -|12      21|- PA11
//       PA7 -|13      20|- PA10 (Reserved for RXD)
//       PB0 -|14      19|- PA9  (Reserved for TXD)
//       PB1 -|15      18|- PA8  (Measure the period at this pin)
//       VSS -|16      17|- VDD
//             ----------
/*int detect_intersection(uint16_t adcval, uint16_t adcval2, uint16_t adccenter){
	return (adccenter>adcval && adccenter>adcval2);
}*/

void main(void)
{	
	UART2_Init();
	printf("entered main");

	bool startFlag = false;
	int node_count = -1;
	int mode = 0;
	int clear_intersection = 100;
	char path[16] = "";
	
	int command = 0;
	signal_flag = 0;
/*
	unsigned short range=0;

	Motor_Init();
	
	initialize_decoder();
	initialize_timer22();

	coll_init();


	while (1)
	{
		coll_loop(&range);
		while(range>50){
			turnRight();		// not too sure what this block does, is it collision detection??? - Charles
		}
	}

*/
	printf("test 1");
	ADC_Init();
	Motor_Init();
	initialize_decoder();
	initialize_timer22();

	printf("pre loop");

	while(!startFlag){
		printf("post loop");

		if (signal_flag) // check if a new signal has been captured
		{
			signal_flag = 0; // reset flag
			//signal_length = pulse_width; // store the pulse width of the captured signal
			printf("pre decode");
			command = decode(pulse_width); // decode the signal length to determine the command
			
			printf("%d %d   ", command, pulse_width);
			
			if (command > 0) {
			//printf("Decoded Command: %d, pulse width: %d\r\n", command, pulse_width); // print the decoded command for debugging

				// Execute the command (this is where you would add your motor control logic)
				switch (command) {
					case 1:
						// Stop
						robotStop();
						printf("Case 1\r\n");
						break;

					case 2:
						// Turn Left
						turnLeft();
						waitms(2000);
						robotStop();
						printf("Turning Left\r\n");
						break;

					case 3:
						// Turn right
						turnRight();
						waitms(2000);
						robotStop();
						printf("Turning right\r\n");
						break;

					case 4:
						// Move forward
						robotForward();
						waitms(2000);
						robotStop();
						printf("Moving Forward\r\n");
						break;

					case 5:
						// Move reverse
						robotBackward();
						waitms(2000);
						robotStop();
						printf("Moving Backward\r\n");
						break;

					case 6:
						// Turn around 180 degrees
						printf("Turning around\r\n");
						break;

					case 7:
						// Cycle modes/pathes
						printf("Cycling modes\r\n");
						
						if (mode < 2) {
							mode++;
						} 
						else {
							mode = 0;
						}

						break;

					case 8:
						// Start predetermined path

						if (mode == 0) {
							strcpy (path, "FLLFRLRS");
						}
						else if (mode == 1) {
							strcpy (path, "LRLRFFS");
						}
						else {
							strcpy(path, "RFRLRLFS");
						}
						
						startFlag = true;
						
						printf("Path %d selected\r\n", mode);
						break;

					case 9:
						// Move forward-right
						turnRight();
						waitms(2000);
						robotStop();
						robotForward();
						waitms(2000);
						robotStop();
						printf("Moving Forward-Right\r\n");
						break;

					case 10:
						// Move forward-left
						turnLeft();
						waitms(2000);
						robotStop();
						robotForward();
						waitms(2000);
						robotStop();
						printf("Moving Forward-Left\r\n");
						break;

					case 11:
						// Move back-right
						turnLeft();
						waitms(2000);
						robotStop();
						robotBackward();
						waitms(2000);
						robotStop();
						printf("Moving Backwards-Right\r\n");
						break;

					case 12:
						// Move reverse
						turnRight();
						waitms(2000);
						robotStop();
						robotBackward();
						waitms(2000);
						robotStop();
						printf("Moving Backwards-left\r\n");
						break;
						
					default:
						printf("robot idling\r\n");
						robotStop();
						break;
				}

			}
			pulse_width = 0;
			command = 0;
		}

		waitms(100);

	}


	PIDState pid;
	PID_Init(&pid, 0.2f, 0.05f, 0.05f); // Kp, Ki, Kd
	float base_speed = 700.0f; // Speed can be between 0 to 1000, tune as we test
	uint16_t adcval;
	uint16_t adcval2;
	uint16_t adccenter;
	
    while (1)
    {

		adcval = ADC_Read_Channel(4); 
		adcval2 = ADC_Read_Channel(6);
		adccenter = ADC_Read_Channel(5);
		float error = (float)adcval - (float)adcval2 ; // Implement these variables later
		float correction = PID_Compute(&pid, error);
		Motor_Drive(base_speed, correction);

		/*if ((detect_intersection(adcval, adcval2, adccenter) == 1) && clear_intersection > 250){
 			node_count++;
			clear_intersection = 0;
			//printf("Intersection detected! Total count: %d\r\n", node_count);
			
			if (path[node_count] == 'F') {
				robotForward();
				waitms(500);
			} else if (path[node_count] == 'L') {
				turnLeft();
				robotForward();
				waitms(500);
			} else if (path[node_count] == 'R') {
				turnRight();
				robotForward();
				waitms(500);
			}
			else if (path[node_count] == 'S') {
				robotStop();

				while(1){
					// robot finished, jut an infinite loop for now, can add something else if we want
				}

				// play ending song???
			}
			else {

			}
		}*/

		//clear_intersection++;
				waitms(PID_DT_MS);
    }
		
}



	

	