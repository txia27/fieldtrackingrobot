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
bool detect_intersection(uint16_t a, uint16_t b, uint16_t c){
	if(c > a && c > b){
		return true;
	}
	else{
		return false;
	}
}

void main(void)
{	
	UART2_Init();

	bool startFlag = false;
	int node_count = -1;
	int mode = 0;
	int clear_intersection = 250;
	int path[8];
	
	int command = 0;
	signal_flag = 0;

	unsigned long ms = 0;
	unsigned long last_time_press1 = 0;
	unsigned long last_time_press2 = 0;
/*


	while (1)
	{
		coll_loop(&range);
		while(range>50){
			turnRight();		// not too sure what this block does, is it collision detection??? - Charles
		}
	}

*/
	ADC_Init();
	Motor_Init();
	initialize_decoder();
	initialize_timer22();

	while(1) {

		ms = 0;
		last_time_press1 = 0;
		last_time_press2 = 0;
		node_count = -1;
		clear_intersection = 250;

		while(!startFlag){

			if (signal_flag) // check if a new signal has been captured
			{
				signal_flag = 0; // reset flag
				//signal_length = pulse_width; // store the pulse width of the captured signal
				command = decode(pulse_width); // decode the signal length to determine the command
				
				if (command > 0) {
				//printf("Decoded Command: %d, pulse width: %d\r\n", command, pulse_width); // print the decoded command for debugging

					// Execute the command (this is where you would add your motor control logic)
					switch (command) {
						case 1:
							// Stop
							robotStop();
							waitms(100);
							//printf("Case 1\r\n");
							break;

						case 2:
							// Turn Left
							turnLeft();
							waitms(100);
							robotStop();
							//printf("Turning Left\r\n");
							break;

						case 3:
							// Turn right
							turnRight();
							waitms(100);
							robotStop();
							//printf("Turning right\r\n");
							break;

						case 4:
							// Move forward
							robotForward();
							waitms(100);
							robotStop();
							//printf("Moving Forward\r\n");
							break;

						case 5:
							// Move reverse
							robotBackward();
							waitms(100);
							robotStop();
							//printf("Moving Backward\r\n");
							break;

						case 6:
							// Turn around 180 degrees
							if (ms - last_time_press1 > COOLDOWN) {
								turnRight();
								waitms(4400);
								//printf("Turning around\r\n");
								last_time_press1 = ms;
							}
							break;

						case 7:
							// Cycle modes/pathes

							if (ms - last_time_press2 > COOLDOWN) {
								//printf("Cycling modes\r\n");
								
								if (mode < 2) {
									mode++;
								} 
								else {
									mode = 0;
								}
								last_time_press2 = ms;
							}

							break;

						case 8:
							// Start predetermined path

							if (mode == 0) {
								int buffer [8] = {0,1,1,0,2,1,2,3};
								size_t size = sizeof(buffer);
								memcpy(path, buffer, size);
							}
							else if (mode == 1) {
								int buffer[8] = {1,2,1,2,0,0,3,3};
								size_t size = sizeof(buffer);
								memcpy(path,buffer,size);
							}
							else {
								int buffer[8] = {2,0,2,1,2,1,0,3};
								size_t size = sizeof(buffer);
								memcpy(path,buffer,size);
							}
							
							startFlag = true;
							
							printf("Path %d selected\r\n", mode);
							break;

						case 9:
							// Move forward-right
							turnRight();
							waitms(100);
							robotStop();
							robotForward();
							waitms(100);
							robotStop();
							//printf("Moving Forward-Right\r\n");
							break;

						case 10:
							// Move forward-left
							turnLeft();
							waitms(100);
							robotStop();
							robotForward();
							waitms(100);
							robotStop();
							//printf("Moving Forward-Left\r\n");
							break;

						case 11:
							// Move back-right
							turnLeft();
							waitms(100);
							robotStop();
							robotBackward();
							waitms(100);
							robotStop();
							//printf("Moving Backwards-Right\r\n");
							break;

						case 12:
							// Move reverse
							turnRight();
							waitms(100);
							robotStop();
							robotBackward();
							waitms(100);
							robotStop();
							//printf("Moving Backwards-left\r\n");
							break;
							
						default:
							//printf("robot idling\r\n");
							robotStop();
							break;
					}

				}
				pulse_width = 0;
				command = 0;
			}
			ms += 100;
			waitms(100);

		}



		PIDState pid;
		PID_Init(&pid, 0.2f, 0.05f, 0.05f); // Kp, Ki, Kd
		float base_speed = 600.0f; // Speed can be between 0 to 1000, tune as we test
		uint16_t adcval;
		uint16_t adcval2;
		uint16_t adccenter;
		
		while (startFlag)
		{

			//adcval = ADC_Read_Channel(4); 
			//adcval2 = ADC_Read_Channel(6);
			//adccenter = ADC_Read_Channel(5);
			ADC_Read_All(&adcval, &adccenter, &adcval2);
			float error = (float)adcval - (float)adcval2 ; // Implement these variables later
			float correction = PID_Compute(&pid, error);
			Motor_Drive(base_speed, correction);
			printf("ADC Values: %d %d %d | %d\r", adcval, adcval2, adccenter, 
			detect_intersection(adcval, adcval2, adccenter));
			fflush(stdout);

			if (signal_flag) // check if a new signal has been captured
			{
				signal_flag = 0; // reset flag
				//signal_length = pulse_width; // store the pulse width of the captured signal
				command = decode(pulse_width); // decode the signal length to determine the command
				
				if (command == 1) {
					robotStop();
					startFlag = false;
					while(!startFlag) {
						if (signal_flag) // check if a new signal has been captured
						{
							signal_flag = 0; // reset flag
							//signal_length = pulse_width; // store the pulse width of the captured signal
							command = decode(pulse_width); // decode the signal length to determine the command
							
							if (command == 8) {
								startFlag = true;
							}
							pulse_width = 0;
							command = 0;
						}
					}
				}
				pulse_width = 0;
				command = 0;
			}

			//Auto Recovery Mode
			if(adcval < 100 && adcval2 < 100 && adccenter < 100){
				robotStop();
				waitms(100);

				robotSpin();
				waitms(1000);

				robotForward();
				waitms(1500);
			}

			if ((detect_intersection(adcval, adcval2, adccenter)) && (clear_intersection > 500) && (adccenter > 100)) {
				printf("Intersection Detected\n");

				node_count++;
				clear_intersection = 0;
				//printf("Intersection detected! Total count: %d\r\n", node_count);
				
				if (path[node_count] == 0) {
					printf("F\n");
					robotForward();
				} else if (path[node_count] == 1) {
					printf("L\n");
					robotStop();
					waitms(2000);
					turnLeft();
					waitms(700);
					robotForward();
				} else if (path[node_count] == 2) {
					printf("R\n");
					robotStop();
					waitms(2000);
					turnRight();
					waitms(700);
					robotForward();
				}
				else if (path[node_count] == 3) {
					robotStop();
					printf("S");

					startFlag = false;

					// play ending song???
				}
				else {
					printf("else\n");
				}
			}

			clear_intersection++;
			waitms(PID_DT_MS);
		}

	}
		
}



	

	