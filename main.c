#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "Common/Include/stm32l051xx.h"
#include "motor.h"
#include "ADC.h"
#include "collision.h"
#include "decoder.h"
#include "Common/Include/serial.h"

#define F_CPU 32000000L

//=========ABSTRACT=========//
// Measures the period of a square wave on pin PA8

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

#define PIN_PERIOD (GPIOA->IDR&BIT8)

// GetPeriod() seems to work fine for frequencies between 300Hz and 600kHz.
// 'n' is used to measure the time of 'n' periods; this increases accuracy.
long int GetPeriod (int n)
{
	int i;
	unsigned int saved_TCNT1a, saved_TCNT1b;
	
	SysTick->LOAD = 0xffffff;  // 24-bit counter set to check for signal present
	SysTick->VAL = 0xffffff; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while (PIN_PERIOD!=0) // Wait for square wave to be 0
	{
		if(SysTick->CTRL & BIT16) return 0;
	}
	SysTick->CTRL = 0x00; // Disable Systick counter

	SysTick->LOAD = 0xffffff;  // 24-bit counter set to check for signal present
	SysTick->VAL = 0xffffff; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while (PIN_PERIOD==0) // Wait for square wave to be 1
	{
		if(SysTick->CTRL & BIT16) return 0;
	}
	SysTick->CTRL = 0x00; // Disable Systick counter
	
	SysTick->LOAD = 0xffffff;  // 24-bit counter reset
	SysTick->VAL = 0xffffff; // load the SysTick counter to initial value
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	for(i=0; i<n; i++) // Measure the time of 'n' periods
	{
		while (PIN_PERIOD!=0) // Wait for square wave to be 0
		{
			if(SysTick->CTRL & BIT16) return 0;
		}
		while (PIN_PERIOD==0) // Wait for square wave to be 1
		{
			if(SysTick->CTRL & BIT16) return 0;
		}
	}
	SysTick->CTRL = 0x00; // Disable Systick counter

	return 0xffffff-SysTick->VAL;
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

 void main(void)
{
	ADC_Init();

	volatile int uart_flag = 0;
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
			turnRight();
		}

		if (signal_flag) // check if a new signal has been captured
		{
			signal_flag = 0; // reset flag
			//signal_length = pulse_width; // store the pulse width of the captured signal
			int command = decode(pulse_width); // decode the signal length to determine the command
			
			if (command > 0) {
			printf("Decoded Command: %d, pulse width: %d\r\n", command, pulse_width); // print the decoded command for debugging

				// Execute the command (this is where you would add your motor control logic)
				switch (command) {
					case 1:
						// Stop
						robotStop();
						printf("Stopping\r\n");
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
						break;

					case 8:
						// Start predetermined path
						printf("Path x selected\r\n");
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
				}

			}
			pulse_width = 0;
		}

		waitms(100);
	}

*/
	Motor_Init();
	PIDState pid;
	PID_Init(&pid, 0.2f, 0.0f, 0.01f); // Kp = 0.1, Ki = 0, Kd = 0.0 (Tune these as we test)

	float base_speed = 600.0f; // Speed can be between 0 to 1000, tune as we test
	uint16_t adcval;
	uint16_t adcval2;
	uint16_t testval = 4011;
    while (1)
    {
        /*uint16_t left   = ADC_Read_Channel(4);
		printf("Left: %d\r\n", left);
        uint16_t center = ADC_Read_Channel(5);
		printf("Center: %d\r\n", center);
        uint16_t right  = ADC_Read_Channel(6);
		printf("Right: %d\r\n", right);

		
       
        for (volatile int i = 0; i < 200000; i++);*/

		
		/*if (egetc() != 0)
		{
			uart_flag = 1;
		}


		if(uart_flag){
			uart_flag = 0;
			Motor_SetPWM(0, 0); // stop the robot
			
			char buf[32];
			float kp, ki, kd;

			printf("\r\nEnter Kp: ");
			fflush(stdout);
			egets_echo(buf,sizeof(buf));
			kp = atof(buf);

			printf("Enter Ki: ");
			fflush(stdout);
			egets_echo(buf,sizeof(buf));
			ki = atof(buf);

			printf("Enter Kd: ");
			fflush(stdout);
			egets_echo(buf,sizeof(buf));
			kd = atof(buf);

			PID_Init(&pid, kp, ki, kd);
			printf("PID updated! Kp=%.3f Ki=%.3f Kd=%.3f\r\n\n\n", kp, ki, kd);
			fflush(stdout);
		}*/

		adcval = ADC_Read_Channel(4);
		adcval2 = ADC_Read_Channel(6);
		float error = (float)adcval - (float)adcval2 ; // Implement these variables later
		float correction = PID_Compute(&pid, error);
		Motor_Drive(base_speed, correction);

		//printf("\x1b[2J\x1b[1;1H");
		//printf("error=%.3f correction=%.3f ", error, correction);
		//printf("leftadc=%d rithadc=%d test=%d\r", adcval, adcval2, testval);

		waitms(PID_DT_MS);
    }
		
		/*Motor_Init();
	PIDState pid;
	PID_Init(&pid, 0.1f, 0.0f, 0.0f); // Kp = 0.1, Ki = 0, Kd = 0.0 (Tune these as we test)

	float base_speed = 600.0f; // Speed can be between 0 to 1000, tune as we test

	while(1){
		uint16_t error = ADC_Read_Channel(4) - ADC_Read_Channel(6); // Implement these variables later
		float correction = PID_Compute(&pid, error);
		Motor_Drive(base_speed, correction);
		waitms(PID_DT_MS);
	}*/
	

	//printf("Peter Lake BBL\r\n");*/
}



	

	