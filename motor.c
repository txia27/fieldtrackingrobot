#include <stdint.h>
#include <stdbool.h>
#include "motor.h"
#include "Common/Include/stm32l051xx.h"

void Motor_Init(void){
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2
    RCC->IOPENR |= BIT0;

    // Set pins as alternate function for PWM
    GPIOA->MODER = (GPIOA->MODER & ~(BIT1|BIT0)) | BIT1; // PA0
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xF << 0)) | (0x2 << 0); // Configure alternate function as timer
	GPIOA->OTYPER &= ~BIT0; // Push-pull

    GPIOA->MODER = (GPIOA->MODER & ~(BIT2|BIT3)) | BIT3; // PA1
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xF << 4)) | (0x2 << 4); // Configure alternate function as timer
    GPIOA->OTYPER &= ~BIT1; // Push-pull

    GPIOA->MODER = (GPIOA->MODER & ~(BIT4|BIT5)) | BIT5; // PA2
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xF << 8)) | (0x2 << 8); // Configure alternate function as timer
    GPIOA->OTYPER &= ~BIT2; // Push-pull,

    GPIOA->MODER = (GPIOA->MODER & ~(BIT6|BIT7)) | BIT7; // PA3
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xF << 12)) | (0x2 << 12); // Configure alternate function as timer
	GPIOA->OTYPER &= ~BIT3; // Push-pull

    //==Configure TIM2==//

    // 1. Prescaler to scale down TIM2 from SysTick(32MHz) to 20 kHz
    TIM2->PSC = 1;

    // 2. Set TIM2 Auto Reload Register (This is the period of the PWM)
    TIM2->ARR = PWM_MAX - 1;

    // 3. Capture and Compare Registers (CCR1-4). This controls the duty cycle. If the counter is below CCR, the pin is high.
    // Start them at 0
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;

    // 4. Output Compare Mode. (Tell each pin to operate in PWM mode 1 and enable the output)
    TIM2->CCMR1 = (6 << 4)  |  // CH1 PWM mode 1
                  (1 << 3)  |  // CH1 preload enable
                  (6 << 12) |  // CH2 PWM mode 1
                  (1 << 11);   // CH2 preload enable

    TIM2->CCMR2 = (6 << 4)  |  // CH3 PWM mode 1
                  (1 << 3)  |  // CH3 preload enable
                  (6 << 12) |  // CH4 PWM mode 1
                  (1 << 11);   // CH4 preload enable

    TIM2->CCER = BIT0 | BIT4 | BIT8 | BIT12;

    // Enable TIM2
    TIM2->CR1 |= BIT0;
}

/*
/ Input range: -1000 to 1000
/ 1000 = Full Forward
/ -1000 = Full Reverse
/ 0 = Brake
*/
void Motor_SetPWM(int left_pwm, int right_pwm){
    if(left_pwm > PWM_MAX) left_pwm = PWM_MAX;
    if(left_pwm < -PWM_MAX) left_pwm = -PWM_MAX;
    if(right_pwm > PWM_MAX) right_pwm = PWM_MAX;
    if(right_pwm < -PWM_MAX) right_pwm = -PWM_MAX;

    TIM2->CCR1 = (PWM_MAX + left_pwm) / 2;               // PA0: Motor A H-Bridge IN1
    TIM2->CCR2 = (PWM_MAX - left_pwm) / 2;               // PA1: Motor A H-Bridge IN2
    TIM2->CCR3 = (PWM_MAX + right_pwm) / 2;              // PA2: Motor B H-Bridge IN1
    TIM2->CCR4 = (PWM_MAX - right_pwm) / 2;              // PA3: Motor B H-Bridge IN2
}

// Tune PID values during testing
void PID_Init(PIDState* pid, float Kp, float Ki, float Kd){
    pid->Kp = Kp;
    pid->Kd = Kd;
    pid->Ki = Ki;
    pid->integral = 0;
    pid->prev_error = 0;
}

// error = adc_left - adc_right
// IF error is negative: right > left, right signal is stronger so robot drifted right
// IF error is positive: left > right, left signal is stronger so robot drifted left
// IF error = 0: left = right, robot is on the line!
float PID_Compute(PIDState* pid, float error){
    pid->integral += error * PID_DT_S;
    float derivative = (error - pid->prev_error) / PID_DT_S;
    pid->prev_error = error;

    if(pid->integral > PWM_MAX) pid->integral = PWM_MAX;
    if(pid->integral < -PWM_MAX) pid->integral = -PWM_MAX;

    return (pid->Kp*error) + (pid->Ki*pid->integral) + (pid->Kd*derivative);
}

void Motor_Drive(float base_speed, float correction){
    float left_pwm = base_speed + correction;
    float right_pwm = base_speed - correction;

    if((int)left_pwm > PWM_MAX) left_pwm = PWM_MAX;
    if((int)left_pwm < -PWM_MAX) left_pwm = -PWM_MAX;
    if((int)right_pwm > PWM_MAX) right_pwm = PWM_MAX;
    if((int)right_pwm < -PWM_MAX) right_pwm = -PWM_MAX;

    if(left_pwm > 0 && left_pwm < PWM_MIN) left_pwm = PWM_MIN;
    if(left_pwm < 0 && left_pwm > -PWM_MIN) left_pwm = -PWM_MIN;
    if(right_pwm > 0 && right_pwm < PWM_MIN) right_pwm = PWM_MIN;
    if(right_pwm < 0 && right_pwm > -PWM_MIN) right_pwm = -PWM_MIN;

    Motor_SetPWM((int)right_pwm,(int)left_pwm);
}

void turnLeft(void){
    Motor_SetPWM(-800,1000); // Turn left
}

void turnRight(void){
    Motor_SetPWM(1000, -800);  // Turn right
}

void robotForward(void){
    Motor_SetPWM(1000,1000);
}

void robotBackward(void){
    Motor_SetPWM(-1000,-1000);
}

void robotStop(void){
    Motor_SetPWM(0,0);
}

void robotSpin(void){
    Motor_SetPWM(1000,-1000);
}
