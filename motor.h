#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

#define PWM_MAX 1000 // Maximum PWM value for full speed
#define PWM_MIN 100 // Minimum PWM value to overcome motor deadzone
#define PID_DT_MS 20 // PID loop time step in milliseconds
#define PID_DT_S (PID_DT_MS / 1000.0f)

//==H-BRIDGE PIN ASSIGNMENTS==//
// 1. Motor A (Left): PWM on PA0 and PA1
// 2. Motor B (Right): PWM on PA2 and PA3

typedef struct PIDState{
    float Kp;
    float Ki;
    float Kd;
    float prev_error;
    float integral;
} PIDState;

void Motor_Init(void); // Setup TIM2 for PWM and PA0-4 for motor control
void Motor_SetPWM(int left_pwm, int right_pwm); // Writes PWM values in the TIM2 compare register
void PID_Init(PIDState* pid, float Kp, float Ki, float Kd); // Initializes the PID struct
float PID_Compute(PIDState* pid, uint16_t error); // Computes PID values
void Motor_Drive(float base_speed, float correction); // High level function that contains the previous functions to drive the motors with a base speed and a correction value from the PID controller

#endif