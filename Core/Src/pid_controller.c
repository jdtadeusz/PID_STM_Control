#include "pid_controller.h"

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float min, float max) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->out_min = min; 
    pid->out_max = max;
    pid->integral_limit = 500.0f; 
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->setpoint = 0.0f; 
}

float PID_Compute(PID_TypeDef *pid, float current_val) {
    // Obliczanie błędu 
    float error = pid->setpoint - current_val;

    // P 
    float P = pid->Kp * error;

    // I 
    pid->integral += error;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    else if (pid->integral < pid->integral_limit) pid->integral = -pid->integral_limit;
    float I = pid->Ki * pid->integral;
    
    // D
    float D = pid->Kd * (error - pid->last_error);
    pid->last_error = error;

    // Suma
    float output = P + I + D;

    if (output > pid->out_max) output = pid->out_max;
    else if (output < pid->out_min) output = pid->out_min;

    return output;
}

void PID_SetSetpoint(PID_TypeDef *pid, float setpoint) {
    pid->setpoint = setpoint;
}