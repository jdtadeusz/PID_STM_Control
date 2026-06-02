#include "pid_controller.h"
#include <math.h>
#include <stdint.h>

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float min, float max) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->out_min = min; 
    pid->out_max = max;
    pid->integral_limit = 500.0f; 
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_measurement = 0.0f;
    pid->setpoint = 0.0f;
    pid->dt = 0.015f; // 15ms default
}

float PID_Compute(PID_TypeDef *pid, float current_val) {
    float error = current_val - pid->setpoint;

    // ANTI-WINDUP (Reset after passing the target setpoint)
    if ((error > 0.0f && pid->last_error < 0.0f) || 
        (error < 0.0f && pid->last_error > 0.0f)) {
        
    } else {
        pid->integral += error * pid->dt; 
    }

    // Proportional Term (P)
    float effective_kp;
    if (error < 0) {
        // Ball is too high — gently decrease throttle
        effective_kp = pid->Kp * 0.4f;
    } else {
        // Ball is too low — normal system response
        effective_kp = pid->Kp;
    }

    float P = effective_kp * error;
    
    // Integral Term (I)
    pid->integral += error * pid->dt;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float I = pid->Ki * pid->integral;
    
    // Derivative Term (D)
    float D = -pid->Kd * (current_val - pid->last_measurement) / pid->dt;
    
    pid->last_error = error;
    pid->last_measurement = current_val;

    float output = P + I + D;

    // Output saturation limits
    if (output > pid->out_max) output = pid->out_max;
    else if (output < pid->out_min) output = pid->out_min;

    return output;
}

void PID_SetSetpoint(PID_TypeDef *pid, float setpoint) {
    pid->setpoint = setpoint;
}

void PID_SetDt(PID_TypeDef *pid, float dt) {
    pid->dt = dt;
}