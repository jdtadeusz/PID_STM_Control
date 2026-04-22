#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#include <stdint.h>

typedef struct{
    // Nastawy
    float Kp; 
    float Ki;
    float Kd;

    // Stan wewn.
    float setpoint;
    float integral; 
    float last_error; 

    // Ograniczenia 
    float out_min; 
    float out_max; 
    float integral_limit; 

    // Czas próbkowania 
    float dt; 
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float min, float max);
float PID_Compute(PID_TypeDef *pid, float current_val);
void PID_SetSetpoint(PID_TypeDef *pid, float setpoint);

#endif