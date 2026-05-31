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
    pid->dt = 0.015f; // 15ms domyślnie
}

float PID_Compute(PID_TypeDef *pid, float current_val) {
    // Obliczanie błędu 
    // Dla czujnika na górze: duża odległość = piłeczka nisko = potrzeba wyższa prędkość
    // Dlatego: error = measurement - setpoint (odwrócone)
    float error = current_val - pid->setpoint;

    // P (proporcjonalny)
    float P = pid->Kp * error;

    // I (całkujący) - z uwzględnieniem czasu próbkowania
    pid->integral += error * pid->dt;
    
    // Ograniczenie całki (anti-windup)
    if (pid->integral > pid->integral_limit) 
        pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) 
        pid->integral = -pid->integral_limit;
    
    float I = pid->Ki * pid->integral;
    
    // D (różniczkujący) - z uwzględnieniem czasu próbkowania
    // Używamy measurement derivative zamiast error derivative (lepiej dla noise)
    float D = pid->Kd * (pid->last_measurement - current_val) / pid->dt;
    
    pid->last_error = error;
    pid->last_measurement = current_val;

    // Suma składników PID
    float output = P + I + D;

    // Nasycenie wyjścia
    if (output > pid->out_max) 
        output = pid->out_max;
    else if (output < pid->out_min) 
        output = pid->out_min;

    return output;
}

void PID_SetSetpoint(PID_TypeDef *pid, float setpoint) {
    pid->setpoint = setpoint;
}

void PID_SetDt(PID_TypeDef *pid, float dt) {
    pid->dt = dt;
}
