#include "pid.h"

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = out_min;
    pid->output_max = out_max;
    pid->integral_max = out_max * 0.5f;
    pid->prev_output = 0.0f;
    pid->output_alpha = 1.0f;   /* 1.0 = 无滤波，改用 I 项做低通 */
}

float PID_Compute(PID_Controller *pid, float setpoint, float measured, float dt)
{
    float error = setpoint - measured;

    pid->integral += error * dt;
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max)
        pid->integral = -pid->integral_max;

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    if (output > pid->output_max) 	
        output = pid->output_max;
    else if (output < pid->output_min)
        output = pid->output_min;

    /* 一阶低通滤波：Y[n] = α·X[n] + (1-α)·Y[n-1]，抑制输出震荡 */
    output = pid->output_alpha * output + (1.0f - pid->output_alpha) * pid->prev_output;
    pid->prev_output = output;

    return output;
}

void PID_Reset(PID_Controller *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_output = 0.0f;
}
