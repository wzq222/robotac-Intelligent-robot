#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float output_min, output_max;
    float integral_max;
    float prev_output;       /* 上一次滤波后的输出 */
    float output_alpha;      /* 输出滤波系数 (0~1)，越小越平滑 */
} PID_Controller;

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_min, float out_max);
float PID_Compute(PID_Controller *pid, float setpoint, float measured, float dt);
void PID_Reset(PID_Controller *pid);

#endif
