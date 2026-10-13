#ifndef __CLAW_H
#define __CLAW_H

/*
 * 爪子舵机（TIM2 PWM，见 Steering.c / Control_steering.c）
 *   PWM1 = pin 1 = PA0  张合
 *   PWM2 = pin 2 = PA1  旋转
 *
 * 角度约 -90 ~ +90，对应 compare = angle * 4 + 530
 *
 * Debug：在 Watch 里改下面两个变量，主循环会周期性刷到舵机。
 */

extern volatile int g_claw_pwm1_angle;
extern volatile int g_claw_pwm2_angle;

void Claw_ApplyAngles(void);
void Claw_Init(void);

/* 巡线任务用（标定完再把宏改成实测值） */
#define CLAW_GRIP_OPEN_ANGLE   (-30)
#define CLAW_DROP_MISSION_STEP 2U

void Claw_OpenGrip(void);

#endif
