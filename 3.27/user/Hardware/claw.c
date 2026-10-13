#include "claw.h"
#include "Control_steering.h"

volatile int g_claw_pwm1_angle = 90;
volatile int g_claw_pwm2_angle = 0;

void Claw_ApplyAngles(void)
{
    servo(1, g_claw_pwm1_angle);
    servo(2, g_claw_pwm2_angle);
}

void Claw_Init(void)
{
    Claw_ApplyAngles();
}

void Claw_OpenGrip(void)
{
    servo(1, CLAW_GRIP_OPEN_ANGLE);
}
