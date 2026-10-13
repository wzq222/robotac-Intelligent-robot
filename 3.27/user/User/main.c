#include "stm32f10x.h"
#include "BSP.h"
#include "Delay.h"
#include "USART2.h"
#include "app_ircomm.h"
#include "line_track.h"
#include "motor.h"
#include "claw.h"

extern volatile uint8_t g_control_flag;

#define BASE_SPEED 30.0f

#define MOTOR_SELFTEST 0

/* 1：主循环只刷 g_claw_pwm1_angle / g_claw_pwm2_angle，Keil Debug 里改变量即看效果
 * 0：正常巡线 */
#define CLAW_ANGLE_DEBUG 0

#if MOTOR_SELFTEST
static void Motor_SelfTest(void)
{
    const int spd = 300;

    Set_Motor1_RPM(0); Set_Motor2_RPM(0);
    Set_Motor3_RPM(0); Set_Motor4_RPM(0);
    Delay_ms(1000);

    Set_Motor1_RPM(spd); Delay_ms(2000); Set_Motor1_RPM(0); Delay_ms(500);
    Set_Motor2_RPM(spd); Delay_ms(2000); Set_Motor2_RPM(0); Delay_ms(500);
    Set_Motor3_RPM(spd); Delay_ms(2000); Set_Motor3_RPM(0); Delay_ms(500);
    Set_Motor4_RPM(spd); Delay_ms(2000); Set_Motor4_RPM(0); Delay_ms(500);

    Set_Motor1_RPM(spd);  Set_Motor4_RPM(spd);
    Set_Motor2_RPM(-spd); Set_Motor3_RPM(-spd);
    Delay_ms(3000);

    Set_Motor1_RPM(0); Set_Motor2_RPM(0);
    Set_Motor3_RPM(0); Set_Motor4_RPM(0);
    Delay_ms(2000);
}
#endif

int main(void)
{
    BSP_Init();
    Delay_Init();
    Claw_Init();

#if CLAW_ANGLE_DEBUG
    while (1)
    {
        Claw_ApplyAngles();
        Delay_ms(50);
    }
#else
    /* 上电爪姿：P1=80(松爪)+P2=0(水平拿模块)，等 1s 后合爪(P1=40)，再等 1s 后开始巡线 */
    Delay_ms(1000);
    g_claw_pwm1_angle = 40;
    Claw_ApplyAngles();
    Delay_ms(1000);

    IR_SendControl(0, 1, 0);
    Delay_ms(1000);

#if MOTOR_SELFTEST
    while (1)
    {
        Motor_SelfTest();
    }
#else
    LineTrack_Init(BASE_SPEED);

    /* 启动前清除积攒的标志，避免拿延时期间残留的传感器数据驱动 PID */
    g_control_flag = 0U;

    while (1)	
    {
        __disable_irq();
        uint8_t has_packet = ir_data_updated;
        ir_data_updated = 0U;
        uint8_t flag = g_control_flag;
        g_control_flag = 0U;
        __enable_irq();

        if (has_packet)
        {
            USART2_ProcessPacket();
        }

        if (flag)
        {
            LineTrack_Update(0.01f);
        }
        __WFI();
    }
#endif
#endif
}
