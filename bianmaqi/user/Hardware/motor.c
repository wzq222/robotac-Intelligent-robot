#include"sys.h"
#include "stm32f10x.h"
#include"motor.h"
void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_Motor;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE);

	/* 让motor1~4使能 */
	GPIO_Motor.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Motor.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Motor.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOB, &GPIO_Motor);
	PBout(12) = 1;

	GPIO_Motor.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOB, &GPIO_Motor);
	PBout(13) = 1;

	GPIO_Motor.GPIO_Pin = GPIO_Pin_14;
	GPIO_Init(GPIOB, &GPIO_Motor);
	PBout(14) = 1;

	GPIO_Motor.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_Motor);
	PBout(15) = 1;

	/* 初始化PWM波引脚 */
	GPIO_Motor.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Motor.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Motor.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIO_Motor);

	GPIO_Motor.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_Motor);

	GPIO_Motor.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_Motor);

	GPIO_Motor.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_Motor);

	GPIO_Motor.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_Motor);

	GPIO_Motor.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOA, &GPIO_Motor);

	GPIO_Motor.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_Motor);

	GPIO_Motor.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOA, &GPIO_Motor);
	
	//初始化TIM1,TIM3时钟
	//TIM3常用来控制
	//TIM3_CH1：PA6
  //TIM3_CH2：PA7
  //TIM3_CH3：PB0
  //TIM3_CH4：PB1
	//四个引脚的pwm，I2C，输出
	//TIM1常用来控制
  //TIM1_CH1 PA8
  //TIM1_CH2 PA9
  //TIM1_CH3 PA10
  //TIM1_CH4 PA11
	//四个引脚的pwm，I2C输出
	//定时器的周期是1000，从0数到999，数满1000就归零，满足一次pwm周期或者触发一次中断。
	TIM_TimeBaseStructure.TIM_Period = (1000-1);
	//调控定时器的预分频器PSC，不分频，TIM1用72MHz，TIM3用36MHz
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	//采样时钟的分频控制，对外界采集来的频率进行分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	//计数器（CNT），让他向上计数，从0开始++
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	//配置OC
	//回复处置，防止出错
	TIM_OCStructInit(&TIM_OCInitStructure);
	//定时器向上计数（0到ARR），CNT:当前计数值，CRR：比较值（占空比）
	//PWM1中CNT<CRR输出高电平，CNT>CRR输出低电平
	//PWM2相反，CNT<CRR输出低电平，CNT>CRR输出高电平
	//高电平在一个周期中的时间越长，转速越快，PCLK1/2的频率太高,以至于CNT计数的过程很快，所以我们基本无法看到从CNT<CRR到CNT>CRR的过程，但是调控CRR导致的高低电平输出时间很明显。
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	//输出使能
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	//PWM输出的有效电平为高电平
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	//初始占空比为0
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	//开启pwm占空比的装载功能，改占空比时更加顺滑，当我开启时，我改掉一个占空比会先放入缓存，然后下个周期生效，不改会导致立刻生效。
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	//开启TIM1的PWM输出开关，只有高级定时器需要，TIM234这种通用的不需要
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
}
void Set_Motor1_RPM(int RPM)
{
	if (RPM > 720) RPM = 720;
	if (RPM < -720) RPM = -720;
	if (RPM == 0) {
		TIM_SetCompare1(TIM3, 1000 - RPM);
		TIM_SetCompare2(TIM3, 1000);
	} else if (RPM > 0) {
		RPM = RPM * 0.8f + 280;
		TIM_SetCompare1(TIM3, 1000 - RPM);
		TIM_SetCompare2(TIM3, 1000);
	} else {
		RPM = RPM * 0.8f - 280;
		TIM_SetCompare1(TIM3, 1000);
		TIM_SetCompare2(TIM3, 1000 + RPM);
	}
}

void Set_Motor2_RPM(int RPM)
{
	if (RPM > 720) RPM = 720;
	if (RPM < -720) RPM = -720;
	if (RPM == 0) {
		TIM_SetCompare3(TIM3, 1000 - RPM);
		TIM_SetCompare4(TIM3, 1000);
	} else if (RPM > 0) {
		RPM = RPM * 0.8f + 280;
		TIM_SetCompare3(TIM3, 1000 - RPM);
		TIM_SetCompare4(TIM3, 1000);
	} else {
		RPM = RPM * 0.8f - 280;
		TIM_SetCompare3(TIM3, 1000);
		TIM_SetCompare4(TIM3, 1000 + RPM);
	}
}

void Set_Motor3_RPM(int RPM)
{
	if (RPM > 720) RPM = 720;
	if (RPM < -720) RPM = -720;
	if (RPM == 0) {
		TIM_SetCompare1(TIM1, 1000 - RPM);
		TIM_SetCompare2(TIM1, 1000);
	} else if (RPM > 0) {
		RPM = RPM * 0.8f + 280;
		TIM_SetCompare1(TIM1, 1000 - RPM);
		TIM_SetCompare2(TIM1, 1000);
	} else {
		RPM = RPM * 0.8f - 280;
		TIM_SetCompare1(TIM1, 1000);
		TIM_SetCompare2(TIM1, 1000 + RPM);
	}
}

void Set_Motor4_RPM(int RPM)
{
	if (RPM > 720) RPM = 720;
	if (RPM < -720) RPM = -720;
	if (RPM == 0) {
		TIM_SetCompare3(TIM1, 1000 - RPM);
		TIM_SetCompare4(TIM1, 1000);
	} else if (RPM > 0) {
		RPM = RPM * 0.8f + 280;
		TIM_SetCompare3(TIM1, 1000 - RPM);
		TIM_SetCompare4(TIM1, 1000);
	} else {
		RPM = RPM * 0.8f - 280;
		TIM_SetCompare3(TIM1, 1000);
		TIM_SetCompare4(TIM1, 1000 + RPM);
	}
}