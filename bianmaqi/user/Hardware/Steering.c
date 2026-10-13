#include "stm32f10x.h"
//舵机初始化
void Steering_Init(){
 GPIO_InitTypeDef GPIO_InitStructure;
 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
 TIM_OCInitTypeDef TIM_OCInitStructure;

 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOA,&GPIO_InitStructure);

 TIM_InternalClockConfig(TIM2);
 //定时时间等于（PSC+1）*（ARR+1）/72MHz(TIM2先我给的72MHz)
 //算出来等于20ms，正好是舵机常用周期
 TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
 TIM_TimeBaseStructure.TIM_Period = 7199;
 TIM_TimeBaseStructure.TIM_Prescaler = 199;
 TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
 TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);

 TIM_OCStructInit(&TIM_OCInitStructure);
 TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //
 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //
 TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//
 TIM_OCInitStructure.TIM_Pulse = 0;
 TIM_OC1Init(TIM2, &TIM_OCInitStructure); // 初始化TIM2 OC1
 TIM_OC2Init(TIM2, &TIM_OCInitStructure); // 初始化TIM2 OC2

 TIM_Cmd(TIM2,ENABLE);

}
