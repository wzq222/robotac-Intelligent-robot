#include "Motor.h"
#include "stm32f10x_exti.h"
#include "stm32f10x.h"
#include "USART2.h"
#include "sys.h"
#include "Steering.h"
#include "Digital_input.h"
#include "encoder.h"

void BSP_Init(){
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  RCC_Configuration();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO | RCC_APB2Periph_TIM1, ENABLE); 
	Motor_Init();
	Steering_Init();
	USART2_Init();
	Digital_Input_Init();
	Encoder_Init();
}

	
