#include "stm32f10x.h"

//初始化正负极
void Digital_Input_Init(){
	/*********************************************
	*** 2x12插针定义，Reset状态下，定义为 ***	**
	** PA12   5V								**
	** PA15   5V								**
	** PB3    5V								**
	** PB4    5V								**
	** PB5    空，防止误接5V和3.3V短路		**
	** PB8    3.3V								**
	** PB9    3.3V								**
	** PC13   3.3V								**
	** PC14   空，防止误接3.3V和GND短路		**
	** PC15   GND								**
	** PB11   GND								**
	** PB10   GND								**
	**********************************************/
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	GPIO_InitTypeDef GPIO_Input;
	GPIO_Input.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Input.GPIO_Speed = GPIO_Speed_50MHz;

	// PC13
	GPIO_Input.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_Input);
	// PC15
	GPIO_Input.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOC, &GPIO_Input);
	// PB11
	GPIO_Input.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_Input);
	// PB10
	GPIO_Input.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOB, &GPIO_Input);
}
