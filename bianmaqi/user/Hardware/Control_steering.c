#include "stm32f10x.h"

void servo(int pin, int angle){
	//0xA0  ->  0x38C
	//现在是-90->+90
	uint16_t inner_angle=(angle*4)+530;
	switch(pin){
		case 1:
			TIM_SetCompare1(TIM2, inner_angle);
			break;
		case 2: 
			TIM_SetCompare2(TIM2, inner_angle);
			break;
		case 3:
			break;
	}
}