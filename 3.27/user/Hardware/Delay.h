#ifndef __DELAY_H
#define __DELAY_H
#include <stdint.h>

void Delay_Init(void);
void Delay_IncTick(void);
uint32_t Delay_GetTick(void);
uint8_t Delay_HasElapsed(uint32_t start, uint32_t period);

void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);

#endif
