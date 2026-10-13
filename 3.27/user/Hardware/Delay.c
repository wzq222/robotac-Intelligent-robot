#include "stm32f10x.h"
#include "Delay.h"

static volatile uint32_t s_delay_tick = 0U;

void Delay_IncTick(void)
{
	s_delay_tick++;
}

uint32_t Delay_GetTick(void)
{
	return s_delay_tick;
}

uint8_t Delay_HasElapsed(uint32_t start, uint32_t period)
{
	return (uint32_t)(Delay_GetTick() - start) >= period;
}

void Delay_Init(void)
{
	SystemCoreClockUpdate();
	s_delay_tick = 0U;

	if (SysTick_Config(SystemCoreClock / 1000U) != 0U)
	{
		while (1)
		{
		}
	}
}

void Delay_us(uint32_t xus)
{
	uint32_t loops_per_us = SystemCoreClock / 8000000U;

	if (loops_per_us == 0U)
	{
		loops_per_us = 1U;
	}

	while (xus--)
	{
		volatile uint32_t i = loops_per_us;
		while (i--)
		{
			__NOP();
		}
	}
}

void Delay_ms(uint32_t xms)
{
	uint32_t start = Delay_GetTick();

	while ((uint32_t)(Delay_GetTick() - start) < xms)
	{
	}
}

void Delay_s(uint32_t xs)
{
	while (xs--)
	{
		Delay_ms(1000U);
	}
}
