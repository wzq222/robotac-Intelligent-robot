#include "USART2.h"
#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>

#define USART2_PACKET_SIZE 100
#define IR_CH_NUM 8

// 公开的八路传感器数据数组，在中断里自动更新
volatile uint16_t ir_data[IR_CH_NUM] = {0};
volatile uint8_t ir_data_updated = 0;

static uint8_t s_rx_buf[USART2_PACKET_SIZE];
static uint8_t s_pending_buf[USART2_PACKET_SIZE];
static uint8_t s_rx_step = 0;
static uint8_t s_rx_started = 0;

// 初始化完成标志
volatile uint8_t s_usart2_init_done = 0;

// 调试：计数器，看中断是否被触发
volatile uint32_t s_irq_count = 0;
volatile uint8_t s_last_byte = 0;

static void USART2_ParseAPacket(const uint8_t *buf)
{
	uint16_t tmp[IR_CH_NUM];
	uint8_t got[IR_CH_NUM] = {0};
	uint8_t count = 0;
	const char *p = (const char *)buf;

	if (p[0] != '$' || p[1] != 'A')
	{
		return;
	}

	for (uint8_t i = 0; i < IR_CH_NUM; i++)
	{
		tmp[i] = 0;
	}

	// 查找形如 xN:VALUE 的字段，直到 '#'
	while (*p != '\0' && *p != '#')
	{
		if (*p == 'x')
		{
			int idx = 0;
			int val = 0;
			if (sscanf(p, "x%d:%d", &idx, &val) == 2)
			{
				if (idx >= 1 && idx <= IR_CH_NUM)
				{
					if (val < 0)
					{
						val = 0;
					}
					if (val > 4096)
					{
						val = 4096;
					}
					if (!got[idx - 1])
					{
						got[idx - 1] = 1;
						count++;
					}
					tmp[idx - 1] = (uint16_t)val;
				}
			}
		}
		p++;
	}

	// 仅在完整收到8路时更新共享数组
	if (count == IR_CH_NUM)
	{
		for (uint8_t i = 0; i < IR_CH_NUM; i++)
		{
			ir_data[i] = tmp[i];
		}
	}
}

void USART2_ProcessPacket(void)
{
	USART2_ParseAPacket(s_pending_buf);
}

static void USART2_RxByteHandler(uint8_t byte)
{
	if (byte == '$')
	{
		s_rx_started = 1;
		s_rx_step = 0;
		s_rx_buf[s_rx_step++] = byte;
		return;
	}

	if (!s_rx_started)
	{
		return;
	}

	if (s_rx_step >= (USART2_PACKET_SIZE - 1))
	{
		s_rx_started = 0;
		s_rx_step = 0;
		return;
	}

	s_rx_buf[s_rx_step++] = byte;

	if (byte == '#')
	{
		s_rx_buf[s_rx_step] = '\0';
		memcpy(s_pending_buf, s_rx_buf, s_rx_step + 1);
		ir_data_updated = 1;
		s_rx_started = 0;
		s_rx_step = 0;
		return;
	}
}

void USART2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART2, ENABLE);
	s_usart2_init_done = 1;
}

void USART2_SendByte(uint8_t byte)
{
	USART_SendData(USART2, byte);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
	{
	}
}

void USART2_SendString(const char *string)
{
	while (*string != '\0')
	{
		USART2_SendByte((uint8_t)*string);
		string++;
	}
}



void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		uint8_t rx = (uint8_t)USART_ReceiveData(USART2);
		s_irq_count++;
		s_last_byte = rx;
		USART2_RxByteHandler(rx);
	}
}