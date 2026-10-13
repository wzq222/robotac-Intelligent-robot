#ifndef __USART2_H
#define __USART2_H

#include <stdint.h>

// 八路传感器数据，自动在中断里更新（无需手动调用函数）
extern volatile uint16_t ir_data[8];
// 新数据标志（可选，用于判断是否有新包）
extern volatile uint8_t ir_data_updated;
// 调试变量
extern volatile uint32_t s_irq_count;     // 中断计数
extern volatile uint8_t s_last_byte;      // 最后接收到的字节
extern volatile uint8_t s_usart2_init_done;  // USART2初始化完成标志
void USART2_Init(void);
void USART2_ProcessPacket(void);
void USART2_SendByte(uint8_t byte);
void USART2_SendString(const char *string);

#endif