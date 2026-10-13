/* app_ircomm.h
 * 独立模块：向八路传感器发送模式选择命令（通过 USART2 PA2 TX）
 */
#ifndef __APP_IRCOMM_H
#define __APP_IRCOMM_H

#include <stdint.h>

// 发送控制包：adjust(校准), aData(模拟量模式开关), dData(数字量模式开关)
// 数据格式: "$<adj>,<a>,<d>#" 例如: "$1,0,1#"
void IR_SendControl(uint8_t adjust, uint8_t aData, uint8_t dData);

#endif
