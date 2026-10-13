#include "app_ircomm.h"
#include "USART2.h"

// 发送控制包到八路模块，格式与示例一致："$<adj>,<a>,<d>#"
void IR_SendControl(uint8_t adjust, uint8_t aData, uint8_t dData)
{
    char send_buf[8] = "$0,0,0#";

    send_buf[1] = (adjust ? '1' : '0');
    send_buf[3] = (aData ? '1' : '0');
    send_buf[5] = (dData ? '1' : '0');

    USART2_SendString(send_buf);
}
