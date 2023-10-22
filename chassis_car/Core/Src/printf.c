#include "stdarg.h"
#include "stdio.h"
#include "main.h"
#include "printf.h"

#define TX_BUF_SIZE 512

uint8_t send_buf[TX_BUF_SIZE];

void usart_printf(const char *format, ...)
{
    va_list args;
    uint32_t length;

    va_start(args, format);
    length = vsnprintf((char *) send_buf, TX_BUF_SIZE, (const char *) format, args);
    va_end(args);
//  HAL_UART_Transmit(&huart8, (uint8_t *)send_buf, length, 0xFFFF);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *) send_buf, length);
}