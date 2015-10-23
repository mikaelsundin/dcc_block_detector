#ifndef _UART_H_
#define _UART_H_
#include <stdint.h>

char read_usart(void);
void write_usart(char ch);
void init_usart(void);

//simple print functions
void print_hex_ln(uint8_t val);
void print_ln(char* str);

#endif
