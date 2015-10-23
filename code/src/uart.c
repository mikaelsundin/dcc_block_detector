#include "stm32f0xx_conf.h"

#include "uart.h"

/**
 * Try to read one byte from usart
 * @return non zero on receive
 */
char read_usart(void){
    if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET){
        return USART_ReceiveData(USART1);
    }else{
        return 0;
    }



}

/**
 * @brief send one char over uart
 * @param ch char to send
 */
void write_usart(char ch){
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){};
    USART_SendData(USART1, ch);
}

/**
 * @brief initialize uart
 */
void init_usart(void){
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init (USART1, &USART_InitStructure);
    USART_Cmd (USART1, ENABLE);

    //configure alternative function for PA9, PA10 to UART1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

    //TX pin
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}


/**
 * @brief simple function to send a uint8_t as hex with an appended newline.
 */
void print_hex_ln(uint8_t val){
    char nibble2hex[] = {'0', '1','2','3','4','5','6','7','8','9','A', 'B', 'C', 'D', 'E', 'F'};

    write_usart( nibble2hex[val>>4 & 0x0F]);
    write_usart( nibble2hex[val>>0 & 0x0F]);
    write_usart('\n');
}

/**
 * @brief simple function to send a string with an appended newline.
 */
void print_ln(char* str){
    while(*str != 0){
        write_usart(*str++);
    }
    write_usart('\n');
}


