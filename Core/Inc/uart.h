/*
 * uart.h
 *
 *  Created on: Jan 29, 2022
 *      Author: Mario Regus
 */

#ifndef __UART_H__
#define __UART_H__



void uartInit(void);

void uart_PrintString(char * str);
void uart_PrintFloat(float value);
void uart_PrintInt(unsigned int value, unsigned char base);

#endif /* __UART_H__ */
