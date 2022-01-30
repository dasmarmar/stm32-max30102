/*
 * uart.c
 *
 *  Created on: Jan 29, 2022
 *      Author: Mario Regus
 */

#include "stm32f4xx_hal.h"
#include "string.h"
#include "stdio.h"
#include "system.h"
#include "float.h"
#include "stdint.h"

UART_HandleTypeDef stLinkUart;

void uartInit(void)
{
	// Change UART instance as needed according to board configuration
	stLinkUart.Instance = USART3;
	stLinkUart.Init.BaudRate = 115200;
	stLinkUart.Init.WordLength = UART_WORDLENGTH_8B;
	stLinkUart.Init.StopBits = UART_STOPBITS_1;
	stLinkUart.Init.Parity = UART_PARITY_NONE;
	stLinkUart.Init.Mode = UART_MODE_TX_RX;
	stLinkUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	stLinkUart.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&stLinkUart) != HAL_OK)
    {
    	errorHandler();
    }
}

void uart_PrintString(char * str)
{
	HAL_UART_Transmit(&stLinkUart, str, strlen(str), HAL_MAX_DELAY);
}

void uart_PrintFloat(float value)
{
	uint8_t buf[12];

	value *= 100;
	sprintf((char*)buf, "%u.%02\r\n",
			(unsigned int)value/100,
			(unsigned int)value % 100);

	HAL_UART_Transmit(&stLinkUart, buf, strlen((char*)buf), HAL_MAX_DELAY);}

void uart_PrintInt(unsigned int value, unsigned char base)
{
	uint8_t buf[12];

	switch(base){
	case 10: sprintf((char*)buf, "%u", value); break;
	case 16: sprintf((char*)buf, "%x", value); break;
	}

	HAL_UART_Transmit(&stLinkUart, buf, strlen((char*)buf), HAL_MAX_DELAY);
}
