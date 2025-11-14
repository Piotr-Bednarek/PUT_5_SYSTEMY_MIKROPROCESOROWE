/*
 * main.cpp
 *
 *  Created on: Nov 14, 2025
 *      Author: janek
 */

#include "UartMessageHandler.hpp"


extern "C" {
#include "main.h"      // CubeMX-generated main.c header
#include "stm32f7xx_hal.h"

// HAL handles generated in main.c
extern UART_HandleTypeDef huart3;
extern uint8_t rx_byte;

// CubeMX init functions (in main.c)
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART3_UART_Init(void);
}

UartMessageHandler uartHandler(&huart3);

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        uartHandler.receiveByte(rx_byte);
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}

extern "C" int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART3_UART_Init();

    // Start UART receive interrupt
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);

    // Main loop
    while(1) {
        HAL_Delay(10);
    }
}
