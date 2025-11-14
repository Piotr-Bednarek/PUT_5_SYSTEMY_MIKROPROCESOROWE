/*
 * UartMessageHandler.hpp
 *
 *  Created on: Nov 14, 2025
 *      Author: janek
 */

#ifndef UARTMESSAGEHANDLER_HPP
#define UARTMESSAGEHANDLER_HPP
//#define BUFFERSIZE = 256

#include "main.h"  // CubeMX header

class UartMessageHandler {
public:
    explicit UartMessageHandler(UART_HandleTypeDef* huart);
    void receiveByte(uint8_t byte); // call from HAL_UART_RxCpltCallback
private:
    UART_HandleTypeDef* huart_;
    char rxBuffer[256];
    uint16_t idx = 0;

    void processMessage();
};

#endif

