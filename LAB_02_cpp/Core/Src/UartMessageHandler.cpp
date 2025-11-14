/*
 * UartMessageHandler.cpp
 *
 *  Created on: Nov 14, 2025
 *      Author: janek
 */

#include "UartMessageHandler.hpp"
#include <cstring>   // for memset
#include <cstdlib>   // for atoi
#include <cstdio>    // for sprintf if needed
#include <string>

// Wrap HAL headers in extern "C"
extern "C" {
#include "stm32f7xx_hal.h"
}

char conditionListPin[3]   = {'1','2','3'};
char conditionListState[2] = {'0','1'};

UartMessageHandler::UartMessageHandler(UART_HandleTypeDef* huart) : huart_(huart) {
    memset(rxBuffer, 0, sizeof(rxBuffer));
}

// called for every received byte
void UartMessageHandler::receiveByte(uint8_t byte) {
    if (byte == '\r') {           // end of message
        rxBuffer[idx] = '\0';
        processMessage();
        idx = 0;
    } else if (idx < sizeof(rxBuffer)-1) {
        rxBuffer[idx++] = byte;
    } else {
        idx = 0;                  // overflow protection
    }
}

void UartMessageHandler::transmitMessage(const char* msg){
	HAL_UART_Transmit(huart_, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void UartMessageHandler::processMessage() {
    // basic validation
    if (rxBuffer[0] != 'L' || rxBuffer[1] != 'D' ||
        strchr(conditionListPin, rxBuffer[2]) == nullptr ||
        strchr(conditionListState, rxBuffer[3]) == nullptr)
    {
        const char* msg = "Wrong message format: LDxy where x=1..3, y=0/1\r\n";
        transmitMessage(msg);
        return;
    }

    if (strlen(rxBuffer)!=4){
    	const char* msg = "WARNING! stick to the format: LDxy where x=1..3, y=0/1\r\n";
    	transmitMessage(msg);

    	//std::string lenStr = std::to_string(strlen((char*)rxBuffer)); check dlugosci wiadomosci jakims cudem jest 4 nie dotkne nigdy debugera moglem to zrobic w 5 sekund ale nie
    	//transmitMessage(lenStr.c_str());
    }

    int ledNumber = rxBuffer[2] - '0';
    int ledState  = rxBuffer[3] - '0';

    switch (ledNumber) {
        case 1:
        	HAL_GPIO_WritePin(GPIOB, LD1_Pin, ledState ? GPIO_PIN_SET : GPIO_PIN_RESET);
        	break;
        case 2:
        	HAL_GPIO_WritePin(GPIOB, LD2_Pin, ledState ? GPIO_PIN_SET : GPIO_PIN_RESET);
        	break;
        case 3:
        	HAL_GPIO_WritePin(GPIOB, LD3_Pin, ledState ? GPIO_PIN_SET : GPIO_PIN_RESET);
        	break;
        default:
        	break;
    }
}








