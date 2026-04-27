/*
 * SPIconfig.c
 *
 *  Created on: 27.04.2026
 *      Author: matteo gassler
 */

#include "SPIconfig.h"

extern SPI_HandleTypeDef hspi1;

// --- SPI Write Funktion ---
void TMC_WriteReg(uint8_t addr, uint32_t value) {
    uint8_t data[5];
    data[0] = addr | 0x80; // MSB auf 1 setzen für Write
    data[1] = (value >> 24) & 0xFF;
    data[2] = (value >> 16) & 0xFF;
    data[3] = (value >> 8)  & 0xFF;
    data[4] = value & 0xFF;

    // CS LOW, Senden, CS HIGH
    HAL_GPIO_WritePin(TMC1_CS_GPIO_Port, TMC1_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, data, 5, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(TMC1_CS_GPIO_Port, TMC1_CS_Pin, GPIO_PIN_SET);
}

// --- SPI Read Funktion ---
uint32_t TMC_ReadReg(uint8_t addr) {
    uint8_t tx_data[5] = {addr & 0x7F, 0, 0, 0, 0}; // MSB auf 0 für Read
    uint8_t rx_data[5] = {0};

    // 1. Transfer: Dem TMC5160 sagen, welches Register wir wollen
    HAL_GPIO_WritePin(TMC1_CS_GPIO_Port, TMC1_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx_data, 5, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(TMC1_CS_GPIO_Port, TMC1_CS_Pin, GPIO_PIN_SET);

    // 2. Transfer: Die eigentlichen Daten abholen
    HAL_GPIO_WritePin(TMC1_CS_GPIO_Port, TMC1_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 5, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(TMC1_CS_GPIO_Port, TMC1_CS_Pin, GPIO_PIN_SET);

    // Bytes zusammenbauen (rx_data[0] ist die Dummy-Antwort der Adresse)
    return ((uint32_t)rx_data[1] << 24) | ((uint32_t)rx_data[2] << 16) |
           ((uint32_t)rx_data[3] << 8)  | rx_data[4];
}
