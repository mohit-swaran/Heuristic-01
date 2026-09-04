/*
 * MMC5983MA_IO.h
 *
 *  Created on: Feb 1, 2026
 *      Author: mohit
 */

#ifndef INC_MMC5983MA_IO_H_
#define INC_MMC5983MA_IO_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    SPI_HandleTypeDef *spiHandle; // Pointer to the HAL SPI handle (e.g., &hspi1)
    GPIO_TypeDef *csPort;         // GPIO Port for Chip Select (e.g., GPIOB)
    uint16_t csPin;               // GPIO Pin for Chip Select (e.g., GPIO_PIN_9)
} MMC5983MA_IO_t;

/* Function Prototypes */

// Initialize the struct with your specific hardware pins
void MMC5983_IO_Init(MMC5983MA_IO_t *io, SPI_HandleTypeDef *hspi, GPIO_TypeDef *port, uint16_t pin);

// Returns true if we get the correct product ID from the device (Device Check)
bool MMC5983_IO_IsConnected(MMC5983MA_IO_t *io);

// Read a single byte from a register
bool MMC5983_IO_ReadSingleByte(MMC5983MA_IO_t *io, uint8_t registerAddress, uint8_t *value);

// Write a single byte into a register
bool MMC5983_IO_WriteSingleByte(MMC5983MA_IO_t *io, uint8_t registerAddress, uint8_t value);

// Read multiple bytes (burst read)
bool MMC5983_IO_ReadMultipleBytes(MMC5983MA_IO_t *io, uint8_t registerAddress, uint8_t *buffer, uint8_t length);

// Write multiple bytes (burst write)
bool MMC5983_IO_WriteMultipleBytes(MMC5983MA_IO_t *io, uint8_t registerAddress, uint8_t *buffer, uint8_t length);

// Helper: Sets a specific bit (Read-Modify-Write)
bool MMC5983_IO_SetRegisterBit(MMC5983MA_IO_t *io, uint8_t registerAddress, uint8_t bitMask);

// Helper: Clears a specific bit (Read-Modify-Write)
bool MMC5983_IO_ClearRegisterBit(MMC5983MA_IO_t *io, uint8_t registerAddress, uint8_t bitMask);

// Helper: Checks if a bit is set
bool MMC5983_IO_IsBitSet(MMC5983MA_IO_t *io, uint8_t registerAddress, uint8_t bitMask);

#endif /* INC_MMC5983MA_IO_H_ */
