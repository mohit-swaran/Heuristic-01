/*
 * MMC5983MA.h
 *
 *  Created on: Feb 1, 2026
 *      Author: mohit
 */

#ifndef INC_MMC5983MA_H_
#define INC_MMC5983MA_H_


#include "MMC5983MA_IO.h" // Include our HAL IO layer
#include "MMC5983MA_CONST.h"
#include <stdint.h>
#include <stdbool.h>


typedef struct {
    MMC5983MA_IO_t io; // The low-level IO handle (SPI + CS)

    // Shadow Registers (to minimize SPI reads for Write-Only registers)
    struct {
        uint8_t internalControl0;
        uint8_t internalControl1;
        uint8_t internalControl2;
        uint8_t internalControl3;
    } shadow;

} MMC5983MA_t;

/* Error Codes */
typedef enum {
    MMC5983MA_OK = 1,
    MMC5983MA_ERROR_SPI,
    MMC5983MA_ERROR_TIMEOUT,
    MMC5983MA_ERROR_ID_MISMATCH
} MMC5983MA_Status;

/* Function Prototypes */

// --- Initialization ---
// Initializes the struct and verifies connection.
// Call this after HAL_SPI_Init() in your main.c
MMC5983MA_Status MMC5983_Init(MMC5983MA_t *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin);

// Polls if device is connected (Checks Product ID)
bool MMC5983_IsConnected(MMC5983MA_t *dev);

// Soft Reset the device
bool MMC5983_SoftReset(MMC5983MA_t *dev);

// --- Measurements ---

// Get X, Y, Z raw values (18-bit unsigned)
bool MMC5983_GetMeasurementXYZ(MMC5983MA_t *dev, uint32_t *x, uint32_t *y, uint32_t *z);

// Get individual axes (Slower, not recommended for Micromouse)
uint32_t MMC5983_GetMeasurementX(MMC5983MA_t *dev);
uint32_t MMC5983_GetMeasurementY(MMC5983MA_t *dev);
uint32_t MMC5983_GetMeasurementZ(MMC5983MA_t *dev);

// Get Temperature (-75C to 125C)
int MMC5983_GetTemperature(MMC5983MA_t *dev);

// Read fields without triggering a new measurement (if using Continuous Mode)
bool MMC5983_ReadFieldsXYZ(MMC5983MA_t *dev, uint32_t *x, uint32_t *y, uint32_t *z);

// --- Configuration (SET/RESET & Bandwidth) ---

// Perform manual SET/RESET (De-gaussing)
bool MMC5983_PerformSetOperation(MMC5983MA_t *dev);
bool MMC5983_PerformResetOperation(MMC5983MA_t *dev);

// Configure Filter Bandwidth (100Hz, 200Hz, 400Hz, 800Hz)
bool MMC5983_SetFilterBandwidth(MMC5983MA_t *dev, uint16_t bandwidth);
uint16_t MMC5983_GetFilterBandwidth(MMC5983MA_t *dev);

// Configure Continuous Mode (Sampling Frequency)
bool MMC5983_SetContinuousModeFrequency(MMC5983MA_t *dev, uint16_t frequency);
bool MMC5983_EnableContinuousMode(MMC5983MA_t *dev);
bool MMC5983_DisableContinuousMode(MMC5983MA_t *dev);

// Configure Periodic SET/RESET (Auto-degauss)
bool MMC5983_EnableAutomaticSetReset(MMC5983MA_t *dev);
bool MMC5983_DisableAutomaticSetReset(MMC5983MA_t *dev);
bool MMC5983_SetPeriodicSetSamples(MMC5983MA_t *dev, uint16_t numberOfSamples);

// --- Interrupts & 3-Wire SPI ---
bool MMC5983_EnableInterrupt(MMC5983MA_t *dev);
bool MMC5983_DisableInterrupt(MMC5983MA_t *dev);
bool MMC5983_ClearMeasDoneInterrupt(MMC5983MA_t *dev, uint8_t measMask);

bool MMC5983_Enable3WireSPI(MMC5983MA_t *dev); // Not recommended for standard use


#endif /* INC_MMC5983MA_H_ */
