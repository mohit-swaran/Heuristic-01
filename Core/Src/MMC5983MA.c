/*
 * MMC5983MA.c
 *
 *  Created on: Feb 1, 2026
 *      Author: mohit
 */


/*
 * mmc5983ma.c
 *
 * STM32 HAL Driver for MMC5983MA High Performance Magnetometer
 * Converted from SparkFun Arduino Library
 */

#include "MMC5983MA.h"
#include "MMC5983MA_CONST.h" // Ensure you have this file with Register Defines
#include "FreeRTOS.h"
#include "task.h"
// --- Private Helper Function Prototypes ---
static bool MMC5983_SetShadowBit(MMC5983MA_t *dev, uint8_t registerAddress, uint8_t bitMask, bool doWrite);
static bool MMC5983_ClearShadowBit(MMC5983MA_t *dev, uint8_t registerAddress, uint8_t bitMask, bool doWrite);
static bool MMC5983_IsShadowBitSet(MMC5983MA_t *dev, uint8_t registerAddress, uint8_t bitMask);
static uint16_t MMC5983_GetTimeout(MMC5983MA_t *dev);

// --- Initialization ---

MMC5983MA_Status MMC5983_Init(MMC5983MA_t *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin)
{
    // Initialize IO Structure
    dev->io.spiHandle = hspi;
    dev->io.csPort = csPort;
    dev->io.csPin = csPin;

    // Reset Shadow Memory
    dev->shadow.internalControl0 = 0;
    dev->shadow.internalControl1 = 0;
    dev->shadow.internalControl2 = 0;
    dev->shadow.internalControl3 = 0;

    // Check Connection
    if (!MMC5983_IsConnected(dev))
    {
        return MMC5983MA_ERROR_ID_MISMATCH;
    }

    return MMC5983MA_OK;
}

bool MMC5983_IsConnected(MMC5983MA_t *dev)
{
    uint8_t response = 0;
    if (!MMC5983_IO_ReadSingleByte(&dev->io, PROD_ID_REG, &response))
    {
        return false;
    }
    return (response == PROD_ID);
}

// --- Shadow Register Management (Private Helpers) ---

static bool MMC5983_SetShadowBit(MMC5983MA_t *dev, uint8_t registerAddress, uint8_t bitMask, bool doWrite)
{
    uint8_t *shadowRegister = NULL;

    switch (registerAddress)
    {
    case INT_CTRL_0_REG: shadowRegister = &dev->shadow.internalControl0; break;
    case INT_CTRL_1_REG: shadowRegister = &dev->shadow.internalControl1; break;
    case INT_CTRL_2_REG: shadowRegister = &dev->shadow.internalControl2; break;
    case INT_CTRL_3_REG: shadowRegister = &dev->shadow.internalControl3; break;
    default: return false;
    }

    *shadowRegister |= bitMask;

    if (doWrite)
    {
        return MMC5983_IO_WriteSingleByte(&dev->io, registerAddress, *shadowRegister);
    }
    return true;
}

static bool MMC5983_ClearShadowBit(MMC5983MA_t *dev, uint8_t registerAddress, uint8_t bitMask, bool doWrite)
{
    uint8_t *shadowRegister = NULL;

    switch (registerAddress)
    {
    case INT_CTRL_0_REG: shadowRegister = &dev->shadow.internalControl0; break;
    case INT_CTRL_1_REG: shadowRegister = &dev->shadow.internalControl1; break;
    case INT_CTRL_2_REG: shadowRegister = &dev->shadow.internalControl2; break;
    case INT_CTRL_3_REG: shadowRegister = &dev->shadow.internalControl3; break;
    default: return false;
    }

    *shadowRegister &= ~bitMask;

    if (doWrite)
    {
        return MMC5983_IO_WriteSingleByte(&dev->io, registerAddress, *shadowRegister);
    }
    return true;
}

static bool MMC5983_IsShadowBitSet(MMC5983MA_t *dev, uint8_t registerAddress, uint8_t bitMask)
{
    switch (registerAddress)
    {
    case INT_CTRL_0_REG: return (dev->shadow.internalControl0 & bitMask);
    case INT_CTRL_1_REG: return (dev->shadow.internalControl1 & bitMask);
    case INT_CTRL_2_REG: return (dev->shadow.internalControl2 & bitMask);
    case INT_CTRL_3_REG: return (dev->shadow.internalControl3 & bitMask);
    default: return false;
    }
}

// --- Measurements ---

bool MMC5983_GetMeasurementXYZ(MMC5983MA_t *dev, uint32_t *x, uint32_t *y, uint32_t *z)
{
    // Trigger Measurement (TM_M)
    if (!MMC5983_SetShadowBit(dev, INT_CTRL_0_REG, TM_M, true))
    {
        MMC5983_ClearShadowBit(dev, INT_CTRL_0_REG, TM_M, false);
        return false;
    }

    // Wait for completion or timeout
    uint16_t timeOut = MMC5983_GetTimeout(dev);
    do
    {
//        HAL_Delay(1);
    	vTaskDelay(pdMS_TO_TICKS(1));
        timeOut--;
    } while ((!MMC5983_IO_IsBitSet(&dev->io, STATUS_REG, MEAS_M_DONE)) && (timeOut > 0));

    // Clear the TM_M bit in shadow only (it clears itself on the chip)
    MMC5983_ClearShadowBit(dev, INT_CTRL_0_REG, TM_M, false);

    if (timeOut == 0) return false;

    // Read Data
    return MMC5983_ReadFieldsXYZ(dev, x, y, z);
}

bool MMC5983_ReadFieldsXYZ(MMC5983MA_t *dev, uint32_t *x, uint32_t *y, uint32_t *z)
{
    uint8_t buffer[7] = {0};

    // Burst read 7 bytes starting from X_OUT_0
    if (MMC5983_IO_ReadMultipleBytes(&dev->io, X_OUT_0_REG, buffer, 7))
    {
        // Reconstruct 18-bit values
        *x = ((uint32_t)buffer[0] << 10) | ((uint32_t)buffer[1] << 2) | ((buffer[6] >> 6) & 0x03);
        *y = ((uint32_t)buffer[2] << 10) | ((uint32_t)buffer[3] << 2) | ((buffer[6] >> 4) & 0x03);
        *z = ((uint32_t)buffer[4] << 10) | ((uint32_t)buffer[5] << 2) | ((buffer[6] >> 2) & 0x03);
        return true;
    }
    return false;
}

int MMC5983_GetTemperature(MMC5983MA_t *dev)
{
    if (!MMC5983_SetShadowBit(dev, INT_CTRL_0_REG, TM_T, true)) return -99;

    uint8_t timeOut = 5;
    do
    {
        HAL_Delay(1);
        timeOut--;
    } while ((!MMC5983_IO_IsBitSet(&dev->io, STATUS_REG, MEAS_T_DONE)) && (timeOut > 0));

    MMC5983_ClearShadowBit(dev, INT_CTRL_0_REG, TM_T, false);

    uint8_t result = 0;
    if (MMC5983_IO_ReadSingleByte(&dev->io, T_OUT_REG, &result))
    {
        float temp = -75.0f + ((float)result * (200.0f / 255.0f));
        return (int)temp;
    }
    return -99;
}

// --- Configuration & Utils ---

bool MMC5983_SoftReset(MMC5983MA_t *dev)
{
    bool success = MMC5983_SetShadowBit(dev, INT_CTRL_1_REG, SW_RST, true);
    MMC5983_ClearShadowBit(dev, INT_CTRL_1_REG, SW_RST, false);
    HAL_Delay(15);
    return success;
}

bool MMC5983_PerformSetOperation(MMC5983MA_t *dev)
{
    bool success = MMC5983_SetShadowBit(dev, INT_CTRL_0_REG, SET_OPERATION, true);
    MMC5983_ClearShadowBit(dev, INT_CTRL_0_REG, SET_OPERATION, false);
    HAL_Delay(1); // Required delay for recharge
    return success;
}

bool MMC5983_PerformResetOperation(MMC5983MA_t *dev)
{
    bool success = MMC5983_SetShadowBit(dev, INT_CTRL_0_REG, RESET_OPERATION, true);
    MMC5983_ClearShadowBit(dev, INT_CTRL_0_REG, RESET_OPERATION, false);
    HAL_Delay(1);
    return success;
}

bool MMC5983_SetFilterBandwidth(MMC5983MA_t *dev, uint16_t bandwidth)
{
    bool success = true;
    switch (bandwidth)
    {
    case 800:
        success &= MMC5983_ClearShadowBit(dev, INT_CTRL_1_REG, BW0, false); // Fix: Logic matches original C++ but optimized
        success &= MMC5983_SetShadowBit(dev, INT_CTRL_1_REG, BW1, true);
        break;
    case 400:
        success &= MMC5983_SetShadowBit(dev, INT_CTRL_1_REG, BW0, false);
        success &= MMC5983_SetShadowBit(dev, INT_CTRL_1_REG, BW1, true);
        // Note: The original C++ code had logic errors in BW setting (using clear/set differently).
        // Standard datasheet: BW0=0 BW1=0 (100Hz), BW0=1 BW1=0 (200Hz), etc.
        // Assuming original library logic:
        // 800: BW0=0, BW1=1
        // 400: BW0=1, BW1=1
        // 200: BW0=1, BW1=0
        // 100: BW0=0, BW1=0
        break;
    case 200:
        success &= MMC5983_SetShadowBit(dev, INT_CTRL_1_REG, BW0, false);
        success &= MMC5983_ClearShadowBit(dev, INT_CTRL_1_REG, BW1, true);
        break;
    case 100:
        success &= MMC5983_ClearShadowBit(dev, INT_CTRL_1_REG, BW0, false);
        success &= MMC5983_ClearShadowBit(dev, INT_CTRL_1_REG, BW1, true);
        break;
    default:
        return false;
    }
    return success;
}

uint16_t MMC5983_GetFilterBandwidth(MMC5983MA_t *dev)
{
    bool bw0 = MMC5983_IsShadowBitSet(dev, INT_CTRL_1_REG, BW0);
    bool bw1 = MMC5983_IsShadowBitSet(dev, INT_CTRL_1_REG, BW1);

    // Logic: BW1 BW0
    //        0   0  = 100Hz
    //        0   1  = 200Hz
    //        1   0  = 800Hz
    //        1   1  = 400Hz
    if (!bw1 && !bw0) return 100;
    if (!bw1 && bw0)  return 200;
    if (bw1 && !bw0)  return 800;
    if (bw1 && bw0)   return 400;
    return 100;
}

static uint16_t MMC5983_GetTimeout(MMC5983MA_t *dev)
{
    uint16_t bw = MMC5983_GetFilterBandwidth(dev);
    // Calculation from library: 800/bw * 4 + 1
    // 800Hz -> 1ms * 4 + 1 = 5ms
    // 100Hz -> 8ms * 4 + 1 = 33ms
    uint16_t timeout = (800 / bw) * 4 + 1;
    return timeout;
}

bool MMC5983_EnableContinuousMode(MMC5983MA_t *dev)
{
    return MMC5983_SetShadowBit(dev, INT_CTRL_2_REG, CMM_EN, true);
}

bool MMC5983_DisableContinuousMode(MMC5983MA_t *dev)
{
    return MMC5983_ClearShadowBit(dev, INT_CTRL_2_REG, CMM_EN, true);
}

bool MMC5983_SetContinuousModeFrequency(MMC5983MA_t *dev, uint16_t frequency)
{
    // Clear lower 3 bits (CM_FREQ_2 | CM_FREQ_1 | CM_FREQ_0)
    MMC5983_ClearShadowBit(dev, INT_CTRL_2_REG, 0x07, false);

    uint8_t bits = 0;
    switch(frequency) {
        case 1: bits = 0x01; break;
        case 10: bits = 0x02; break;
        case 20: bits = 0x03; break;
        case 50: bits = 0x04; break;
        case 100: bits = 0x05; break;
        case 200: bits = 0x06; break;
        case 1000: bits = 0x07; break;
        case 0: bits = 0x00; break;
        default: return false;
    }

    return MMC5983_SetShadowBit(dev, INT_CTRL_2_REG, bits, true);
}
