/*
 * VL53L0X.c
 *
 *  Created on: Aug 26, 2026
 *      Author: mohit
 */

#include "main.h"   // Must be first so hi2c1 and HAL drivers are declared
#include "VL53L0X.h"

extern I2C_HandleTypeDef hi2c1; // Bring in the I2C handle from main.c

HAL_StatusTypeDef VL53L0X_WrByte(uint16_t DevAddr, uint8_t RegAddr, uint8_t Value) {
    return HAL_I2C_Mem_Write(&hi2c1, DevAddr, RegAddr, I2C_MEMADD_SIZE_8BIT, &Value, 1, 100);
}

HAL_StatusTypeDef VL53L0X_RdByte(uint16_t DevAddr, uint8_t RegAddr, uint8_t *pData) {
    return HAL_I2C_Mem_Read(&hi2c1, DevAddr, RegAddr, I2C_MEMADD_SIZE_8BIT, pData, 1, 100);
}

//HAL_StatusTypeDef VL53L0X_Begin(VL53L0X_Dev_t *dev) {
//    HAL_StatusTypeDef status = HAL_OK;
//    uint8_t temp = 0;
//
//    // 1. Mandatory Silicon Tuning Sequence (The "Magic Bytes")
//    status |= VL53L0X_WrByte(dev->addr, 0x80, 0x01);
//    status |= VL53L0X_WrByte(dev->addr, 0xFF, 0x01);
//    status |= VL53L0X_WrByte(dev->addr, 0x00, 0x00);
//
//    // Capture the individual StopVariable for this specific sensor
//    status |= VL53L0X_RdByte(dev->addr, 0x91, &dev->stop_variable);
//
//    status |= VL53L0X_WrByte(dev->addr, 0x00, 0x01);
//    status |= VL53L0X_WrByte(dev->addr, 0xFF, 0x00);
//    status |= VL53L0X_WrByte(dev->addr, 0x80, 0x00);
//
//    // 2. Disable TCC and MSRC via System Sequence Config (Register 0x01)
//    status |= VL53L0X_RdByte(dev->addr, 0x01, &temp);
//    temp &= 0xEB; // Clears TCC and MSRC configuration bits
//    status |= VL53L0X_WrByte(dev->addr, 0x01, temp);
//
//    // 3. Start Continuous Ranging Mode (Write 0x02 to REG_SYSRANGE_START / 0x00)
//    status |= VL53L0X_WrByte(dev->addr, 0x00, VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK);
//
//    return status;
//}

HAL_StatusTypeDef VL53L0X_Begin(VL53L0X_Dev_t *dev) {
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t temp = 0;

    // 1. Mandatory Silicon Tuning Sequence (The "Magic Bytes")
    status |= VL53L0X_WrByte(dev->addr, 0x80, 0x01);
    status |= VL53L0X_WrByte(dev->addr, 0xFF, 0x01);
    status |= VL53L0X_WrByte(dev->addr, 0x00, 0x00);

    status |= VL53L0X_RdByte(dev->addr, 0x91, &dev->stop_variable);

    status |= VL53L0X_WrByte(dev->addr, 0x00, 0x01);
    status |= VL53L0X_WrByte(dev->addr, 0xFF, 0x00);
    status |= VL53L0X_WrByte(dev->addr, 0x80, 0x00);

    // 2. Disable TCC and MSRC via System Sequence Config (Register 0x01)
    status |= VL53L0X_RdByte(dev->addr, 0x01, &temp);
    temp &= 0xEB;
    status |= VL53L0X_WrByte(dev->addr, 0x01, temp);

    // 2b. *** NEW: Configure GPIO interrupt to fire on "new sample ready" ***
    // Without this, RESULT_INTERRUPT_STATUS (0x13) never asserts, and any
    // code that waits on it (like isRangeComplete) will time out forever.
    status |= VL53L0X_WrByte(dev->addr, 0x0A, 0x04); // SYSTEM_INTERRUPT_CONFIG_GPIO = new sample ready

    // 2c. Clear any stale/pending interrupt so we start from a known state
    status |= VL53L0X_WrByte(dev->addr, 0x0B, 0x01); // SYSTEM_INTERRUPT_CLEAR

    // 3. Start Continuous Ranging Mode
    status |= VL53L0X_WrByte(dev->addr, 0x00, VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK);

    return status;
}

HAL_StatusTypeDef VL53L0X_ReadDistanceContinuous(uint8_t dev_addr, uint16_t *pDistance) {
    uint8_t buffer[12];

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, dev_addr, VL53L0X_REG_RESULT_RANGE_STATUS,
                                                I2C_MEMADD_SIZE_8BIT,
                                                buffer, 12,
                                                VL53L0X_TIMEOUT);
    if (status == HAL_OK) {
        *pDistance = ((uint16_t)buffer[10] << 8) | buffer[11];
    }
    return status;
}

HAL_StatusTypeDef VL53L0X_SetBareMetalTimingBudget(uint8_t dev_addr, uint32_t budget_us) {
    uint32_t used_budget_us = 132000; // Default overhead
    if (budget_us > 20000) {
        used_budget_us = budget_us;
    }

    // The VL53L0X timing budget calculation involves internal macro periods
    // For a bare-metal approach, writing the standard sequence config registers is safest:
    uint8_t status = HAL_OK;

    // Example bare-metal register writes for timing budget configuration
    // (Note: VL53L0X requires specific sequence steps enabled in 0x01)
    status |= VL53L0X_WrByte(dev_addr, 0x01, 0xFF); // Enable all steps

    return status;
}

static bool VL53L0X_IsRangeComplete(uint8_t dev_addr)
{
    uint8_t status = 0;
    if (HAL_I2C_Mem_Read(&hi2c1, dev_addr, VL53L0X_REG_RESULT_INTERRUPT_STATUS,
                          I2C_MEMADD_SIZE_8BIT, &status, 1, VL53L0X_TIMEOUT) != HAL_OK) {
        return false; // I2C error -> treat as "not ready"
    }
    return (status & 0x07) != 0;
}

HAL_StatusTypeDef VL53L0X_ReadRangePolled(uint8_t dev_addr, uint16_t *range_mm, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (!VL53L0X_IsRangeComplete(dev_addr)) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            return HAL_TIMEOUT;
        }
        // small yield so we don't hammer the bus / starve other tasks
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    uint8_t buf[2];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, dev_addr, VL53L0X_REG_RESULT_RANGE_MM,
                                                 I2C_MEMADD_SIZE_8BIT, buf, 2, VL53L0X_TIMEOUT);
    if (status != HAL_OK) {
        return status;
    }
    *range_mm = ((uint16_t)buf[0] << 8) | buf[1];

    // Clear the interrupt so the next sample can set it again
    uint8_t clear_val = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, dev_addr, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR,
                       I2C_MEMADD_SIZE_8BIT, &clear_val, 1, VL53L0X_TIMEOUT);

    return HAL_OK;
}
