/*
 * VL53L0X.h
 *
 *  Created on: Aug 26, 2026
 *      Author: mohit
 */

#ifndef INC_VL53L0X_H_
#define INC_VL53L0X_H_

#include <stdint.h>
#include "main.h" // Needed for GPIO_TypeDef and HAL types
#include "FreeRTOS.h"
#include "task.h"      // <-- add: for vTaskDelay / pdMS_TO_TICKS
#include <stdbool.h>   // for bool/true/false

//Result Reg
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS   0x13
#define VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR    0x0B
#define VL53L0X_REG_RESULT_RANGE_STATUS       0x14
#define VL53L0X_REG_RESULT_RANGE_MM           0x1E

//i2c Default addr
#define VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS 0x8A

//Ranging Add
#define VL53L0X_REG_SYSRANGE_START 0x00
#define VL53L0X_REG_SYSRANGE_MODE_MASK 0x0F
#define VL53L0X_REG_SYSRANGE_MODE_START_STOP 0x01

#define VL53L0X_REG_OSC_CALIBRATE_VAL 0x00f8
#define VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD 0x0004

#define VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK 0x02
#define VL53L0X_TIMEOUT 100

typedef struct {
    uint8_t addr;              // Unique 8-bit I2C address (e.g., 0x60) - Matches your main struct usage!
    GPIO_TypeDef* xshut_port;  // GPIO Port for XSHUT pin
    uint16_t xshut_pin;        // GPIO Pin for XSHUT
    uint8_t stop_variable;     // Each sensor needs to store its own StopVariable!
    uint16_t distance_mm;      // Latest ranging result
} VL53L0X_Dev_t;

typedef int8_t VL53L0X_Error;

#define VL53L0X_ERROR_NONE ((VL53L0X_Error)0)
#define VL53L0X_ERROR ((VL53L0X_Error)-1)

//internal
HAL_StatusTypeDef VL53L0X_WrByte(uint16_t DevAddr, uint8_t RegAddr, uint8_t Value);
HAL_StatusTypeDef VL53L0X_RdByte(uint16_t DevAddr, uint8_t RegAddr, uint8_t *pData);
HAL_StatusTypeDef VL53L0X_Begin(VL53L0X_Dev_t *dev);
HAL_StatusTypeDef VL53L0X_ReadDistanceContinuous(uint8_t dev_addr, uint16_t *pDistance);
HAL_StatusTypeDef VL53L0X_SetBareMetalTimingBudget(uint8_t dev_addr, uint32_t budget_us);
HAL_StatusTypeDef VL53L0X_ReadRangePolled(uint8_t dev_addr, uint16_t *range_mm, uint32_t timeout_ms);

static bool VL53L0X_IsRangeComplete(uint8_t dev_addr);

#endif /* INC_VL53L0X_H_ */
