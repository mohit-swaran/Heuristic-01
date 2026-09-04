/*
 * MMC5983MA_IO.c
 * Updated for X-CUBE-MEMS1 Custom Bus Integration
 */

#include "MMC5983MA_IO.h"
#include "custom_bus.h" // Access to CUSTOM_SPI1_Send / Receive

// Define the Read Flag for the MMC5983MA (MSB = 1)
#define READ_REG_FLAG(x) (0x80 | x)

// --- Public Functions ---

bool MMC5983_IO_IsConnected(MMC5983MA_IO_t *io)
{
    uint8_t response = 0;
    // 0x2F is the Product ID register for MMC5983MA
    return MMC5983_IO_ReadSingleByte(io, 0x2F, &response);
}

bool MMC5983_IO_WriteSingleByte(MMC5983MA_IO_t *io, uint8_t registerAddress, uint8_t value)
{
    int32_t ret;

    // Select the device
    HAL_GPIO_WritePin(io->csPort, io->csPin, GPIO_PIN_RESET);

    // Send Register Address (MSB = 0 for Write)
    ret = BSP_SPI1_Send(&registerAddress, 1);

    if (ret == 0) {
        // Send Value
        ret = BSP_SPI1_Send(&value, 1);
    }

    // Deselect the device
    HAL_GPIO_WritePin(io->csPort, io->csPin, GPIO_PIN_SET);

    return (ret == 0);
}

bool MMC5983_IO_ReadSingleByte(MMC5983MA_IO_t *io, uint8_t registerAddress, uint8_t *value)
{
    int32_t ret;
    uint8_t addr = READ_REG_FLAG(registerAddress);

    // Select the device
    HAL_GPIO_WritePin(io->csPort, io->csPin, GPIO_PIN_RESET);

    // Send Register Address with Read Flag
    ret = BSP_SPI1_Send(&addr, 1);

    if (ret == 0) {
        // Receive the data byte
        ret = BSP_SPI1_Recv(value, 1);
    }

    // Deselect the device
    HAL_GPIO_WritePin(io->csPort, io->csPin, GPIO_PIN_SET);

    return (ret == 0);
}

bool MMC5983_IO_WriteMultipleBytes(MMC5983MA_IO_t *io, uint8_t registerAddress, uint8_t *buffer, uint8_t length)
{
    int32_t ret;

    // Select the device
    HAL_GPIO_WritePin(io->csPort, io->csPin, GPIO_PIN_RESET);

    // Send Starting Register Address
    ret = BSP_SPI1_Send(&registerAddress, 1);

    if (ret == 0) {
        // Burst Write
        ret = BSP_SPI1_Send(buffer, length);
    }

    // Deselect the device
    HAL_GPIO_WritePin(io->csPort, io->csPin, GPIO_PIN_SET);

    return (ret == 0);
}

bool MMC5983_IO_ReadMultipleBytes(MMC5983MA_IO_t *io, uint8_t registerAddress, uint8_t *buffer, uint8_t length)
{
    int32_t ret;
    uint8_t addr = READ_REG_FLAG(registerAddress);

    // Select the device
    HAL_GPIO_WritePin(io->csPort, io->csPin, GPIO_PIN_RESET);

    // Send Starting Register Address with Read Flag
    ret = BSP_SPI1_Send(&addr, 1);

    if (ret == 0) {
        // Burst Read
        ret = BSP_SPI1_Recv(buffer, length);
    }

    // Deselect the device
    HAL_GPIO_WritePin(io->csPort, io->csPin, GPIO_PIN_SET);

    return (ret == 0);
}

// --- Read-Modify-Write Helpers (These remain logically the same) ---

bool MMC5983_IO_SetRegisterBit(MMC5983MA_IO_t *io, uint8_t registerAddress, uint8_t bitMask)
{
    uint8_t value = 0;
    if (!MMC5983_IO_ReadSingleByte(io, registerAddress, &value)) return false;
    value |= bitMask;
    return MMC5983_IO_WriteSingleByte(io, registerAddress, value);
}

bool MMC5983_IO_ClearRegisterBit(MMC5983MA_IO_t *io, uint8_t registerAddress, uint8_t bitMask)
{
    uint8_t value = 0;
    if (!MMC5983_IO_ReadSingleByte(io, registerAddress, &value)) return false;
    value &= ~bitMask;
    return MMC5983_IO_WriteSingleByte(io, registerAddress, value);
}

bool MMC5983_IO_IsBitSet(MMC5983MA_IO_t *io, uint8_t registerAddress, uint8_t bitMask)
{
    uint8_t value = 0;
    if (!MMC5983_IO_ReadSingleByte(io, registerAddress, &value)) return false;
    return (value & bitMask) ? true : false;
}
