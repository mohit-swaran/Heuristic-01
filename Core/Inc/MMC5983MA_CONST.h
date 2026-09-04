/*
 * MMC5983MA_CONST.h
 *
 *  Created on: Feb 1, 2026
 *      Author: mohit
 */

#ifndef INC_MMC5983MA_CONST_H_
#define INC_MMC5983MA_CONST_H_

// Macro for invoking the callback if the function pointer is valid
#define SAFE_CALLBACK(cb, code) \
    if (cb != nullptr)          \
    {                           \
        cb(code);               \
    }


// Registers definitions
#define X_OUT_0_REG    0x00
#define X_OUT_1_REG    0x01
#define Y_OUT_0_REG    0x02
#define Y_OUT_1_REG    0x03
#define Z_OUT_0_REG    0x04
#define Z_OUT_1_REG    0x05
#define XYZ_OUT_2_REG  0x06
#define T_OUT_REG      0x07
#define STATUS_REG     0x08
#define INT_CTRL_0_REG 0x09
#define INT_CTRL_1_REG 0x0a
#define INT_CTRL_2_REG 0x0b
#define INT_CTRL_3_REG 0x0c
#define PROD_ID_REG    0x2f
#define DUMMY          0x00

// Constants definitions
#define I2C_ADDR       0x30
#define PROD_ID        0x30

// Bits definitions
#define MEAS_M_DONE                 (1 << 0)
#define MEAS_T_DONE                 (1 << 1)
#define OTP_READ_DONE               (1 << 4)
#define TM_M                        (1 << 0)
#define TM_T                        (1 << 1)
#define INT_MEAS_DONE_EN            (1 << 2)
#define SET_OPERATION               (1 << 3)
#define RESET_OPERATION             (1 << 4)
#define AUTO_SR_EN                  (1 << 5)
#define OTP_READ                    (1 << 6)
#define BW0                         (1 << 0)
#define BW1                         (1 << 1)
#define X_INHIBIT                   (1 << 2)
#define YZ_INHIBIT                  (3 << 3)
#define SW_RST                      (1 << 7)
#define CM_FREQ_0                   (1 << 0)
#define CM_FREQ_1                   (1 << 1)
#define CM_FREQ_2                   (1 << 2)
#define CMM_EN                      (1 << 3)
#define PRD_SET_0                   (1 << 4)
#define PRD_SET_1                   (1 << 5)
#define PRD_SET_2                   (1 << 6)
#define EN_PRD_SET                  (1 << 7)
#define ST_ENP                      (1 << 1)
#define ST_ENM                      (1 << 2)
#define SPI_3W                      (1 << 6)
#define X2_MASK                     (3 << 6)
#define Y2_MASK                     (3 << 4)
#define Z2_MASK                     (3 << 2)
#define XYZ_0_SHIFT                 10
#define XYZ_1_SHIFT                 2

typedef enum MMC5983MA_ERROR{
  NONE,
  I2C_INITIALIZATION_ERROR,
  SPI_INITIALIZATION_ERROR,
  INVALID_DEVICE,
  BUS_ERROR,
  INVALID_FILTER_BANDWIDTH,
  INVALID_CONTINUOUS_FREQUENCY,
  INVALID_PERIODIC_SAMPLES
};

#endif /* INC_MMC5983MA_CONST_H_ */
