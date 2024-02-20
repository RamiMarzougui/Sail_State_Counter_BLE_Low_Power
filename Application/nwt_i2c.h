/*
 * nwt_i2c.h
 *
 *  Created on: 11 juil. 2023
 *      Author: gaeta
 */

#ifndef NWT_I2C_H


/*********************************************************************
 * INCLUDES
 */
#include <ti/drivers/I2C.h>
/*********************************************************************
 * CONSTANTS
 */
#define NWT_I2C_H

/* Sensor address */
//#define SENSOR_ADDR  0x6A //for dev kit
#define SENSOR_ADDR  0x6B //for final product

/* Sensor result registers */
#define ACC_Z_LOW_REG 0x28 //LSM6DSV16BX
#define ACC_Z_HI_REG 0x29
#define WAKE_UP_THS 0x5B //Wake up threshold

#define MLC1_SRC 0x70
#define OUTY_A 0x2A
#define OUTX_A 0x2C
#define OUTZ_A 0x28

//#define FSM_STATUS 0x36 //LSM6DSOX
#define FSM_STATUS 0x4A //LSM6DSV16BX

#define WHOAMI_REG 0x0F

#define FIFO_DATA_OUT_TAG 0x78
#define FIFO_DATA_OUT_BYTE_0 0x79

/**** MLC feature TAG ****/
#define MLC_feature 0x1C

/**** Variance Identifiers ****/
#define VARIANCE_ACC_Z_2F_ID 0x0376
#define VARIANCE_ACC_V_ID 0x0378
#define VARIANCE_ACC_Z_1F_ID 0x0370

/* Accelerometer control register */
#define CTRL1_XL_REG 0x10
#define CTRL6_C 0x15
#define CTRL5_C 0x14

/*********************************************************************
*  EXTERNAL VARIABLES
*/
/*
 * Data structure containing accelerometer data
 * Updated at accelerometer frequency
 */
struct Acc {
    float Acc_X;
    float Acc_Y;
    float Acc_Z;
};

typedef enum {
    LOW_POWER = 0xA,
    ULTRA_LOW_POWER = 0xB,
    ERROR_S = 0xF,
} Sensor_PowerMode;


extern uint8_t txBuffer[8];
extern uint8_t rxBuffer[8];
extern I2C_Transaction i2cTransaction;

/*********************************************************************
 * FUNCTIONS
 */

/*
 * @fn      I2C_createTask
 *
 * @brief   Task creation function for I2C.
 */
extern void I2C_createTask(void);

/*
 * @fn        I2C_MLC_Features
 *
 * @brief     Read MLC Features values
 */

extern void I2C_MLC_Features(void);

/*
 * @fn        I2C_MLC_FSM_LP_MODE
 *
 * @brief     Enter LP MODE
 */
extern void I2C_MLC_FSM_LP_MODE(void);

/*
 * @fn        I2C_MLC_FSM_ULP_MODE
 *
 * @brief     Enter ULP MODE
 */
extern void I2C_MLC_FSM_ULP_MODE(void);

/*
 * @fn        I2C_GET_SENSOR_ODR
 *
 * @brief     Get SENSOR Accelerometer ODR value
 */

Sensor_PowerMode I2C_GET_SENSOR_ODR(void);

/*
 * @fn        uint16_to_half
 *
 * @brief     Function to convert a 16-bit unsigned integer to half-precision float
 */

extern float uint16_to_half(uint16_t value);

/*
 * @fn        half_to_uint16
 *
 * @brief     Function to convert a half-precision float to a 16-bit unsigned integer
 */

extern uint16_t half_to_uint16(float value);

/*
 * @fn      I2C_read_fsm
 *
 * @brief   Read mlc result
 */
extern void I2C_read_fsm(void);

/*
 * @fn      I2C_read_feature
 *
 * @brief   Read mlc feature
 */
extern void I2C_read_feature(void);

/*
 * @fn      I2C_read_wake_up_thr
 *
 * @brief   Read WAKE UP threshold
 */
extern void I2C_read_wake_up_thr(void);

/*********************************************************************
 * @fn      I2C_read_acc
 *
 * @brief   Read the XL
 */
extern void I2C_read_acc(void);


/*
 * @fn      read_MLC_trehshold
 *
 * @brief   Read the MLC thresholds (OFF threshold + ON/F1 threshold)
 */

extern void read_MLC_trehshold(void);

#endif /* NWT_I2C_H_ */
