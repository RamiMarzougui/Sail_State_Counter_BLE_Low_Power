/*
 * BQ27427.h
 *
 *  Created on: 12 sept. 2023
 *      Author: rami
 */

#ifndef LIB_BQ27427_BQ27427_H_
#define LIB_BQ27427_BQ27427_H_

/*================== Includes =============================================*/
#include <BQ27427_DEFINITIONS.h>
#include <stdint.h>
#include <stdbool.h>

/*================== Macros and Definitions ===============================*/
#define BQ27427_I2C_TIMEOUT 20000

typedef int16_t (*i2c_write_byte)(uint8_t, uint8_t, uint8_t *, uint8_t);
typedef int16_t (*i2c_read_byte)(uint8_t, uint8_t, uint8_t *, uint8_t);

int16_t BQ27427_i2cWriteBytes(uint8_t reg, uint8_t *data, uint8_t len);
int16_t BQ27427_i2cReadBytes(uint8_t reg, uint8_t *data, uint8_t len);

typedef struct {
    /** Component mandatory fields **/
    uint8_t BQ27427_i2c_address;
    i2c_write_byte  write_reg;
    i2c_read_byte   read_reg;
} BQ27427_ctx_t;

typedef struct {
    uint16_t CellVoltage; //Cell voltage in mV
    int16_t CellCurrent; //Current measurement in mA (max, average or standby)
    uint16_t CellCapacity; //Capacity measurement in mAh
    int16_t CellPower; //Average power in mAh. >0 indicates charging
    uint16_t CellSoC; //State of charge measurement in % (filtered or unfiltered)
    uint8_t CellSoH; //State of health measurement in %, or status bits
    uint16_t CellTemperature; //Temperature measurement in °C
    uint16_t ICTemperature; //Internal IC temperature in °C
    //Flags
    uint8_t FastCHG; //Fast charging allowed
    uint8_t FullCHG; //Full charge detected
    uint8_t DSCH; //Battery is discharging

} BQ27427_Data;



// Parameters for the current() function, to specify which current to read
typedef enum {
    AVG,  // Average Current (DEFAULT)
    STBY, // Standby Current
    MAX   // Max Current
} current_measure;

// Parameters for the capacity() function, to specify which capacity to read
typedef enum {
    REMAIN,     // Remaining Capacity (DEFAULT)
    FULL,       // Full Capacity
    AVAIL,      // Available Capacity
    AVAIL_FULL, // Full Available Capacity
    REMAIN_F,   // Remaining Capacity Filtered
    REMAIN_UF,  // Remaining Capacity Unfiltered
    FULL_F,     // Full Capacity Filtered
    FULL_UF,    // Full Capacity Unfiltered
    DESIGN,     // Design Capacity
    TRUE_REMAIN //
} capacity_measure;

// Parameters for the soc() function
typedef enum {
    FILTERED,  // State of Charge Filtered (DEFAULT)
    UNFILTERED // State of Charge Unfiltered
} soc_measure;

// Parameters for the soh() function
typedef enum {
    PERCENT,  // State of Health Percentage (DEFAULT)
    SOH_STAT  // State of Health Status Bits
} soh_measure;

// Parameters for the temperature function
typedef enum {
    BATTERY,      // Battery Temperature (DEFAULT)
    INTERNAL_TEMP // Internal IC Temperature
} temp_measure;

// Parameters for the setGPOUTFunction() funciton
typedef enum {
    SOC_INT, // Set GPOUT to SOC_INT functionality
    BAT_LOW  // Set GPOUT to BAT_LOW functionality
} gpout_function;

/*================== Function Prototypes ==================================*/

/*****************************************************************************
 ************************** Initialization Functions *************************
 *****************************************************************************/
bool BQ27427_init (BQ27427_ctx_t *dev);
bool BQ27427_setCapacity (uint16_t capacity);
bool BQ27427_setHibernateCurrent(uint16_t current_mA);
bool BQ27427_setDesignEnergy (uint16_t energy);
bool BQ27427_setTerminateVoltageMin (uint16_t voltage);
bool BQ27427_setChargeVChgTermination(uint16_t voltage);
bool BQ27427_setTaperRateTime (uint16_t rate);
bool BQ27427_setTaperRateVoltage(uint16_t voltage);

/*****************************************************************************
 ********************** Battery Characteristics Functions ********************
 *****************************************************************************/

uint16_t BQ27427_voltage (void);
int16_t BQ27427_current (current_measure type);
uint16_t BQ27427_capacity (capacity_measure type);
int16_t BQ27427_power (void);
uint16_t BQ27427_soc (soc_measure type);
uint8_t BQ27427_soh (soh_measure type);
uint16_t BQ27427_temperature (temp_measure type);

/*****************************************************************************
 ************************** GPOUT Control Functions **************************
 *****************************************************************************/

bool BQ27427_GPOUTPolarity (void);
bool BQ27427_setGPOUTPolarity (bool activeHigh);
bool BQ27427_GPOUTFunction (void);
bool BQ27427_setSLEEPenable(bool enable);
bool BQ27427_setGPOUTFunction (gpout_function function);
uint8_t BQ27427_SOC1SetThreshold (void);
uint8_t BQ27427_ReadBoardOffset(void);
bool BQ27427_WriteBoardOffset(void);
uint8_t BQ27427_SOC1ClearThreshold (void);
bool BQ27427_set_BI_PU_EN(bool detect_bat_pin_enable);
bool BQ27427_setSOC1Thresholds (uint8_t set, uint8_t clear);
uint8_t BQ27427_SOCFSetThreshold (void);
uint8_t BQ27427_SOCFClearThreshold (void);
bool BQ27427_setSOCFThresholds (uint8_t set, uint8_t clear);
bool BQ27427_socFlag (void);
bool BQ27427_socfFlag (void);
bool BQ27427_itporFlag (void);
bool BQ27427_initComp(void);
bool BQ27427_fcFlag (void);
bool BQ27427_chgFlag (void);
bool BQ27427_dsgFlag (void);
uint8_t BQ27427_sociDelta (void);
bool BQ27427_setSOCIDelta (uint8_t delta);
bool BQ27427_pulseGPOUT (void);

bool BQ27427_unseal (void);
bool BQ27427_softReset (void);
uint8_t BQ27427_computeBlockChecksum (void);
/*****************************************************************************
 *************************** Control Sub-Commands ****************************
 *****************************************************************************/
uint16_t BQ27427_deviceType (void);
bool BQ27427_enterConfig (bool userControl);
bool BQ27427_exitConfig (bool resim);
uint16_t BQ27427_flags (void);
uint16_t BQ27427_status (void);
bool BQ27427_SET_HIBERNATE(void);
bool BQ27427_CLEAR_HIBERNATE(void);
void BQ27427_Full_Reset(void);

void BQ27427_Shutdown_EN(void);
void BQ27427_Enter_SHUTDOWN(void);

#endif
