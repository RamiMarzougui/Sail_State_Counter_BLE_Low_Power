/*
 * BQ27427.c
 *
 *  Created on: 12 sept. 2023
 *      Author: rami
 */


/*================== Includes =============================================*/
#include "BQ27427.h"
#include <ti/drivers/I2C.h>
#include <stddef.h>
#include "shared.h"
#include "led_debug.h"
/*================== Constant and Variable Definitions ====================*/
static BQ27427_ctx_t ctx = {0};
/*======================== Functions =====================================*/
/**
  * @brief  Read generic device register
  *
  *
  */
int16_t BQ27427_i2cReadBytes(uint8_t reg, uint8_t *data, uint8_t len)
{

    I2C_Transaction Transaction;
    uint8_t txBuffer[1];
    int16_t ret = 1;

    /* Prepare Tx Buffer */
    txBuffer[0] = reg;

    /* Read I2C transaction setup */
    Transaction.writeBuf   = txBuffer;
    Transaction.writeCount = 1;
    Transaction.readBuf    = data;
    Transaction.readCount  = len;
    Transaction.targetAddress = BQ27427_I2C_ADDRESS;


    if (!I2C_transfer(i2c, &Transaction))
    {
            ret = 0; //Error
    }

    return ret;

}


/**
  * @brief  Write generic device register
  *
  *
  */
int16_t BQ27427_i2cWriteBytes(uint8_t reg, uint8_t *data, uint8_t len)
{
    I2C_Transaction Transaction;
    uint8_t txBuffer[len+1];
    int16_t ret = 1;

    /* Prepare Tx Buffer */
    txBuffer[0] = reg;
    for(uint32_t i=1; i<len+1;i++)
        txBuffer[i] = data[i-1];

    /* Write I2C transaction setup */
    Transaction.writeBuf = txBuffer;
    Transaction.writeCount = len+1;
    Transaction.readCount  = 0;
    Transaction.targetAddress = BQ27427_I2C_ADDRESS;

    if (!I2C_transfer(i2c, &Transaction))
    {
            ret = 0; //Error
    }

    return ret;
}



static bool BQ27427_sealed (void);
static bool BQ27427_seal (void);

static uint16_t BQ27427_opConfig (void);
static bool BQ27427_writeOpConfig (uint16_t value);

static uint16_t BQ27427_readWord (uint16_t subAddress);
static uint16_t BQ27427_readControlWord (uint16_t function);
static bool BQ27427_executeControlWord (uint16_t function);

/*****************************************************************************
 ************************** Extended Data Commands ***************************
 *****************************************************************************/

static bool BQ27427_blockDataControl (void);
static bool BQ27427_blockDataClass (uint8_t id);
static bool BQ27427_blockDataOffset (uint8_t offset);
static uint8_t BQ27427_blockDataChecksum (void);
static uint8_t BQ27427_readBlockData (uint8_t offset);
static bool BQ27427_writeBlockData (uint8_t offset, uint8_t data);

static bool BQ27427_writeBlockChecksum (uint8_t csum);
static uint8_t BQ27427_readExtendedData (uint8_t classID, uint8_t offset);
static bool BQ27427_writeExtendedData (uint8_t classID, uint8_t offset, uint8_t *data, uint8_t len);

static volatile bool sealFlag = false; // Global to identify that IC was previously sealed
static volatile bool userConfigControl = false; // Global to identify that user has control over

static uint8_t constrain(const uint8_t x, const uint8_t a, const uint8_t b);


/**
 * Initializes I2C and verifies communication with the BQ27427.
 * Must be called before using any other functions.
 *
 * @return true if communication was successful.
 * */
bool BQ27427_init(BQ27427_ctx_t *dev) {


    if (dev == NULL)
        return false;

    ctx.read_reg = dev->read_reg;
    ctx.write_reg = dev->write_reg;
    ctx.BQ27427_i2c_address = dev->BQ27427_i2c_address;

    // Read deviceType from BQ27427
    if (BQ27427_deviceType() == BQ27427_DEVICE_ID) {
        return true; // If device ID is valid, return true
    } else
        return false;

}

/**
 * Configures the design capacity of the connected battery.
 *
 * @param capacity of battery (unsigned 16-bit value)
 * @return true if capacity successfully set.
 * */
bool BQ27427_setCapacity(uint16_t capacity) {
    // Write to STATE subclass (82) of BQ27427 extended memory.
    // Offset 0x0A (10)
    // Design capacity is a 2-byte piece of data - MSB first
    // Unit: mAh
    uint8_t capacityData[2] = {capacity >> 8, capacity & 0x00FF};
    return BQ27427_writeExtendedData(BQ27427_ID_STATE, 6, capacityData, 2);
}


/**
 * Configures the Hibernate I the gauge enters HIBERNATE mode.
 *
 * @param mA (unsigned 16-bit value)
 * @return true if capacity successfully set.
 * */
bool BQ27427_setHibernateCurrent(uint16_t current_mA) {
    // Write to STATE subclass (68) of BQ27427 extended memory.
    // Offset 0x07 (7)
    // Hibernate I is a 2-byte piece of data - MSB first
    // Unit: mA
    uint8_t current_mA_Data[2] = {current_mA >> 8, current_mA & 0x00FF};
    return BQ27427_writeExtendedData(BQ27427_ID_STATE, 23, current_mA_Data, 2);
}

/**
 * Configures the design energy of the connected battery.
 *
 * @param energy of battery (unsigned 16-bit value)
 * @return true if energy successfully set.
*/
bool BQ27427_setDesignEnergy(uint16_t energy) {
    // Write to STATE subclass (82) of BQ27427 extended memory.
    // Offset 0x0C (12)
    // Design energy is a 2-byte piece of data - MSB first
    // Unit: mWh
    uint8_t energyData[2] = {energy >> 8, energy & 0x00FF};
    return BQ27427_writeExtendedData(BQ27427_ID_STATE, 8, energyData, 2);
}

/**
 * Configures terminate voltage (lowest operational voltage of battery powered circuit)
 *
 * @param voltage of battery (unsigned 16-bit value)
 * @return true if energy successfully set.
 * */
bool BQ27427_setTerminateVoltageMin(uint16_t voltage) {
    // Write to STATE subclass (82) of BQ27427 extended memory.
    // Offset 0x0F (16)
    // Termiante voltage is a 2-byte piece of data - MSB first
    // Unit: mV
    // Min 2500, Max 3700
    if (voltage < 2500) voltage = 2500;
    if (voltage > 3700) voltage = 3700;

    uint8_t tvData[2] = {voltage >> 8, voltage & 0x00FF};
    return BQ27427_writeExtendedData(BQ27427_ID_STATE, 10, tvData, 2);
}

bool BQ27427_setChargeVChgTermination(uint16_t voltage){
    // V at Chg Term
    // Write to STATE subclass (82) of BQ27427 extended memory.
    // Offset 0x21 (33)
    // Termiante voltage is a 2-byte piece of data - MSB first
    // Unit: mV
    // Min 0, Max 5000

    uint8_t tvData[2] = {voltage >> 8, voltage & 0x00FF};
    return BQ27427_writeExtendedData(BQ27427_ID_STATE, 33, tvData, 2);

}

// Configures taper rate of connected battery.
bool BQ27427_setTaperRateTime(uint16_t rate) {
    // Write to STATE subclass (82) of BQ27427 extended memory.
    // Offset 0x1B (27)
    // Termiante voltage is a 2-byte piece of data - MSB first
    // Unit: 0.1 Hr
    // Max 2000
    // default 100
    if (rate > 2000)
        rate = 2000;

    uint8_t trData[2] = {rate >> 8, rate & 0x00F};
    return BQ27427_writeExtendedData(BQ27427_ID_STATE, 21, trData, 2);
}

bool BQ27427_setTaperRateVoltage(uint16_t voltage) {
    // Write to STATE subclass (82) of BQ27427 extended memory.
    // Offset 0x1B (27)
    // Termiante voltage is a 2-byte piece of data - MSB first
    // Unit: mV
    // Max 2000
    // default 4100
    if (voltage > 5000)
        voltage = 5000;

    uint8_t trData[2] = {voltage >> 8, voltage & 0x00FF};
    return BQ27427_writeExtendedData(BQ27427_ID_STATE, 29, trData, 2);
}

/*****************************************************************************
 ********************** Battery Characteristics Functions ********************
 *****************************************************************************/

/**
 * Reads and returns the battery voltage
 *
 * @return battery voltage in mV
 * */
uint16_t BQ27427_voltage(void) {
    return BQ27427_readWord(BQ27427_COMMAND_VOLTAGE);
}

/**
 * Reads and returns the specified current measurement
 * @param current_measure enum specifying current value to be read
 * @return specified current measurement in mA. <0 --> charging
 * */
int16_t BQ27427_current(current_measure type) {
    //type = AVG;
    int16_t current = 0;
    switch (type) {
        case AVG:
            current = (int16_t) BQ27427_readWord(BQ27427_COMMAND_AVG_CURRENT);
            break;
        case STBY:
            current = (int16_t) BQ27427_readWord(BQ27427_COMMAND_STDBY_CURRENT);
            break;
        case MAX:
            current = (int16_t) BQ27427_readWord(BQ27427_COMMAND_MAX_CURRENT);
            break;
    }

    return current;
}

/**
 * Reads and returns the specified capacity measurement
 *
 * @param capacity_measure enum specifying capacity value to be read
 * @return specified capacity measurement in mAh.
 * */
uint16_t BQ27427_capacity(capacity_measure type) {
    //type = REMAIN;
    uint16_t capacity = 0;
    switch (type) {
        case REMAIN:
            return BQ27427_readWord(BQ27427_COMMAND_REM_CAPACITY);
            break;
        case FULL:
            return BQ27427_readWord(BQ27427_COMMAND_FULL_CAPACITY);
            break;
        case AVAIL:
            capacity = BQ27427_readWord(BQ27427_COMMAND_NOM_CAPACITY);
            break;
        case AVAIL_FULL:
            capacity = BQ27427_readWord(BQ27427_COMMAND_AVAIL_CAPACITY);
            break;
        case REMAIN_F:
            capacity = BQ27427_readWord(BQ27427_COMMAND_REM_CAP_FIL);
            break;
        case REMAIN_UF:
            capacity = BQ27427_readWord(BQ27427_COMMAND_REM_CAP_UNFL);
            break;
        case FULL_F:
            capacity = BQ27427_readWord(BQ27427_COMMAND_FULL_CAP_FIL);
            break;
        case FULL_UF:
            capacity = BQ27427_readWord(BQ27427_COMMAND_FULL_CAP_UNFL);
            break;
        case TRUE_REMAIN:
            capacity = BQ27427_readWord(BQ27427_COMMAND_TRUE_REM_CAP);
            break;
        case DESIGN:
            capacity = BQ27427_readWord(BQ27427_EXTENDED_CAPACITY);

        default:
            return BQ27427_readWord(BQ27427_COMMAND_REM_CAPACITY);
    }

    return capacity;
}

/**
 * Reads and returns measured average power
 *
 * @return average power in mAh.
 * */
int16_t BQ27427_power(void) {
    return (int16_t) BQ27427_readWord(BQ27427_COMMAND_AVG_POWER);
}

/**
 * Reads and returns specified state of charge measurement
 *
 * @param soc_measure enum specifying filtered or unfiltered measurement
 * @return specified state of charge measurement in %
 * */
uint16_t BQ27427_soc(soc_measure type) {
    //type = FILTERED;
    uint16_t socRet = 0;
    switch (type) {
        case FILTERED:
            socRet = BQ27427_readWord(BQ27427_COMMAND_SOC);
            break;
        case UNFILTERED:
            socRet = BQ27427_readWord(BQ27427_COMMAND_SOC_UNFL);
            break;
    }

    return socRet;
}

/**
 * Reads and returns specified state of health measurement
 *
 * @param soh_measure enum specifying filtered or unfiltered measurement
 * @return specified state of health measurement in %, or status bits
 * */
uint8_t BQ27427_soh(soh_measure type) {
    //type = PERCENT;
    uint16_t sohRaw = BQ27427_readWord(BQ27427_COMMAND_SOH);
    uint8_t sohStatus = sohRaw >> 8;
    uint8_t sohPercent = sohRaw & 0x00FF;

    if (type == PERCENT)
        return sohPercent;
    else
        return sohStatus;
}

/**
 * Reads and returns specified temperature measurement
 *
 * @param temp_measure enum specifying internal or battery measurement
 * @return specified temperature measurement in degrees C
 * */
uint16_t BQ27427_temperature(temp_measure type) {
    //type = BATTERY;
    uint16_t temp = 0;
    float temp_k = 0.0f;
    uint16_t temp_C = 0;
    switch (type) {
        case BATTERY:
            temp = BQ27427_readWord(BQ27427_COMMAND_TEMP);
            break;
        case INTERNAL_TEMP:
            temp = BQ27427_readWord(BQ27427_COMMAND_INT_TEMP);
            break;
    }

    temp_k = (float) temp * 0.1;
    temp = (uint16_t) (temp_k - 273.15);


    return temp;
}

/*****************************************************************************
 ************************** GPOUT Control Functions **************************
 *****************************************************************************/

/**
 * Get GPOUT polarity setting (active-high or active-low)
 *
 * @return true if active-high, false if active-low
 * */
bool BQ27427_GPOUTPolarity(void) {
    uint16_t opConfigRegister = BQ27427_opConfig();

    return (opConfigRegister & BQ27427_OPCONFIG_GPIOPOL);
}

bool BQ27427_setSLEEPenable(bool enable) {
    uint16_t oldOpConfig = BQ27427_opConfig();

    // Check to see if we need to update opConfig:
    if ((enable && (oldOpConfig & BQ27427_OPCONFIG_GPIOPOL)) ||
        (!enable && !(oldOpConfig & BQ27427_OPCONFIG_GPIOPOL)))
        return true;

    uint16_t newOpConfig = oldOpConfig;
    if (enable)
        newOpConfig |= BQ27427_OPCONFIG_SLEEP;
    else
        newOpConfig &= ~(BQ27427_OPCONFIG_SLEEP);

    return BQ27427_writeOpConfig(newOpConfig);
}



/**
 * Set GPOUT polarity to active-high or active-low
 *
 * @param activeHigh is true if active-high, false if active-low
 * @return true on success
 * */
bool BQ27427_setGPOUTPolarity(bool activeHigh) {
    uint16_t oldOpConfig = BQ27427_opConfig();

    // Check to see if we need to update opConfig:
    if ((activeHigh && (oldOpConfig & BQ27427_OPCONFIG_GPIOPOL)) ||
        (!activeHigh && !(oldOpConfig & BQ27427_OPCONFIG_GPIOPOL)))
        return true;

    uint16_t newOpConfig = oldOpConfig;
    if (activeHigh)
        newOpConfig |= BQ27427_OPCONFIG_GPIOPOL;
    else
        newOpConfig &= ~(BQ27427_OPCONFIG_GPIOPOL);

    return BQ27427_writeOpConfig(newOpConfig);
}



/**
 * Get GPOUT function (BAT_LOW or SOC_INT)
 *
 * @return true if BAT_LOW or false if SOC_INT
 * */
bool BQ27427_GPOUTFunction(void) {
    uint16_t opConfigRegister = BQ27427_opConfig();

    return (opConfigRegister & BQ27427_OPCONFIG_BATLOWEN);
}

/**
 * Set GPOUT function to BAT_LOW or SOC_INT
 *
 * @param function should be either BAT_LOW or SOC_INT
 * @return true on success
 * */
bool BQ27427_setGPOUTFunction(gpout_function function) {
    uint16_t oldOpConfig = BQ27427_opConfig();

    // Check to see if we need to update opConfig:
    if ((function && (oldOpConfig & BQ27427_OPCONFIG_BATLOWEN)) ||
        (!function && !(oldOpConfig & BQ27427_OPCONFIG_BATLOWEN)))
        return true;

    // Modify BATLOWN_EN bit of opConfig:
    uint16_t newOpConfig = oldOpConfig;
    if (function)
        newOpConfig |= BQ27427_OPCONFIG_BATLOWEN;
    else
        newOpConfig &= ~(BQ27427_OPCONFIG_BATLOWEN);

    // Write new opConfig
    return BQ27427_writeOpConfig(newOpConfig);
}


bool BQ27427_set_BI_PU_EN(bool detect_bat_pin_enable) {
    uint16_t oldOpConfig = BQ27427_opConfig();

    // Check to see if we need to update opConfig:
    if ((detect_bat_pin_enable && (oldOpConfig & BQ27427_OPCONFIG_GPIOPOL)) ||
        (!detect_bat_pin_enable && !(oldOpConfig & BQ27427_OPCONFIG_GPIOPOL)))
        return true;

    uint16_t newOpConfig = oldOpConfig;
    if (detect_bat_pin_enable)
        newOpConfig |= BQ27427_OPCONFIG_BI_PU_EN;
    else
        newOpConfig &= ~(BQ27427_OPCONFIG_BI_PU_EN);

    return BQ27427_writeOpConfig(newOpConfig);

}
/**
 * Get SOC1_Set Threshold - threshold to set the alert flag
 *
 * @return state of charge value between 0 and 100%
 * */
uint8_t BQ27427_SOC1SetThreshold(void)
{
    return BQ27427_readExtendedData(BQ27427_ID_DISCHARGE, 0);
}

uint8_t BQ27427_ReadBoardOffset(void)
{
    return BQ27427_readExtendedData(64, 0);

}

bool BQ27427_WriteBoardOffset(void)
{
    uint8_t offset = 0x44;
    return (BQ27427_writeExtendedData(64, 0, &offset,1));
}

/**
 * Get SOC1_Clear Threshold - threshold to clear the alert flag
 *
 * @return state of charge value between 0 and 100%
 * */
uint8_t BQ27427_SOC1ClearThreshold(void)
{
    return BQ27427_readExtendedData(BQ27427_ID_DISCHARGE, 1);
}

/**
 * Set the SOC1 set and clear thresholds to a percentage
 *
 * @param set and clear percentages between 0 and 100. clear > set.
 * @return true on success
 * */
bool BQ27427_setSOC1Thresholds(uint8_t set, uint8_t clear)
{
    uint8_t thresholds[2];
    thresholds[0] = constrain(set, 0, 100);
    thresholds[1] = constrain(clear, 0, 100);
    return BQ27427_writeExtendedData(BQ27427_ID_DISCHARGE, 0, thresholds, 2);
}

/**
 * Get SOCF_Set Threshold - threshold to set the alert flag
 *
 * @return state of charge value between 0 and 100%
 * */
uint8_t BQ27427_SOCFSetThreshold(void)
{
    return BQ27427_readExtendedData(BQ27427_ID_DISCHARGE, 2);
}

/**
 * Get SOCF_Clear Threshold - threshold to clear the alert flag
 *
 * @return state of charge value between 0 and 100%
 * */
uint8_t BQ27427_SOCFClearThreshold(void)
{
    return BQ27427_readExtendedData(BQ27427_ID_DISCHARGE, 3);
}

/**
 * Set the SOCF set and clear thresholds to a percentage
 *
 * @param set and clear percentages between 0 and 100. clear > set.
 * @return true on success
 * */
bool BQ27427_setSOCFThresholds(uint8_t set, uint8_t clear)
{
    uint8_t thresholds[2];
    thresholds[0] = constrain(set, 0, 100);
    thresholds[1] = constrain(clear, 0, 100);
    return BQ27427_writeExtendedData(BQ27427_ID_DISCHARGE, 2, thresholds, 2);
}

/**
 * Check if the SOC1 flag is set in flags()
 *
 * @return true if flag is set
 * */
bool BQ27427_socFlag(void)
{
    uint16_t flagState = BQ27427_flags();

    return flagState & BQ27427_FLAG_SOC1;
}

/**
 * Check if the SOCF flag is set in flags()
 *
 * @return true if flag is set
 * */
bool BQ27427_socfFlag(void)
{
    uint16_t flagState = BQ27427_flags();

    return flagState & BQ27427_FLAG_SOCF;

}

/**
 * Check if the ITPOR flag is set in flags()
 *
 * @return true if flag is set
 * */
bool BQ27427_itporFlag(void)
{
    uint16_t flagState = BQ27427_flags();

    return flagState & BQ27427_FLAG_ITPOR;
}

bool BQ27427_initComp(void){

    uint16_t stat = BQ27427_status();
    return stat & BQ27427_STATUS_INITCOMP;

}

/**
 * Check if the FC flag is set in flags()
 *
 * @return true if flag is set
 * */
bool BQ27427_fcFlag(void)
{
    uint16_t flagState = BQ27427_flags();

    return flagState & BQ27427_FLAG_FC;
}

/**
 * Check if the CHG flag is set in flags()
 *
 * @return true if flag is set
 * */
bool BQ27427_chgFlag(void)
{
    uint16_t flagState = BQ27427_flags();

    return flagState & BQ27427_FLAG_CHG;
}

/**
 * Check if the DSG flag is set in flags()
 *
 * @return true if flag is set
 * */
bool BQ27427_dsgFlag(void)
{
    uint16_t flagState = BQ27427_flags();

    return flagState & BQ27427_FLAG_DSG;
}

/**
 * Get the SOC_INT interval delta
 *
 * @return interval percentage value between 1 and 100
 * */
uint8_t BQ27427_sociDelta(void)
{
    return BQ27427_readExtendedData(BQ27427_ID_STATE, 26);
}

/**
 * Set the SOC_INT interval delta to a value between 1 and 100
 *
 * @param interval percentage value between 1 and 100
 * @return true on success
 * */
bool BQ27427_setSOCIDelta(uint8_t delta)
{
    uint8_t soci = constrain(delta, 0, 100);
    return BQ27427_writeExtendedData(BQ27427_ID_STATE, 26, &soci, 1);
}

/**
 * Pulse the GPOUT pin - must be in SOC_INT mode
 *
 * @return true on success
 * */
bool BQ27427_pulseGPOUT(void)
{
    return BQ27427_executeControlWord(BQ27427_CONTROL_PULSE_SOC_INT);
}

bool BQ27427_SET_HIBERNATE(void)
{
    return BQ27427_executeControlWord(BQ27427_CONTROL_SET_HIBERNATE);
}

bool BQ27427_CLEAR_HIBERNATE(void)
{
    return BQ27427_executeControlWord(BQ27427_CONTROL_CLEAR_HIBERNATE);
}

/*****************************************************************************
 *************************** Control Sub-Commands ****************************
 *****************************************************************************/

/**
 * Read the device type - should be 0x0421
 *
 * @return 16-bit value read from DEVICE_TYPE subcommand
 * */
uint16_t BQ27427_deviceType(void)
{
    return BQ27427_readControlWord(BQ27427_CONTROL_DEVICE_TYPE);
}

/**
 * Enter configuration mode - set userControl if calling from an App
 * and you want control over when to exitConfig.
 *
 * @param userControl is true if the App is handling entering
 * and exiting config mode (should be false in library calls).
 * @return true on success
 * */
bool BQ27427_enterConfig(bool userControl) {

    if (userControl)
    {
        userConfigControl = true;

        if (BQ27427_sealed())
        {
            sealFlag = true;
            BQ27427_unseal(); // Must be unsealed before making changes
        }

        if (BQ27427_executeControlWord(BQ27427_CONTROL_SET_CFGUPDATE))
        {
            int16_t timeout = BQ27427_I2C_TIMEOUT;
            while ((timeout--) && (!(BQ27427_flags() & BQ27427_FLAG_CFGUPMODE)))
                //delay_ms(10);
            delay(10);

            if (timeout > 0)
                return true;
            else
                return false;
        }
        else
            return false;
    }
    else
        return false;

}

/**
 * Exit configuration mode with the option to perform a resimulation
 *
 * @param resim is true if resimulation should be performed after exiting
 * @return true on success
 * */
bool BQ27427_exitConfig(bool resim) {

    //resim = true;
    // There are two methods for exiting config mode:
    //    1. Execute the EXIT_CFGUPDATE command
    //    2. Execute the SOFT_RESET command
    // EXIT_CFGUPDATE exits config mode _without_ an OCV (open-circuit voltage)
    // measurement, and without resimulating to update unfiltered-SoC and SoC.
    // If a new OCV measurement or resimulation is desired, SOFT_RESET or
    // EXIT_RESIM should be used to exit config mode.
    if (resim) {
        if (BQ27427_softReset()) {
            int16_t timeout = BQ27427_I2C_TIMEOUT;
            while ((timeout--) && ((BQ27427_flags() & BQ27427_FLAG_CFGUPMODE)))
                //delay_ms(1);
            delay(1);
            if (timeout > 0) {
                if (sealFlag) BQ27427_seal(); // Seal back up if we IC was sealed coming in
                return true;
            }
        }
        return false;
    } else {
        return BQ27427_executeControlWord(BQ27427_CONTROL_EXIT_CFGUPDATE);
    }
}

/**
 * Read the flags() command
 *
 * @return 16-bit representation of flags() command register
 * */
uint16_t BQ27427_flags(void) {
    return BQ27427_readWord(BQ27427_COMMAND_FLAGS);
}

/**
 * Read the CONTROL_STATUS subcommand of control()
 *
 * @return 16-bit representation of CONTROL_STATUS subcommand
 * */
uint16_t BQ27427_status(void) {
    return BQ27427_readControlWord(BQ27427_CONTROL_STATUS);
}

/***************************** Private Functions *****************************/

/**
 * Check if the BQ27427-G1A is sealed or not.
 *
 * @return true if the chip is sealed
 * */
static bool BQ27427_sealed(void) {
    uint16_t stat = BQ27427_status();
    return stat & BQ27427_STATUS_SS;
}

/**
 * Seal the BQ27427-G1A
 *
 * @return true on success
 * */
static bool BQ27427_seal(void) {
    return BQ27427_readControlWord(BQ27427_CONTROL_SEALED);
}

/**
 * UNseal the BQ27427-G1A
 *
 * @return true on success
 * */
bool BQ27427_unseal(void) {
    // To unseal the BQ27427, write the key to the control
    // command. Then immediately write the same key to control again.
    if (BQ27427_readControlWord(BQ27427_UNSEAL_KEY)) {
        return BQ27427_readControlWord(BQ27427_UNSEAL_KEY);
    }
    return false;
}

/**
 * Read the 16-bit opConfig register from extended data
 *
 * @return opConfig register contents
 * */
static uint16_t BQ27427_opConfig(void) {
    return BQ27427_readWord(BQ27427_EXTENDED_OPCONFIG);
}

/**
 * Write the 16-bit opConfig register in extended data
 *
 * @param New 16-bit value for opConfig
 * @return true on success
 * */
static bool BQ27427_writeOpConfig(uint16_t value) {

    uint8_t opConfigData[2] = {value >> 8, value & 0x00FF};

    // OpConfig register location: BQ27427_ID_REGISTERS id, offset 0
    return BQ27427_writeExtendedData(BQ27427_ID_REGISTERS, 0, opConfigData, 2);
}

/**
 * Issue a soft-reset to the BQ27427-G1A
 *
 * @return true on success
 * */
bool BQ27427_softReset(void) {
    return BQ27427_executeControlWord(BQ27427_CONTROL_SOFT_RESET);
}

void BQ27427_Full_Reset(void) {
    BQ27427_enterConfig(true);
    BQ27427_executeControlWord(BQ27427_CONTROL_RESET);
    BQ27427_exitConfig(true);

}


void BQ27427_Shutdown_EN(void) {
    BQ27427_enterConfig(true);
    BQ27427_executeControlWord(BQ27427_CONTROL_SHUTDOWN_ENABLE);
    BQ27427_exitConfig(true);

}

void BQ27427_Enter_SHUTDOWN(void) {
    BQ27427_enterConfig(true);
    BQ27427_executeControlWord(BQ27427_CONTROL_SHUTDOWN);
    BQ27427_exitConfig(true);

}

/**
 * Read a 16-bit command word from the BQ27427-G1A
 *
 * @param subAddress is the command to be read from
 * @return 16-bit value of the command's contents
 * */
static uint16_t BQ27427_readWord(uint16_t subAddress) {
    uint8_t data[2];
    BQ27427_i2cReadBytes(subAddress, data, 2);
    return ((uint16_t) data[1] << 8) | data[0];
}

/**
 * Read a 16-bit subcommand() from the BQ27427-G1A's control()
 *
 * @param function is the subcommand of control() to be read
 * @return 16-bit value of the subcommand's contents
 * */
static uint16_t BQ27427_readControlWord(uint16_t function) {

    uint8_t command[2] = {function & 0x00FF, function >> 8};
    uint8_t data[2] = {0, 0};

    BQ27427_i2cWriteBytes((uint8_t) 0, command, 2);

    if (BQ27427_i2cReadBytes((uint8_t) 0, data, 2)) {
        return ((uint16_t) data[1] << 8) | data[0];
    }

    return false;
}

/**
 * Execute a subcommand() from the BQ27427-G1A's control()
 *
 * @param function is the subcommand of control() to be executed
 * @return true on success
 * */
static bool BQ27427_executeControlWord(uint16_t function) {

    uint8_t command[2] = {function & 0x00FF, function >> 8};

    if (BQ27427_i2cWriteBytes((uint8_t) 0, command, 2))
        return true;

    return false;
}

/*****************************************************************************
 ************************** Extended Data Commands ***************************
 *****************************************************************************/

/**
 * Issue a BlockDataControl() command to enable BlockData access
 *
 * @return true on success
 * */
static bool BQ27427_blockDataControl(void)
{
    uint8_t enableByte = 0x00;
    return BQ27427_i2cWriteBytes(BQ27427_EXTENDED_CONTROL, &enableByte, 1);
}

/**
    Issue a DataClass() command to set the data class to be accessed

    @param id is the id number of the class
    @return true on success
*/
static bool BQ27427_blockDataClass(uint8_t id)
{
    return BQ27427_i2cWriteBytes(BQ27427_EXTENDED_DATACLASS, &id, 1);
}

/**
    Issue a DataBlock() command to set the data block to be accessed

    @param offset of the data block
    @return true on success
*/
static bool BQ27427_blockDataOffset(uint8_t offset)
{
    return BQ27427_i2cWriteBytes(BQ27427_EXTENDED_DATABLOCK, &offset, 1);
}

/**
 * Read the current checksum using BlockDataCheckSum()
 *
 * @return true on success
 * */
static uint8_t BQ27427_blockDataChecksum(void)
{
    uint8_t csum;
    BQ27427_i2cReadBytes(BQ27427_EXTENDED_CHECKSUM, &csum, 1);
    return csum;
}

/**
 * @return Use BlockData() to read a byte from the loaded extended data
 *
 * @param offset of data block byte to be read
 * @return true on success
 * */
static uint8_t BQ27427_readBlockData(uint8_t offset)
{
    uint8_t ret;
    uint8_t address = offset + BQ27427_EXTENDED_BLOCKDATA;
    BQ27427_i2cReadBytes(address, &ret, 1);
    return ret;
}

/**
 * @return Use BlockData() to write a byte to an offset of the loaded data
 *
 * @param offset is the position of the byte to be written
 * data is the value to be written
 * @return true on success
 * */
static bool BQ27427_writeBlockData(uint8_t offset, uint8_t data)
{
    uint8_t address = offset + BQ27427_EXTENDED_BLOCKDATA;
    return BQ27427_i2cWriteBytes(address, &data, 1);
}

/**
 * Read all 32 bytes of the loaded extended data and compute a
 * checksum based on the values.
 *
 * @return 8-bit checksum value calculated based on loaded data
 * */
uint8_t BQ27427_computeBlockChecksum(void)
{
    uint8_t data[32];
    BQ27427_i2cReadBytes(BQ27427_EXTENDED_BLOCKDATA, data, 32);

    uint8_t csum = 0;
    for (uint8_t i = 0; i < 32; i++) {
        csum += data[i];
    }
    csum = 255 - csum;

    return csum;
}

/**
 * Use the BlockDataCheckSum() command to write a checksum value
 *
 * @param csum is the 8-bit checksum to be written
 * @return true on success
 * */
static bool BQ27427_writeBlockChecksum(uint8_t csum)
{
    return BQ27427_i2cWriteBytes(BQ27427_EXTENDED_CHECKSUM, &csum, 1);
}

/**
 * Read a byte from extended data specifying a class ID and position offset
 *
 * @param classID is the id of the class to be read from
 *          offset is the byte position of the byte to be read
 * @return 8-bit value of specified data
 * */
static uint8_t BQ27427_readExtendedData(uint8_t classID, uint8_t offset) {
    uint8_t retData = 0;
    if (!userConfigControl) BQ27427_enterConfig(false);

    if (!BQ27427_blockDataControl()) // // enable block data memory control
        return false; // Return false if enable fails
    if (!BQ27427_blockDataClass(classID)) // Write class ID using DataBlockClass()
        return false;

    BQ27427_blockDataOffset(offset / 32); // Write 32-bit block offset (usually 0)

    BQ27427_computeBlockChecksum(); // Compute checksum going in
    uint8_t oldCsum = BQ27427_blockDataChecksum();
    retData = BQ27427_readBlockData(offset % 32); // Read from offset (limit to 0-31)

    if (!userConfigControl) BQ27427_exitConfig(true);

    return retData;
}

/**
 * Write a specified number of bytes to extended data specifying a
 * class ID, position offset.
 *
 * @param classID is the id of the class to be read from
 *          offset is the byte position of the byte to be read
 *          data is the data buffer to be written
 *          len is the number of bytes to be written
 * @return true on success
 * */
static bool BQ27427_writeExtendedData(uint8_t classID, uint8_t offset, uint8_t * data, uint8_t len) {
    if (len > 32)
        return false;

    if (!userConfigControl) BQ27427_enterConfig(false);

    if (!BQ27427_blockDataControl()) // // enable block data memory control
        return false; // Return false if enable fails
    if (!BQ27427_blockDataClass(classID)) // Write class ID using DataBlockClass()
        return false;

    BQ27427_blockDataOffset(offset / 32); // Write 32-bit block offset (usually 0)
    BQ27427_computeBlockChecksum(); // Compute checksum going in
    uint8_t oldCsum = BQ27427_blockDataChecksum();

    // Write data bytes:
    for (int i = 0; i < len; i++) {
        // Write to offset, mod 32 if offset is greater than 32
        // The blockDataOffset above sets the 32-bit block
        BQ27427_writeBlockData((offset % 32) + i, data[i]);
    }

    // Write new checksum using BlockDataChecksum (0x60)
    uint8_t newCsum = BQ27427_computeBlockChecksum(); // Compute the new checksum
    BQ27427_writeBlockChecksum(newCsum);

    if (!userConfigControl) BQ27427_exitConfig(true);

    return true;
}


static uint8_t constrain(const uint8_t x, const uint8_t a, const uint8_t b) {
    if(x < a) {
        return a;
    }else if(b < x) {
        return b;
    }else
        return x;
}
