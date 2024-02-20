/*
 *  EEPROM.c
 *
 *  API for the 24CW640T-I/CS1668 EEPROM
 *
 *  Created on: 7 sept. 2023
 *      Author: rami
 */

/*================== Includes =============================================*/
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <ti/sysbios/knl/Task.h>
#include "EEPROM.h"
#include "shared.h"
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/sysbios/BIOS.h>
#include "ti_drivers_config.h"
/*================== Macros and Definitions ===============================*/

/*================== Constant and Variable Definitions ====================*/
/* Error Counters */
uint32_t M24xxx_eepromError_wrNb = 0;
uint32_t M24xxx_eepromError_rdNb = 0;
uint32_t M24xxx_eepromError_wrNbMax = 0;
uint32_t M24xxx_eepromError_rdNbMax = 0;
uint32_t M24xxx_eepromError_watReady = 0;
/* Functions return value */
EEPROM_return return_result;

void EEPROM_Init(void)
{
    //No write protection to unlock in this case
    //For now, nothing to initialize
}


EEPROM_return EEPROM_Write(uint16_t Memory_Address, uint8_t * aTxBuffer, uint16_t buffSize)
{

    EEPROM_return status = EEPROM_OK;

    //Check if address is within range
    if(Memory_Address > EEPROM_LAST_ADDRESS) return EEPROM_ADDRESS_OOR;

    uint8_t TxTransaction[buffSize+2];
    I2C_Transaction i2cTransaction;
    i2cTransaction.writeBuf = TxTransaction;
    i2cTransaction.writeCount = buffSize+2;
    i2cTransaction.readCount  = 0;
    i2cTransaction.targetAddress = EEPROM_ADD;

    TxTransaction[0] = Memory_Address >> 8;
    TxTransaction[1] = Memory_Address;

    //Fill the rest of the buffer with DATA
    for(uint32_t i=2;i<buffSize+2;i++)
        TxTransaction[i] = aTxBuffer[i-2];


    //Write DATA into EEPROM
    if (!I2C_transfer(i2c, &i2cTransaction))
    {
            status = EEPROM_ERROR;
    }

    return(status);

}


EEPROM_return EEPROM_Read(uint16_t  Memory_Address, uint8_t * aRxBuffer, uint16_t buffSize)
{
    EEPROM_return status = EEPROM_OK;

    //Check if address is within range
    if(Memory_Address > EEPROM_LAST_ADDRESS) return EEPROM_ADDRESS_OOR;

    uint8_t TxTransaction[2];
    I2C_Transaction i2cTransaction;
    i2cTransaction.writeBuf = TxTransaction;
    i2cTransaction.readBuf = (void*) aRxBuffer;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readCount  = buffSize;
    i2cTransaction.targetAddress = EEPROM_ADD;

    TxTransaction[0] = Memory_Address >> 8;
    TxTransaction[1] = Memory_Address;

    //Read DATA from EEPROM
    if (!I2C_transfer(i2c, &i2cTransaction))
    {
            status = EEPROM_ERROR;
    }

    return(status);

}
