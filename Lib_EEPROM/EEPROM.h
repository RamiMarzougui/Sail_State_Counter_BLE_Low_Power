/*
 *  EEPROM.h
 *
 *  API for the 24CW640T-I/CS1668 EEPROM
 *
 *  Created on: 7 sept. 2023
 *      Author: rami
 */

/* *****************************************************
 * EEPROM Memory Map
 * Author: Rami
 * Date: September 8, 2023
 * Description: EEPROM memory map for the Sens2Sail application
 *
 * Memory Map:
 * -----------
 *  Address    | Description
 * -----------------------------------------------------
 *  0x0000     | SW Version Major
 *  0x0001     | SW Version Minor
 *  0x0002     | SW Version Bug Fixes
 *  0x0004     | SN MSB
 *  0x0104     | SN LSB
 *  0x0124     | X
 *  0x0324     | X
 *  0x03A4     | X
 *  0x03B4     | X
 * -----------------------------------------------------
 * Total EEPROM Size: 0x2000 (8192 bytes)
 *
 * Usage Notes:
 * - The EEPROM is divided into various sections for
 *   configuration, user data, calibration, event logs (maybe) ?
 * *****************************************************/

#ifndef LIB_EEPROM_EEPROM_H_
#define LIB_EEPROM_EEPROM_H_

/*================== Includes =============================================*/
#include <ti/drivers/I2C.h>
/*================== Macros and Definitions ===============================*/
#define EEPROM_24Cxxx_SIZE            65536 //64KB --> * 1024
#define EEPROM_24Cxxx_PAGESIZE       32
#define EEPROM_ADD 0x50
#define EEPROM_LAST_ADDRESS 0x1FFF
#define EEPROM_24Cxxx_MAX_TRIALS               10
#define EEPROM_24Cxxx_DEV_READY_TIMEOUT         50

#define SW_VERSION_ADD 0x00
#define SN_ADD 0x03
/**
 * Possible return values when interacting with the EEPROM
 */
typedef enum{
    EEPROM_OK,   /*!< Read or Write operation OK      */
    EEPROM_ERROR,   /*!< Error trying to read or write the EEPROM    */
    EEPROM_ADDRESS_OOR,  /*!< Address out of range    */
}EEPROM_return;
/*================== Constant and Variable Definitions ====================*/
/* Error Counters */
extern uint32_t M24xxx_eepromError_wrNb;
extern uint32_t M24xxx_eepromError_rdNb;
extern uint32_t M24xxx_eepromError_wrNbMax;
extern uint32_t M24xxx_eepromError_rdNbMax;
extern uint32_t M24xxx_eepromError_watReady ;
/*================== Function Prototypes ==================================*/
/**
 * @brief   Initialize the EEPROM and unlock write protection if it exists
 *
 * @details
 *
 */
void EEPROM_Init(void);

/**
 * @brief   Writes a number of bytes into the EEPROM starting from a certain address
 *
 * @details
 *
 *
 * @param   Memory_Address    Starting address to write at
 * @param   aTxBuffer         Buffer that contains the data to write
 * @param   buffSize          Number of bytes to write into EEPROM
 *
 * @return  EEPROM_OK if no error occurred, otherwise EEPROM_ERROR or EEPROM_ADDRESS_OOR if address is out of range
 */
EEPROM_return EEPROM_Write(uint16_t Memory_Address, uint8_t * aTxBuffer, uint16_t buffSize);

/**
 * @brief   Read a number of bytes from the EEPROM starting from a certain address
 *
 * @details
 *
 *
 * @param   Memory_Address    Starting address to read from
 * @param   aRxBuffer         Buffer that will contain the read data
 * @param   buffSize          Number of bytes to read from EEPROM
 *
 * @return  EEPROM_OK if no error occurred, otherwise EEPROM_ERROR or EEPROM_ADDRESS_OOR if address is out of range
 */
EEPROM_return EEPROM_Read(uint16_t  Memory_Address, uint8_t * aRxBuffer, uint16_t buffSize);


#endif /* LIB_EEPROM_EEPROM_H_ */
