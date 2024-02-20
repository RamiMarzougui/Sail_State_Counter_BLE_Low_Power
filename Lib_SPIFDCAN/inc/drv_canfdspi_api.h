/*
 *  drv_canfdspi_api.h
 *
 *  API function prototypes for the CAN FD SPI controller
 *
 *  Created on: 7 sept. 2023
 *      Author: rami
 */

#ifndef _DRV_CANFDSPI_API_H
#define _DRV_CANFDSPI_API_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "drv_canfdspi_defines.h"
#include <ecode.h>
#include <ti/drivers/SPI.h>

#ifdef __cplusplus  // Provide C++ Compatibility
extern "C" {
#endif


typedef uint8_t CANFDSPI_MODULE_ID;

// *****************************************************************************
// *****************************************************************************
//! Reset DUT

Ecode_t DRV_CANFDSPI_Reset(CANFDSPI_MODULE_ID index, SPI_Handle controllerSpi);

// *****************************************************************************
// *****************************************************************************
// Section: SPI Access Functions

// *****************************************************************************
//! SPI Read Byte

Ecode_t DRV_CANFDSPI_ReadByte(CANFDSPI_MODULE_ID index, uint16_t address,
        uint8_t *rxd, SPI_Handle controllerSpi);

// *****************************************************************************
//! SPI Write Byte

Ecode_t DRV_CANFDSPI_WriteByte(CANFDSPI_MODULE_ID index, uint16_t address,
        uint8_t txd, SPI_Handle controllerSpi);

// *****************************************************************************
//! SPI Read Word

int8_t DRV_CANFDSPI_ReadWord(CANFDSPI_MODULE_ID index, uint16_t address,
        uint32_t *rxd);

// *****************************************************************************
//! SPI Write Word

Ecode_t DRV_CANFDSPI_WriteWord(CANFDSPI_MODULE_ID index, uint16_t address,
        uint32_t txd);

/// *****************************************************************************
//! SPI Read Word

Ecode_t DRV_CANFDSPI_ReadHalfWord(CANFDSPI_MODULE_ID index, uint16_t address,
        uint16_t *rxd);

// *****************************************************************************
//! SPI Write Word

Ecode_t DRV_CANFDSPI_WriteHalfWord(CANFDSPI_MODULE_ID index, uint16_t address,
        uint16_t txd);

// *****************************************************************************
//! SPI Read Byte Array

Ecode_t DRV_CANFDSPI_ReadByteArray(CANFDSPI_MODULE_ID index, uint16_t address,
        uint8_t *rxd, uint16_t nBytes);

// *****************************************************************************
//! SPI Write Byte Array

Ecode_t DRV_CANFDSPI_WriteByteArray(CANFDSPI_MODULE_ID index, uint16_t address,
        uint8_t *txd, uint16_t nBytes);

// *****************************************************************************
//! SPI SFR Write Byte Safe

int8_t DRV_CANFDSPI_WriteByteSafe(CANFDSPI_MODULE_ID index, uint16_t address,
        uint8_t txd);

// *****************************************************************************
//! SPI RAM Write Word Safe

int8_t DRV_CANFDSPI_WriteWordSafe(CANFDSPI_MODULE_ID index, uint16_t address,
        uint32_t txd);

// *****************************************************************************
//! SPI Read Byte Array with CRC

int8_t DRV_CANFDSPI_ReadByteArrayWithCRC(CANFDSPI_MODULE_ID index, uint16_t address,
        uint8_t *rxd, uint16_t nBytes, bool fromRam, bool* crcIsCorrect);

// *****************************************************************************
//! SPI Write Byte Array with CRC

int8_t DRV_CANFDSPI_WriteByteArrayWithCRC(CANFDSPI_MODULE_ID index, uint16_t address,
        uint8_t *txd, uint16_t nBytes, bool fromRam);

// *****************************************************************************
//! SPI Read Word Array

Ecode_t DRV_CANFDSPI_ReadWordArray(CANFDSPI_MODULE_ID index, uint16_t address,
        uint32_t *rxd, uint16_t nWords);

// *****************************************************************************
//! SPI Write Word Array

int8_t DRV_CANFDSPI_WriteWordArray(CANFDSPI_MODULE_ID index, uint16_t address,
        uint32_t *txd, uint16_t nWords);


// *****************************************************************************
// *****************************************************************************
// Section: Configuration

// *****************************************************************************
//! CAN Control register configuration

Ecode_t DRV_CANFDSPI_Configure(CANFDSPI_MODULE_ID index, CAN_CONFIG* config);

// *****************************************************************************
//! Reset Configure object to reset values
int8_t DRV_CANFDSPI_ConfigureObjectReset(CAN_CONFIG* config);


// *****************************************************************************
// *****************************************************************************
// Section: Operating mode

// *****************************************************************************
//! Select Operation Mode

Ecode_t DRV_CANFDSPI_OperationModeSelect(CANFDSPI_MODULE_ID index,
        CAN_OPERATION_MODE opMode);

// *****************************************************************************
//! Get Operation Mode

CAN_OPERATION_MODE DRV_CANFDSPI_OperationModeGet(CANFDSPI_MODULE_ID index);

// *****************************************************************************
//! Enable Low Power Mode

int8_t DRV_CANFDSPI_LowPowerModeEnable(CANFDSPI_MODULE_ID index);

// *****************************************************************************
//! Disable Low Power Mode

int8_t DRV_CANFDSPI_LowPowerModeDisable(CANFDSPI_MODULE_ID index);


// *****************************************************************************
// *****************************************************************************
// Section: CAN Transmit

// *****************************************************************************
//! Configure Transmit FIFO

Ecode_t DRV_CANFDSPI_TransmitChannelConfigure(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel, CAN_TX_FIFO_CONFIG* config);

// *****************************************************************************
//! Reset TransmitChannelConfigure object to reset values

int8_t DRV_CANFDSPI_TransmitChannelConfigureObjectReset(CAN_TX_FIFO_CONFIG* config);

// *****************************************************************************
//! Configure Transmit Queue

Ecode_t DRV_CANFDSPI_TransmitQueueConfigure(CANFDSPI_MODULE_ID index,
        CAN_TX_QUEUE_CONFIG* config);

// *****************************************************************************
//! Reset TransmitQueueConfigure object to reset values

int8_t DRV_CANFDSPI_TransmitQueueConfigureObjectReset(CAN_TX_QUEUE_CONFIG* config);

// *****************************************************************************
//! TX Channel Load

Ecode_t DRV_CANFDSPI_TransmitChannelLoad(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel, CAN_TX_MSGOBJ* txObj,
        uint8_t *txd, uint32_t txdNumBytes, bool flush);

// *****************************************************************************
//! TX Queue Load

static inline int8_t DRV_CANFDSPI_TransmitQueueLoad(CANFDSPI_MODULE_ID index,
        CAN_TX_MSGOBJ* txObj,
        uint8_t *txd, uint32_t txdNumBytes, bool flush)
{
    return DRV_CANFDSPI_TransmitChannelLoad(index, CAN_TXQUEUE_CH0, txObj, txd, txdNumBytes, flush);
}

// *****************************************************************************
//! TX Channel Flush

int8_t DRV_CANFDSPI_TransmitChannelFlush(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel);

// *****************************************************************************
//! Transmit Channel Status Get

int8_t DRV_CANFDSPI_TransmitChannelStatusGet(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel, CAN_TX_FIFO_STATUS* status);

// *****************************************************************************
//! Transmit FIFO Reset

int8_t DRV_CANFDSPI_TransmitChannelReset(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel);

// *****************************************************************************
//! Transmit FIFO Update

Ecode_t DRV_CANFDSPI_TransmitChannelUpdate(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel, bool flush);

// *****************************************************************************
//! TX Queue Flush

static inline int8_t DRV_CANFDSPI_TransmitQueueFlush(CANFDSPI_MODULE_ID index)
{
    return DRV_CANFDSPI_TransmitChannelFlush(index, CAN_TXQUEUE_CH0);
}

// *****************************************************************************
//! Transmit Queue Status Get

static inline int8_t DRV_CANFDSPI_TransmitQueueStatusGet(CANFDSPI_MODULE_ID index,
        CAN_TX_FIFO_STATUS* status)
{
    return DRV_CANFDSPI_TransmitChannelStatusGet(index, CAN_TXQUEUE_CH0, status);
}

// *****************************************************************************
//! Transmit Queue Reset

static inline int8_t DRV_CANFDSPI_TransmitQueueReset(CANFDSPI_MODULE_ID index)
{
    return DRV_CANFDSPI_TransmitChannelReset(index, CAN_TXQUEUE_CH0);
}

// *****************************************************************************
//! Transmit Queue Update

static inline int8_t DRV_CANFDSPI_TransmitQueueUpdate(CANFDSPI_MODULE_ID index, bool flush)
{
    return DRV_CANFDSPI_TransmitChannelUpdate(index, CAN_TXQUEUE_CH0, flush);
}

// *****************************************************************************
//! Request transmissions using TXREQ register

int8_t DRV_CANFDSPI_TransmitRequestSet(CANFDSPI_MODULE_ID index,
        CAN_TXREQ_CHANNEL txreq);

// *****************************************************************************
//! Get TXREQ register

int8_t DRV_CANFDSPI_TransmitRequestGet(CANFDSPI_MODULE_ID index,
        uint32_t* txreq);

// *****************************************************************************
//! Abort transmission of single FIFO

int8_t DRV_CANFDSPI_TransmitChannelAbort(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel);

// *****************************************************************************
//! Abort transmission of TXQ

static inline int8_t DRV_CANFDSPI_TransmitQueueAbort(CANFDSPI_MODULE_ID index)
{
    return DRV_CANFDSPI_TransmitChannelAbort(index, CAN_TXQUEUE_CH0);
}

// *****************************************************************************
//! Abort All transmissions

int8_t DRV_CANFDSPI_TransmitAbortAll(CANFDSPI_MODULE_ID index);

// *****************************************************************************
//! Set Transmit Bandwidth Sharing Delay
int8_t DRV_CANFDSPI_TransmitBandWidthSharingSet(CANFDSPI_MODULE_ID index,
        CAN_TX_BANDWITH_SHARING txbws);


// *****************************************************************************
// *****************************************************************************
// Section: CAN Receive

// *****************************************************************************
//! Filter Object Configuration

Ecode_t DRV_CANFDSPI_FilterObjectConfigure(CANFDSPI_MODULE_ID index,
        CAN_FILTER filter, CAN_FILTEROBJ_ID* id);

// *****************************************************************************
//! Filter Mask Configuration

Ecode_t DRV_CANFDSPI_FilterMaskConfigure(CANFDSPI_MODULE_ID index,
        CAN_FILTER filter, CAN_MASKOBJ_ID* mask);

// *****************************************************************************
//! Link Filter to FIFO

Ecode_t DRV_CANFDSPI_FilterToFifoLink(CANFDSPI_MODULE_ID index,
        CAN_FILTER filter, CAN_FIFO_CHANNEL channel, bool enable);

// *****************************************************************************
//! Filter Enable

int8_t DRV_CANFDSPI_FilterEnable(CANFDSPI_MODULE_ID index, CAN_FILTER filter);

// *****************************************************************************
//! Filter Disable

int8_t DRV_CANFDSPI_FilterDisable(CANFDSPI_MODULE_ID index, CAN_FILTER filter);

// *****************************************************************************
//! Set Device Net Filter Count
int8_t DRV_CANFDSPI_DeviceNetFilterCountSet(CANFDSPI_MODULE_ID index,
        CAN_DNET_FILTER_SIZE dnfc);

// *****************************************************************************
//! Configure Receive FIFO

Ecode_t DRV_CANFDSPI_ReceiveChannelConfigure(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel, CAN_RX_FIFO_CONFIG* config);

// *****************************************************************************
//! Reset ReceiveChannelConfigure object to reset value

int8_t DRV_CANFDSPI_ReceiveChannelConfigureObjectReset(CAN_RX_FIFO_CONFIG* config);

// *****************************************************************************
//! Receive Channel Status Get

int8_t DRV_CANFDSPI_ReceiveChannelStatusGet(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel, CAN_RX_FIFO_STATUS* status);

// *****************************************************************************
//! Get Received Message

Ecode_t DRV_CANFDSPI_ReceiveMessageGet(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel, CAN_RX_MSGOBJ* rxObj,
        uint8_t *rxd, uint8_t nBytes);

// *****************************************************************************
//! Receive FIFO Reset

int8_t DRV_CANFDSPI_ReceiveChannelReset(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel);

// *****************************************************************************
//! Receive FIFO Update


Ecode_t DRV_CANFDSPI_ReceiveChannelUpdate(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel);


// *****************************************************************************
// *****************************************************************************
// Section: Transmit Event FIFO

// *****************************************************************************
//! Transmit Event FIFO Status Get

int8_t DRV_CANFDSPI_TefStatusGet(CANFDSPI_MODULE_ID index,
        CAN_TEF_FIFO_STATUS* status);

// *****************************************************************************
//! Get Transmit Event FIFO Message

int8_t DRV_CANFDSPI_TefMessageGet(CANFDSPI_MODULE_ID index,
        CAN_TEF_MSGOBJ* tefObj);

// *****************************************************************************
//! Transmit Event FIFO Reset

int8_t DRV_CANFDSPI_TefReset(CANFDSPI_MODULE_ID index);

// *****************************************************************************
//! Transmit Event FIFO Update

int8_t DRV_CANFDSPI_TefUpdate(CANFDSPI_MODULE_ID index);

// *****************************************************************************
//! Configure Transmit Event FIFO

Ecode_t DRV_CANFDSPI_TefConfigure(CANFDSPI_MODULE_ID index, CAN_TEF_CONFIG* config);

// *****************************************************************************
//! Reset TefConfigure object to reset value

int8_t DRV_CANFDSPI_TefConfigureObjectReset(CAN_TEF_CONFIG* config);


// *****************************************************************************
// *****************************************************************************
// Section: Module Events

// *****************************************************************************
//! Module Event Get


int8_t DRV_CANFDSPI_ModuleEventGet(CANFDSPI_MODULE_ID index,
        CAN_MODULE_EVENT* flags);

// *****************************************************************************
//! Module Event Enable


Ecode_t DRV_CANFDSPI_ModuleEventEnable(CANFDSPI_MODULE_ID index,
        CAN_MODULE_EVENT flags);

// *****************************************************************************
//! Module Event Disable


int8_t DRV_CANFDSPI_ModuleEventDisable(CANFDSPI_MODULE_ID index,
        CAN_MODULE_EVENT flags);

// *****************************************************************************
//! Module Event Clear

Ecode_t DRV_CANFDSPI_ModuleEventClear(CANFDSPI_MODULE_ID index,
        CAN_MODULE_EVENT flags);

// *****************************************************************************
//! Get RX Code

int8_t DRV_CANFDSPI_ModuleEventRxCodeGet(CANFDSPI_MODULE_ID index,
        CAN_RXCODE* rxCode);

// *****************************************************************************
//! Get TX Code

int8_t DRV_CANFDSPI_ModuleEventTxCodeGet(CANFDSPI_MODULE_ID index,
        CAN_TXCODE* txCode);

// *****************************************************************************
//! Get Filter Hit

int8_t DRV_CANFDSPI_ModuleEventFilterHitGet(CANFDSPI_MODULE_ID index,
        CAN_FILTER* filterHit);

// *****************************************************************************
//! Get ICODE

int8_t DRV_CANFDSPI_ModuleEventIcodeGet(CANFDSPI_MODULE_ID index,
        CAN_ICODE* icode);

// *****************************************************************************
// *****************************************************************************
// Section: Transmit FIFO Events

// *****************************************************************************
//! Transmit FIFO Event Get

Ecode_t DRV_CANFDSPI_TransmitChannelEventGet(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel, CAN_TX_FIFO_EVENT* flags);

// *****************************************************************************
//! Transmit Queue Event Get

static inline int8_t DRV_CANFDSPI_TransmitQueueEventGet(CANFDSPI_MODULE_ID index,
        CAN_TX_FIFO_EVENT* flags)
{
    return DRV_CANFDSPI_TransmitChannelEventGet(index, CAN_TXQUEUE_CH0, flags);
}

// *****************************************************************************
//! Get pending interrupts of all transmit FIFOs

int8_t DRV_CANFDSPI_TransmitEventGet(CANFDSPI_MODULE_ID index, uint32_t* txif);

// *****************************************************************************
//! Get pending TXATIF of all transmit FIFOs

int8_t DRV_CANFDSPI_TransmitEventAttemptGet(CANFDSPI_MODULE_ID index,
        uint32_t* txatif);

// *****************************************************************************
//! Transmit FIFO Index Get

int8_t DRV_CANFDSPI_TransmitChannelIndexGet(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel, uint8_t* idx);

// *****************************************************************************
//! Transmit FIFO Event Enable


Ecode_t DRV_CANFDSPI_TransmitChannelEventEnable(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel, CAN_TX_FIFO_EVENT flags);

// *****************************************************************************
//! Transmit FIFO Event Disable

int8_t DRV_CANFDSPI_TransmitChannelEventDisable(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel, CAN_TX_FIFO_EVENT flags);

// *****************************************************************************
//! Transmit FIFO Event Clear

int8_t DRV_CANFDSPI_TransmitChannelEventAttemptClear(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel);


// *****************************************************************************
//! Transmit Queue Index Get

static inline int8_t DRV_CANFDSPI_TransmitQueueIndexGet(CANFDSPI_MODULE_ID index, uint8_t* idx)
{
    return DRV_CANFDSPI_TransmitChannelIndexGet(index, CAN_TXQUEUE_CH0, idx);
}

// *****************************************************************************
//! Transmit Queue Event Enable


static inline int8_t DRV_CANFDSPI_TransmitQueueEventEnable(CANFDSPI_MODULE_ID index,
        CAN_TX_FIFO_EVENT flags)
{
    return DRV_CANFDSPI_TransmitChannelEventEnable(index, CAN_TXQUEUE_CH0, flags);
}

// *****************************************************************************
//! Transmit Queue Event Disable


static inline int8_t DRV_CANFDSPI_TransmitQueueEventDisable(CANFDSPI_MODULE_ID index,
        CAN_TX_FIFO_EVENT flags)
{
    return DRV_CANFDSPI_TransmitChannelEventDisable(index, CAN_TXQUEUE_CH0, flags);
}

// *****************************************************************************
//! Transmit Queue Event Clear


static inline int8_t DRV_CANFDSPI_TransmitQueueEventAttemptClear(CANFDSPI_MODULE_ID index)
{
    return DRV_CANFDSPI_TransmitChannelEventAttemptClear(index, CAN_TXQUEUE_CH0);
}


// *****************************************************************************
// *****************************************************************************
// Section: Receive FIFO Events

// *****************************************************************************
//! Receive FIFO Event Get

Ecode_t DRV_CANFDSPI_ReceiveChannelEventGet(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel, CAN_RX_FIFO_EVENT* flags);

// *****************************************************************************
//! Get pending interrupts of all receive FIFOs

int8_t DRV_CANFDSPI_ReceiveEventGet(CANFDSPI_MODULE_ID index, uint32_t* rxif);

// *****************************************************************************
//!Get pending RXOVIF of all receive FIFOs

int8_t DRV_CANFDSPI_ReceiveEventOverflowGet(CANFDSPI_MODULE_ID index, uint32_t* rxovif);

// *****************************************************************************
//! Receive FIFO Index Get


int8_t DRV_CANFDSPI_ReceiveChannelIndexGet(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel, uint8_t* idx);

// *****************************************************************************
//! Receive FIFO Event Enable


Ecode_t DRV_CANFDSPI_ReceiveChannelEventEnable(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel, CAN_RX_FIFO_EVENT flags);

// *****************************************************************************
//! Receive FIFO Event Disable


int8_t DRV_CANFDSPI_ReceiveChannelEventDisable(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel, CAN_RX_FIFO_EVENT flags);

// *****************************************************************************
//! Receive FIFO Event Clear


int8_t DRV_CANFDSPI_ReceiveChannelEventOverflowClear(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel);


// *****************************************************************************
// *****************************************************************************
// Section: Transmit Event FIFO Events

// *****************************************************************************
//! Transmit Event FIFO Event Get


int8_t DRV_CANFDSPI_TefEventGet(CANFDSPI_MODULE_ID index,
        CAN_TEF_FIFO_EVENT* flags);

// *****************************************************************************
//! Transmit Event FIFO Event Enable


int8_t DRV_CANFDSPI_TefEventEnable(CANFDSPI_MODULE_ID index,
        CAN_TEF_FIFO_EVENT flags);

// *****************************************************************************
//! Transmit Event FIFO Event Disable


int8_t DRV_CANFDSPI_TefEventDisable(CANFDSPI_MODULE_ID index,
        CAN_TEF_FIFO_EVENT flags);

// *****************************************************************************
//! Transmit Event FIFO Event Clear

int8_t DRV_CANFDSPI_TefEventOverflowClear(CANFDSPI_MODULE_ID index);


// *****************************************************************************
// *****************************************************************************
// Section: Error Handling

// *****************************************************************************
//! Transmit Error Count Get

int8_t DRV_CANFDSPI_ErrorCountTransmitGet(CANFDSPI_MODULE_ID index,
        uint8_t* tec);

// *****************************************************************************
//! Receive Error Count Get

int8_t DRV_CANFDSPI_ErrorCountReceiveGet(CANFDSPI_MODULE_ID index,
        uint8_t* rec);

// *****************************************************************************
//! Error State Get

int8_t DRV_CANFDSPI_ErrorStateGet(CANFDSPI_MODULE_ID index,
        CAN_ERROR_STATE* flags);

// *****************************************************************************
//! Error Counts and Error State Get


int8_t DRV_CANFDSPI_ErrorCountStateGet(CANFDSPI_MODULE_ID index,
        uint8_t* tec, uint8_t* rec, CAN_ERROR_STATE* flags);

// *****************************************************************************
//! Get Bus Diagnostic Registers: all data at once, since we want to keep them in synch

int8_t DRV_CANFDSPI_BusDiagnosticsGet(CANFDSPI_MODULE_ID index,
        CAN_BUS_DIAGNOSTIC* bd);

// *****************************************************************************
//! Clear Bus Diagnostic Registers

int8_t DRV_CANFDSPI_BusDiagnosticsClear(CANFDSPI_MODULE_ID index);


// *****************************************************************************
// *****************************************************************************
// Section: ECC

// *****************************************************************************
//! Enable ECC

int8_t DRV_CANFDSPI_EccEnable(CANFDSPI_MODULE_ID index);

// *****************************************************************************
//! Disable ECC

int8_t DRV_CANFDSPI_EccDisable(CANFDSPI_MODULE_ID index);

// *****************************************************************************
//! ECC Event Get

int8_t DRV_CANFDSPI_EccEventGet(CANFDSPI_MODULE_ID index,
        CAN_ECC_EVENT* flags);

// *****************************************************************************
//! Set ECC Parity

int8_t DRV_CANFDSPI_EccParitySet(CANFDSPI_MODULE_ID index,
        uint8_t parity);

// *****************************************************************************
//! Get ECC Parity

int8_t DRV_CANFDSPI_EccParityGet(CANFDSPI_MODULE_ID index,
        uint8_t* parity);

// *****************************************************************************
//! Get ECC Error Address

int8_t DRV_CANFDSPI_EccErrorAddressGet(CANFDSPI_MODULE_ID index,
        uint16_t* a);

// *****************************************************************************
//! ECC Event Enable

int8_t DRV_CANFDSPI_EccEventEnable(CANFDSPI_MODULE_ID index,
        CAN_ECC_EVENT flags);

// *****************************************************************************
//! ECC Event Disable

int8_t DRV_CANFDSPI_EccEventDisable(CANFDSPI_MODULE_ID index,
        CAN_ECC_EVENT flags);

// *****************************************************************************
//! ECC Event Clear

int8_t DRV_CANFDSPI_EccEventClear(CANFDSPI_MODULE_ID index,
        CAN_ECC_EVENT flags);

// *****************************************************************************
//! Initialize RAM

Ecode_t DRV_CANFDSPI_RamInit(CANFDSPI_MODULE_ID index, uint8_t d);


// *****************************************************************************
// *****************************************************************************
// Section: CRC

// *****************************************************************************
//! CRC Event Enable

int8_t DRV_CANFDSPI_CrcEventEnable(CANFDSPI_MODULE_ID index,
        CAN_CRC_EVENT flags);

// *****************************************************************************
//! CRC Event Disable

int8_t DRV_CANFDSPI_CrcEventDisable(CANFDSPI_MODULE_ID index,
        CAN_CRC_EVENT flags);

// *****************************************************************************
//! CRC Event Clear

int8_t DRV_CANFDSPI_CrcEventClear(CANFDSPI_MODULE_ID index,
        CAN_CRC_EVENT flags);

// *****************************************************************************
//! CRC Event Get

int8_t DRV_CANFDSPI_CrcEventGet(CANFDSPI_MODULE_ID index, CAN_CRC_EVENT* flags);

// *****************************************************************************
//! Get CRC Value from device

int8_t DRV_CANFDSPI_CrcValueGet(CANFDSPI_MODULE_ID index, uint16_t* crc);


// *****************************************************************************
// *****************************************************************************
// Section: Time Stamp

// *****************************************************************************
//! Time Stamp Enable

int8_t DRV_CANFDSPI_TimeStampEnable(CANFDSPI_MODULE_ID index);

// *****************************************************************************
//! Time Stamp Disable

int8_t DRV_CANFDSPI_TimeStampDisable(CANFDSPI_MODULE_ID index);

// *****************************************************************************
//! Time Stamp Get

int8_t DRV_CANFDSPI_TimeStampGet(CANFDSPI_MODULE_ID index, uint32_t* ts);

// *****************************************************************************
//! Time Stamp Set

int8_t DRV_CANFDSPI_TimeStampSet(CANFDSPI_MODULE_ID index, uint32_t ts);

// *****************************************************************************
//! Time Stamp Mode Configure

int8_t DRV_CANFDSPI_TimeStampModeConfigure(CANFDSPI_MODULE_ID index,
        CAN_TS_MODE mode);

// *****************************************************************************
//! Time Stamp Prescaler Set

int8_t DRV_CANFDSPI_TimeStampPrescalerSet(CANFDSPI_MODULE_ID index,
        uint16_t ps);


// *****************************************************************************
// *****************************************************************************
// Section: Oscillator and Bit Time

// *****************************************************************************
//! Enable oscillator to wake-up from sleep

int8_t DRV_CANFDSPI_OscillatorEnable(CANFDSPI_MODULE_ID index);

// *****************************************************************************
//! Set Oscillator Control

Ecode_t DRV_CANFDSPI_OscillatorControlSet(CANFDSPI_MODULE_ID index,
        CAN_OSC_CTRL ctrl);

int8_t DRV_CANFDSPI_OscillatorControlObjectReset(CAN_OSC_CTRL* ctrl);


// *****************************************************************************
//! Get Oscillator Status

int8_t DRV_CANFDSPI_OscillatorStatusGet(CANFDSPI_MODULE_ID index,
        CAN_OSC_STATUS* status);

// *****************************************************************************
//! Configure Bit Time registers (based on CAN clock speed)

Ecode_t DRV_CANFDSPI_BitTimeConfigure(CANFDSPI_MODULE_ID index,
        CAN_BITTIME_SETUP bitTime, CAN_SSP_MODE sspMode,
        CAN_SYSCLK_SPEED clk);

// *****************************************************************************
//! Configure Nominal bit time for 40MHz system clock

Ecode_t DRV_CANFDSPI_BitTimeConfigureNominal40MHz(CANFDSPI_MODULE_ID index,
        CAN_BITTIME_SETUP bitTime);

// *****************************************************************************
//! Configure Data bit time for 40MHz system clock

Ecode_t DRV_CANFDSPI_BitTimeConfigureData40MHz(CANFDSPI_MODULE_ID index,
        CAN_BITTIME_SETUP bitTime, CAN_SSP_MODE sspMode);

// *****************************************************************************
//! Configure Nominal bit time for 20MHz system clock

Ecode_t DRV_CANFDSPI_BitTimeConfigureNominal20MHz(CANFDSPI_MODULE_ID index,
        CAN_BITTIME_SETUP bitTime);

// *****************************************************************************
//! Configure Data bit time for 20MHz system clock

Ecode_t DRV_CANFDSPI_BitTimeConfigureData20MHz(CANFDSPI_MODULE_ID index,
        CAN_BITTIME_SETUP bitTime, CAN_SSP_MODE sspMode);

// *****************************************************************************
//! Configure Nominal bit time for 10MHz system clock

Ecode_t DRV_CANFDSPI_BitTimeConfigureNominal10MHz(CANFDSPI_MODULE_ID index,
        CAN_BITTIME_SETUP bitTime);

// *****************************************************************************
//! Configure Data bit time for 10MHz system clock

Ecode_t DRV_CANFDSPI_BitTimeConfigureData10MHz(CANFDSPI_MODULE_ID index,
        CAN_BITTIME_SETUP bitTime, CAN_SSP_MODE sspMode);


// *****************************************************************************
// *****************************************************************************
// Section: GPIO

// *****************************************************************************
//! Initialize GPIO Mode

Ecode_t DRV_CANFDSPI_GpioModeConfigure(CANFDSPI_MODULE_ID index,
        GPIO_PIN_MODE gpio0, GPIO_PIN_MODE gpio1);

// *****************************************************************************
//! Initialize GPIO Direction

int8_t DRV_CANFDSPI_GpioDirectionConfigure(CANFDSPI_MODULE_ID index,
        GPIO_PIN_DIRECTION gpio0, GPIO_PIN_DIRECTION gpio1);

// *****************************************************************************
//! Enable Transceiver Standby Control

int8_t DRV_CANFDSPI_GpioStandbyControlEnable(CANFDSPI_MODULE_ID index);

// *****************************************************************************
//! Disable Transceiver Standby Control

int8_t DRV_CANFDSPI_GpioStandbyControlDisable(CANFDSPI_MODULE_ID index);

// *****************************************************************************
//! Configure Open Drain Interrupts

int8_t DRV_CANFDSPI_GpioInterruptPinsOpenDrainConfigure(CANFDSPI_MODULE_ID index,
        GPIO_OPEN_DRAIN_MODE mode);

// *****************************************************************************
//! Configure Open Drain TXCAN

int8_t DRV_CANFDSPI_GpioTransmitPinOpenDrainConfigure(CANFDSPI_MODULE_ID index,
        GPIO_OPEN_DRAIN_MODE mode);

// *****************************************************************************
//! GPIO Output Pin Set

int8_t DRV_CANFDSPI_GpioPinSet(CANFDSPI_MODULE_ID index,
        GPIO_PIN_POS pos, GPIO_PIN_STATE latch);

// *****************************************************************************
//! GPIO Input Pin Read

int8_t DRV_CANFDSPI_GpioPinRead(CANFDSPI_MODULE_ID index,
        GPIO_PIN_POS pos, GPIO_PIN_STATE* state);

// *****************************************************************************
//! Configure CLKO Pin

int8_t DRV_CANFDSPI_GpioClockOutputConfigure(CANFDSPI_MODULE_ID index,
        GPIO_CLKO_MODE mode);


// *****************************************************************************
// *****************************************************************************
// Section: Miscellaneous

// *****************************************************************************
//! DLC to number of actual data bytes conversion

uint32_t DRV_CANFDSPI_DlcToDataBytes(CAN_DLC dlc);

// *****************************************************************************
//! FIFO Index Get

int8_t DRV_CANFDSPI_FifoIndexGet(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel, uint8_t* mi);

// *****************************************************************************
//! Calculate CRC16

uint16_t DRV_CANFDSPI_CalculateCRC16(uint8_t* data, uint16_t size);

// *****************************************************************************
//! Data bytes to DLC conversion

CAN_DLC DRV_CANFDSPI_DataBytesToDlc(uint8_t n);


void DRV_SPI_Transfer(SPI_Handle handle,
              const void *txBuffer,
              void * rxBuffer,
              int count);


#endif // _DRV_CANFDSPI_API_H
