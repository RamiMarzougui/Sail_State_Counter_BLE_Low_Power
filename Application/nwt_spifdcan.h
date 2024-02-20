/*
 * nwt_spifdcan.h
 *
 *  Created on: 7 sept. 2023
 *      Author: rami
 */

#ifndef APPLICATION_NWT_SPIFDCAN_H_
#define APPLICATION_NWT_SPIFDCAN_H_

/*================== Includes =============================================*/
#include <ti/drivers/SPI.h>
#include "drv_spi.h"
#include "drv_canfdspi_api.h"
#include "drv_canfdspi_register.h"

/*================== Macros and Definitions ===============================*/
#define CONFIG_SPI_CONTROLLER           0

#define INTB_CAN_PIN 20

//Filter Mask ID
#define FILTER_MASK_ID 0x110

//CAN MSG ID
#define MSG_ID_SET_THRESHOLD 0x110
#define MSG_CHANGE_ULP_MODE_STATE 0x111
#define MSG_SET_REAL_STATE 0x112
#define MSG_SET_THR_DELTA 0x113
#define MSG_SET_MLC_CONFIG_TYPE 0x114
#define MSG_SET_THR_WAKEUP 0x115

// Switch states
typedef struct {
    bool S1;
    bool S2;
    bool S3;
    bool S4;
} APP_SwitchState;

// Payload
typedef struct {
    bool On;
    uint8_t Dlc;
    bool Mode;
    uint8_t Counter;
    uint8_t Delay;
    bool BRS;
} APP_Payload;

typedef enum {
    // Initialization
    APP_STATE_INIT = 0,
    APP_STATE_REQUEST_CONFIG,
    APP_STATE_WAIT_FOR_CONFIG,
    APP_STATE_INIT_TXOBJ,

    // POR signaling
    APP_STATE_FLASH_LEDS,

    // Transmit and Receive
    APP_STATE_TRANSMIT,
    APP_STATE_RECEIVE,
    APP_STATE_PAYLOAD,

    // Switch monitoring
    APP_STATE_SWITCH_CHANGED
} APP_STATES;


typedef struct {
    /* The application's current state */
    APP_STATES state;
    /* TODO: Define any additional data used by the application. */
} APP_DATA;

/*================== Constant and Variable Definitions ====================*/
extern uint8_t controllerRxBuffer[6];
extern uint8_t controllerTxBuffer[6];
extern SPI_Handle controllerSpi;

/*================== Function Prototypes ==================================*/
/**
 * @brief   FDCAN Task main function
 *
 * @details
 *
 */
void FDCAN_taskFxn(void);

/**
 * @brief   Create the FDCAN Task
 *
 * @details
 *
 */
void FDCAN_createTask(void);

/**
 * @brief   Initialize the FDCAN Controller and set its configuration
 *
 * @details
 *
 *
 * @param   can_bitrate    The desired Bit Rate for the FDCAN controller (CAN_500K_1M, CAN_500K_2M, .....)
 *
 */
void FDCAN_Init(CAN_BITTIME_SETUP can_bitrate);

/**
 * @brief   Transmit a message object over the CAN BUS if the TX FIFO is not full
 *
 * @details
 *
 *
 * @param   txObj       Transfer message object
 * @param   txd         Buffer that contains the frame data to transmit
 *
 */
void FDCAN_TransmitFRAME(CAN_TX_MSGOBJ* txObj, uint8_t *txd);

/**
 * @brief   Get all CAN messages in RX FIFO and store them in an array of messages
 *
 * @details
 *
 *
 * @param   rxd       CAN Messages Holder
 *
 */
void FDCAN_ReceiveFRAMES(uint8_t * rxd);

/**
 * @brief   Callback function for the GPIO interrupt on INTB_CAN PIN
 *
 * @details
 *
 */
void DataRxIntFxn(void);

/**
 * @brief   Fill Tx Buffer with Data for CAN message
 *
 * @details
 *
 *
 * @param   txd       Tx Data Buffer
 *
 */
void FillFrameDATA(uint8_t* txd);

/**
 * @brief   Correct Current Sign
 *
 * @details
 *
 */
void FlipCurrentSign(void);

/**
 * @brief   Process Received FDCAN Messages
 *
 * @details
 *
 */
void FDCAN_ProcessRxFRAMES(void);

#endif /* APPLICATION_NWT_SPIFDCAN_H_ */
