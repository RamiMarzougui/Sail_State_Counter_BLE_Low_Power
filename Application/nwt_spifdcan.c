/*
 * nwt_spifdcan.c
 *
 *  Created on: 7 sept. 2023
 *      Author: rami
 */

/*================== Includes =============================================*/
#include <nwt_spifdcan.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include "EEPROM.h"
#include "shared.h"
#include "scif.h"
#include "unico.h"
#include "nwt_i2c.h"
/* Driver configuration */
#include "ti_drivers_config.h"
#include "drv_spi.h"
#include "drv_canfdspi_api.h"
#include "drv_canfdspi_register.h"

/*================== Macros and Definitions ===============================*/
//! Use RX and TX Interrupt pins to check FIFO status
#define APP_USE_RX_INT
//#define APP_USE_TX_INT
//#define APP_USE_RX_TX_INT

// Transmit Channels
#define APP_TX_FIFO CAN_FIFO_CH2

// Receive Channels
#define APP_RX_FIFO CAN_FIFO_CH1

#define APP_BUFFER_SIZE             10

#define SW_VERSION_ADD 0x00

#define MAX_TXQUEUE_ATTEMPTS 50

#define FDCAN_Task_Period 4100 //33.33 ms
#define FDCAN_TASK_STACK_SIZE                   1024
#define FDCAN_TASK_PRIORITY                     1

/*================== Constant and Variable Definitions ====================*/
// Receive objects
CAN_RX_MSGOBJ rxObj;
uint8_t rxMsgs[16][MAX_DATA_BYTES];
//Counter for Set Threshold messages
uint8_t rxSetTHRMsgCnt = 0;
//Counter for Change ULP MODE state messages
uint8_t rxChangeULPMsgCnt = 0;
//Counter for Real State Set messages
uint8_t rxSetRealStateMsgCnt = 0;
//Counter for Set Delta messages
uint8_t rxSetTHRdeltaMsgCnt = 0;
//Counter for Change MLC Config Type messages
uint8_t rxChangeMLCconfigTypeCnt = 0;
//Counter for Change WAKEUP Threshold messages
uint8_t rxSetTHRwakeupMsgCnt = 0;
CAN_RX_FIFO_EVENT rxFlags;
bool RxMsgFlag = false;

// Transmission and reception buffers
static char rx_buffer[APP_BUFFER_SIZE];
static char tx_buffer[APP_BUFFER_SIZE];

uint8_t fdcanTaskStack[FDCAN_TASK_STACK_SIZE];
Task_Struct fdcanTask;

uint32_t cpt_F1;
uint32_t cpt_ON;

uint16_t new_threshold;

BQ27427_Data Battery_Cell_Data;


void FDCAN_createTask(void)
{
    Task_Params FDCANtaskParams;

    /* Configure task */
    Task_Params_init(&FDCANtaskParams);
    FDCANtaskParams.stack = fdcanTaskStack;
    FDCANtaskParams.stackSize = FDCAN_TASK_STACK_SIZE;
    FDCANtaskParams.priority = FDCAN_TASK_PRIORITY;

    Task_construct(&fdcanTask, FDCAN_taskFxn, &FDCANtaskParams, NULL);
}

void FDCAN_taskFxn(void)
{
#ifdef FINAL_VERSION
    //Block task in here
    SemaphoreP_pend(sem_fdcan_thread, SemaphoreP_WAIT_FOREVER); //Block Task here until Self-Check Test is over
#endif

    uint32_t i;
    bool transferOK;
    int32_t status;
    /* Soft Version and Serial Number */
    uint8_t sw_version[3];
    uint8_t serial_number[2];


    /****** Read SW version & serial number stored in EEPROM *****/
    EEPROM_Read(SW_VERSION_ADD, sw_version, 3);
    EEPROM_Read(SN_ADD, serial_number, 2);
    /*************************************************************/

    /******************** Init FDCAN ******************************/
    FDCAN_Init(CAN_500K_1M); //500Kbit/s - 1Mbit/s
    /**************************************************************/

    /******************** Define CAN Frame ************************/
    CAN_TX_MSGOBJ txObj; //TX Message Object
    txObj.bF.id.SID = 0x044;  //Standard or Base ID
    txObj.bF.id.EID = 0;
    txObj.bF.ctrl.FDF = 1; // CAN FD frame
    txObj.bF.ctrl.BRS = 1; // Switch bit rate
    txObj.bF.ctrl.IDE = 0; // Standard frame
    txObj.bF.ctrl.RTR = 0; // Not a remote frame request
    txObj.bF.ctrl.DLC = CAN_DLC_64; // 64 data bytes
    //Sequence doesn't get transmitted but will be stored in TEF
    txObj.bF.ctrl.SEQ = 1;
    /***************************************************************/

    /******** CAN Frame Data (SW Version + Fuel Gauge Data) ********/
    uint8_t txd[MAX_DATA_BYTES] = {0};
    uint8_t nmb = 0x00;
    //Initialize transmit data
    for(uint8_t i=0; i < 3; i++) txd[i] = sw_version[i];
    for(uint8_t i=0; i < 2; i++) txd[37+i] = serial_number[i];
    /**************************************************************/

    while(1)
    {

    /******************** Collect Battery Cell Data ***********************/
        Battery_Cell_Data.CellVoltage = BQ27427_voltage();
        Battery_Cell_Data.CellCurrent = BQ27427_current(AVG); //Average current
        Battery_Cell_Data.CellPower = BQ27427_power();
        Battery_Cell_Data.CellSoC = BQ27427_soc(FILTERED);
        Battery_Cell_Data.CellSoH = BQ27427_soh(PERCENT); //Percentage
        Battery_Cell_Data.CellTemperature = BQ27427_temperature(BATTERY); //CellTemperature
        Battery_Cell_Data.ICTemperature = BQ27427_temperature(INTERNAL_TEMP); //Internal IC temperature
        Battery_Cell_Data.CellCapacity = BQ27427_capacity(FULL); //Full capacity
        //Flags
        Battery_Cell_Data.FastCHG = BQ27427_chgFlag();
        Battery_Cell_Data.FullCHG = BQ27427_fcFlag();
        Battery_Cell_Data.DSCH = BQ27427_dsgFlag();
    /**********************************************************************/

    /*************************** Read SC counter ***************************/
        cpt_F1 =  ((uint32_t) scifTaskData.count.output.F1Msb << 16) | (uint32_t) scifTaskData.count.output.F1Lsb;
        cpt_ON =  ((uint32_t) scifTaskData.count.output.OnMsb << 16) | (uint32_t) scifTaskData.count.output.OnLsb;
    /***********************************************************************/

    /**************** Fill CAN Frame with collected Data ******************/
         FillFrameDATA(txd);
    /**********************************************************************/

    /********************** Send CAN Frame *******************************/
         FDCAN_TransmitFRAME(&txObj, txd);
    /*********************************************************************/

    /************************ Receive CAN Frames ****************************/
         FDCAN_ReceiveFRAMES(&rxMsgs[0][0]);
         FDCAN_ProcessRxFRAMES();
    /************************************************************************/

         Task_sleep(FDCAN_Task_Period);

    }

    GPIO_disableInt(INTB_CAN_PIN);

}

void FDCAN_Init(CAN_BITTIME_SETUP can_bitrate)
{
    REG_CiMASK mObj;
    CAN_CONFIG config;
    REG_CiFLTOBJ fObj;
    bool ramInitialized;

    DRV_CANFDSPI_Reset(DRV_CANFDSPI_INDEX_0, controllerSpi);

    // OScillator Configuration : divide by 10
    CAN_OSC_CTRL oscCtrl;
    DRV_CANFDSPI_OscillatorControlObjectReset(&oscCtrl);
    oscCtrl.ClkOutDivide = OSC_CLKO_DIV1;
    DRV_CANFDSPI_OscillatorControlSet(DRV_CANFDSPI_INDEX_0, oscCtrl);

    // Enable ECC and initialize RAM
    DRV_CANFDSPI_EccEnable(DRV_CANFDSPI_INDEX_0);

    if (!ramInitialized) {
      DRV_CANFDSPI_RamInit(DRV_CANFDSPI_INDEX_0, 0xff);
      ramInitialized = true;
    }

    DRV_CANFDSPI_ConfigureObjectReset(&config);
    config.IsoCrcEnable = 1;
    config.StoreInTEF = 0;
    //config.TXQEnable = 1;
    DRV_CANFDSPI_Configure(DRV_CANFDSPI_INDEX_0, &config);


    // FIFO 1 : Transmit FIFO; 5 messages, 64 byte maximum payload, low priority
    CAN_TX_FIFO_CONFIG txfConfig;
    DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txfConfig);
    txfConfig.FifoSize = 7;
    txfConfig.PayLoadSize = CAN_PLSIZE_64;
    txfConfig.TxPriority = 1;
    DRV_CANFDSPI_TransmitChannelConfigure(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH2, &txfConfig);

    // FIFO 2 : Receive FIFO; 16 messages, 64 byte maximum payload, time stamping enabled
    CAN_RX_FIFO_CONFIG rxfConfig;
    rxfConfig.FifoSize = 20;
    rxfConfig.PayLoadSize = CAN_PLSIZE_8;
    //rxfConfig.RxTimeStampEnable = 1;
    DRV_CANFDSPI_ReceiveChannelConfigure(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH1, &rxfConfig);

    // Setup RX Filter
    fObj.word = 0;
    fObj.bF.SID = FILTER_MASK_ID;
    fObj.bF.EXIDE = 0;
    fObj.bF.SID11 = 1; //Standard ID only
    fObj.bF.EID = 0x00;

    DRV_CANFDSPI_FilterObjectConfigure(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, &fObj.bF);

    // Setup RX Mask
    mObj.word = 0;
    mObj.bF.MSID = FILTER_MASK_ID;
    mObj.bF.MIDE = 0;
    mObj.bF.MEID = 0x00;
    DRV_CANFDSPI_FilterMaskConfigure(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, &mObj.bF);

    // Link FIFO and Filter
    DRV_CANFDSPI_FilterToFifoLink(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, CAN_FIFO_CH1, true);

    /************ Configuration of interrupts **************************/
    DRV_CANFDSPI_ModuleEventClear(DRV_CANFDSPI_INDEX_0, CAN_ALL_EVENTS);
    //Configure receive interrupt for TXQ and FIFO 2
    DRV_CANFDSPI_ReceiveChannelEventEnable(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH1, CAN_RX_FIFO_NOT_EMPTY_EVENT);
    DRV_CANFDSPI_ModuleEventEnable(DRV_CANFDSPI_INDEX_0, CAN_RX_EVENT);
    /******************************************************************/

    DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0, can_bitrate, CAN_SSP_MODE_AUTO, CAN_SYSCLK_20M); //System Clock 20MHz in this case
    DRV_CANFDSPI_GpioModeConfigure(DRV_CANFDSPI_INDEX_0, GPIO_MODE_INT, GPIO_MODE_INT);
    DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_0, CAN_NORMAL_MODE);

    /******************** Configure INTB_CAN **********************/
    GPIO_setConfig(INTB_CAN_PIN, GPIO_CFG_INPUT);
    //Configure interrupt on INTB_CAN
    GPIO_setConfig(INTB_CAN_PIN, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    //Set Callback
    GPIO_setCallback(INTB_CAN_PIN, DataRxIntFxn);
    //Enable Interrupt
    GPIO_enableInt(INTB_CAN_PIN);
}


void FDCAN_TransmitFRAME( CAN_TX_MSGOBJ* txObj, uint8_t *txd)
{
    bool flush = true;
    CAN_TX_FIFO_EVENT txFlags;
    uint8_t nb_attempts = 0;

    // Check that FIFO is not full
    DRV_CANFDSPI_TransmitChannelEventGet(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH2, &txFlags);

    while(!(txFlags & CAN_TX_FIFO_NOT_FULL_EVENT) && (nb_attempts<MAX_TXQUEUE_ATTEMPTS)) nb_attempts++;
    if (txFlags & CAN_TX_FIFO_NOT_FULL_EVENT) {
    //Load message and transmit
        DRV_CANFDSPI_TransmitChannelLoad(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH2, txObj, txd, DRV_CANFDSPI_DlcToDataBytes(txObj->bF.ctrl.DLC), flush);
    }
}


void FDCAN_ReceiveFRAMES(uint8_t *rxd)
{
    /********************* Check RX FIFO ********************************/
    if(RxMsgFlag)
    {
        do
        {
            DRV_CANFDSPI_ReceiveMessageGet(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH1, &rxObj, rxd, MAX_DATA_BYTES);
            DRV_CANFDSPI_ReceiveChannelEventGet(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH1, &rxFlags);
            if(rxObj.bF.id.SID == MSG_ID_SET_THRESHOLD) rxSetTHRMsgCnt++; //Set Threshold message has been received
            else if(rxObj.bF.id.SID == MSG_CHANGE_ULP_MODE_STATE) rxChangeULPMsgCnt++; //Change ULP Mode message has been received
            else if(rxObj.bF.id.SID == MSG_SET_REAL_STATE) rxSetRealStateMsgCnt++; //Set Real State message has been received
            else if(rxObj.bF.id.SID == MSG_SET_THR_DELTA) rxSetTHRdeltaMsgCnt++; //Set Threshold message has been received
            else if(rxObj.bF.id.SID == MSG_SET_MLC_CONFIG_TYPE) rxChangeMLCconfigTypeCnt++; //Change MLC Config Type message has been received
            else if(rxObj.bF.id.SID == MSG_SET_THR_WAKEUP) rxSetTHRwakeupMsgCnt++; //Set Wakeup threshold message has been received

        }while(rxFlags & CAN_RX_FIFO_NOT_EMPTY_EVENT);

        RxMsgFlag = false;
    }
    /*********************************************************************/
}

void FDCAN_ProcessRxFRAMES(void)
{

    /* Handle MLC Threshold configuration through CAN */
    float newThreshold = 0.0f;
    uint16_t newThreshold_c = 0;

    //Change ULP Mode Activation State
    if(rxChangeULPMsgCnt > 0)
    {
      if(rxMsgs[0][0] == 0) FlagActivationULP = 0; //ULP Mode Activation OFF
      else if(rxMsgs[0][0] == 1) FlagActivationULP = 1; //ULP Mode Activation ON

      rxChangeULPMsgCnt = 0; //Reset Msg counter
    }

    //Set Threshold
    if(rxSetTHRMsgCnt > 0)
    {

        if(mlc_config == ONE_FEATURE)
        {
            //Modify OFF Threshold
            fsm_mlc_conf_LP_one_feature[160].data = rxMsgs[0][2]; //LSB
            fsm_mlc_conf_LP_one_feature[161].data = rxMsgs[0][3]; //MSB

            //Modify ON/F1 Threshold
            fsm_mlc_conf_LP_one_feature[164].data = rxMsgs[0][0]; //LSB
            fsm_mlc_conf_LP_one_feature[165].data = rxMsgs[0][1]; //MSB
        }
        else if(mlc_config == TWO_FEATURES)
        {
            //Modify F1 Threshold (VAR_Z)
            fsm_mlc_conf_LP_two_features[166].data = rxMsgs[0][2]; //LSB
            fsm_mlc_conf_LP_two_features[167].data = rxMsgs[0][3]; //MSB

            //Modify ON/OFF Threshold (VAR_V)
            fsm_mlc_conf_LP_two_features[170].data = rxMsgs[0][0]; //LSB
            fsm_mlc_conf_LP_two_features[171].data = rxMsgs[0][1]; //MSB
        }

      reconfig_mlc_flag = 1; //Set Re-Configuration flag

      rxSetTHRMsgCnt = 0; //Reset Msg counter
    }

    //Set Real State
    if(rxSetRealStateMsgCnt > 0)
    {

      if(rxMsgs[0][0] == 1)  //ON - Active
      {

          if(mlc_config == ONE_FEATURE)
          {
              //The new threshold is equal to the moving average of the variance on Z-axis plus a delta
              newThreshold = uint16_to_half(SensorData.MOVING_AVG_Z) + delta_thr;
              newThreshold_c = half_to_uint16(newThreshold); //Convert to uint16

              //Modify ON/F1 Threshold
              fsm_mlc_conf_LP_one_feature[164].data = newThreshold_c; //ON/F1 Threshold : LSB
              fsm_mlc_conf_LP_one_feature[165].data = newThreshold_c >> 8; //ON/F1 Threshold : MSB
          }
          else if(mlc_config == TWO_FEATURES)
          {
              //The new threshold is equal to the moving average of the variance V plus a delta
              newThreshold = uint16_to_half(SensorData.MOVING_AVG_V) + delta_thr;
              newThreshold_c = half_to_uint16(newThreshold); //Convert to uint16

              //Modify ON/OFF Threshold (VAR_V)
              fsm_mlc_conf_LP_two_features[170].data = newThreshold_c; //ON/OFF Threshold : LSB
              fsm_mlc_conf_LP_two_features[171].data = newThreshold_c >> 8; //ON/OFF Threshold : MSB
          }


          //Set Re-configure Flag
          reconfig_mlc_flag = 1; //Re-Configure MLC

      }
      else if(rxMsgs[0][0] == 2) //F1 - Flapping
      {

          if(mlc_config == ONE_FEATURE)
          {
              //The new threshold is equal to the moving average of the variance on Z-axis minus a delta
              newThreshold = uint16_to_half(SensorData.MOVING_AVG_Z) - delta_thr;
              newThreshold_c = half_to_uint16(newThreshold); //Convert to uint16

              //Modify ON/F1 Threshold
              fsm_mlc_conf_LP_one_feature[164].data = newThreshold_c; //ON/F1 Threshold : LSB
              fsm_mlc_conf_LP_one_feature[165].data = newThreshold_c >> 8; //ON/F1 Threshold : MSB
          }
          else if(mlc_config == TWO_FEATURES)
          {
              //The new threshold is equal to the moving average of the variance on Z-axis minus a delta
              newThreshold = uint16_to_half(SensorData.MOVING_AVG_Z) - delta_thr;
              newThreshold_c = half_to_uint16(newThreshold); //Convert to uint16

              //Modify F1 Threshold (VAR_Z)
              fsm_mlc_conf_LP_two_features[166].data = newThreshold_c; //F1 Threshold : LSB
              fsm_mlc_conf_LP_two_features[167].data = newThreshold_c >> 8; //F1 Threshold : MSB
          }

          reconfig_mlc_flag = 1; //Re-Configure MLC
      }
      else if(rxMsgs[0][0] == 0) //OFF
      {
          if(mlc_config == ONE_FEATURE)
          {
              //The new threshold is equal to the moving average of the variance on Z-axis plus a delta
              newThreshold = uint16_to_half(SensorData.MOVING_AVG_Z) + delta_thr_off;
              newThreshold_c = half_to_uint16(newThreshold); //Convert to uint16

              //Modify OFF Threshold
              fsm_mlc_conf_LP_one_feature[160].data = newThreshold_c; //OFF Threshold : LSB
              fsm_mlc_conf_LP_one_feature[161].data = newThreshold_c >> 8; //OFF Threshold : MSB
          }
          else if(mlc_config == TWO_FEATURES)
          {
              //The new threshold is equal to the moving average of the variance V minus a delta
              newThreshold = uint16_to_half(SensorData.MOVING_AVG_V) - delta_thr_off;
              newThreshold_c = half_to_uint16(newThreshold); //Convert to uint16

              //Modify ON/OFF Threshold (VAR_V)
              fsm_mlc_conf_LP_two_features[170].data = newThreshold_c; //ON/OFF Threshold : LSB
              fsm_mlc_conf_LP_two_features[171].data = newThreshold_c >> 8; //ON/OFF Threshold : MSB
          }

          reconfig_mlc_flag = 1; //Re-Configure MLC
      }

      rxSetRealStateMsgCnt = 0; //Reset Msg Counter

    }

    //Set Threshold Delta
    if(rxSetTHRdeltaMsgCnt > 0)
    {
      //Set new threshold delta
      delta_thr = uint16_to_half(((uint16_t) rxMsgs[0][1] << 8) | (uint16_t) rxMsgs[0][0]);

      rxSetTHRdeltaMsgCnt = 0; //Reset Msg Counter
    }

    //Change MLC Config Type (1 feature / 2 features)
    if(rxChangeMLCconfigTypeCnt > 0)
    {
        if(rxMsgs[0][0] == 1) //1 feature
        {
            mlc_config = ONE_FEATURE;
            reconfig_mlc_flag = 1; //Re-Configure MLC
        }
        else if(rxMsgs[0][0] == 2) //2 features
        {
            mlc_config = TWO_FEATURES;
            reconfig_mlc_flag = 1; //Re-Configure MLC
        }

        rxChangeMLCconfigTypeCnt = 0; //Reset Msg counter
    }

    if(rxSetTHRwakeupMsgCnt > 0)
    {
        if(rxMsgs[0][0] <= 63) //Value received is within range of wake-up threshold
        {
            fsm_mlc_conf_ULP[29].data = rxMsgs[0][0]; //0x5B register
        }

        rxSetTHRwakeupMsgCnt = 0; //Reset Msg counter
    }

}


void FillFrameDATA(uint8_t* txd)
{
    /*
     * Byte 0 : SW version Major
     * Byte 1 : SW version Minor
     * Byte 2 : SW version Bug fixes
     * Byte 3 : Cell Voltage MSB (mV)
     * Byte 4 : Cell Voltage LSB
     * Byte 5 : Cell Current MSB (mA)
     * Byte 6 : Cell Current LSB
     * Byte 7 : Cell Power MSB
     * Byte 8 : Cell Power LSB
     * Byte 9 : Cell SoC MSB
     * Byte 10 : Cell SoC LSB
     * Byte 11 : Cell SoH (%)
     * Byte 12 : Cell Temperature MSB (°C)
     * Byte 13 : Cell Temperature LSB
     * Byte 14 : Cell Capacity MSB
     * Byte 15 : Cell Capacity LSB
     */

    txd[3] = Battery_Cell_Data.CellVoltage >> 8;
    txd[4] = Battery_Cell_Data.CellVoltage;

    txd[5] = Battery_Cell_Data.CellCurrent >> 8;
    txd[6] = Battery_Cell_Data.CellCurrent;

    txd[7] =  Battery_Cell_Data.CellPower >> 8;
    txd[8] =  Battery_Cell_Data.CellPower;

    txd[9] = Battery_Cell_Data.CellSoC >> 8;
    txd[10] = Battery_Cell_Data.CellSoC;

    txd[11] = Battery_Cell_Data.CellSoH;

    txd[12] = Battery_Cell_Data.CellTemperature >> 8;
    txd[13] = Battery_Cell_Data.CellTemperature;

    txd[14] = Battery_Cell_Data.CellCapacity >> 8;
    txd[15] = Battery_Cell_Data.CellCapacity;

    /*
     * Byte 16 : Linear Acceleration Z MSB
     * Byte 17 : Linear Acceleration Z LSB
     * Byte 18 : Linear Acceleration Y MSB
     * Byte 19 : Linear Acceleration Y LSB
     * Byte 20 : Linear Acceleration X MSB
     * Byte 21 : Linear Acceleration X LSB
     * Byte 22 : MLC SRC
     * Byte 23 : FSM STATUS
     * Byte 24 : MODE ULP Activation
     * Byte 25 : Feature 1 MSB
     * Byte 26 : Feature 1 LSB
     * Byte 27 : F1 counter
     * Byte 28 : F1 counter
     * Byte 29 : F1 counter
     * Byte 30 : F1 counter
     * Byte 31 : ON counter
     * Byte 32 : ON counter
     * Byte 33 : ON counter
     * Byte 34 : ON counter
     * Byte 35 : Sensor current Power Mode
     * Byte 36 : Calculated threshold
     * Byte 37 : Serial Number MSB
     * Byte 38 : Serial Number LSB
     * Byte 39 : OFF Threshold MSB
     * Byte 40 : OFF Threshold LSB
     * Byte 41 : ON/F1 Threshold MSB
     * Byte 42 : ON/F1 Threshold LSB
     * Byte 43 : Moving Avg Var Z MSB
     * Byte 44 : Moving Avg Var Z LSB
     * Byte 45 : I2C Error Flag
     * Byte 46 : Feature 2 MSB
     * Byte 47 : Feature 2 LSB
     * Byte 48 : Moving Avg Var V MSB
     * Byte 49 : Moving Avg Var V LSB
     * Byte 50 : MLC Config Type
     * Byte 51 : WAKE UP Threshold
     */

    //Sensor Data (X, Y, Z) linear acceleration
    txd[16] = SensorData.OUTZ_A_v >> 8;
    txd[17] = SensorData.OUTZ_A_v;

    txd[18] = SensorData.OUTY_A_v >> 8;
    txd[19] = SensorData.OUTY_A_v;

    txd[20] = SensorData.OUTX_A_v >> 8;
    txd[21] = SensorData.OUTX_A_v;

    //MLC SRC
    txd[22] = SensorData.MLC_SRC;

    //FSM STATUS
    txd[23] = SensorData.FSM_status;

    //ULP activation
    txd[24] = FlagActivationULP;

    //Feature 1 value
    txd[25] = SensorData.Feature_1 >> 8;
    txd[26] = SensorData.Feature_1;

    //Counters
    txd[27] = cpt_F1 >> 24; //F1
    txd[28] = cpt_F1 >> 16;
    txd[29] = cpt_F1 >> 8;
    txd[30] = cpt_F1;

    txd[31] = cpt_ON >> 24; //ON
    txd[32] = cpt_ON >> 16;
    txd[33] = cpt_ON >> 8;
    txd[34] = cpt_ON;

    //Current Power Mode
    txd[35] = sensorCurrentPM;

    //Calculated threshold
    txd[36] = calculated_threshold;

    //MLC OFF Threshold
    txd[39] = SensorData.OFF_Threshold >> 8;
    txd[40] = SensorData.OFF_Threshold;

    //MLC ON/F1 Threshold
    txd[41] = SensorData.ON_F1_Threshold >> 8;
    txd[42] = SensorData.ON_F1_Threshold;

    //Moving average Var Z
    txd[43] = SensorData.MOVING_AVG_Z >> 8;
    txd[44] = SensorData.MOVING_AVG_Z;

    //Error flag
    txd[45] = i2c_error_flag;

    //Feature 2 value
    txd[46] = SensorData.Feature_2 >> 8;
    txd[47] = SensorData.Feature_2;

    //Moving average Var V
    txd[48] = SensorData.MOVING_AVG_V >> 8;
    txd[49] = SensorData.MOVING_AVG_V;

    //MLC Config Type
    txd[50] = mlc_config;

    //Wake up threshold
    txd[51] = SensorData.WAKE_UP_Threshold;

    //Debug
    txd[52] = DebugCurrentPM;


}


/*
 *  ======== DataRxIntFxn ========
 *  Callback function for the GPIO interrupt on INTB_CAN_PIN
 */

void DataRxIntFxn(void)
{
    RxMsgFlag = true; //Set Flag to indicate a new message in the RX FIFO
}

void FlipCurrentSign(void)
{
    uint8_t CC_Gain;
    uint8_t CheckSum;
    /****************** Reverse Current Sign ****************************/
    //Unseal
    uint8_t txBuffer[3];
    uint8_t rxBuffer[3];
    I2C_Transaction i2cTransaction;
    /* Ping I2C transaction setup */
    i2cTransaction.writeBuf   = txBuffer;
    i2cTransaction.writeCount = 3;
    i2cTransaction.readBuf    = rxBuffer;
    i2cTransaction.readCount  = 0;
    i2cTransaction.targetAddress = BQ27427_I2C_ADDRESS;

    txBuffer[0] = 0x00;
    txBuffer[1] = 0x00;
    txBuffer[2] = 0x80;
    if (!I2C_transfer(i2c, &i2cTransaction))
    {
            //while(1);
    }

    if (!I2C_transfer(i2c, &i2cTransaction))
    {
            //while(1);
    }

    //BQ27427_enterConfig(true);


    //write 0x13 0x00 to I2C command/register 0x00
    txBuffer[0] = 0x00;
    txBuffer[1] = 0x13;
    txBuffer[2] = 0x00;
    if (!I2C_transfer(i2c, &i2cTransaction))
    {
            //while(1);
    }

    //Poll flags (0x06) and wait for CFGUPMODE bit 4 is set
    while(!(BQ27427_flags() & BQ27427_FLAG_CFGUPMODE));

    //Enable Block data memory control
    i2cTransaction.writeCount = 2;

    txBuffer[0] = 0x61;
    txBuffer[1] = 0x00;

    if (!I2C_transfer(i2c, &i2cTransaction))
    {
            //while(1);
    }

    //Write 0x69 0x00 to I2C command/register 0x3E (this selects sub-class Calibration)
    txBuffer[0] = 0x3E;
    txBuffer[1] = 0x69;
    if (!I2C_transfer(i2c, &i2cTransaction))
    {
            //while(1);
    }
    //Offset
    txBuffer[0] = 0x3F;
    txBuffer[1] = 0x00;
    if (!I2C_transfer(i2c, &i2cTransaction))
    {
            //while(1);
    }

    //Read one byte from register 0x45 (this is the byte from CC_Gain that holds the sign in bit 7)
    i2cTransaction.writeCount = 1;
    i2cTransaction.readCount  = 1;
    txBuffer[0] = 0x45;
    if (!I2C_transfer(i2c, &i2cTransaction))
    {
            //while(1);
    }
    CC_Gain = rxBuffer[0];


    //Read Gain again
    i2cTransaction.writeCount = 1;
    i2cTransaction.readCount  = 1;
    txBuffer[0] = 0x45;
    if (!I2C_transfer(i2c, &i2cTransaction))
    {
            //while(1);
    }
    CC_Gain = rxBuffer[0];


    //Read one byte from register 0x60 (this is the check-sum)
    txBuffer[0] = 0x60;
    if (!I2C_transfer(i2c, &i2cTransaction))
    {
            //while(1);
    }
    CheckSum = rxBuffer[0];

    //Flip bit 7 from the byte read in step 4 and write it to register 0x45
    CC_Gain = CC_Gain ^ 0x80;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readCount  = 0;
    txBuffer[0] = 0x45;
    txBuffer[1] = CC_Gain;
    if (!I2C_transfer(i2c, &i2cTransaction))
    {
            //while(1);
    }

    //Flip bit 7 from the byte read in step 5 and write it to register 0x60
    i2cTransaction.writeCount = 2;
    i2cTransaction.readCount  = 0;


    CheckSum = BQ27427_computeBlockChecksum();

    txBuffer[0] = 0x60;
    txBuffer[1] = CheckSum;
    if (!I2C_transfer(i2c, &i2cTransaction))
    {
            //while(1);
    }

   BQ27427_exitConfig(true);
}
