/******************************************************************************

 @file  simple_peripheral_oad_onchip.c

 @brief This file contains the OAD sample application based on
        simple_peripheral for use with the CC2650 Bluetooth Low Energy
        Protocol Stack.

 Group: WCS, BTS
 Target Device: cc13xx_cc26xx

 ******************************************************************************
 
 Copyright (c) 2017-2023, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#include <ti/display/Display.h>


/* to deal with semaphore*/
#include "shared.h"
#include <ti/drivers/dpl/SemaphoreP.h>
#include "unico.h"
#include "nwt_i2c.h"
#include "EEPROM.h"

#if (!(defined __TI_COMPILER_VERSION__) && !(defined __GNUC__))
#include <intrinsics.h>
#endif

#include <ti/drivers/utils/List.h>

#include "scif.h"
#include "nwt_sc.h"

#include <icall.h>
#include "util.h"
#include "onboard.h"
#include <bcomdef.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

#include <devinfoservice.h>
#include <simple_gatt_profile.h>

// Used for OAD Reset Service APIs
#include "oad_reset_service.h"
// Needed for HAL_SYSTEM_RESET()
//#include "hal_mcu.h"

//#include "peripheral.h"

#ifdef USE_RCOSC
#include <rcosc_calibration.h>
#endif //USE_RCOSC

#include <ti_drivers_config.h>
#include <board_key.h>

#include <menu/two_btn_menu.h>

#include "ti_ble_config.h"
#ifdef LED_DEBUG
#include <ti/drivers/GPIO.h>
#endif //LED_DEBUG

#include "simple_peripheral_oad_onchip_menu.h"
#include "simple_peripheral_oad_onchip.h"

#include <ti/sysbios/BIOS.h>
#include <math.h>

// Used for imgHdr_t structure
#include <common/cc26xx/oad/oad_image_header.h>
#include "oad.h"
#include <common/cc26xx/flash_interface/flash_interface.h>


/*********************************************************************
 * CONSTANTS
 */
// How often to perform periodic event (in ms)
#define SP_PERIODIC_EVT_PERIOD               5000

// Offset into the scanRspData string the software version info is stored
#define OAD_SOFT_VER_OFFSET                   15

// Task configuration
#define SP_TASK_PRIORITY                     1

#ifndef SP_TASK_STACK_SIZE
#define SP_TASK_STACK_SIZE                   2048
#endif

// Application events
#define SP_STATE_CHANGE_EVT                  0
#define SP_CHAR_CHANGE_EVT                   1
#define SP_KEY_CHANGE_EVT                    2
#define SP_ADV_EVT                           3
#define SP_PAIR_STATE_EVT                    4
#define SP_PASSCODE_EVT                      5
#define SP_PERIODIC_EVT                      6
#define SP_READ_RPA_EVT                      7
#define SP_SEND_PARAM_UPDATE_EVT             8
#define SP_CONN_EVT                          9
#define SP_OAD_RESET_EVT                     10


#define SP_OAD_QUEUE_EVT                     OAD_QUEUE_EVT       // Event_Id_01
#define SP_OAD_COMPLETE_EVT                  OAD_DL_COMPLETE_EVT // Event_Id_02
#define SP_OAD_NO_MEM_EVT                    OAD_OUT_OF_MEM_EVT  // Event_Id_03

// Internal Events for RTOS application
#define SP_ICALL_EVT                         ICALL_MSG_EVENT_ID  // Event_Id_31
#define SP_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

// Bitwise OR of all RTOS events to pend on
#define SP_ALL_EVENTS                        (SP_ICALL_EVT             | \
                                              SP_QUEUE_EVT             | \
                                              SP_OAD_QUEUE_EVT         | \
                                              SP_OAD_COMPLETE_EVT      | \
                                              SP_OAD_NO_MEM_EVT)

// Size of string-converted device address ("0xXXXXXXXXXXXX")
#define SP_ADDR_STR_SIZE     15

// Row numbers for two-button menu
#define SP_ROW_SEPARATOR_1   (TBM_ROW_APP + 0)
#define SP_ROW_STATUS_1      (TBM_ROW_APP + 1)
#define SP_ROW_STATUS_2      (TBM_ROW_APP + 2)
#define SP_ROW_CONNECTION    (TBM_ROW_APP + 3)
#define SP_ROW_ADVSTATE      (TBM_ROW_APP + 4)
#define SP_ROW_RSSI          (TBM_ROW_APP + 5)
#define SP_ROW_IDA           (TBM_ROW_APP + 6)
#define SP_ROW_RPA           (TBM_ROW_APP + 7)
#define SP_ROW_DEBUG         (TBM_ROW_APP + 8)

// For storing the active connections
#define SP_RSSI_TRACK_CHNLS        1            // Max possible channels can be GAP_BONDINGS_MAX
#define SP_MAX_RSSI_STORE_DEPTH    5
#define SP_INVALID_HANDLE          0xFFFF
#define RSSI_2M_THRSHLD           -30           // -80 dB rssi
#define RSSI_1M_THRSHLD           -40           // -90 dB rssi
#define RSSI_S2_THRSHLD           -50           // -100 dB rssi
#define RSSI_S8_THRSHLD           -60           // -120 dB rssi
#define SP_PHY_NONE                LL_PHY_NONE  // No PHY set
#define AUTO_PHY_UPDATE            0xFF

// Spin if the expression is not true
#define SIMPLEPERIPHERAL_ASSERT(expr) if (!(expr)) simple_peripheral_spin();
/*********************************************************************
 * TYPEDEFS
 */

// Auto connect available groups
enum
{
  AUTOCONNECT_DISABLE = 0,              // Disable
  AUTOCONNECT_GROUP_A = 1,              // Group A
  AUTOCONNECT_GROUP_B = 2               // Group B
};

// App event passed from stack modules. This type is defined by the application
// since it can queue events to itself however it wants.
typedef struct
{
  uint8_t event;                // event type
  void    *pData;               // pointer to message
} spEvt_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t state;
  uint16_t connHandle;
  uint8_t status;
} spPairStateData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t deviceAddr[B_ADDR_LEN];
  uint16_t connHandle;
  uint8_t uiInputs;
  uint8_t uiOutputs;
  uint32_t numComparison;
} spPasscodeData_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
  uint32_t event;
  void *pBuf;
} spGapAdvEventData_t;

// Container to store information from clock expiration using a flexible array
// since data is not always needed
typedef struct
{
  uint8_t event;                //
  uint8_t data[];
} spClockEventData_t;

// List element for parameter update and PHY command status lists
typedef struct
{
  List_Elem elem;
  uint16_t  connHandle;
} spConnHandleEntry_t;

// Connected device information
typedef struct
{
  uint16_t         connHandle;                        // Connection Handle
  Clock_Struct*    pUpdateClock;                      // pointer to clock struct
  int8_t           rssiArr[SP_MAX_RSSI_STORE_DEPTH];
  uint8_t          rssiCntr;
  int8_t           rssiAvg;
  bool             phyCngRq;                          // Set to true if PHY change request is in progress
  uint8_t          currPhy;
  uint8_t          rqPhy;
  uint8_t          phyRqFailCnt;                      // PHY change request count
  bool             isAutoPHYEnable;                   // Flag to indicate auto phy change
} spConnRec_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

uint8_t SelfCheckFlags;
uint16_t CellVoltage_s; //Cell voltage in mV
uint16_t CellCurrent_s; //Current measurement in mA (max, average or standby)
uint16_t CellSoC_s; //State of charge measurement in %

// Display Interface
extern Display_Handle dispHandle;
extern SemaphoreP_Handle sem_display;
extern bool ble_activate;

extern const imgHdr_t _imgHdr;

// Task configuration
Task_Struct spTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(spTaskStack, 8)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__ ((aligned (8)))
#else
#pragma data_alignment=8
#endif

uint8_t spTaskStack[SP_TASK_STACK_SIZE];

#define APP_EVT_EVENT_MAX 0x9
char *appEventStrings[] = {
  "APP_STATE_CHANGE_EVT     ",
  "APP_CHAR_CHANGE_EVT      ",
  "APP_KEY_CHANGE_EVT       ",
  "APP_ADV_EVT              ",
  "APP_PAIR_STATE_EVT       ",
  "APP_PASSCODE_EVT         ",
  "APP_READ_RPA_EVT         ",
  "APP_PERIODIC_EVT         ",
  "APP_SEND_PARAM_UPDATE_EVT",
  "APP_CONN_EVT             ",
};

//reset connection handle
uint16_t resetConnHandle = LINKDB_CONNHANDLE_INVALID;

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Queue object used for app messages
static Queue_Struct appMsgQueue;
static Queue_Handle appMsgQueueHandle;

// Clock instance for internal periodic events. Only one is needed since
// GattServApp will handle notifying all connected GATT clients
static Clock_Struct clkPeriodic;
// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;

// Memory to pass periodic event ID to clock handler
spClockEventData_t argPeriodic =
{ .event = SP_PERIODIC_EVT };

// Memory to pass RPA read event ID to clock handler
spClockEventData_t argRpaRead =
{ .event = SP_READ_RPA_EVT };

// Per-handle connection info
static spConnRec_t connList[MAX_NUM_BLE_CONNS];

// Current connection handle as chosen by menu
static uint16_t menuConnHandle = LINKDB_CONNHANDLE_INVALID;

// List to store connection handles for set phy command status's
static List_List setPhyCommStatList;

// List to store connection handles for queued param updates
static List_List paramUpdateList;

// Auto connect Disabled/Enabled {0 - Disabled, 1- Group A , 2-Group B, ...}
uint8_t autoConnect = AUTOCONNECT_DISABLE;

// Variable used to store the number of messages pending once OAD completes
// The application cannot reboot until all pending messages are sent
static uint8_t numPendingMsgs = 0;
static bool oadWaitReboot = false;

// Flag to be stored in NV that tracks whether service changed
// indications needs to be sent out
static uint32_t  sendSvcChngdOnNextBoot = FALSE;

// Advertising handles
uint8 advHandleLegacy;
uint8 advHandleLongRange;

extern float acc;
extern int fsm;
int received_msg;
uint8_t calculated_threshold;

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = {0};


//flag for ble adevrtising
bool advertising_initialized = false;

uint8_t sw_v[3] = {0}; //Software Version
uint8_t sn[2] = {0}; //Serial Number

uint16_t iter_counter = 0;

uint32_t cptF1;
uint32_t cptON;

bool SelfC_state;

//Delta threshold (for ON/F1)
float delta_thr = 0.1;
//Delta threshold (for OFF state)
float delta_thr_off = 0.001;

//Re-Configure MLC flag
uint8_t reconfig_mlc_flag = 0;
/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimplePeripheral_init( void );
static void SimplePeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t SimplePeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimplePeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimplePeripheral_processGapMessage(gapEventHdr_t *pMsg);
static void SimplePeripheral_advCallback(uint32_t event, void *pBuf, uintptr_t arg);
static void SimplePeripheral_processAdvEvent(spGapAdvEventData_t *pEventData);
static void SimplePeripheral_processAppMsg(spEvt_t *pMsg);
static void SimplePeripheral_processCharValueChangeEvt(uint8_t paramId);
static void SimplePeripheral_performPeriodicTask(void);
static void SimplePeripheral_updateRPA(void);
static void SimplePeripheral_clockHandler(UArg arg);
static void SimplePeripheral_passcodeCb(uint8_t *pDeviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs,
                                        uint32_t numComparison);
static void SimplePeripheral_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status);
static void SimplePeripheral_processPairState(spPairStateData_t *pPairState);
static void SimplePeripheral_processPasscode(spPasscodeData_t *pPasscodeData);
static void SimplePeripheral_charValueChangeCB(uint8_t paramId);
static status_t SimplePeripheral_enqueueMsg(uint8_t event, void *pData);
void SimplePeripheral_processOadResetWriteCB(uint16_t connHandle,
                                             uint16_t bim_var);
static void SimplePeripheral_keyChangeHandler(uint8 keys);
static void SimplePeripheral_handleKeys(uint8_t keys);
static void SimplePeripheral_processOadResetEvt(oadResetWrite_t *resetEvt);
static void SimplePeripheral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static void SimplePeripheral_initPHYRSSIArray(void);
static void SimplePeripheral_updatePHYStat(uint16_t eventCode, uint8_t *pMsg);
static uint8_t SimplePeripheral_addConn(uint16_t connHandle);
static uint8_t SimplePeripheral_getConnIndex(uint16_t connHandle);
static uint8_t SimplePeripheral_removeConn(uint16_t connHandle);
static void SimplePeripheral_processParamUpdate(uint16_t connHandle);
static uint8_t SimplePeripheral_processL2CAPMsg(l2capSignalEvent_t *pMsg);
static status_t SimplePeripheral_startAutoPhyChange(uint16_t connHandle);
static status_t SimplePeripheral_stopAutoPhyChange(uint16_t connHandle);
static status_t SimplePeripheral_setPhy(uint16_t connHandle, uint8_t allPhys,
                                        uint8_t txPhy, uint8_t rxPhy,
                                        uint16_t phyOpts);
static uint8_t SimplePeripheral_clearConnListEntry(uint16_t connHandle);
static void SimplePeripheral_menuSwitchCb(tbmMenuObj_t* pMenuObjCurr,
                                          tbmMenuObj_t* pMenuObjNext);
static void SimplePeripheral_connEvtCB(Gap_ConnEventRpt_t *pReport);
static void SimplePeripheral_processConnEvt(Gap_ConnEventRpt_t *pReport);

static void PreStartupSelfCheck(void);

static void Set_HallCAP_INT(void);
/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Bond Manager Callbacks
static gapBondCBs_t SimplePeripheral_BondMgrCBs =
{
  SimplePeripheral_passcodeCb,       // Passcode callback
  SimplePeripheral_pairStateCb       // Pairing/Bonding state Callback
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t SimplePeripheral_simpleProfileCBs =
{
  SimplePeripheral_charValueChangeCB // Characteristic value change callback
};

static oadResetWriteCB_t SimplePeripheral_oadResetCBs =
{
  SimplePeripheral_processOadResetWriteCB // Write Callback.
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      simple_peripheral_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void simple_peripheral_spin(void)
{
  volatile uint8_t x = 0;

  while(1)
  {
    x++;
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_createTask
 *
 * @brief   Task creation function for the Simple Peripheral OAD User App.
 */
void SimplePeripheral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = spTaskStack;
  taskParams.stackSize = SP_TASK_STACK_SIZE;
  taskParams.priority = SP_TASK_PRIORITY;

  Task_construct(&spTask, SimplePeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimplePeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 */
static void SimplePeripheral_init(void)
{
  // Create the menu
  SimplePeripheral_buildMenu();
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

#if 0
  /* Configure the LED pin */
  GPIO_setConfig(CONFIG_GPIO_GLED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
  GPIO_setConfig(CONFIG_GPIO_RLED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

  uint_least8_t activeLed;
  uint8_t blinkCnt = 16;

  if (blinkCnt < 12)
  {
    activeLed = CONFIG_GPIO_GLED;
  }
  else
  {
    activeLed = CONFIG_GPIO_RLED;
  }
  for(uint8_t numBlinks = 0; numBlinks < blinkCnt; ++numBlinks)
  {
    GPIO_toggle(activeLed);
    // Sleep for 100ms, sys-tick for BLE-Stack is 10us,
    // Task sleep is in # of ticks
    Task_sleep(10000);
  }
  GPIO_write(activeLed, 0);
#endif //LED_DEBUG

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueueHandle = Util_constructQueue(&appMsgQueue);

  // Create one-shot clock for internal periodic events.
  Util_constructClock(&clkPeriodic, SimplePeripheral_clockHandler,
                      SP_PERIODIC_EVT_PERIOD, 0, false, (UArg)&argPeriodic);

  uint8_t swVer[OAD_SW_VER_LEN];
  OAD_getSWVersion(swVer, OAD_SW_VER_LEN);

  // Set the Device Name characteristic in the GAP GATT Service
  // For more information, see the section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Configure GAP
  {
    uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;

    // Pass all parameter update requests to the app for it to decide
    GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
  }

  // Setup the GAP Bond Manager. For more information see the GAP Bond Manager
  // section in the User's Guide
  setBondManagerParameters();

  // Initialize GATT attributes
  GGS_AddService(GAP_SERVICE);                 // GAP GATT Service
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT Service
  DevInfo_AddService();                        // Device Information Service
  SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile

  Reset_addService((oadUsrAppCBs_t *)&SimplePeripheral_oadResetCBs);

  // Setup the SimpleProfile Characteristic Values
  // For more information, see the GATT and GATTServApp sections in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  {
    uint8_t charValue1 = 1;
    uint8_t charValue2 = 2;
    uint8_t charValue3[SIMPLEPROFILE_CHAR3_LEN];
    uint8_t charValue4[SIMPLEPROFILE_CHAR4_LEN];
    uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
    uint8_t charValue6 = 6;

    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t),
                               &charValue1);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint8_t),
                               &charValue2);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, SIMPLEPROFILE_CHAR3_LEN,
                               charValue3);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN,
                               charValue4);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN,
                               charValue5);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR6, sizeof(uint8_t),
                                   &charValue6);
  }

  // Register callback with SimpleGATTprofile
  SimpleProfile_RegisterAppCBs(&SimplePeripheral_simpleProfileCBs);

  // Start Bond Manager and register callback
  VOID GAPBondMgr_Register(&SimplePeripheral_BondMgrCBs);

  // Register with GAP for HCI/Host messages. This is needed to receive HCI
  // events. For more information, see the HCI section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  // Set default values for Data Length Extension
  // Extended Data Length Feature is already enabled by default
  {
    // Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
    // Some brand smartphone is essentially needing 251/2120, so we set them here.
    #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
    #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

    // This API is documented in hci.h
    // See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
  }

  // Initialize GATT Client
  GATT_InitClient();

  // Init key debouncer
  Board_initKeys(SimplePeripheral_keyChangeHandler);

  // Initialize Connection List
  SimplePeripheral_clearConnListEntry(LINKDB_CONNHANDLE_ALL);

  //Initialize GAP layer for Peripheral role and register to receive GAP events
  GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, addrMode, &pRandomAddress);

  // Initialize array to store connection handle and RSSI values
  SimplePeripheral_initPHYRSSIArray();

  // The type of display is configured based on the BOARD_DISPLAY_USE...
  // preprocessor definitions
  dispHandle = Display_open(Display_Type_ANY, NULL);

  // Initialize Two-Button Menu module
  TBM_SET_TITLE(&spMenuMain, "Simple Peripheral");
  tbm_setItemStatus(&spMenuMain, SP_ITEM_SELECT_OAD_DBG, SP_ITEM_SELECT_CONN);

  tbm_initTwoBtnMenu(dispHandle, &spMenuMain, 4, SimplePeripheral_menuSwitchCb);
  Display_printf(dispHandle, SP_ROW_SEPARATOR_1, 0, "====================");

  uint8_t versionStr[OAD_SW_VER_LEN + 1];

  memcpy(versionStr, swVer, OAD_SW_VER_LEN);

  // Add in Null terminator
  versionStr[OAD_SW_VER_LEN] = 0;

  // Display Image version
  Display_print1(dispHandle, 0, 0, "SBP On-chip OAD v%s",
                 versionStr);

  /*
   * When switching from persistent app back to the user application for the
   * for the first time after an OAD the device must send a service changed
   * indication. This will cause any peers to rediscover services.
   *
   * To prevent sending a service changed IND on every boot, a flag is stored
   * in NV to determine whether or not the service changed IND needs to be
   * sent
   */
  uint8_t status = osal_snv_read(BLE_NVID_CUST_START,
                                  sizeof(sendSvcChngdOnNextBoot),
                                  (uint8 *)&sendSvcChngdOnNextBoot);
  if(status != SUCCESS)
  {
    /*
     * On first boot the NV item will not have yet been initialzed, and the read
     * will fail. Do a write to set the initial value of the flash in NV
     */
     osal_snv_write(BLE_NVID_CUST_START, sizeof(sendSvcChngdOnNextBoot),
                    (uint8 *)&sendSvcChngdOnNextBoot);
  }

}

/*********************************************************************
 * @fn      SimplePeripheral_taskFxn
 *
 * @brief   Application task entry point for the Simple Peripheral.
 *
 * @param   a0, a1 - not used.
 */
static void SimplePeripheral_taskFxn(UArg a0, UArg a1)
{

/* Soft Version and Serial Number */
//    uint8_t sw_version[3];
//    uint8_t serial_number[2];
//
//    sw_version[0] = 0;
//    sw_version[1] = 3;
//    sw_version[2] = 16;
//
//    serial_number[0] = 1;
//    serial_number[1] = 5;
//    /****** Read SW version & serial number stored in EEPROM *****/
//    EEPROM_Write(SW_VERSION_ADD, sw_version, 3);
//    EEPROM_Write(SN_ADD, serial_number, 2);
//    /*************************************************************/

  // Initialize application
  SimplePeripheral_init();

  /****************** Init Fuel Gauge **************************/
  BQ27427_init(&BQ27427);
  BQ27427_Full_Reset();
  /*************************************************************/

  /************** Fuel Gauge Config **************/
  BQ27427_enterConfig(true);
  BQ27427_setCapacity(50);
  BQ27427_setDesignEnergy(50 * 2.75f);
  BQ27427_setTerminateVoltageMin(2750);
  BQ27427_setTaperRateTime(10 * 50 / 3);
  BQ27427_setChargeVChgTermination(4200);
  //Set Sleep current
  BQ27427_setHibernateCurrent(1); //50mA
  //Enable Sleep Mode
  BQ27427_setSLEEPenable(true);
  BQ27427_exitConfig(true);
  //FlipCurrentSign(); //Correct Current Gain
  /********************************************/

  SelfCheck.self_check_state = true;
  SelfC_state = true;

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, SP_ALL_EVENTS,
                        ICALL_TIMEOUT_PREDEFINE);

#ifdef FINAL_VERSION
    /******** Pre Startup Self Check **************/
    if(SelfCheck.self_check_state)
    {
        PreStartupSelfCheck();
        iter_counter++; //Increment iteration counter : Pre Startup Check should only last about 1 minute
    }
    else //Self Check Test is over --> Bluetooth should now be activated only via hall effect sensor
    {
        Set_HallCAP_INT();
        // Wait on sem indefinitely
        SemaphoreP_pend(sem_magn, BIOS_WAIT_FOREVER);

        // Enable advertising
        sp_enable_advertising();
        iter_counter++; //Increment iteration counter
    }

    SemaphoreP_post(sem_i2c_thread);
    /**********************************************/
#else
    PreStartupSelfCheck();
#endif

    config_ble_service();

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      // Fetch any available messages that might have been sent from the stack
      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8_t safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = SimplePeripheral_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & SP_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueueHandle))
        {
          spEvt_t *pMsg = (spEvt_t *)Util_dequeueMsg(appMsgQueueHandle);
          if (pMsg)
          {
            // Process message.
            SimplePeripheral_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }
      // OAD events
      if(events & SP_OAD_NO_MEM_EVT)
      {
        // The OAD module is unable to allocate memory, print failure, cancel OAD
        Display_print0(dispHandle, SP_ROW_STATUS_1, 0,
                        "OAD malloc fail, cancelling OAD");
        OAD_cancel();
#if 0
        // Diplay is not enabled in persist app so use LED
        GPIO_write(CONFIG_GPIO_RLED, 1);
#endif //LED_DEBUG
      }
       // OAD queue processing
      if(events & SP_OAD_QUEUE_EVT)
      {
        // Process the OAD Message Queue
        uint8_t status = OAD_processQueue();

        // If the OAD state machine encountered an error, print it
        // Return codes can be found in oad_constants.h
        if(status == OAD_DL_COMPLETE)
        {
          Display_print0(dispHandle, SP_ROW_STATUS_1, 0, "OAD DL Complete, wait for Enable");
        }
        else if(status == OAD_IMG_ID_TIMEOUT)
        {
          Display_print0(dispHandle, SP_ROW_STATUS_1, 0, "ImgID Timeout, disconnecting");

          // This may be an attack, terminate the link,
          // Note HCI_DISCONNECT_REMOTE_USER_TERM seems to most closet reason for
          // termination at this state
          MAP_GAP_TerminateLinkReq(OAD_getactiveCxnHandle(), HCI_DISCONNECT_REMOTE_USER_TERM);
        }
        else if(status != OAD_SUCCESS)
        {
          Display_print1(dispHandle, SP_ROW_STATUS_1, 0, "OAD Error: %d", status);
        }

      }

      if(events & SP_OAD_COMPLETE_EVT)
      {
        // Register for L2CAP Flow Control Events
        L2CAP_RegisterFlowCtrlTask(selfEntity);
      }

    }

    /*SemaphoreP_post(sem_i2c_thread);
        #ifndef FINAL_VERSION
            SemaphoreP_post(sem_fdcan_thread);
        #endif*/

#ifdef FINAL_VERSION
    /************ Pre Startup Check Test ************/
        if(SelfCheck.self_check_state)
        {
            if(iter_counter == 5245)
            {
                SelfC_state = false;
                 //Update flags
                SelfCheckFlags = ((uint8_t) (SelfC_state)<<4) | ((uint8_t) (SelfCheck.hall_effect_sensor_out)<<3) | ((uint8_t) (SelfCheck.fuel_gauge)<<2) | ((uint8_t) (SelfCheck.eeprom_present)<<1) | (uint8_t) (SelfCheck.sensor_present);
            }
            else if(iter_counter > 5333) // 500 = 0.5*500s//60 seconds --> Turn off pre startup check | Start main program tasks
            {
                SelfCheck.self_check_state = false;
                // Stop advertising
                sp_disable_advertising();
                iter_counter = 0; //Reset iter counter
                /************ Release main program tasks *********/
                SemaphoreP_post(sem_i2c_thread);
                #ifndef FINAL_VERSION
                    SemaphoreP_post(sem_fdcan_thread);
                #endif
                /*************************************************/
            }
        }
        else
        {
            if(iter_counter < 8000)
            {
                SemaphoreP_post(sem_magn); //Post semaphore to keep the iterations going : Bluetooth Task runs for 60 seconds
                SemaphoreP_post(sem_i2c_thread);
             }
            else
            {
                // Stop advertising
                sp_disable_advertising();
                iter_counter = 0; //Block Task iterations and reset counter
            }
        }
    /************************************************/
#endif
  }
}


//Config of Ble services profil
void config_ble_service(void)
{
    uint8_t CHAR3_DATA[SIMPLEPROFILE_CHAR3_LEN]; //Acc Z + FSM STATUS + threshold + SC counters + Self-Check Test Flags + Software Version + Serial Number

    /************ Read SC counter *****************/
    cptF1 =  ((uint32_t) scifTaskData.count.output.F1Msb << 16) | (uint32_t) scifTaskData.count.output.F1Lsb;
    cptON =  ((uint32_t) scifTaskData.count.output.OnMsb << 16) | (uint32_t) scifTaskData.count.output.OnLsb;
    /*********************************************/

    /* XL
     * Multiplie la valeur l'accéléro par 100
     * 1.43 devient 143, On lit 0x8F sur le BLE (143 en decimal)
    */

    int acc_conv;
    acc_conv = (int)round(acc*100);

    CHAR3_DATA[0] = acc_conv; //Acc Z
    CHAR3_DATA[1] = SensorData.FSM_status; //FSM STATUS
    CHAR3_DATA[2] = calculated_threshold; //Threshold value
    /********* SC Counters ******/
    CHAR3_DATA[3] = cptON >> 24;
    CHAR3_DATA[4] = cptON >> 16;
    CHAR3_DATA[5] = cptON >> 8;
    CHAR3_DATA[6] = cptON;
    CHAR3_DATA[7] = cptF1 >> 24;
    CHAR3_DATA[8] = cptF1 >> 16;
    CHAR3_DATA[9] = cptF1 >> 8;
    CHAR3_DATA[10] = cptF1;
    /* Self-Check */
    //Software version and Serial Number
    CHAR3_DATA[11] = SelfCheckFlags;
    CHAR3_DATA[12] = sw_v[0];
    CHAR3_DATA[13] = sw_v[1];
    CHAR3_DATA[14] = sw_v[2];
    CHAR3_DATA[15] = sn[0];
    CHAR3_DATA[16] = sn[1];
    //Fuel Gauge Data
    CHAR3_DATA[17] = CellVoltage_s>>8;
    CHAR3_DATA[18] = CellVoltage_s;
    CHAR3_DATA[19] = CellCurrent_s>>8;
    CHAR3_DATA[20] = CellCurrent_s;
    CHAR3_DATA[21] = CellSoC_s>>8;
    CHAR3_DATA[22] = CellSoC_s;
    //ON-F1 Threshold
    CHAR3_DATA[23] = SensorData.ON_F1_Threshold>>8;
    CHAR3_DATA[24] = SensorData.ON_F1_Threshold;
    //WakeUp counter
    CHAR3_DATA[25] = cptWU >> 24;
    CHAR3_DATA[26] = cptWU >> 16;
    CHAR3_DATA[27] = cptWU >> 8;
    CHAR3_DATA[28] = cptWU;

    //Set CHAR3 value
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, SIMPLEPROFILE_CHAR3_LEN, CHAR3_DATA);
}

void sp_disable_advertising(void)
{
    // Stop advertising
    GapAdv_disable(advHandleLongRange);
    GapAdv_disable(advHandleLegacy);
}

void sp_enable_advertising(void)
{
    bStatus_t status = FAILURE;

    //enable advertising only if previously initialized
    if(advertising_initialized)
    {
        // Enable legacy advertising for set #1
        status = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);

        // Enable long range advertising for set #2
        status = GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
    }
}

//Config threshold of mlc
void config_mlc_threshold(void)
{
    float newThreshold = 0.0f;
    uint16_t newThreshold_c = 0;

    /* Read the truth state of the sail (by user)
     * 2 = On
     * 4 = F1
     * */
    SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &received_msg);

    if(received_msg == 2)  //ON
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
    else if(received_msg == 4) //F1
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
    else if(received_msg == 0) //OFF
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
    else if(received_msg == 5) //Reset counters
    {
        cptF1 = 0; //F1 counter
        cptON = 0; //ON counter
        cptWU = 0; //WakeUp counter
        /********** Reset SC counters *******/
        scifTaskData.count.output.F1Msb = 0;
        scifTaskData.count.output.F1Lsb = 0;
        scifTaskData.count.output.OnMsb = 0;
        scifTaskData.count.output.OnLsb = 0;
    }
    else if(received_msg == 6) //Activate SelfCheck test
    {
        SelfCheck.self_check_state = true;
        SelfC_state = true; //Re-launch SelfCheck test
        //Update SelfCheck Flags
        SelfCheckFlags = ((uint8_t) (SelfC_state)<<4) | ((uint8_t) (SelfCheck.hall_effect_sensor_out)<<3) | ((uint8_t) (SelfCheck.fuel_gauge)<<2) | ((uint8_t) (SelfCheck.eeprom_present)<<1) | (uint8_t) (SelfCheck.sensor_present);
        config_ble_service();
        iter_counter = 0; //Reset iter counter
    }

    SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &received_msg);
}

/*********************************************************************
 * @fn      SimplePeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  // Always dealloc pMsg unless set otherwise
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      SimplePeripheral_processGapMessage((gapEventHdr_t*) pMsg);
      break;

    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = SimplePeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch(pMsg->status)
      {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
        // Process HCI Command Complete Events here
        {
          SimplePeripheral_processCmdCompleteEvt((hciEvt_CmdComplete_t *) pMsg);
          break;
        }

        case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
          AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
          break;

        // HCI Commands Events
        case HCI_COMMAND_STATUS_EVENT_CODE:
        {
          hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *)pMsg;
          switch ( pMyMsg->cmdOpcode )
          {
            case HCI_LE_SET_PHY:
            {
              if (pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
              {
                Display_printf(dispHandle, SP_ROW_STATUS_1, 0,
                        "PHY Change failure, peer does not support this");
              }
              else
              {
                Display_printf(dispHandle, SP_ROW_STATUS_1, 0,
                               "PHY Update Status Event: 0x%x",
                               pMyMsg->cmdStatus);
              }

              SimplePeripheral_updatePHYStat(HCI_LE_SET_PHY, (uint8_t *)pMsg);
              break;
            }

            default:
              break;
          }
          break;
        }

        // LE Events
        case HCI_LE_EVENT_CODE:
        {
          hciEvt_BLEPhyUpdateComplete_t *pPUC =
            (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

          // A Phy Update Has Completed or Failed
          if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
          {
            if (pPUC->status != SUCCESS)
            {
              Display_printf(dispHandle, SP_ROW_STATUS_1, 0,
                             "PHY Change failure");
            }
            else
            {
              // Only symmetrical PHY is supported.
              // rxPhy should be equal to txPhy.
              Display_printf(dispHandle, SP_ROW_STATUS_2, 0,
                             "PHY Updated to %s",
                             (pPUC->rxPhy == HCI_PHY_1_MBPS) ? "1M" :
                             (pPUC->rxPhy == HCI_PHY_2_MBPS) ? "2M" :
                              "CODED");
            }

            SimplePeripheral_updatePHYStat(HCI_BLE_PHY_UPDATE_COMPLETE_EVENT, (uint8_t *)pMsg);
          }
          break;
        }

        default:
          break;
      }

      break;
    }

    case L2CAP_SIGNAL_EVENT:
      // Process L2CAP signal
      safeToDealloc = SimplePeripheral_processL2CAPMsg((l2capSignalEvent_t *)pMsg);
      break;

    default:
      // do nothing
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimplePeripheral_processL2CAPMsg
 *
 * @brief   Process L2CAP messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processL2CAPMsg(l2capSignalEvent_t *pMsg)
{
  uint8_t safeToDealloc = TRUE;
  static bool firstRun = TRUE;

  switch (pMsg->opcode)
  {
    case L2CAP_NUM_CTRL_DATA_PKT_EVT:
    {
      /*
      * We cannot reboot the device immediately after receiving
      * the enable command, we must allow the stack enough time
      * to process and respond to the OAD_EXT_CTRL_ENABLE_IMG
      * command. This command will determine the number of
      * packets currently queued up by the LE controller.
      * BIM var is already set via OadPersistApp_processOadWriteCB
      */
      if(firstRun)
      {
        firstRun = false;

        // We only want to set the numPendingMsgs once
        numPendingMsgs = MAX_NUM_PDU - pMsg->cmd.numCtrlDataPktEvt.numDataPkt;

        // Wait until all PDU have been sent on cxn events
        Gap_RegisterConnEventCb(SimplePeripheral_connEvtCB,
                                  GAP_CB_REGISTER,
                                  GAP_CB_CONN_EVENT_ALL,
                                  resetConnHandle);
                                  //OAD_getactiveCxnHandle());
                                  //pMsg->connHandle);
                                  //0);

        /* Set the flag so that the connection event callback will
         * be processed in the context of a pending OAD reboot
         */
        oadWaitReboot = true;
      }

      break;
    }

    default:
      // do nothing
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimplePeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    OAD_setBlockSize(pMsg->msg.mtuEvt.MTU);
    Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      SimplePeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimplePeripheral_processAppMsg(spEvt_t *pMsg)
{
  bool dealloc = TRUE;

  switch (pMsg->event)
  {
    case SP_CHAR_CHANGE_EVT:
      SimplePeripheral_processCharValueChangeEvt(*(uint8_t*)(pMsg->pData));
      break;

    case SP_KEY_CHANGE_EVT:
      SimplePeripheral_handleKeys(*(uint8_t*)(pMsg->pData));
      break;

    case SP_ADV_EVT:
      SimplePeripheral_processAdvEvent((spGapAdvEventData_t*)(pMsg->pData));
      break;

    case SP_PAIR_STATE_EVT:
      SimplePeripheral_processPairState((spPairStateData_t*)(pMsg->pData));
      break;

    case SP_PASSCODE_EVT:
      SimplePeripheral_processPasscode((spPasscodeData_t*)(pMsg->pData));
      break;

    case SP_PERIODIC_EVT:
      SimplePeripheral_performPeriodicTask();
      break;

    case SP_READ_RPA_EVT:
      SimplePeripheral_updateRPA();
      break;

    case SP_SEND_PARAM_UPDATE_EVT:
    {
      // Extract connection handle from data
      uint16_t connHandle = *(uint16_t *)(((spClockEventData_t *)pMsg->pData)->data);

      SimplePeripheral_processParamUpdate(connHandle);

      // This data is not dynamically allocated
      dealloc = FALSE;
      break;
    }

    case SP_CONN_EVT:
      SimplePeripheral_processConnEvt((Gap_ConnEventRpt_t *)(pMsg->pData));
      break;

    case SP_OAD_RESET_EVT:
      SimplePeripheral_processOadResetEvt((oadResetWrite_t *)(pMsg->pData));
      break;

    default:
      // Do nothing.
      break;
  }

  // Free message data if it exists and we are to dealloc
  if ((dealloc == TRUE) && (pMsg->pData != NULL))
  {
    ICall_free(pMsg->pData);
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 */
static void SimplePeripheral_processGapMessage(gapEventHdr_t *pMsg)
{
  switch(pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      bStatus_t status = FAILURE;

      gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;

      if(pPkt->hdr.status == SUCCESS)
      {
        // Store the system ID
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = pPkt->devAddr[0];
        systemId[1] = pPkt->devAddr[1];
        systemId[2] = pPkt->devAddr[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = pPkt->devAddr[5];
        systemId[6] = pPkt->devAddr[4];
        systemId[5] = pPkt->devAddr[3];

        // Set Device Info Service Parameter
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "Initialized");

        // Setup and start Advertising
        // For more information, see the GAP section in the User's Guide:
        // http://software-dl.ti.com/lprf/ble5stack-latest/

        // Create Advertisement set #1 and assign handle
        status = GapAdv_create(&SimplePeripheral_advCallback, &advParams1,
                               &advHandleLegacy);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Load advertising data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
                                     sizeof(advData1), advData1);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Load scan response data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_SCAN_RSP,
                                     sizeof(scanResData1), scanResData1);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Set event mask for set #1
        status = GapAdv_setEventMask(advHandleLegacy,
                                     GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                     GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                     GAP_ADV_EVT_MASK_SET_TERMINATED);

        // Enable legacy advertising for set #1
        status = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Create Advertisement set #2 and assign handle
        status = GapAdv_create(&SimplePeripheral_advCallback, &advParams2,
                               &advHandleLongRange);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Load advertising data for set #2 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLongRange, GAP_ADV_DATA_TYPE_ADV,
                                     sizeof(advData2), advData2);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Set event mask for set #2
        status = GapAdv_setEventMask(advHandleLongRange,
                                     GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                     GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                     GAP_ADV_EVT_MASK_SET_TERMINATED);

        // Enable long range advertising for set #2
        status = GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Display device address
        Display_printf(dispHandle, SP_ROW_IDA, 0, "%s Addr: %s",
                       (addrMode <= ADDRMODE_RANDOM) ? "Dev" : "ID",
                       Util_convertBdAddr2Str(pPkt->devAddr));

        //adertising initialized
        advertising_initialized = true;

        if (addrMode > ADDRMODE_RANDOM)
        {
          SimplePeripheral_updateRPA();

          // Create one-shot clock for RPA check event.
          Util_constructClock(&clkRpaRead, SimplePeripheral_clockHandler,
                              READ_RPA_PERIOD, 0, true,
                              (UArg) &argRpaRead);
        }
      }

      break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
      gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t *)pMsg;

      // Display the amount of current connections
      uint8_t numActive = linkDB_NumActive();
      Display_printf(dispHandle, SP_ROW_STATUS_2, 0, "Num Conns: %d",
                     (uint16_t)numActive);

      if (pPkt->hdr.status == SUCCESS)
      {
        // Add connection to list and start RSSI
        SimplePeripheral_addConn(pPkt->connectionHandle);

        // Display the address of this connection
        Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "Connected to %s",
                       Util_convertBdAddr2Str(pPkt->devAddr));

        // Enable connection selection option
        tbm_setItemStatus(&spMenuMain, SP_ITEM_SELECT_CONN, TBM_ITEM_NONE);

        // Start Periodic Clock.
        Util_startClock(&clkPeriodic);
      }

      if (numActive < MAX_NUM_BLE_CONNS)
      {
        // Start advertising since there is room for more connections
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      }
      else
      {
        // Stop advertising since there is no room for more connections
        GapAdv_disable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        GapAdv_disable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      }

      break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
      gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t *)pMsg;

      // Display the amount of current connections
      uint8_t numActive = linkDB_NumActive();
      Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "Device Disconnected!");
      Display_printf(dispHandle, SP_ROW_STATUS_2, 0, "Num Conns: %d",
                     (uint16_t)numActive);

      // Remove the connection from the list and disable RSSI if needed
      SimplePeripheral_removeConn(pPkt->connectionHandle);

      // If no active connections
      if (numActive == 0)
      {
        // Stop periodic clock
        Util_stopClock(&clkPeriodic);

        // Disable Connection Selection option
        tbm_setItemStatus(&spMenuMain, TBM_ITEM_NONE, SP_ITEM_SELECT_CONN);
      }

      // Start advertising since there is room for more connections
      GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);

      // Clear remaining lines
      Display_clearLine(dispHandle, SP_ROW_CONNECTION);

      // Cancel the OAD if one is going on
      // A disconnect forces the peer to re-identify
      //OAD_cancel(); //ToDo: Remove: PK

      break;
    }

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
    {
      gapUpdateLinkParamReqReply_t rsp;
      gapUpdateLinkParamReqEvent_t *pReq = (gapUpdateLinkParamReqEvent_t *)pMsg;

      rsp.connectionHandle = pReq->req.connectionHandle;
      rsp.signalIdentifier = pReq->req.signalIdentifier;

      // Only accept connection intervals with slave latency of 0
      // This is just an example of how the application can send a response
      if(pReq->req.connLatency == 0)
      {
        rsp.intervalMin = pReq->req.intervalMin;
        rsp.intervalMax = pReq->req.intervalMax;
        rsp.connLatency = pReq->req.connLatency;
        rsp.connTimeout = pReq->req.connTimeout;
        rsp.accepted = TRUE;
      }
      else
      {
        rsp.accepted = FALSE;
      }

      // Send Reply
      VOID GAP_UpdateLinkParamReqReply(&rsp);

      break;
    }

    case GAP_LINK_PARAM_UPDATE_EVENT:
    {
      gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;

      // Get the address from the connection handle
      linkDBInfo_t linkInfo;
      linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

      if(pPkt->status == SUCCESS)
      {
        // Display the address of the connection update
        Display_printf(dispHandle, SP_ROW_STATUS_2, 0, "Link Param Updated: %s",
                       Util_convertBdAddr2Str(linkInfo.addr));
      }
      else
      {
        // Display the address of the connection update failure
        Display_printf(dispHandle, SP_ROW_STATUS_2, 0,
                       "Link Param Update Failed 0x%x: %s", pPkt->opcode,
                       Util_convertBdAddr2Str(linkInfo.addr));
      }

      // Check if there are any queued parameter updates
      spConnHandleEntry_t *connHandleEntry = (spConnHandleEntry_t *)List_get(&paramUpdateList);
      if (connHandleEntry != NULL)
      {
        // Attempt to send queued update now
        SimplePeripheral_processParamUpdate(connHandleEntry->connHandle);

        // Free list element
        ICall_free(connHandleEntry);
      }

      break;
    }

#if defined ( NOTIFY_PARAM_UPDATE_RJCT )
    case GAP_LINK_PARAM_UPDATE_REJECT_EVENT:
    {
      linkDBInfo_t linkInfo;
      gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;

      // Get the address from the connection handle
      linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

      // Display the address of the connection update failure
      Display_printf(dispHandle, SP_ROW_STATUS_2, 0,
                     "Peer Device's Update Request Rejected 0x%x: %s", pPkt->opcode,
                     Util_convertBdAddr2Str(linkInfo.addr));

      break;
    }
#endif
    default:
      Display_clearLines(dispHandle, SP_ROW_STATUS_1, SP_ROW_STATUS_2);
      break;
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramId - parameter Id of the value that was changed.
 *
 * @return  None.
 */
static void SimplePeripheral_charValueChangeCB(uint8_t paramId)
{
  uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

  if (pValue)
  {
    *pValue = paramId;

    if (SimplePeripheral_enqueueMsg(SP_CHAR_CHANGE_EVT, pValue) != SUCCESS)
    {
      ICall_freeMsg(pValue);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processOadWriteCB
 *
 * @brief   Process a write request to the OAD reset service
 *
 * @param   connHandle - the connection Handle this request is from.
 * @param   bim_var    - bim_var to set before resetting.
 *
 * @return  None.
 */
void SimplePeripheral_processOadWriteCB(uint8_t event, uint16_t arg)
{
  Event_post(syncEvent, event);
}

/*********************************************************************
 * @fn      SimplePeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 */
static void SimplePeripheral_processCharValueChangeEvt(uint8_t paramId)
{
    uint8_t newValue;

    switch(paramId)
    {
      case SIMPLEPROFILE_CHAR1:
          SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &newValue);
          config_mlc_threshold();
        break;

  //    case SIMPLEPROFILE_CHAR3:
  //      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &newValue);
  //      break;

      case SIMPLEPROFILE_CHAR6:
          SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR6, &newValue);
        break;

      default:
        // should not reach here!
        break;
    }
}

/*********************************************************************
 * @fn      SimplePeripheral_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (SP_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimplePeripheral_performPeriodicTask(void)
{
  uint8_t valueToCopy;

  // Call to retrieve the value of the third characteristic in the profile
  if (SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &valueToCopy) == SUCCESS)
  {
    // Call to set that value of the fourth characteristic in the profile.
    // Note that if notifications of the fourth characteristic have been
    // enabled by a GATT client device, then a notification will be sent
    // every time this function is called.
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                               &valueToCopy);
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_updateRPA
 *
 * @brief   Read the current RPA from the stack and update display
 *          if the RPA has changed.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimplePeripheral_updateRPA(void)
{
  // Read the current RPA.
  // The parameters for the call to HCI_LE_ReadLocalResolvableAddressCmd
  // are not needed to be accurate to retrieve the local resolvable address.
  // The 1st parameter can be any of ADDRMODE_PUBLIC and ADDRMODE_RANDOM.
  // The 2nd parameter only has to be not NULL.
  // The result will come with HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS
  // complete event.
  HCI_LE_ReadLocalResolvableAddressCmd(0, rpa);
}

/*********************************************************************
 * @fn      SimplePeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimplePeripheral_clockHandler(UArg arg)
{
  spClockEventData_t *pData = (spClockEventData_t *)arg;

  if (pData->event == SP_PERIODIC_EVT)
  {
    // Start the next period
    Util_startClock(&clkPeriodic);

    // Post event to wake up the application
    SimplePeripheral_enqueueMsg(SP_PERIODIC_EVT, NULL);
  }
  else if (pData->event == SP_READ_RPA_EVT)
  {
    // Start the next period
    Util_startClock(&clkRpaRead);

    // Post event to read the current RPA
    SimplePeripheral_enqueueMsg(SP_READ_RPA_EVT, NULL);
  }
  else if (pData->event == SP_SEND_PARAM_UPDATE_EVT)
  {
    // Send message to app
    SimplePeripheral_enqueueMsg(SP_SEND_PARAM_UPDATE_EVT, pData);
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   keys - bitmap of pressed keys
 *
 * @return  none
 */
static void SimplePeripheral_keyChangeHandler(uint8_t keys)
{
  uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

  if (pValue)
  {
    *pValue = keys;

    if(SimplePeripheral_enqueueMsg(SP_KEY_CHANGE_EVT, pValue) != SUCCESS)
    {
      ICall_freeMsg(pValue);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   keys - bit field for key events. Valid entries:
 *                 KEY_LEFT
 *                 KEY_RIGHT
 */
static void SimplePeripheral_handleKeys(uint8_t keys)
{
//  if (keys & KEY_LEFT)
//  {
//    // Check if the key is still pressed. Workaround for possible bouncing.
//    if (GPIO_read(CONFIG_GPIO_BTN1) == 0)
//    {
//      tbm_buttonLeft();
//    }
//  }
//  else if (keys & KEY_RIGHT)
//  {
//    // Check if the key is still pressed. Workaround for possible bouncing.
//    if (GPIO_read(CONFIG_GPIO_BTN2) == 0)
//    {
//      tbm_buttonRight();
//    }
//  }
}

/*********************************************************************
 * @fn      SimplePeripheral_doSetConnPhy
 *
 * @brief   Set PHY preference.
 *
 * @param   index - 0: 1M PHY
 *                  1: 2M PHY
 *                  2: 1M + 2M PHY
 *                  3: CODED PHY (Long range)
 *                  4: 1M + 2M + CODED PHY
 *
 * @return  always true
 */
bool SimplePeripheral_doSetConnPhy(uint8 index)
{
  bool status = TRUE;

  static uint8_t phy[] = {
    HCI_PHY_1_MBPS, HCI_PHY_2_MBPS, HCI_PHY_1_MBPS | HCI_PHY_2_MBPS,
    HCI_PHY_CODED, HCI_PHY_1_MBPS | HCI_PHY_2_MBPS | HCI_PHY_CODED,
    AUTO_PHY_UPDATE
  };

  uint8_t connIndex = SimplePeripheral_getConnIndex(menuConnHandle);
  SIMPLEPERIPHERAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Set Phy Preference on the current connection. Apply the same value
  // for RX and TX.
  // If auto PHY update is not selected and if auto PHY update is enabled, then
  // stop auto PHY update
  // Note PHYs are already enabled by default in build_config.opt in stack project.
  if(phy[index] != AUTO_PHY_UPDATE)
  {
    // Cancel RSSI reading  and auto phy changing
    SimplePeripheral_stopAutoPhyChange(connList[connIndex].connHandle);

    SimplePeripheral_setPhy(menuConnHandle, 0, phy[index], phy[index], 0);

    Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "PHY preference: %s",
                   TBM_GET_ACTION_DESC(&spMenuConnPhy, index));
  }
  else
  {
    // Start RSSI read for auto PHY update (if it is disabled)
    SimplePeripheral_startAutoPhyChange(menuConnHandle);
  }

  return status;
}
/*********************************************************************
 * @fn      SimplePeripheral_advCallback
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void SimplePeripheral_advCallback(uint32_t event, void *pBuf, uintptr_t arg)
{
  spGapAdvEventData_t *pData = ICall_malloc(sizeof(spGapAdvEventData_t));

  if (pData)
  {
    pData->event = event;
    pData->pBuf = pBuf;

    if(SimplePeripheral_enqueueMsg(SP_ADV_EVT, pData) != SUCCESS)
    {
      ICall_freeMsg(pData);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 */
static void SimplePeripheral_processAdvEvent(spGapAdvEventData_t *pEventData)
{
  switch (pEventData->event)
  {
    case GAP_EVT_ADV_START_AFTER_ENABLE:
      Display_printf(dispHandle, SP_ROW_ADVSTATE, 0, "Adv Set %d Enabled",
                     *(uint8_t *)(pEventData->pBuf));
      break;

    case GAP_EVT_ADV_END_AFTER_DISABLE:
      Display_printf(dispHandle, SP_ROW_ADVSTATE, 0, "Adv Set %d Disabled",
                     *(uint8_t *)(pEventData->pBuf));
      break;

    case GAP_EVT_ADV_START:
      break;

    case GAP_EVT_ADV_END:
      break;

    case GAP_EVT_ADV_SET_TERMINATED:
    {
      GapAdv_setTerm_t *advSetTerm = (GapAdv_setTerm_t *)(pEventData->pBuf);

      Display_printf(dispHandle, SP_ROW_ADVSTATE, 0, "Adv Set %d disabled after conn %d",
                     advSetTerm->handle, advSetTerm->connHandle );
    }
    break;

    case GAP_EVT_SCAN_REQ_RECEIVED:
      break;

    case GAP_EVT_INSUFFICIENT_MEMORY:
      break;

    default:
      break;
  }

  // All events have associated memory to free except the insufficient memory
  // event
  if (pEventData->event != GAP_EVT_INSUFFICIENT_MEMORY)
  {
    ICall_free(pEventData->pBuf);
  }
}


/*********************************************************************
 * @fn      SimplePeripheral_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SimplePeripheral_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  spPairStateData_t *pData = ICall_malloc(sizeof(spPairStateData_t));

  // Allocate space for the event data.
  if (pData)
  {
    pData->state = state;
    pData->connHandle = connHandle;
    pData->status = status;

    // Queue the event.
    if(SimplePeripheral_enqueueMsg(SP_PAIR_STATE_EVT, pData) != SUCCESS)
    {
      ICall_freeMsg(pData);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_passcodeCb
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void SimplePeripheral_passcodeCb(uint8_t *pDeviceAddr,
                                        uint16_t connHandle,
                                        uint8_t uiInputs,
                                        uint8_t uiOutputs,
                                        uint32_t numComparison)
{
  spPasscodeData_t *pData = ICall_malloc(sizeof(spPasscodeData_t));

  // Allocate space for the passcode event.
  if (pData )
  {
    pData->connHandle = connHandle;
    memcpy(pData->deviceAddr, pDeviceAddr, B_ADDR_LEN);
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    if(SimplePeripheral_enqueueMsg(SP_PASSCODE_EVT, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void SimplePeripheral_processPairState(spPairStateData_t *pPairData)
{
  uint8_t state = pPairData->state;
  uint8_t status = pPairData->status;

  switch (state)
  {
    case GAPBOND_PAIRING_STATE_STARTED:
      Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Pairing started");
      break;

    case GAPBOND_PAIRING_STATE_COMPLETE:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Pairing success");
      }
      else
      {
        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Pairing fail: %d", status);
      }
      break;

    case GAPBOND_PAIRING_STATE_ENCRYPTED:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Encryption success");
      }
      else
      {
        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Encryption failed: %d", status);
      }
      break;

    case GAPBOND_PAIRING_STATE_BOND_SAVED:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Bond save success");
      }
      else
      {
        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Bond save failed: %d", status);
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void SimplePeripheral_processPasscode(spPasscodeData_t *pPasscodeData)
{
  // Display passcode to user
  if (pPasscodeData->uiOutputs != 0)
  {
    Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Passcode: %d",
                   B_APP_DEFAULT_PASSCODE);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(pPasscodeData->connHandle , SUCCESS,
                         B_APP_DEFAULT_PASSCODE);
}

/*********************************************************************
 * @fn      SimplePeripheral_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void SimplePeripheral_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
  // Enqueue the event for processing in the app context.
  if(SimplePeripheral_enqueueMsg(SP_CONN_EVT, pReport) != SUCCESS)
  {
    ICall_freeMsg(pReport);
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void SimplePeripheral_processConnEvt(Gap_ConnEventRpt_t *pReport)
{
  /* If we are waiting for an OAD Reboot, process connection events to ensure
   * that we are not waiting to send data before restarting
   */
  if(oadWaitReboot)
  {
    // Wait until all pending messages are sent
    if(numPendingMsgs == 0)
    {
      // Store the flag to indicate that a service changed IND will
      // be sent at the next boot
      sendSvcChngdOnNextBoot = TRUE;

      uint8_t status = osal_snv_write(BLE_NVID_CUST_START,
                                      sizeof(sendSvcChngdOnNextBoot),
                                      (uint8 *)&sendSvcChngdOnNextBoot);
      if(status != SUCCESS)
      {
        Display_print1(dispHandle, 5, 0, "SNV WRITE FAIL: %d", status);
      }

      // Reset the system
      SystemReset();
    }
    else
    {
      numPendingMsgs--;
    }
  }
  else
  {
    // Get index from handle
    uint8_t connIndex = SimplePeripheral_getConnIndex(pReport->handle);

    // If auto phy change is enabled
    if (connList[connIndex].isAutoPHYEnable == TRUE)
    {
      // Read the RSSI
      HCI_ReadRssiCmd(pReport->handle);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 */
static status_t SimplePeripheral_enqueueMsg(uint8_t event, void *pData)
{
  uint8_t success;
  spEvt_t *pMsg = ICall_malloc(sizeof(spEvt_t));

  // Create dynamic pointer to message.
  if(pMsg)
  {
    pMsg->event = event;
    pMsg->pData = pData;

    // Enqueue the message.
    success = Util_enqueueMsg(appMsgQueueHandle, syncEvent, (uint8_t *)pMsg);
    return (success) ? SUCCESS : FAILURE;
  }

  return(bleMemAllocError);
}

/*********************************************************************
 * @fn      SimplePeripheral_doSelectConn
 *
 * @brief   Select a connection to communicate with
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool SimplePeripheral_doSelectConn(uint8_t index)
{
  menuConnHandle = connList[index].connHandle;

  // Set the menu title and go to this connection's context
  TBM_SET_TITLE(&spMenuPerConn, TBM_GET_ACTION_DESC(&spMenuSelectConn, index));

  // Clear non-connection-related message
  Display_clearLine(dispHandle, SP_ROW_CONNECTION);

  tbm_goTo(&spMenuPerConn);

  return (true);
}

/*********************************************************************
 * @fn      SimplePeripheral_addConn
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          if there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_addConn(uint16_t connHandle)
{
  uint8_t i;
  uint8_t status = bleNoResources;

  // Try to find an available entry
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == LINKDB_CONNHANDLE_INVALID)
    {
#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
      spClockEventData_t *paramUpdateEventData;

      // Allocate data to send through clock handler
      paramUpdateEventData = ICall_malloc(sizeof(spClockEventData_t) +
                                          sizeof (uint16_t));
      if(paramUpdateEventData)
      {
        paramUpdateEventData->event = SP_SEND_PARAM_UPDATE_EVT;
        *((uint16_t *)paramUpdateEventData->data) = connHandle;

        // Create a clock object and start
        connList[i].pUpdateClock
          = (Clock_Struct*) ICall_malloc(sizeof(Clock_Struct));

        if (connList[i].pUpdateClock)
        {
          Util_constructClock(connList[i].pUpdateClock,
                              SimplePeripheral_clockHandler,
                              SEND_PARAM_UPDATE_DELAY, 0, true,
                              (UArg) paramUpdateEventData);
        }
      }
      else
      {
        status = bleMemAllocError;
      }
#endif

      // Found available entry to put a new connection info in
      connList[i].connHandle = connHandle;

      // Set default PHY to 1M
      connList[i].currPhy = HCI_PHY_1_MBPS;

      break;
    }
  }

  return status;
}

/*********************************************************************
 * @fn      SimplePeripheral_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_getConnIndex(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      return i;
    }
  }

  return(MAX_NUM_BLE_CONNS);
}

/*********************************************************************
 * @fn      SimplePeripheral_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  SUCCESS if connHandle found valid index or bleInvalidRange
 *          if index wasn't found. LINKDB_CONNHANDLE_ALL will always succeed.
 */
static uint8_t SimplePeripheral_clearConnListEntry(uint16_t connHandle)
{
  uint8_t i;
  // Set to invalid connection index initially
  uint8_t connIndex = MAX_NUM_BLE_CONNS;

  if(connHandle != LINKDB_CONNHANDLE_ALL)
  {
    // Get connection index from handle
    connIndex = SimplePeripheral_getConnIndex(connHandle);
    if(connIndex >= MAX_NUM_BLE_CONNS)
	{
	  return(bleInvalidRange);
	}
  }

  // Clear specific handle or all handles
  for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if((connIndex == i) || (connHandle == LINKDB_CONNHANDLE_ALL))
    {
      connList[i].connHandle = LINKDB_CONNHANDLE_INVALID;
      connList[i].currPhy = 0;
      connList[i].phyCngRq = 0;
      connList[i].phyRqFailCnt = 0;
      connList[i].rqPhy = 0;
      memset(connList[i].rssiArr, 0, SP_MAX_RSSI_STORE_DEPTH);
      connList[i].rssiAvg = 0;
      connList[i].rssiCntr = 0;
      connList[i].isAutoPHYEnable = FALSE;
    }
  }

  return(SUCCESS);
}

/*********************************************************************
 * @fn      SimplePeripheral_removeConn
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_removeConn(uint16_t connHandle)
{
  uint8_t connIndex = SimplePeripheral_getConnIndex(connHandle);

  if(connIndex != MAX_NUM_BLE_CONNS)
  {
    Clock_Struct* pUpdateClock = connList[connIndex].pUpdateClock;

    if (pUpdateClock != NULL)
    {
      // Stop and destruct the RTOS clock if it's still alive
      if (Util_isActive(pUpdateClock))
      {
        Util_stopClock(pUpdateClock);
      }

      // Destruct the clock object
      Clock_destruct(pUpdateClock);
      // Free clock struct
      ICall_free(pUpdateClock);
    }
    // Stop Auto PHY Change
    SimplePeripheral_stopAutoPhyChange(connHandle);
    // Clear Connection List Entry
    SimplePeripheral_clearConnListEntry(connHandle);
  }

  return connIndex;
}

/*********************************************************************
 * @fn      SimplePeripheral_processParamUpdate
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static void SimplePeripheral_processParamUpdate(uint16_t connHandle)
{
  gapUpdateLinkParamReq_t req;
  uint8_t connIndex;

  req.connectionHandle = connHandle;
#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
  req.connLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
  req.connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
  req.intervalMin = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
  req.intervalMax = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
#endif

  connIndex = SimplePeripheral_getConnIndex(connHandle);
  SIMPLEPERIPHERAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Deconstruct the clock object
  Clock_destruct(connList[connIndex].pUpdateClock);
  // Free clock struct
  ICall_free(connList[connIndex].pUpdateClock);
  connList[connIndex].pUpdateClock = NULL;

  // Send parameter update
  bStatus_t status = GAP_UpdateLinkParamReq(&req);

  // If there is an ongoing update, queue this for when the udpate completes
  if (status == bleAlreadyInRequestedMode)
  {
    spConnHandleEntry_t *connHandleEntry = ICall_malloc(sizeof(spConnHandleEntry_t));
    if (connHandleEntry)
    {
      connHandleEntry->connHandle = connHandle;

      List_put(&paramUpdateList, (List_Elem *)&connHandleEntry);
    }
  }
}

/*********************************************************************
 * @fn      SimpleCentral_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimplePeripheral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  uint8_t status = pMsg->pReturnParam[0];

  //Find which command this command complete is for
  switch (pMsg->cmdOpcode)
  {
    case HCI_READ_RSSI:
    {
      // Display RSSI value, if RSSI is higher than threshold, change to faster PHY
      if (status == SUCCESS)
      {
        uint16_t handle = BUILD_UINT16(pMsg->pReturnParam[1], pMsg->pReturnParam[2]);

        uint8_t index = SimplePeripheral_getConnIndex(handle);
        SIMPLEPERIPHERAL_ASSERT(index < MAX_NUM_BLE_CONNS);

        connList[index].rssiArr[connList[index].rssiCntr++] =
                                                  (int8_t)pMsg->pReturnParam[3];
        connList[index].rssiCntr %= SP_MAX_RSSI_STORE_DEPTH;

        int16_t sum_rssi = 0;
        for(uint8_t cnt=0; cnt<SP_MAX_RSSI_STORE_DEPTH; cnt++)
        {
          sum_rssi += connList[index].rssiArr[cnt];
        }
        connList[index].rssiAvg = (uint32_t)(sum_rssi/SP_MAX_RSSI_STORE_DEPTH);

        Display_printf(dispHandle, SP_ROW_RSSI, 0,
                       "RSSI:-%d, AVG RSSI:-%d",
                       (uint32_t)(-(int8_t)pMsg->pReturnParam[3]),
                       (uint32_t)(-sum_rssi/SP_MAX_RSSI_STORE_DEPTH));

        uint8_t phyRq = SP_PHY_NONE;
        uint8_t phyRqS = SP_PHY_NONE;
        uint8_t phyOpt = LL_PHY_OPT_NONE;

        if(connList[index].phyCngRq == FALSE)
        {
          if((connList[index].rssiAvg >= RSSI_2M_THRSHLD) &&
             (connList[index].currPhy != HCI_PHY_2_MBPS) &&
             (connList[index].currPhy != SP_PHY_NONE))
          {
            // try to go to higher data rate
            phyRqS = phyRq = HCI_PHY_2_MBPS;
          }
          else if((connList[index].rssiAvg < RSSI_2M_THRSHLD) &&
                  (connList[index].rssiAvg >= RSSI_1M_THRSHLD) &&
                  (connList[index].currPhy != HCI_PHY_1_MBPS) &&
                  (connList[index].currPhy != SP_PHY_NONE))
          {
            // try to go to legacy regular data rate
            phyRqS = phyRq = HCI_PHY_1_MBPS;
          }
          else if((connList[index].rssiAvg >= RSSI_S2_THRSHLD) &&
                  (connList[index].rssiAvg < RSSI_1M_THRSHLD) &&
                  (connList[index].currPhy != SP_PHY_NONE))
          {
            // try to go to lower data rate S=2(500kb/s)
            phyRqS = HCI_PHY_CODED;
            phyOpt = LL_PHY_OPT_S2;
            phyRq = BLE5_CODED_S2_PHY;
          }
          else if(connList[index].rssiAvg < RSSI_S2_THRSHLD )
          {
            // try to go to lowest data rate S=8(125kb/s)
            phyRqS = HCI_PHY_CODED;
            phyOpt = LL_PHY_OPT_S8;
            phyRq = BLE5_CODED_S8_PHY;
          }
          if((phyRq != SP_PHY_NONE) &&
             // First check if the request for this phy change is already not honored then don't request for change
             (((connList[index].rqPhy == phyRq) &&
               (connList[index].phyRqFailCnt < 2)) ||
              (connList[index].rqPhy != phyRq)))
          {
            //Initiate PHY change based on RSSI
            SimplePeripheral_setPhy(connList[index].connHandle, 0,
                                    phyRqS, phyRqS, phyOpt);
            connList[index].phyCngRq = TRUE;

            // If it a request for different phy than failed request, reset the count
            if(connList[index].rqPhy != phyRq)
            {
              // then reset the request phy counter and requested phy
              connList[index].phyRqFailCnt = 0;
            }

            if(phyOpt == LL_PHY_OPT_NONE)
            {
              connList[index].rqPhy = phyRq;
            }
            else if(phyOpt == LL_PHY_OPT_S2)
            {
              connList[index].rqPhy = BLE5_CODED_S2_PHY;
            }
            else
            {
              connList[index].rqPhy = BLE5_CODED_S8_PHY;
            }

          }
        } // end of if(connList[index].phyCngRq == FALSE)
      } // end of if (status == SUCCESS)
      break;
    }

    case HCI_LE_READ_PHY:
    {
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, SP_ROW_RSSI + 2, 0, "RXPh: %d, TXPh: %d",
                       pMsg->pReturnParam[3], pMsg->pReturnParam[4]);
      }
      break;
    }

    case HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS:
    {
      uint8_t* pRpaNew = &(pMsg->pReturnParam[1]);

      if (memcmp(pRpaNew, rpa, B_ADDR_LEN))
      {
        // If the RPA has changed, update the display
        Display_printf(dispHandle, SP_ROW_RPA, 0, "RP Addr: %s",
                       Util_convertBdAddr2Str(pRpaNew));
        memcpy(rpa, pRpaNew, B_ADDR_LEN);
      }
      break;
    }

    default:
      break;
  } // end of switch (pMsg->cmdOpcode)
}

/*********************************************************************
* @fn      SimplePeripheral_initPHYRSSIArray
*
* @brief   Initializes the array of structure/s to store data related
*          RSSI based auto PHy change
*
* @param   connHandle - the connection handle
*
* @param   addr - pointer to device address
*
* @return  index of connection handle
*/
static void SimplePeripheral_initPHYRSSIArray(void)
{
  //Initialize array to store connection handle and RSSI values
  memset(connList, 0, sizeof(connList));
  for (uint8_t index = 0; index < MAX_NUM_BLE_CONNS; index++)
  {
    connList[index].connHandle = SP_INVALID_HANDLE;
  }
}
/*********************************************************************
      // Set default PHY to 1M
 * @fn      SimplePeripheral_startAutoPhyChange
 *
 * @brief   Start periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 * @param   devAddr - device address
 *
 * @return  SUCCESS: Terminate started
 *          bleIncorrectMode: No link
 *          bleNoResources: No resources
 */
static status_t SimplePeripheral_startAutoPhyChange(uint16_t connHandle)
{
  status_t status = FAILURE;

  // Get connection index from handle
  uint8_t connIndex = SimplePeripheral_getConnIndex(connHandle);
  SIMPLEPERIPHERAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Start Connection Event notice for RSSI calculation
  Gap_RegisterConnEventCb(SimplePeripheral_connEvtCB, GAP_CB_REGISTER, GAP_CB_CONN_EVENT_ALL, connHandle);

  // Flag in connection info if successful
  if (status == SUCCESS)
  {
    connList[connIndex].isAutoPHYEnable = TRUE;
  }

  return status;
}

/*********************************************************************
 * @fn      SimplePeripheral_stopAutoPhyChange
 *
 * @brief   Cancel periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 *
 * @return  SUCCESS: Operation successful
 *          bleIncorrectMode: No link
 */
static status_t SimplePeripheral_stopAutoPhyChange(uint16_t connHandle)
{
  // Get connection index from handle
  uint8_t connIndex = SimplePeripheral_getConnIndex(connHandle);
  SIMPLEPERIPHERAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Stop connection event notice
  Gap_RegisterConnEventCb(NULL, GAP_CB_UNREGISTER, GAP_CB_CONN_EVENT_ALL, connHandle);

  // Also update the phychange request status for active RSSI tracking connection
  connList[connIndex].phyCngRq = FALSE;
  connList[connIndex].isAutoPHYEnable = FALSE;

  return SUCCESS;
}

/*********************************************************************
 * @fn      SimplePeripheral_setPhy
 *
 * @brief   Call the HCI set phy API and and add the handle to a
 *          list to match it to an incoming command status event
 */
static status_t SimplePeripheral_setPhy(uint16_t connHandle, uint8_t allPhys,
                                        uint8_t txPhy, uint8_t rxPhy,
                                        uint16_t phyOpts)
{
  // Allocate list entry to store handle for command status
  spConnHandleEntry_t *connHandleEntry = ICall_malloc(sizeof(spConnHandleEntry_t));

  if (connHandleEntry)
  {
    connHandleEntry->connHandle = connHandle;

    // Add entry to the phy command status list
    List_put(&setPhyCommStatList, (List_Elem *)connHandleEntry);

    // Send PHY Update
    HCI_LE_SetPhyCmd(connHandle, allPhys, txPhy, rxPhy, phyOpts);
  }

  return SUCCESS;
}

/*********************************************************************
* @fn      SimplePeripheral_updatePHYStat
*
* @brief   Update the auto phy update state machine
*
* @param   connHandle - the connection handle
*
* @return  None
*/
static void SimplePeripheral_updatePHYStat(uint16_t eventCode, uint8_t *pMsg)
{
  uint8_t connIndex;

  switch (eventCode)
  {
    case HCI_LE_SET_PHY:
    {
      // Get connection handle from list
      spConnHandleEntry_t *connHandleEntry =
                           (spConnHandleEntry_t *)List_get(&setPhyCommStatList);

      if (connHandleEntry)
      {
        // Get index from connection handle
        connIndex = SimplePeripheral_getConnIndex(connHandleEntry->connHandle);

        ICall_free(connHandleEntry);

        // Is this connection still valid?
        if (connIndex < MAX_NUM_BLE_CONNS)
        {
          hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *)pMsg;

          if (pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
          {
            // Update the phychange request status for active RSSI tracking connection
            connList[connIndex].phyCngRq = FALSE;
            connList[connIndex].phyRqFailCnt++;
          }
        }
      }
      break;
    }

    // LE Event - a Phy update has completed or failed
    case HCI_BLE_PHY_UPDATE_COMPLETE_EVENT:
    {
      hciEvt_BLEPhyUpdateComplete_t *pPUC =
                                     (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

      if(pPUC)
      {
        // Get index from connection handle
        connIndex = SimplePeripheral_getConnIndex(pPUC->connHandle);

        // Is this connection still valid?
        if (connIndex < MAX_NUM_BLE_CONNS)
        {
          // Update the phychange request status for active RSSI tracking connection
          connList[connIndex].phyCngRq = FALSE;

          if (pPUC->status == SUCCESS)
          {
            connList[connIndex].currPhy = pPUC->rxPhy;
          }
          if(pPUC->rxPhy != connList[connIndex].rqPhy)
          {
            connList[connIndex].phyRqFailCnt++;
          }
          else
          {
            // Reset the request phy counter and requested phy
            connList[connIndex].phyRqFailCnt = 0;
            connList[connIndex].rqPhy = 0;
          }
        }
      }

      break;
    }

    default:
      break;
  } // end of switch (eventCode)
}

/*********************************************************************
 * @fn      SimplePeripheral_menuSwitchCb
 *
 * @brief   Detect menu context switching
 *
 * @param   pMenuObjCurr - the current menu object
 * @param   pMenuObjNext - the menu object the context is about to switch to
 *
 * @return  none
 */
static void SimplePeripheral_menuSwitchCb(tbmMenuObj_t* pMenuObjCurr,
                                       tbmMenuObj_t* pMenuObjNext)
{
  uint8_t NUMB_ACTIVE_CONNS = linkDB_NumActive();

  // interested in only the events of
  // entering scMenuConnect, spMenuSelectConn, and scMenuMain for now
  if (pMenuObjNext == &spMenuSelectConn)
  {
    static uint8_t* pAddrs;
    uint8_t* pAddrTemp;

    if (pAddrs != NULL)
    {
      ICall_free(pAddrs);
    }

    // Allocate buffer to display addresses
    pAddrs = ICall_malloc(NUMB_ACTIVE_CONNS * SP_ADDR_STR_SIZE);

    if (pAddrs == NULL)
    {
      TBM_SET_NUM_ITEM(&spMenuSelectConn, 0);
    }
    else
    {
      uint8_t i;

      TBM_SET_NUM_ITEM(&spMenuSelectConn, MAX_NUM_BLE_CONNS);

      pAddrTemp = pAddrs;

      // Add active connection info to the menu object
      for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
      {
        if (connList[i].connHandle != LINKDB_CONNHANDLE_INVALID)
        {
          // Get the address from the connection handle
          linkDBInfo_t linkInfo;
          linkDB_GetInfo(connList[i].connHandle, &linkInfo);
          // This connection is active. Set the corresponding menu item with
          // the address of this connection and enable the item.
          memcpy(pAddrTemp, Util_convertBdAddr2Str(linkInfo.addr),
                 SP_ADDR_STR_SIZE);
          TBM_SET_ACTION_DESC(&spMenuSelectConn, i, pAddrTemp);
          tbm_setItemStatus(&spMenuSelectConn, (1 << i), SP_ITEM_NONE);
          pAddrTemp += SP_ADDR_STR_SIZE;
        }
        else
        {
          // This connection is not active. Disable the corresponding menu item.
          tbm_setItemStatus(&spMenuSelectConn, SP_ITEM_NONE, (1 << i));
        }
      }
    }
  }
  else if (pMenuObjNext == &spMenuMain)
  {
    // Now we are not in a specific connection's context

    // Clear connection-related message
    Display_clearLine(dispHandle, SP_ROW_CONNECTION);
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processOadResetEvt
 *
 * @brief   Process a write request to the OAD reset service
 *
 * @param   resetEvt - The oadResetWrite_t struct containing reset data
 *
 * @return  None.
 */
static void SimplePeripheral_processOadResetEvt(oadResetWrite_t *resetEvt)
{
  /* We cannot reboot the device immediately after receiving
   * the enable command, we must allow the stack enough time
   * to process and responsd to the OAD_EXT_CTRL_ENABLE_IMG
   * command. The current implementation will wait one cxn evt
   */
  // Register for L2CAP Flow Control Events
  L2CAP_RegisterFlowCtrlTask(selfEntity);

  resetConnHandle = resetEvt->connHandle;

  uint8_t status = FLASH_FAILURE;
  //read the image validation bytes and set it appropriately.
  imgHdr_t imgHdr = {0};
  if(flash_open())
  {
    status = readFlash(0x0, (uint8_t *)&imgHdr, OAD_IMG_HDR_LEN);
  }

  if ((FLASH_SUCCESS == status) && ( imgHdr.fixedHdr.imgVld != 0))
  {
    if ( OAD_evenBitCount(imgHdr.fixedHdr.imgVld) )
    {
      imgHdr.fixedHdr.imgVld = imgHdr.fixedHdr.imgVld << 1;
      writeFlash((uint32_t)FLASH_ADDRESS(0, IMG_VALIDATION_OFFSET),
                 (uint8_t *)&(imgHdr.fixedHdr.imgVld), sizeof(imgHdr.fixedHdr.imgVld));
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processOadResetWriteCB
 *
 * @brief   Process a write request to the OAD reset service
 *
 * @param   connHandle - the connection Handle this request is from.
 * @param   bim_var    - bim_var to set before resetting.
 *
 * @return  None.
 */
void SimplePeripheral_processOadResetWriteCB(uint16_t connHandle,
                                      uint16_t bim_var)
{
    // Allocate memory for OAD EVT payload, the app task must free this later
    oadResetWrite_t *oadResetWriteEvt = ICall_malloc(sizeof(oadResetWrite_t));

    oadResetWriteEvt->connHandle = connHandle;
    oadResetWriteEvt->bim_var = bim_var;

    // This function will enqueue the messsage and wake the application
    SimplePeripheral_enqueueMsg(SP_OAD_RESET_EVT, (uint8_t *)oadResetWriteEvt);
}

/*********************************************************************
 * @fn      SimplePeripheral_doAutoConnect
 *
 * @brief   Enable/Disable peripheral as AutoConnect node.
 *
 * @param   index - 0 : Disable AutoConnect
 *                  1 : Enable Group A
 *                  2 : Enable Group B
 *
 * @return  always true
 */
bool SimplePeripheral_doAutoConnect(uint8_t index)
{
    if (index == 1)
    {
      if (autoConnect != AUTOCONNECT_GROUP_A)
      {
        GapAdv_disable(advHandleLongRange);
        GapAdv_disable(advHandleLegacy);
        advData1[2] = 'G';
        advData1[3] = 'A';
        advData2[2] = 'G';
        advData2[3] = 'A';
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        autoConnect = AUTOCONNECT_GROUP_A;
      }
    }
    else if (index == 2)
    {
      if (autoConnect != AUTOCONNECT_GROUP_B)
      {
        GapAdv_disable(advHandleLongRange);
        GapAdv_disable(advHandleLegacy);
        advData1[2] = 'G';
        advData1[3] = 'B';
        advData2[2] = 'G';
        advData2[3] = 'B';
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        autoConnect = AUTOCONNECT_GROUP_B;
      }
    }
    else
    {
      if (autoConnect)
      {
        GapAdv_disable(advHandleLongRange);
        GapAdv_disable(advHandleLegacy);
        advData1[2] = 'S';
        advData1[3] = 'P';
        advData2[2] = 'S';
        advData2[3] = 'P';
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        autoConnect = AUTOCONNECT_DISABLE;
      }
    }
    //tbm_goTo(&spMenuMain);

    return (true);
}


/*********************************************************************
*********************************************************************/


//System Integrity Check
void PreStartupSelfCheck(void)
{
    uint8_t txBuff[2];
    uint8_t rxBuff[2];
    I2C_Transaction i2cPingTransaction;
    unsigned int state; //Hall Effect Sensor output pin state

    //Configure input pin (Hall Effect Sensor)
    GPIO_setConfig(CAP_HALL, GPIO_CFG_INPUT);

    i2cPingTransaction.writeBuf   = txBuff;
    i2cPingTransaction.writeCount = 1;
    i2cPingTransaction.readBuf    = rxBuff;
    i2cPingTransaction.readCount  = 0;

    /************* SENSOR Detection ****************/
    i2cPingTransaction.targetAddress = SENSOR_ADDR;

    /* Ping sensor */
    if (I2C_transfer(i2c, &i2cPingTransaction)) SelfCheck.sensor_present = true;
    else SelfCheck.sensor_present = false;
    /***********************************************/

    /************* EEPROM Detection ****************/
    i2cPingTransaction.targetAddress = EEPROM_ADD;

    /* Ping sensor */
    if (I2C_transfer(i2c, &i2cPingTransaction)) SelfCheck.eeprom_present = true;
    else SelfCheck.eeprom_present = false;
    /***********************************************/

    /************* Fuel Gauge Detection ************/
    i2cPingTransaction.targetAddress = BQ27427_I2C_ADDRESS;

    /* Ping sensor */
    if (I2C_transfer(i2c, &i2cPingTransaction)) SelfCheck.fuel_gauge = true;
    else SelfCheck.fuel_gauge = false;
    /***********************************************/

    /************ Hall effect sensor ***************/
    //Read Sensor output pin
    state = GPIO_read(CAP_HALL);

    if(state == 0) SelfCheck.hall_effect_sensor_out = true;
    else SelfCheck.hall_effect_sensor_out = false;
    /***********************************************/

    /*************** Set Flags ********************/
    /*
     *  Byte Flags Explanation:
     *
     *  7  6  5  4  3  2  1  0
     *  |  |  |  |  |  |  |  |
     *  |  |  |  |  |  |  |  Bit 0: (LSB) Sensor Presence (1 : Sensor Detected, 0 : Sensor not detected)
     *  |  |  |  |  |  |  Bit 1: EEPROM Presence (1 : EEPROM Detected, 0 : EEPROM not detected)
     *  |  |  |  |  |  Bit 2: Fuel Gauge Presence (1 : Fuel Gauge Detected, 0 : Fuel Gauge not detected)
     *  |  |  |  |  Bit 3: Hall Effect Sensor (1 : Magnetic Field Detected, 0 : No Magnetic Field detected)
     *  |  |  |  Bit 4 : Self Check State (1 : Self Check is ON, 0 : Self Check is OFF)
     *  |  |  Bit 5
     *  |  Bit 6
     *  Bit 7: (MSB)
     *
     */
    SelfCheckFlags = ((uint8_t) (SelfC_state)<<4) | ((uint8_t) (SelfCheck.hall_effect_sensor_out)<<3) | ((uint8_t) (SelfCheck.fuel_gauge)<<2) | ((uint8_t) (SelfCheck.eeprom_present)<<1) | (uint8_t) (SelfCheck.sensor_present);
    /**********************************************/

    /******** Collect Software V and Serial N ******/
    if(SelfCheck.eeprom_present) //If EEPROM is present
    {
        //Read Software version + Product Serial Number (for BLE CHAR)
        EEPROM_Read(SW_VERSION_ADD, sw_v, 3);
        EEPROM_Read(SN_ADD, sn, 2);
    }

    /**********************************************/


    /********* Collect Fuel Gauge Data *********/
    if(SelfCheck.fuel_gauge) //If Fuel Gauge is present
    {
        CellVoltage_s = BQ27427_voltage();
        CellCurrent_s = BQ27427_current(AVG) + 3000; //Offset : 3000
        CellSoC_s = BQ27427_soc(FILTERED);
    }
    /*******************************************/

}

void Set_HallCAP_INT(void)
{
    //Configure Interrupt
    GPIO_setConfig(CAP_HALL, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setCallback(CAP_HALL, MAGFieldDetFxn);
    GPIO_enableInt(CAP_HALL);
}
