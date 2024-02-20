/*
 * nwt_sc.c
 *
 *  Created on: 21 juil. 2023
 *      Author: gaeta
 */

#include "scif.h"
/* macro, define a function which set the desired bit to one
 * ex : BV(3) = 0b00001000
*/
#define BV(x)    (1 << (x))


#include <ti/drivers/dpl/SemaphoreP.h>
#include "shared.h"
#include "nwt_sc.h"
#include "nwt_i2c.h"
#include <icall_ble_api.h>

//#include "ti_ble_gatt_service.h"

/* TI-RTOS header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Event.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

#include "hal_types.h"

/* Task configuration */
#define SC_TASK_STACK_SIZE                   2048
#define SC_TASK_PRIORITY                     3
uint8_t scTaskStack[SC_TASK_STACK_SIZE];
Task_Struct scTask;

/* Application event object */
static Event_Struct appEventObj;
static Event_Handle appEventHandle;

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

/* Semaphore to deals with Sensor controller and block other thread */
SemaphoreP_Handle sem_magn;
SemaphoreP_Handle sem_sc_thread;
SemaphoreP_Handle sem_sc_event;

/* Var for BLE advertising */
uint8 advHandleLegacy;
uint8 advHandleLongRange;

//Sensor current Power Mode (LP or ULP)
Sensor_PowerMode sensorCurrentPM = LOW_POWER;

//Debug Mode current Power Mode (LP or ULP)
Sensor_PowerMode DebugCurrentPM = LOW_POWER;


//ULP Mode Activation
#ifdef FINAL_VERSION
uint8_t FlagActivationULP = 1; //Activated At Start
#else
uint8_t FlagActivationULP = 0; //Deactivated At Start
#endif

//ULP Jitter Flag
uint8_t FlagULP_j = 1;

//WakeUp counter
uint32_t cptWU = 0;

//I2C target address
static uint8_t targetAddress;

#include "nwt_sc.h"

void scCtrlReadyCallback(void)
{

} // scCtrlReadyCallback

void scTaskAlertCallback(void)
{
    SemaphoreP_post(sem_sc_event);
} // scTaskAlertCallback



void processTaskAlert(void)
{
    // Clear the ALERT interrupt source
    scifClearAlertIntSource();

    // Do SC Task processing here
    //cptF1 = scifTaskData.count.output.F1Lsb;
    //cptON = scifTaskData.count.output.OnLsb;
    //uint8_t cptF2 = scifTaskData.count.output.TotalF2;
    //Display_printf(dispHandle, 0, 0, "   CPT SC ON = %u \n", cptOn);
    //Display_printf(dispHandle, 0, 0, "   CPT SC F1 = %u \n", cptF1);
    //Display_printf(dispHandle, 0, 0, "   CPT SC F2 = %u \n", cptF2);

    // Acknowledge the ALERT event
    scifAckAlertEvents();
} // processTaskAlert

/*
 *  ======== MAGFieldDetFxn ========
 *  Callback function for the GPIO interrupt on CAP_HALL
 */
void MAGFieldDetFxn(uint_least8_t index)
{
    //Magnetic Field Detected ---> Enable advertising & Start counter in REBOOT thread
    SemaphoreP_post(sem_magn);
    SemaphoreP_post(sem_reboot_thread);


}

void SC_disable_advertising(void)
{
    // Stop advertising
    GapAdv_disable(advHandleLongRange);
    GapAdv_disable(advHandleLegacy);
}

void SC_enable_advertising(void)
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

void write_data_ble(void)
{

    uint8_t valueToCopy;
    valueToCopy = 5;
    //SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t),
                                   //&valueToCopy);
    /*simpleProfile_SetParameter(1, sizeof(uint8_t),
                                       &valueToCopy);
    simpleProfile_SetParameter(2, sizeof(uint8_t),
                                       &valueToCopy);*/
}

/*********************************************************************
 * @fn      SC_taskFxn
 *
 * @brief   Application task entry point for SC.
 *
 * @param   a0, a1 - not used.
 */
void SC_taskFxn(UArg a0, UArg a1)
{

    //Wait for semaphore at the start
    SemaphoreP_pend(sem_sc_thread, BIOS_WAIT_FOREVER); //Block task here until Self-Check Test is over

    /* Initialize TI-RTOS objects */
    Event_construct(&appEventObj, NULL);
    appEventHandle = Event_handle(&appEventObj);

    /* Initialize the Sensor Controller
     * OSAL = OS Abstraction layer, layer to interface SC with different OS
     */
    scifOsalInit();
    //scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
    scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);
    scifInit(&scifDriverSetup);

    // Start Sensor Controller task
    scifStartTasksNbl(BV(SCIF_COUNT_TASK_ID));

    // ******************************************************************
    // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
    // ******************************************************************
    // Register the current thread as an ICall dispatcher application
    // so that the application can send and receive messages.
    ICall_registerApp(&selfEntity, &syncEvent);
    //---------------------------------------------------------------

    while (1) {

        // Wait for event to occur
        SemaphoreP_pend(sem_sc_event, BIOS_WAIT_FOREVER);

        // Call process function
        processTaskAlert();

        /*************** Switch POWER MODE on wake-up ******************/
        sensorCurrentPM = I2C_GET_SENSOR_ODR(); //Read ODR to determine power mode

        switch(sensorCurrentPM)
        {
            case LOW_POWER :
                //Switch to ULP Mode
                if(FlagActivationULP == 1) //Only if ULP mode Activation is ON
                {
                    I2C_MLC_FSM_ULP_MODE();
                    sensorCurrentPM = ULTRA_LOW_POWER;
                }
                break;
            case ULTRA_LOW_POWER :
                    //Switch to LP Mode
                    I2C_MLC_Features(); /* Batching in FIFO buffer of MLC features */
                    I2C_MLC_FSM_LP_MODE();
                    sensorCurrentPM = LOW_POWER;
                    //Increment WakeUp counter each time we transition from ULP to LP mode
                    cptWU++;
                break;
            case ERROR_S :
                //ToDo : Add something to do when an error has been detected
                break;
            default :
                break;
        }
        /***************************************************************/


        //Set Debug Current Power Mode variable
        if(DebugCurrentPM == LOW_POWER) DebugCurrentPM = ULTRA_LOW_POWER;
        else DebugCurrentPM = LOW_POWER;

    }
}

/*********************************************************************
 * @fn      SC_createTask
 *
 * @brief   Task creation function for Sensor Controller.
 */
void SC_createTask(void)
{
    Task_Params taskParams;

  /* Configure task */
  Task_Params_init(&taskParams);
  taskParams.stack = scTaskStack;
  taskParams.stackSize = SC_TASK_STACK_SIZE;
  taskParams.priority = SC_TASK_PRIORITY;

  Task_construct(&scTask, SC_taskFxn, &taskParams, NULL);
}

