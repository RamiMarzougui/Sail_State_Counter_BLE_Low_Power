/*
 * nwt_reboot.c
 *
 *  Created on: 26 jan. 2024
 *      Author: rami
 */

#include <ti/drivers/dpl/SemaphoreP.h>
#include "shared.h"
#include "nwt_sc.h"
#include "nwt_i2c.h"
#include <icall_ble_api.h>

/* TI-RTOS header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Event.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

#include "hal_types.h"

#include <driverlib/sys_ctrl.h>

/* Task configuration */
#define REBOOT_Task_Period 200 * 100 //200 ms
#define REBOOT_TASK_STACK_SIZE                   2048
#define REBOOT_TASK_PRIORITY                     1
#define HARD_REBOOT_DURATION 25
uint8_t rebootTaskStack[REBOOT_TASK_STACK_SIZE];
Task_Struct rebootTask;

/* Magnet Presence Counter */
uint32_t mag_counter = 0;

/* Application event object */
static Event_Struct appEventObj;
static Event_Handle appEventHandle;

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;


/*********************************************************************
 * @fn      REBOOT_taskFxn
 *
 * @brief   Application task entry point for REBOOT.
 *
 * @param   a0, a1 - not used.
 */
void REBOOT_taskFxn(UArg a0, UArg a1)
{
    uint32_t pinState;

    int16_t CellCurrent_t;

    while (1) {

        //Wait for semaphore forever
        SemaphoreP_pend(sem_reboot_thread, BIOS_WAIT_FOREVER); //Block task here until magnet is detected

        if(mag_counter>HARD_REBOOT_DURATION) //5 seconds
        {
            mag_counter = 0; //Reset counter
            //RESET
            SysCtrlSystemReset();
        }
        else
        {
            //Read MagnDet pin state
            pinState = GPIO_read(CAP_HALL);

            //Read current value
            //CellCurrent_t = BQ27427_current(AVG);

            //ABS
            //if(CellCurrent_t<0) CellCurrent_t = CellCurrent_t * (-1);

            if (pinState == 0) { //&& (CellCurrent_t < 10)
                //Pin is low --> Magnet is still present : increment counter and keep checking
                mag_counter++;
                SemaphoreP_post(sem_reboot_thread); //Post semaphore to keep checking pin state
            }
            else
            {
                mag_counter = 0; //Reset counter and block task
            }
        }

        Task_sleep(REBOOT_Task_Period);
    }
}

/*********************************************************************
 * @fn      REBOOT_createTask
 *
 * @brief   Task creation function for HARD REBOOT
 */
void REBOOT_createTask(void)
{
  Task_Params taskParams;

  /* Configure task */
  Task_Params_init(&taskParams);
  taskParams.stack = rebootTaskStack;
  taskParams.stackSize = REBOOT_TASK_STACK_SIZE;
  taskParams.priority = REBOOT_TASK_PRIORITY;

  Task_construct(&rebootTask, REBOOT_taskFxn, &taskParams, NULL);
}

