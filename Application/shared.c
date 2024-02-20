/*
 * shared.c
 *
 *  Created on: 17 juil. 2023
 *      Author: gaeta
 */

#include "shared.h"
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/display/Display.h>

Display_Handle dispHandle;
SemaphoreP_Handle sem_display;
SemaphoreP_Handle sem_sc_thread;
SemaphoreP_Handle sem_sc_event;
SemaphoreP_Handle sem_i2c_thread;
SemaphoreP_Handle sem_fdcan_thread;
SemaphoreP_Handle sem_reboot_thread;

MLCconfig_t mlc_config = ONE_FEATURE;

SelfCheck_t SelfCheck = {
     .fuel_gauge = false,
     .eeprom_present = false,
     .hall_effect_sensor_out = false,
     .sensor_present = false,
     .self_check_state = true,
};

void init_display(void)
{
    dispHandle = NULL;
    Display_init();
    //dispHandle = Display_open(Display_Type_UART, NULL);
    dispHandle = Display_open(Display_Type_UART, NULL);
    //Display_printf(dispHandle, 0, 0, "Display Initialized ! \n");
}

/* Create display semaphore */
void init_semaphore(void)
{
    // Init semaphore for display
    sem_display = SemaphoreP_createBinary(1);

    // Init semaphore sensor controller
    sem_magn = SemaphoreP_createBinary(0);
    sem_sc_event = SemaphoreP_createBinary(0); //blocked
    sem_sc_thread = SemaphoreP_createBinary(0); //blocked initially

    //Init semaphore I2C
    sem_i2c_thread = SemaphoreP_createBinary(0); //Block the I2C task initially

    //Init semaphore FDCAN
    sem_fdcan_thread = SemaphoreP_createBinary(0); //Block the FDCAN task initially

    //Init semaphore REBOOT
    sem_reboot_thread = SemaphoreP_createBinary(0); //Block the REBOOT task initially
}
