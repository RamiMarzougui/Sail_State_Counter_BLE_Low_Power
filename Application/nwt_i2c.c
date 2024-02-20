/*
 * Copyright (c) 2018-2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== i2ctmp.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <ti/sysbios/knl/Task.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/display/Display.h>
#include <ti/sysbios/BIOS.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* to deal with semaphore*/
#include "nwt_i2c.h"
#include "led_debug.h"
#include "shared.h"
#include <ti/drivers/dpl/SemaphoreP.h>
#include "unico.h"


/* Task configuration */
#define I2C_TASK_STACK_SIZE                   2048
#define I2C_TASK_PRIORITY                     2
uint8_t i2cTaskStack[I2C_TASK_STACK_SIZE];
Task_Struct i2cTask;

// Init Display Interface
extern SemaphoreP_Handle sem_display;
extern Display_Handle dispHandle;

float acc = 0;

static struct Acc acc_struct;

static uint8_t targetAddress;
//static Display_Handle display;

static void i2cErrorHandler(I2C_Transaction *transaction, Display_Handle dispHandle);

/* Global Var I2C */
uint8_t txBuffer[8];
uint8_t rxBuffer[8];
I2C_Transaction i2cTransaction;

SensorDATA_t SensorData;

//Variance_Z moving average calculation
static uint8_t first_value_Z = 1;
static uint8_t first_value_V = 1;
float variance_Z_values[MOV_AVG_SIZE] = {0.0};
float variance_V_values[MOV_AVG_SIZE] = {0.0};
float moving_avg_var_z = 0.0;
float moving_avg_var_v = 0.0;
float var_z_sum = 0.0;
float var_v_sum = 0.0;

bool i2c_error_flag = false;

/* Init Var I2C */

/*********************************************************************
 * @fn      I2C_taskFxn
 *
 * @brief   Application task entry point for I2C.
 *
 * @param   a0, a1 - not used.
 */
void I2C_taskFxn(UArg a0, UArg a1)
{


#ifdef FINAL_VERSION
    //Wait for semaphore at the start
    SemaphoreP_pend(sem_i2c_thread, BIOS_WAIT_FOREVER); //Block task here until Self-Check Test is done
#endif

    /* Call driver init functions */
    Display_init();

    Display_printf(dispHandle, 0, 0, "   Display Initialized ! \n");

    /* Ping I2C transaction setup */
    i2cTransaction.writeBuf   = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf    = rxBuffer;
    i2cTransaction.readCount  = 0;
    i2cTransaction.targetAddress = SENSOR_ADDR;

    /* Ping sensor */
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        targetAddress = SENSOR_ADDR;
    }

    /* If we never assigned a target address */
    if (targetAddress == 0)
    {
        I2C_close(i2c);
        while (1) {}
    }

    i2cTransaction.targetAddress = targetAddress;

    /* Batching in FIFO buffer of MLC features */
    I2C_MLC_Features();

    /* Config Sensor + Init MLC and FSM */
    I2C_MLC_FSM_LP_MODE();

    /* Read MLC Thresholds */
    read_MLC_trehshold();

    //Release Sensor Controller Task
    SemaphoreP_post(sem_sc_thread);

    while(1)
    {
        #ifdef FINAL_VERSION
            SemaphoreP_pend(sem_i2c_thread, BIOS_WAIT_FOREVER);
        #endif
        /*********************** Check if MLC needs to be re-configured **************/
         if(reconfig_mlc_flag == 1)
         {
             //Re-configure MLC
             /* Batching in FIFO buffer of MLC features */
             I2C_MLC_Features();

             /* Config Sensor with new threshold + Init MLC and FSM */
             I2C_MLC_FSM_LP_MODE();

             /* Read MLC Thresholds */
             read_MLC_trehshold(); //Read threshold for verification + CAN transfer

             reconfig_mlc_flag = 0; //Reset flag
         }
        /*****************************************************************************/
         if(sensorCurrentPM == LOW_POWER)
         {
             //Read Linear Accelerations
             I2C_read_acc();
             //Read FSM and MLC Status
             I2C_read_fsm();
             //Read Feature
             I2C_read_feature();
             //Read WAKE UP Threshold
             I2C_read_wake_up_thr();
         }
        /************************************************/
        Task_sleep(3333); //30Hz
    }


    I2C_close(i2c);
    Display_printf(dispHandle, 0, 0, "I2C closed!");

}

/*
 *  ======== i2cErrorHandler ========
 */
static void i2cErrorHandler(I2C_Transaction *transaction, Display_Handle sem_display)
{
    switch (transaction->status)
    {
        case I2C_STATUS_TIMEOUT:
            //SemaphoreP_pend(sem_display,SemaphoreP_WAIT_FOREVER);
            Display_printf(dispHandle, 0, 0, "I2C transaction timed out!");
            //SemaphoreP_post(sem_display);
            break;
        case I2C_STATUS_CLOCK_TIMEOUT:
            //SemaphoreP_pend(sem_display,SemaphoreP_WAIT_FOREVER);
            Display_printf(dispHandle, 0, 0, "I2C serial clock line timed out!");
            //SemaphoreP_post(sem_display);
            break;
        case I2C_STATUS_ADDR_NACK:
            //SemaphoreP_pend(sem_display,SemaphoreP_WAIT_FOREVER);
            Display_printf(dispHandle,
                           0,
                           0,
                           "I2C target address 0x%x not"
                           " acknowledged!",
                           transaction->targetAddress);
            //SemaphoreP_post(sem_display);
            break;
        case I2C_STATUS_DATA_NACK:
            //SemaphoreP_pend(sem_display,SemaphoreP_WAIT_FOREVER);
            Display_printf(dispHandle, 0, 0, "I2C data byte not acknowledged!");
            //SemaphoreP_post(sem_display);
            break;
        case I2C_STATUS_ARB_LOST:
            //SemaphoreP_pend(sem_display,SemaphoreP_WAIT_FOREVER);
            Display_printf(dispHandle, 0, 0, "I2C arbitration to another controller!");
            //SemaphoreP_post(sem_display);
            break;
        case I2C_STATUS_INCOMPLETE:
            //SemaphoreP_pend(sem_display,SemaphoreP_WAIT_FOREVER);
            Display_printf(dispHandle, 0, 0, "I2C transaction returned before completion!");
            //SemaphoreP_post(sem_display);
            break;
        case I2C_STATUS_BUS_BUSY:
            //SemaphoreP_pend(sem_display,SemaphoreP_WAIT_FOREVER);
            Display_printf(dispHandle, 0, 0, "I2C bus is already in use!");
            //SemaphoreP_post(sem_display);
            break;
        case I2C_STATUS_CANCEL:
            //SemaphoreP_pend(sem_display,SemaphoreP_WAIT_FOREVER);
            Display_printf(dispHandle, 0, 0, "I2C transaction cancelled!");
            //SemaphoreP_post(sem_display);
            break;
        case I2C_STATUS_INVALID_TRANS:
            //SemaphoreP_pend(sem_display,SemaphoreP_WAIT_FOREVER);
            Display_printf(dispHandle, 0, 0, "I2C transaction invalid!");
            //SemaphoreP_post(sem_display);
            break;
        case I2C_STATUS_ERROR:
            //SemaphoreP_pend(sem_display,SemaphoreP_WAIT_FOREVER);
            Display_printf(dispHandle, 0, 0, "I2C generic error!");
            //SemaphoreP_post(sem_display);
            break;
        default:
            //SemaphoreP_pend(sem_display,SemaphoreP_WAIT_FOREVER);
            Display_printf(dispHandle, 0, 0, "I2C undefined error case!");
            //SemaphoreP_post(sem_display);
            break;
    }
}


/*********************************************************************
 * @fn      I2C_read_feature
 *
 * @brief   Read mlc feature
 */
void I2C_read_feature(void)
{
    uint8_t TAG_sensor;
    uint16_t Identifier;
    uint16_t Value;

    /* Read sensor data's */
    i2cTransaction.readCount = 1;
    i2cTransaction.writeCount = 1;
    txBuffer[0] = FIFO_DATA_OUT_TAG;

    if (I2C_transfer(i2c, &i2cTransaction))
    {
        TAG_sensor = rxBuffer[0] >> 3; //TAG_SENSOR[4:0]
    }
    else
    {
        i2cErrorHandler(&i2cTransaction, sem_display);
    }

    //Check if TAG_SENSOR == MLC feature
    if(TAG_sensor == MLC_feature)
    {
        //Read FIFO Data bytes
        i2cTransaction.readCount = 6;
        i2cTransaction.writeCount = 1;
        txBuffer[0] = FIFO_DATA_OUT_BYTE_0;


        if (I2C_transfer(i2c, &i2cTransaction))
        {
            Identifier = ((uint16_t) rxBuffer[3] << 8) | ((uint16_t) rxBuffer[2]) ;
        }
        else
        {
            i2cErrorHandler(&i2cTransaction, sem_display);
        }
    }

    if((Identifier == VARIANCE_ACC_Z_1F_ID) || (Identifier == VARIANCE_ACC_Z_2F_ID)) //Z-Axis Acceleration Variance
    {
        Value = ((uint16_t) rxBuffer[1] << 8) | ((uint16_t) rxBuffer[0]);
        //Store Data for CAN transfer
        SensorData.Feature_1 = Value;

        //Store value in array for moving average calculation
        if(first_value_Z == 1)
        {
            //Fill the whole array with the received value
            for(uint8_t i = 0; i < MOV_AVG_SIZE; i++) variance_Z_values[i] = uint16_to_half(Value);
            first_value_Z = 0;
        }
        else
        {
            //Write the new value at the end of the array and delete the oldest one
            for(uint8_t i=0; i < MOV_AVG_SIZE-1; i++) variance_Z_values[i] = variance_Z_values[i+1];
            variance_Z_values[MOV_AVG_SIZE-1] = uint16_to_half(Value);
        }

        //Calculate the moving average
        var_z_sum = 0.0;
        for(uint8_t i=0; i < MOV_AVG_SIZE; i++) var_z_sum += variance_Z_values[i];
        moving_avg_var_z = (float) var_z_sum/MOV_AVG_SIZE;
        SensorData.MOVING_AVG_Z = half_to_uint16(moving_avg_var_z); //Convert to uint16 and store for CAN transfer

    }
    else if(Identifier == VARIANCE_ACC_V_ID)
    {
        Value = ((uint16_t) rxBuffer[1] << 8) | ((uint16_t) rxBuffer[0]);
        //Store Data for CAN transfer
        SensorData.Feature_2 = Value;

        //Store value in array for moving average calculation
        if(first_value_V == 1)
        {
            //Fill the whole array with the received value
            for(uint8_t i = 0; i < MOV_AVG_SIZE; i++) variance_V_values[i] = uint16_to_half(Value);
            first_value_V = 0;
        }
        else
        {
            //Write the new value at the end of the array and delete the oldest one
            for(uint8_t i=0; i < MOV_AVG_SIZE-1; i++) variance_V_values[i] = variance_V_values[i+1];
            variance_V_values[MOV_AVG_SIZE-1] = uint16_to_half(Value);
        }

        //Calculate the moving average
        var_v_sum = 0.0;
        for(uint8_t i=0; i < MOV_AVG_SIZE; i++) var_v_sum += variance_V_values[i];
        moving_avg_var_v = (float) var_v_sum/MOV_AVG_SIZE;
        SensorData.MOVING_AVG_V = half_to_uint16(moving_avg_var_v); //Convert to uint16 and store for CAN transfer

    }


}


/*
 * @fn      I2C_read_wake_up_thr
 *
 * @brief   Read WAKE UP threshold
 */
void I2C_read_wake_up_thr(void)
{

    //FUNC_CFG_ACCESS : MAIN PAGE
    txBuffer[0] = 0x01;
    txBuffer[1] = 0x00;

    i2cTransaction.readCount = 0;
    i2cTransaction.writeCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction) == false)
    {
        return;
    }

    //Read Wake up Threshold
    i2cTransaction.readCount = 1;
    i2cTransaction.writeCount = 1;

    txBuffer[0] = WAKE_UP_THS;

    if (I2C_transfer(i2c, &i2cTransaction))
    {
        SensorData.WAKE_UP_Threshold = rxBuffer[0]; //0x5B register
    }
    else
    {
        i2cErrorHandler(&i2cTransaction, sem_display);
    }

    //FUNC_CFG_ACCESS : MAIN PAGE
    txBuffer[0] = 0x01;
    txBuffer[1] = 0x00;

    i2cTransaction.readCount = 0;
    i2cTransaction.writeCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction) == false)
    {
        return;
    }

}

/**********************************************************************
 * @fn        I2C_MLC_Features
 *
 * @brief     Read MLC Features values
 */

void I2C_MLC_Features(void)
{
    i2cTransaction.readCount = 0;
    i2cTransaction.writeCount = 2;
    txBuffer[0] = 0x01;
    txBuffer[1] = 0x04; //Global Reset

    if (I2C_transfer(i2c, &i2cTransaction) == false)
    {
        I2C_close(i2c);
    }

    delay(500);

    for (int i = 0; i < 41; i++) {
       txBuffer[0] = read_mlc_features[i].address;
       txBuffer[1] = read_mlc_features[i].data;

       i2cTransaction.readCount = 0;
       i2cTransaction.writeCount = 2;

       /* Send Sensor configuration */
       if (I2C_transfer(i2c, &i2cTransaction) == false)
       {
           i2c_error_flag = true;
           //I2C_close(i2c);
           break;
       }


    }
}

/*********************************************************************
 * @fn      I2C_createTask
 *
 * @brief   Task creation function for I2C.
 */
void I2C_createTask(void)
{
  Task_Params taskParams;

  /* Configure task */
  Task_Params_init(&taskParams);
  taskParams.stack = i2cTaskStack;
  taskParams.stackSize = I2C_TASK_STACK_SIZE;
  taskParams.priority = I2C_TASK_PRIORITY;

  Task_construct(&i2cTask, I2C_taskFxn, &taskParams, NULL);
  //Task_construct(&i2cTask, I2C_debug, &taskParams, NULL);
}

/*********************************************************************
 * @fn      I2C_read_acc
 *
 * @brief   Read the XL
 */
void I2C_read_acc(void)
{
    /* Read sensor data's */
    i2cTransaction.readCount = 2;
    i2cTransaction.writeCount = 1;
    txBuffer[0] = ACC_Z_LOW_REG; //Z-Axis
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        acc_struct.Acc_Z = (int16_t) ((rxBuffer[1] << 8) | (rxBuffer[0]));
        acc = acc_struct.Acc_Z*0.061/1000;
        //Store Data for CAN transfer
        SensorData.OUTZ_A_v = (rxBuffer[1] << 8) | (rxBuffer[0]);
    }
    else
    {
        i2cErrorHandler(&i2cTransaction, sem_display);
    }


    //Y-Axis
    txBuffer[0] = OUTY_A; //Y-Axis
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        //Store Data for CAN transfer
        SensorData.OUTY_A_v = (rxBuffer[1] << 8) | (rxBuffer[0]);
    }
    else
    {
        i2cErrorHandler(&i2cTransaction, sem_display);
    }

    //X-Axis
    txBuffer[0] = OUTX_A; //X-Axis
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        //Store Data for CAN transfer
        SensorData.OUTX_A_v = (rxBuffer[1] << 8) | (rxBuffer[0]);
    }
    else
    {
        i2cErrorHandler(&i2cTransaction, sem_display);
    }
}


/*********************************************************************
 * @fn      read_MLC_trehshold
 *
 * @brief   Read the MLC thresholds (OFF threshold + ON/F1 threshold)
 */

void read_MLC_trehshold(void)
{
    uint8_t OFF_threshold_LSB;
    uint8_t OFF_threshold_MSB;
    uint8_t ON_F1_threshold_LSB;
    uint8_t ON_F1_threshold_MSB;

    uint8_t F1_threshold_LSB;
    uint8_t F1_threshold_MSB;
    uint8_t ON_OFF_threshold_LSB;
    uint8_t ON_OFF_threshold_MSB;

    //FUNC_CFG_ACCESS
    txBuffer[0] = 0x01;
    txBuffer[1] = 0x00;

    i2cTransaction.readCount = 0;
    i2cTransaction.writeCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction) == false)
    {
        return;
    }

    //FUNC_CFG_ACCESS : Enables access to the embedded functions
    txBuffer[0] = 0x01;
    txBuffer[1] = 0x80;
    if (I2C_transfer(i2c, &i2cTransaction) == false)
    {
        return;
    }

    //R/W Page
    txBuffer[0] = 0x17;
    txBuffer[1] = 0x20; //Read Page
    if (I2C_transfer(i2c, &i2cTransaction) == false)
    {
        return;
    }

    //PAGE SEL
    txBuffer[0] = 0x02;
    txBuffer[1] = 0x31;
    if (I2C_transfer(i2c, &i2cTransaction) == false)
    {
        return;
    }

    if(mlc_config == ONE_FEATURE)
    {
        /**************** OFF THR : READ LSB ****************/
        //Page Address
        txBuffer[0] = 0x08;
        txBuffer[1] = 0x7E; //Address : OFF threshold LSB
        /* Send Sensor configuration */
        if (I2C_transfer(i2c, &i2cTransaction) == false)
        {
            return;
        }

        //Read threshold
        i2cTransaction.readCount = 1;
        i2cTransaction.writeCount = 1;
        txBuffer[0] = 0x09;
        /* Read LSB */
        if (I2C_transfer(i2c, &i2cTransaction) == false)
        {
            return;
        }

        OFF_threshold_LSB = rxBuffer[0];
        /***************************************************/

        /**************** OFF THR : READ MSB **************/
        txBuffer[0] = 0x08;
        txBuffer[1] = 0x7F; //Address : OFF threshold MSB

        i2cTransaction.readCount = 0;
        i2cTransaction.writeCount = 2;
        if (I2C_transfer(i2c, &i2cTransaction) == false)
        {
            return;
        }

        i2cTransaction.readCount = 1;
        i2cTransaction.writeCount = 1;
        txBuffer[0] = 0x09;
        /* Read MSB */
        if (I2C_transfer(i2c, &i2cTransaction) == false)
        {
            return;
        }

        OFF_threshold_MSB = rxBuffer[0];
        /*************************************************/

        //Store Threshold for CAN transfer
        SensorData.OFF_Threshold = ((uint16_t) OFF_threshold_MSB) << 8 | (uint16_t) OFF_threshold_LSB;

        /**************** ON/F1 THR : READ LSB ****************/
        //Page Address
        txBuffer[0] = 0x08;
        txBuffer[1] = 0x82; //Address : ON-F1 threshold LSB

        i2cTransaction.readCount = 0;
        i2cTransaction.writeCount = 2;

        /* Send Sensor configuration */
        if (I2C_transfer(i2c, &i2cTransaction) == false)
        {
            return;
        }

        //Read threshold
        i2cTransaction.readCount = 1;
        i2cTransaction.writeCount = 1;
        txBuffer[0] = 0x09;

        /* Read LSB */
        if (I2C_transfer(i2c, &i2cTransaction) == false)
        {
            return;
        }

        ON_F1_threshold_LSB = rxBuffer[0];
        /******************************************************/

        /**************** ON/F1 THR : READ MSB ****************/
        txBuffer[0] = 0x08;
        txBuffer[1] = 0x83; //Address : ON-F1 threshold MSB

        i2cTransaction.readCount = 0;
        i2cTransaction.writeCount = 2;

        if (I2C_transfer(i2c, &i2cTransaction) == false)
        {
            return;
        }


        i2cTransaction.readCount = 1;
        i2cTransaction.writeCount = 1;
        txBuffer[0] = 0x09;

        /* Read MSB */
        if (I2C_transfer(i2c, &i2cTransaction) == false)
        {
            return;
        }

        ON_F1_threshold_MSB = rxBuffer[0];
        /******************************************************/

        //Store Threshold for CAN transfer
        SensorData.ON_F1_Threshold = ((uint16_t) ON_F1_threshold_MSB) << 8 | (uint16_t) ON_F1_threshold_LSB;
    }
    else if(mlc_config == TWO_FEATURES)
    {
        /**************** F1 THR : READ LSB ****************/
        //Page Address
        txBuffer[0] = 0x08;
        txBuffer[1] = 0x86; //Address : F1 threshold (VAR_on_ACC_Z) LSB
        /* Send Sensor configuration */
        if (I2C_transfer(i2c, &i2cTransaction) == false)
        {
            return;
        }

        //Read threshold
        i2cTransaction.readCount = 1;
        i2cTransaction.writeCount = 1;
        txBuffer[0] = 0x09;
        /* Read LSB */
        if (I2C_transfer(i2c, &i2cTransaction) == false)
        {
            return;
        }

        F1_threshold_LSB = rxBuffer[0];
        /***************************************************/

        /**************** F1 THR : READ MSB **************/
        txBuffer[0] = 0x08;
        txBuffer[1] = 0x87; //Address : F1 threshold (VAR_on_ACC_Z) MSB

        i2cTransaction.readCount = 0;
        i2cTransaction.writeCount = 2;
        if (I2C_transfer(i2c, &i2cTransaction) == false)
        {
            return;
        }

        i2cTransaction.readCount = 1;
        i2cTransaction.writeCount = 1;
        txBuffer[0] = 0x09;
        /* Read MSB */
        if (I2C_transfer(i2c, &i2cTransaction) == false)
        {
            return;
        }

        F1_threshold_MSB = rxBuffer[0];
        /*************************************************/

        //Store Threshold for CAN transfer
        SensorData.ON_F1_Threshold = ((uint16_t) F1_threshold_MSB) << 8 | (uint16_t) F1_threshold_LSB;

        /**************** ON/OFF THR : READ LSB ****************/
        //Page Address
        txBuffer[0] = 0x08;
        txBuffer[1] = 0x8A; //Address : ON/OFF threshold (VAR_on_ACC_V) LSB

        i2cTransaction.readCount = 0;
        i2cTransaction.writeCount = 2;

        /* Send Sensor configuration */
        if (I2C_transfer(i2c, &i2cTransaction) == false)
        {
            return;
        }

        //Read threshold
        i2cTransaction.readCount = 1;
        i2cTransaction.writeCount = 1;
        txBuffer[0] = 0x09;

        /* Read LSB */
        if (I2C_transfer(i2c, &i2cTransaction) == false)
        {
            return;
        }

        ON_OFF_threshold_LSB = rxBuffer[0];
        /******************************************************/

        /**************** ON/OFF THR : READ MSB ****************/
        txBuffer[0] = 0x08;
        txBuffer[1] = 0x8B; //Address : ON/OFF threshold (VAR_on_ACC_V) MSB

        i2cTransaction.readCount = 0;
        i2cTransaction.writeCount = 2;

        if (I2C_transfer(i2c, &i2cTransaction) == false)
        {
            return;
        }


        i2cTransaction.readCount = 1;
        i2cTransaction.writeCount = 1;
        txBuffer[0] = 0x09;

        /* Read MSB */
        if (I2C_transfer(i2c, &i2cTransaction) == false)
        {
            return;
        }

        ON_OFF_threshold_MSB = rxBuffer[0];
        /******************************************************/

        //Store Threshold for CAN transfer
        SensorData.OFF_Threshold = ((uint16_t) ON_OFF_threshold_MSB) << 8 | (uint16_t) ON_OFF_threshold_LSB;
    }




    //FUNC_CFG_ACCESS : EMBEDDED
    txBuffer[0] = 0x01;
    txBuffer[1] = 0x80;

    i2cTransaction.readCount = 0;
    i2cTransaction.writeCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction) == false)
    {
        return;
    }

    //PAGE RW --- 0
    txBuffer[0] = 0x17;
    txBuffer[1] = 0x00;
    if (I2C_transfer(i2c, &i2cTransaction) == false)
    {
        return;
    }

    txBuffer[0] = 0x04;
    txBuffer[1] = 0x00;
    if (I2C_transfer(i2c, &i2cTransaction) == false)
    {
        return;
    }

    txBuffer[0] = 0x05;
    txBuffer[1] = 0x11;
    if (I2C_transfer(i2c, &i2cTransaction) == false)
    {
        return;
    }

    //PAGE SEL --- 01
    txBuffer[0] = 0x02;
    txBuffer[1] = 0x01;
    if (I2C_transfer(i2c, &i2cTransaction) == false)
    {
        return;
    }

    //FUNC_CFG_ACCESS : Main
    txBuffer[0] = 0x01;
    txBuffer[1] = 0x00;
    if (I2C_transfer(i2c, &i2cTransaction) == false)
    {
        return;
    }
}

/*********************************************************************
 * @fn      I2C_read_fsm
 *
 * @brief   Read mlc result
 */
void I2C_read_fsm(void)
{
    /* Read sensor data's */
    i2cTransaction.readCount = 2;
    i2cTransaction.writeCount = 1;
    txBuffer[0] = FSM_STATUS;
    int fsm_bin = 0;

    if (I2C_transfer(i2c, &i2cTransaction))
    {
        fsm_bin = rxBuffer[0];

        //Store Data for CAN + BLE transfer
        if(fsm_bin != 0) SensorData.FSM_status = rxBuffer[0];

    }
    else
    {
        i2cErrorHandler(&i2cTransaction, sem_display);
    }

    //MLC SRC
    i2cTransaction.readCount = 1;
    i2cTransaction.writeCount = 1;
    txBuffer[0] = MLC1_SRC;

    if (I2C_transfer(i2c, &i2cTransaction))
    {
        //Store Data for CAN transfer
        SensorData.MLC_SRC = rxBuffer[0];
    }
    else
    {
        i2cErrorHandler(&i2cTransaction, sem_display);
    }

}
/**********************************************************************
 * @fn        I2C_MLC_FSM_LP_MODE
 *
 * @brief     Enter LP MODE
 */

void I2C_MLC_FSM_LP_MODE(void)
{
    bool flag_error_config = false;

    //Check MLC Config type (1 or 2 features)

    if(mlc_config == ONE_FEATURE)
    {
        for (int i = 0; i < 190; i++) {

           txBuffer[0] = fsm_mlc_conf_LP_one_feature[i].address;
           txBuffer[1] = fsm_mlc_conf_LP_one_feature[i].data;

           i2cTransaction.readCount = 0;
           i2cTransaction.writeCount = 2;

           /* Send Sensor configuration */
           if (I2C_transfer(i2c, &i2cTransaction) == false)
           {
               flag_error_config = true;

               i2c_error_flag = true;

               //I2C_close(i2c);
               break;
           }

        }
    }
    else if(mlc_config == TWO_FEATURES)
    {
        for (int i = 0; i < 196; i++) {

           txBuffer[0] = fsm_mlc_conf_LP_two_features[i].address;
           txBuffer[1] = fsm_mlc_conf_LP_two_features[i].data;

           i2cTransaction.readCount = 0;
           i2cTransaction.writeCount = 2;

           /* Send Sensor configuration */
           if (I2C_transfer(i2c, &i2cTransaction) == false)
           {
               flag_error_config = true;

               i2c_error_flag = true;

               //I2C_close(i2c);
               break;
           }

        }
    }


}


void Set_ON_F1_Threshold(uint16_t new_thr)
{

    //FUNC_CFG_ACCESS
    txBuffer[0] = 0x01;
    txBuffer[1] = 0x00;

    i2cTransaction.readCount = 0;
    i2cTransaction.writeCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction) == false)
    {
        return;
    }


    //FUNC_CFG_ACCESS : Enables access to the embedded functions
    txBuffer[0] = 0x01;
    txBuffer[1] = 0x80;

    i2cTransaction.readCount = 0;
    i2cTransaction.writeCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction) == false)
    {
        return;

    }

    //R/W Page
    txBuffer[0] = 0x17;
    txBuffer[1] = 0x40; //Write Page

    i2cTransaction.readCount = 0;
    i2cTransaction.writeCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction) == false)
    {
        return;

    }

    //PAGE SEL
    txBuffer[0] = 0x02;
    txBuffer[1] = 0x31;

    i2cTransaction.readCount = 0;
    i2cTransaction.writeCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction) == false)
    {
        return;
    }

    //Page Address
    txBuffer[0] = 0x08;
    txBuffer[1] = 0x8A; //Address : ON-F1 threshold (LSB)

    i2cTransaction.readCount = 0;
    i2cTransaction.writeCount = 2;

    /* Send Sensor configuration */
    if (I2C_transfer(i2c, &i2cTransaction) == false)
    {
        return;
    }

    //Write threshold : LSB
    txBuffer[0] = 0x09; //PAGE Data Register
    txBuffer[1] = new_thr;

    i2cTransaction.readCount = 0;
    i2cTransaction.writeCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction) == false)
    {
        return;
    }

    //Write threshold : MSB
    txBuffer[0] = 0x09; //PAGE Data Register
    txBuffer[1] = new_thr >> 8;

    i2cTransaction.readCount = 0;
    i2cTransaction.writeCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction) == false)
    {
        return;
    }

    //FUNC_CFG_ACCESS : EMBEDDED
    txBuffer[0] = 0x01;
    txBuffer[1] = 0x80;

    i2cTransaction.readCount = 0;
    i2cTransaction.writeCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction) == false)
    {
        return;
    }

    //FUNC_CFG_ACCESS : Main
    txBuffer[0] = 0x01;
    txBuffer[1] = 0x00;

    i2cTransaction.readCount = 0;
    i2cTransaction.writeCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction) == false)
    {
        return;
    }
}

/**********************************************************************
 * @fn        I2C_MLC_FSM_ULP_MODE
 *
 * @brief     Enter ULP MODE
 */

void I2C_MLC_FSM_ULP_MODE(void)
{
    bool flag_error_config = false;

    //Global Reset
    i2cTransaction.readCount = 0;
    i2cTransaction.writeCount = 2;
    txBuffer[0] = 0x01;
    txBuffer[1] = 0x04; //Global Reset

    if (I2C_transfer(i2c, &i2cTransaction) == false)
    {
        I2C_close(i2c);
    }

    delay(500);

    for (int i = 0; i < 41; i++) {

       txBuffer[0] = fsm_mlc_conf_ULP[i].address;
       txBuffer[1] = fsm_mlc_conf_ULP[i].data;

       i2cTransaction.readCount = 0;
       i2cTransaction.writeCount = 2;

       /* Send Sensor configuration */
       if (I2C_transfer(i2c, &i2cTransaction) == false)
       {
           flag_error_config = true;

           i2c_error_flag = true;

           //I2C_close(i2c);
           break;
       }

    }

}

/**********************************************************************
 * @fn        I2C_GET_SENSOR_ODR
 *
 * @brief     Get SENSOR Accelerometer ODR value
 */

Sensor_PowerMode I2C_GET_SENSOR_ODR(void)
{
    Sensor_PowerMode return_value;
    //Read ODR value
    i2cTransaction.readCount = 1;
    i2cTransaction.writeCount = 1;

    txBuffer[0] = CTRL1_XL_REG;

    //I2C Transaction
    if(I2C_transfer(i2c, &i2cTransaction) == false)
    {
        return(ERROR_S);
    }

    switch(rxBuffer[0] & 0x0F)
    {
        case 0x03 : //15 Hz
            return_value = ULTRA_LOW_POWER;
            break;
        case 0x04 : //30 Hz
            return_value = LOW_POWER;
            break;
        default :
            return_value = ERROR_S;
            break;
    }

    return(return_value);

}


/**********************************************************************
 * @fn        uint16_to_half
 *
 * @brief     Function to convert a 16-bit unsigned integer to half-precision float
 */
float uint16_to_half(uint16_t value) {
    uint32 floatBits = 0;

    // Extract the components of the 16-bit integer (bit manipulation)
    uint32 sign = (value & 0x8000) << 16;
    uint32 exponent = (value & 0x7C00) << 13;
    uint32 significand = (value & 0x03FF) << 13;

    if (exponent == 0x7C00) {
        floatBits = sign | 0x7F800000 | significand;
    } else if (exponent == 0) {
        floatBits = sign | significand;
    } else {
        exponent += 0x38000000;
        floatBits = sign | exponent | significand;
    }

    // Cast the bit pattern back to a float
    float result;
    memcpy(&result, &floatBits, sizeof(float));
    return result;
}

/**********************************************************************
 * @fn        half_to_uint16
 *
 * @brief     Function to convert a half-precision float to a 16-bit unsigned integer
 */

uint16_t half_to_uint16(float value) {
    // Extract the components of the floating-point number
    uint32_t floatBits;
    memcpy(&floatBits, &value, sizeof(float));

    uint16_t sign = (floatBits >> 31) & 0x0001;
    uint16_t exponent = (floatBits >> 23) & 0x00FF;
    uint16_t significand = (floatBits >> 13) & 0x03FF;

    //Special cases for NaN and infinity
    if (exponent == 0xFF) {
        if (significand == 0) {
            // NaN :not a number
            return 0x7FFF;
        } else {
            // Infinity
            return (sign << 15) | 0x7C00;
        }
    }

    // Normalized or subnormal numbers
    exponent = exponent ? (exponent - 127 + 15) : 0;

    // Combine the sign, exponent, and significand bits to form the 16-bit representation
    return (sign << 15) | (exponent << 10) | significand;
}


