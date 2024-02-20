/******************************************************************************

 @file  main.c

 @brief main entry of the BLE stack sample application.

 Group: WCS, BTS
 Target Device: cc13xx_cc26xx

 ******************************************************************************
 
 Copyright (c) 2013-2023, Texas Instruments Incorporated
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

/*******************************************************************************
 * INCLUDES
 */

#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/GPIO.h>
#include <ti/sysbios/BIOS.h>
#include <icall.h>
#include "hal_assert.h"
#include "bcomdef.h"
#include "simple_peripheral_oad_onchip.h"
#ifdef PTM_MODE
#include "npi_task.h"
#endif // PTM_MODE

/* Shared resources  */
#include "nwt_i2c.h"
#include "nwt_sc.h"
#include "nwt_reboot.h"
#include <nwt_spifdcan.h>
#include "shared.h"
#include "BQ27427.h"
#include "EEPROM.h"
/* Header files required to enable instruction fetch cache */
#include <inc/hw_memmap.h>
#include <driverlib/vims.h>

#ifndef USE_DEFAULT_USER_CFG
#include "ble_user_config.h"
// BLE user defined configuration
icall_userCfg_t user0Cfg = BLE_USER_CFG;
#endif // USE_DEFAULT_USER_CFG

#include <ti/display/Display.h>
/*******************************************************************************
 * MACROS
 */
#define POWER_SAVING
/*******************************************************************************
 * CONSTANTS
 */

/*******************************************************************************
 * FUNCTIONS
 */
void I2C_INIT(void);
void SPI_INIT(void);
void Error_Handler(void);
/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */
I2C_Handle i2c;
I2C_Params i2cParams;
SPI_Handle controllerSpi;
SPI_Params spiParams;

BQ27427_ctx_t BQ27427;
/*******************************************************************************
 * EXTERNS
 */

extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

extern Display_Handle dispHandle;

/*******************************************************************************
 * @fn          Main
 *
 * @brief       Application Main
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
int main()
{

  //Power_shutdown(0,0);

  /* Register Application callback to trap asserts raised in the Stack */
  RegisterAssertCback(AssertHandler);

  Board_initGeneral();

  // Enable iCache prefetching
  VIMSConfigure(VIMS_BASE, TRUE, TRUE);
  // Enable cache
  VIMSModeSet(VIMS_BASE, VIMS_MODE_ENABLED);

  /* Update User Configuration of the stack */
  user0Cfg.appServiceInfo->timerTickPeriod = Clock_tickPeriod;
  user0Cfg.appServiceInfo->timerMaxMillisecond  = ICall_getMaxMSecs();

  /* Init semaphore */
  init_semaphore();

  /* Init display */
  init_display();

  /* Initialize ICall module */
  ICall_init();

  /* Start tasks of external images - Priority 5 */
  ICall_createRemoteTasks();

#ifdef PTM_MODE
  /* Start task for NPI task */
  NPITask_createTask(ICALL_SERVICE_CLASS_BLE);
#endif // PTM_MODE

  GPIO_init();

  /************** I2C Init ***************/
  I2C_INIT();
  /****************************************/

  GPIO_setConfig(24, GPIO_CFG_INPUT); //PIN GPOUT (INPUT)

#ifndef FINAL_VERSION
  /************** SPI Init ***************/
  SPI_INIT(); //For CAN FD Controller
  /****************************************/
#endif

  /************* Init Fuel Gauge **********/
  BQ27427_ctx_t BQ27427 = {
          .BQ27427_i2c_address = BQ27427_I2C_ADDRESS,
          .read_reg = BQ27427_i2cReadBytes,
          .write_reg = BQ27427_i2cWriteBytes,
  };
  /****************************************/

//  const PowerCC26XX_Config PowerCC26XX_config = {
//   .policyInitFxn = NULL,
//   .policyFxn = &PowerCC26XX_standbyPolicy,
//   .calibrateFxn = &PowerCC26XX_calibrate,
//   .enablePolicy = FALSE,
//   .calibrateRCOSC_LF = TRUE,
//   .calibrateRCOSC_HF = TRUE,
//  };
//
//  Power_enablePolicy();
//
//  setLowPowerMode();

  /* Launch Tasks */
  I2C_createTask();
  SimplePeripheral_createTask();
  SC_createTask();
  REBOOT_createTask();

#ifndef FINAL_VERSION
 FDCAN_createTask();
#endif

  /* enable interrupts and start SYS/BIOS */
  BIOS_start();

  return 0;
}


void I2C_INIT(void)
{
    I2C_init();
    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(0, &i2cParams);
    if (i2c == NULL)  Error_Handler(); //Failed to open I2C module Todo: Add Debug display messages
}

void SPI_INIT(void)
{
    SPI_init();
    /* Open SPI as controller */
    SPI_Params_init(&spiParams);
    spiParams.frameFormat = SPI_POL1_PHA1;
    spiParams.bitRate     = 500000;
    controllerSpi         = SPI_open(CONFIG_SPI_CONTROLLER, &spiParams);
    if(controllerSpi == NULL) Error_Handler(); //Failed to open SPI module Todo: Add Debug display messages
}

void Error_Handler(void)
{
    //ToDO : Define what to do in case a critical error has been detected on the system
    while(1);
}

/*******************************************************************************
 * @fn          AssertHandler
 *
 * @brief       This is the Application's callback handler for asserts raised
 *              in the stack.  When EXT_HAL_ASSERT is defined in the Stack Wrapper
 *              project this function will be called when an assert is raised,
 *              and can be used to observe or trap a violation from expected
 *              behavior.
 *
 *              As an example, for Heap allocation failures the Stack will raise
 *              HAL_ASSERT_CAUSE_OUT_OF_MEMORY as the assertCause and
 *              HAL_ASSERT_SUBCAUSE_NONE as the assertSubcause.  An application
 *              developer could trap any malloc failure on the stack by calling
 *              HAL_ASSERT_SPINLOCK under the matching case.
 *
 *              An application developer is encouraged to extend this function
 *              for use by their own application.  To do this, add hal_assert.c
 *              to your project workspace, the path to hal_assert.h (this can
 *              be found on the stack side). Asserts are raised by including
 *              hal_assert.h and using macro HAL_ASSERT(cause) to raise an
 *              assert with argument assertCause.  the assertSubcause may be
 *              optionally set by macro HAL_ASSERT_SET_SUBCAUSE(subCause) prior
 *              to asserting the cause it describes. More information is
 *              available in hal_assert.h.
 *
 * input parameters
 *
 * @param       assertCause    - Assert cause as defined in hal_assert.h.
 * @param       assertSubcause - Optional assert subcause (see hal_assert.h).
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void AssertHandler(uint8 assertCause, uint8 assertSubcause)
{
  // Open the display if the app has not already done so
  if ( !dispHandle )
  {
    dispHandle = Display_open(Display_Type_ANY, NULL);
  }

  Display_print0(dispHandle, 0, 0, ">>>STACK ASSERT");

  // check the assert cause
  switch (assertCause)
  {
    case HAL_ASSERT_CAUSE_OUT_OF_MEMORY:
      Display_print0(dispHandle, 0, 0, "***ERROR***");
      Display_print0(dispHandle, 2, 0, ">> OUT OF MEMORY!");
      break;

    case HAL_ASSERT_CAUSE_INTERNAL_ERROR:
      // check the subcause
      if (assertSubcause == HAL_ASSERT_SUBCAUSE_FW_INERNAL_ERROR)
      {
        Display_print0(dispHandle, 0, 0, "***ERROR***");
        Display_print0(dispHandle, 2, 0, ">> INTERNAL FW ERROR!");
      }
      else
      {
        Display_print0(dispHandle, 0, 0, "***ERROR***");
        Display_print0(dispHandle, 2, 0, ">> INTERNAL ERROR!");
      }
      break;

    case HAL_ASSERT_CAUSE_ICALL_ABORT:
      Display_print0(dispHandle, 0, 0, "***ERROR***");
      Display_print0(dispHandle, 2, 0, ">> ICALL ABORT!");
      HAL_ASSERT_SPINLOCK;
      break;

    case HAL_ASSERT_CAUSE_ICALL_TIMEOUT:
      Display_print0(dispHandle, 0, 0, "***ERROR***");
      Display_print0(dispHandle, 2, 0, ">> ICALL TIMEOUT!");
      HAL_ASSERT_SPINLOCK;
      break;

    case HAL_ASSERT_CAUSE_WRONG_API_CALL:
      Display_print0(dispHandle, 0, 0, "***ERROR***");
      Display_print0(dispHandle, 2, 0, ">> WRONG API CALL!");
      HAL_ASSERT_SPINLOCK;
      break;

  default:
      Display_print0(dispHandle, 0, 0, "***ERROR***");
      Display_print0(dispHandle, 2, 0, ">> DEFAULT SPINLOCK!");
      HAL_ASSERT_SPINLOCK;
  }

  return;
}


/*******************************************************************************
 * @fn          smallErrorHook
 *
 * @brief       Error handler to be hooked into TI-RTOS.
 *
 * input parameters
 *
 * @param       eb - Pointer to Error Block.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void smallErrorHook(Error_Block *eb)
{
  for (;;);
}

/*******************************************************************************
 */
