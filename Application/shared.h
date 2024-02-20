/*
 * shared.h
 *
 *  Created on: 13 juil. 2023
 *      Author: gaeta
 */

#ifndef APPLICATION_SHARED_H_
#define APPLICATION_SHARED_H_

//to switch between CAN and final version
#define FINAL_VERSION

#include <ti/display/Display.h>
#include <ti/drivers/dpl/SemaphoreP.h>

#include "hal_types.h"
#include "nwt_i2c.h"
#include "BQ27427.h"
#include <ti/drivers/I2C.h>

/* Hall effect sensor : DRV5032 */
#define CAP_HALL 12

//Variance moving average size
#define MOV_AVG_SIZE 10


/* Pre-Startup Self-Check structure */
typedef struct {
    bool sensor_present;
    bool eeprom_present;
    bool fuel_gauge;
    bool hall_effect_sensor_out;
    bool self_check_state;
} SelfCheck_t;

/* Sensor Data */
typedef struct {
    uint16_t OUTZ_A_v; //Linear acceleration Z-Axis
    uint16_t OUTY_A_v; //Linear acceleration Y-Axis
    uint16_t OUTX_A_v; //Linear acceleration X-Axis
    /********** MLC *********/
    uint8_t FSM_status; //Finite state machine status
    uint8_t MLC_SRC; //Machine learning core source register
    uint16_t Feature_1; //MLC feature 1
    uint16_t Feature_2; //MLC feature 2
    uint16_t OFF_Threshold; //MLC OFF Threshold
    uint16_t ON_F1_Threshold; //MLC ON-F1 Threshold
    uint16_t MOVING_AVG_Z; //A moving average calculated over MOV_AVG_SIZE values (converted to uint16)
    uint16_t MOVING_AVG_V; //A moving average calculated over MOV_AVG_SIZE values (converted to uint16)
    uint8_t WAKE_UP_Threshold; //OFF-WAKE UP Threshold
} SensorDATA_t;

/* MLC Config Type */
typedef enum {
    ONE_FEATURE = 1,
    TWO_FEATURES = 2
} MLCconfig_t;

extern MLCconfig_t mlc_config;

//An array to contain the last MOV_AVG_SIZE values read of the feature (variance_Z)
extern float variance_Z_values[MOV_AVG_SIZE];

//An array to contain the last MOV_AVG_SIZE values read of the feature (variance_V)
extern float variance_V_values[MOV_AVG_SIZE];

//The moving average value (VAR_Z)
extern float moving_avg_var_z;

//The moving average value (VAR_V)
extern float moving_avg_var_v;

//Threshold Delta (ON/F1 state)
extern float delta_thr;
//Threshold Delta (OFF state)
extern float delta_thr_off;

//Re-Config MLC
extern uint8_t reconfig_mlc_flag;

//I2C write fail flag
extern bool i2c_error_flag;

extern SelfCheck_t SelfCheck;

extern SensorDATA_t SensorData;

extern Sensor_PowerMode sensorCurrentPM;

extern uint8_t SelfCheckFlags;

extern uint8_t FlagActivationULP;

extern Sensor_PowerMode DebugCurrentPM;

//Transition from ULP to LP mode COUNTER
extern uint32_t cptWU; //WakeUp

//Flag for ULP Mode jitter correction
extern uint8_t FlagULP_j;

extern uint16_t CellVoltage_s; //Cell voltage in mV
extern uint16_t CellCurrent_s; //Current measurement in mA (max, average or standby)
extern uint16_t CellSoC_s; //State of charge measurement in %


/* Block the main program during the self-check test */
extern SemaphoreP_Handle sem_i2c_thread;
extern SemaphoreP_Handle sem_fdcan_thread;
/* Block the reboot task until magnet is detected */
extern SemaphoreP_Handle sem_reboot_thread;

/* handle global for the UART display */
extern Display_Handle dispHandle;

/* for display
 *unused, because semaphore already in the function display
 */
extern SemaphoreP_Handle sem_display;

/* Semaphore to pilot which task is allowed to run
 * The main task is the SC
 * I2C and BLE tasks are slave
 */
extern SemaphoreP_Handle sem_magn;

/* Semaphore to pilot which task is allowed to run
 * The main task is the SC
 * I2C and BLE tasks are slave
 */
extern SemaphoreP_Handle sem_sc_thread;

/* Semaphore to block the SC thread while no event detected */
extern SemaphoreP_Handle sem_sc_event;

/* Var for BLE advertising */
extern uint8 advHandleLegacy;
extern uint8 advHandleLongRange;
extern bool advertising_initialized;

/* Var for MLC threshold */
extern int sail_real_state;
extern uint8_t calculated_threshold;

/* Fuel Gauge Driver */
extern BQ27427_ctx_t BQ27427;
extern BQ27427_Data Battery_Cell_Data;

/* For I2C */
extern I2C_Handle i2c;
extern I2C_Params i2cParams;
extern float acc;
extern int mlc;


/* Function */
extern void init_display(void);
extern void init_semaphore(void);
extern void config_ble_service(void);

extern void sp_disable_advertising(void);
extern void sp_enable_advertising(void);

#endif /* APPLICATION_SHARED_H_ */
