/*
 *  ======== ti_radio_config.c ========
 *  Configured RadioConfig module definitions
 *
 *  DO NOT EDIT - This file is generated for the CC2652P1FSIP
 *  by the SysConfig tool.
 *
 *  Radio Config module version : 1.17.2
 *  SmartRF Studio data version : 2.29.0
 */

#include "ti_radio_config.h"
#include DeviceFamily_constructPath(rf_patches/rf_patch_cpe_bt5.h)
#include DeviceFamily_constructPath(rf_patches/rf_patch_mce_bt5.h)

// Custom overrides
#include <ti/ble5stack/icall/inc/ble_overrides.h>


// *********************************************************************************
//   RF Frontend configuration
// *********************************************************************************
// RF design based on: LP_CC2652PSIP

// TX Power tables
// The RF_TxPowerTable_DEFAULT_PA_ENTRY and RF_TxPowerTable_HIGH_PA_ENTRY macros are defined in RF.h.
// The following arguments are required:
// RF_TxPowerTable_DEFAULT_PA_ENTRY(bias, gain, boost, coefficient)
// RF_TxPowerTable_HIGH_PA_ENTRY(bias, ibboost, boost, coefficient, ldoTrim)
// See the Technical Reference Manual for further details about the "txPower" Command field.
// The PA settings require the CCFG_FORCE_VDDR_HH = 0 unless stated otherwise.

// 2400 MHz, 5 dBm
RF_TxPowerTable_Entry txPowerTable_2400_pa5[TXPOWERTABLE_2400_PA5_SIZE] =
{
    {-20, RF_TxPowerTable_DEFAULT_PA_ENTRY(6, 3, 0, 4) }, // 0x08C6
    {-18, RF_TxPowerTable_DEFAULT_PA_ENTRY(7, 3, 0, 4) }, // 0x08C7
    {-15, RF_TxPowerTable_DEFAULT_PA_ENTRY(9, 3, 0, 0) }, // 0x00C9
    {-12, RF_TxPowerTable_DEFAULT_PA_ENTRY(12, 3, 0, 0) }, // 0x00CC
    {-10, RF_TxPowerTable_DEFAULT_PA_ENTRY(15, 3, 0, 4) }, // 0x08CF
    {-9, RF_TxPowerTable_DEFAULT_PA_ENTRY(16, 3, 0, 4) }, // 0x08D0
    {-6, RF_TxPowerTable_DEFAULT_PA_ENTRY(21, 3, 0, 4) }, // 0x08D5
    {-5, RF_TxPowerTable_DEFAULT_PA_ENTRY(24, 3, 0, 7) }, // 0x0ED8
    {-3, RF_TxPowerTable_DEFAULT_PA_ENTRY(32, 3, 0, 19) }, // 0x26E0
    {0, RF_TxPowerTable_DEFAULT_PA_ENTRY(30, 2, 0, 19) }, // 0x269E
    {1, RF_TxPowerTable_DEFAULT_PA_ENTRY(35, 2, 0, 19) }, // 0x26A3
    {2, RF_TxPowerTable_DEFAULT_PA_ENTRY(35, 1, 0, 28) }, // 0x3863
    {3, RF_TxPowerTable_DEFAULT_PA_ENTRY(27, 0, 0, 44) }, // 0x581B
    {4, RF_TxPowerTable_DEFAULT_PA_ENTRY(42, 0, 0, 69) }, // 0x8A2A
    {5, RF_TxPowerTable_DEFAULT_PA_ENTRY(62, 0, 0, 54) }, // 0x6C3E
    RF_TxPowerTable_TERMINATION_ENTRY
};

// 2400 MHz, 5 + 10 dBm
RF_TxPowerTable_Entry txPowerTable_2400_pa5_10[TXPOWERTABLE_2400_PA5_10_SIZE] =
{
    {-20, RF_TxPowerTable_DEFAULT_PA_ENTRY(6, 3, 0, 4) }, // 0x08C6
    {-18, RF_TxPowerTable_DEFAULT_PA_ENTRY(7, 3, 0, 4) }, // 0x08C7
    {-15, RF_TxPowerTable_DEFAULT_PA_ENTRY(9, 3, 0, 0) }, // 0x00C9
    {-12, RF_TxPowerTable_DEFAULT_PA_ENTRY(12, 3, 0, 0) }, // 0x00CC
    {-10, RF_TxPowerTable_DEFAULT_PA_ENTRY(15, 3, 0, 4) }, // 0x08CF
    {-9, RF_TxPowerTable_DEFAULT_PA_ENTRY(16, 3, 0, 4) }, // 0x08D0
    {-6, RF_TxPowerTable_DEFAULT_PA_ENTRY(21, 3, 0, 4) }, // 0x08D5
    {-5, RF_TxPowerTable_DEFAULT_PA_ENTRY(24, 3, 0, 7) }, // 0x0ED8
    {-3, RF_TxPowerTable_DEFAULT_PA_ENTRY(32, 3, 0, 19) }, // 0x26E0
    {0, RF_TxPowerTable_DEFAULT_PA_ENTRY(30, 2, 0, 19) }, // 0x269E
    {1, RF_TxPowerTable_DEFAULT_PA_ENTRY(35, 2, 0, 19) }, // 0x26A3
    {2, RF_TxPowerTable_DEFAULT_PA_ENTRY(35, 1, 0, 28) }, // 0x3863
    {3, RF_TxPowerTable_DEFAULT_PA_ENTRY(27, 0, 0, 44) }, // 0x581B
    {4, RF_TxPowerTable_DEFAULT_PA_ENTRY(42, 0, 0, 69) }, // 0x8A2A
    {5, RF_TxPowerTable_DEFAULT_PA_ENTRY(62, 0, 0, 54) }, // 0x6C3E
    {6, RF_TxPowerTable_HIGH_PA_ENTRY(32, 0, 1, 32, 16) }, // 0x104120
    {7, RF_TxPowerTable_HIGH_PA_ENTRY(35, 0, 1, 35, 20) }, // 0x144723
    {8, RF_TxPowerTable_HIGH_PA_ENTRY(42, 0, 1, 44, 20) }, // 0x14592A
    {9, RF_TxPowerTable_HIGH_PA_ENTRY(23, 1, 1, 28, 20) }, // 0x143957
    {10, RF_TxPowerTable_HIGH_PA_ENTRY(30, 1, 1, 35, 20) }, // 0x14475E
    RF_TxPowerTable_TERMINATION_ENTRY
};



//*********************************************************************************
//  RF Setting:   BLE, 2 Mbps, LE 2M
//
//  PHY:          bt5le2mp10
//  Setting file: setting_bt5_le_2m_10_dbm.json
//*********************************************************************************

// PARAMETER SUMMARY
// NB! Setting RF parameters in this design has no effect as no RF commands are selected.

// TI-RTOS RF Mode Object
RF_Mode RF_modeBle =
{
    .rfMode = RF_MODE_AUTO,
    .cpePatchFxn = &rf_patch_cpe_bt5,
    .mcePatchFxn = &rf_patch_mce_bt5,
    .rfePatchFxn = 0
};

// Overrides for CMD_BLE5_RADIO_SETUP_PA
uint32_t pOverrides_bleCommon[] =
{
    // override_ble5_setup_override_common_10_dbm.json
    // Bluetooth 5: Reconfigure to 35 us pilot tone length for high output power PA
    HW_REG_OVERRIDE(0x6024,0x5B20),
    // Bluetooth 5: Compensate for 35 us pilot tone length
    (uint32_t)0x01640263,
    // Bluetooth 5: Default to no CTE. 
    HW_REG_OVERRIDE(0x5328,0x0000),
    // Synth: Increase mid code calibration time to 5 us
    (uint32_t)0x00058683,
    // Synth: Increase mid code calibration time to 5 us
    HW32_ARRAY_OVERRIDE(0x4004,1),
    // Synth: Increase mid code calibration time to 5 us
    (uint32_t)0x38183C30,
    // Set VCTRIM to 0 for 20 dBm PA
    ADI_HALFREG_OVERRIDE(1,26,0xF,0x0),
    // Set VCTRIM to 0 for 20 dBm PA
    ADI_HALFREG_OVERRIDE(1,27,0x7,0x0),
    // Bluetooth 5: Move synth start code
    HW_REG_OVERRIDE(0x4064,0x3C),
    // Bluetooth 5: Set DTX gain -5% for 1 Mbps
    (uint32_t)0x00E787E3,
    // Bluetooth 5: Set DTX threshold 1 Mbps
    (uint32_t)0x00950803,
    // Bluetooth 5: Set DTX gain -2.5% for 2 Mbps
    (uint32_t)0x00F487F3,
    // Bluetooth 5: Set DTX threshold 2 Mbps
    (uint32_t)0x012A0823,
    // Bluetooth 5: Set synth fine code calibration interval
    HW32_ARRAY_OVERRIDE(0x4020,1),
    // Bluetooth 5: Set synth fine code calibration interval
    (uint32_t)0x41005F00,
    // Bluetooth 5: Adapt to synth fine code calibration interval
    (uint32_t)0xC0040141,
    // Bluetooth 5: Adapt to synth fine code calibration interval
    (uint32_t)0x0007DD44,
    // Bluetooth 5: Set enhanced TX shape
    (uint32_t)0x000D8C73,
    // ti/ble5stack/icall/inc/ble_overrides.h
    BLE_STACK_OVERRIDES(),
    (uint32_t)0xFFFFFFFF
};

// Overrides for CMD_BLE5_RADIO_SETUP_PA
uint32_t pOverrides_ble1Mbps[] =
{
    // override_ble5_setup_override_1mbps_10_dbm.json
    // Bluetooth 5: Reconfigure pilot tone length for high output power PA
    HW_REG_OVERRIDE(0x5320,0x0690),
    // Bluetooth 5: Compensate for modified pilot tone length
    (uint32_t)0x018F02A3,
    // override_ble5_symbol_error_tracking.json
    // Symbol tracking: timing correction
    HW_REG_OVERRIDE(0x50D4,0x00F9),
    // Symbol tracking: reduce sample delay
    HW_REG_OVERRIDE(0x50E0,0x0087),
    // Symbol tracking: demodulation order
    HW_REG_OVERRIDE(0x50F8,0x0014),
    (uint32_t)0xFFFFFFFF
};

// Overrides for CMD_BLE5_RADIO_SETUP_PA
uint32_t pOverrides_ble2Mbps[] =
{
    // override_ble5_setup_override_2mbps_residual_per.json
    // PHY: Use MCE RAM (patch), RFE ROM
    MCE_RFE_OVERRIDE(1,0,2,0,3,2),
    // Rx: increase AGC hysteresis (HIGH_GAIN=15, LOW_GAIN=11)
    HW_REG_OVERRIDE(0x6098,0x75FB),
    // override_ble5_setup_override_2mbps_10_dbm.json
    // Bluetooth 5: Reconfigure pilot tone length for high output power PA
    HW_REG_OVERRIDE(0x5320,0x0690),
    // Bluetooth 5: Compensate for modified pilot tone length
    (uint32_t)0x012D02A3,
    // Bluetooth 5: increase low gain AGC delay for 2 Mbps
    HW_REG_OVERRIDE(0x60A4,0x7D00),
    // override_ble5_symbol_error_tracking.json
    // Symbol tracking: timing correction
    HW_REG_OVERRIDE(0x50D4,0x00F9),
    // Symbol tracking: reduce sample delay
    HW_REG_OVERRIDE(0x50E0,0x0087),
    // Symbol tracking: demodulation order
    HW_REG_OVERRIDE(0x50F8,0x0014),
    (uint32_t)0xFFFFFFFF
};

// Overrides for CMD_BLE5_RADIO_SETUP_PA
uint32_t pOverrides_bleCoded[] =
{
    // override_ble5_setup_override_coded_10_dbm.json
    // Bluetooth 5: Reconfigure pilot tone length for high output power PA
    HW_REG_OVERRIDE(0x5320,0x0690),
    // Bluetooth 5: Compensate syncTimeadjust
    (uint32_t)0x07E502A3,
    // Rx: Set AGC reference level to 0x21 (default: 0x2E)
    HW_REG_OVERRIDE(0x609C,0x0021),
    (uint32_t)0xFFFFFFFF
};

// Overrides for CMD_BLE5_RADIO_SETUP_PA
uint32_t pOverrides_bleTxStd[] =
{
    // override_txstd_placeholder.json
    // TX Standard power override
    TX_STD_POWER_OVERRIDE(0x6C3E),
    // The ANADIV radio parameter based on LO divider and front end settings
    (uint32_t)0x05320703,
    // override_txstd_settings.json
    // Bluetooth 5: Set RTIM offset to default for standard PA
    (uint32_t)0x00008783,
    // Bluetooth 5: Set synth mux to default value for standard PA
    (uint32_t)0x050206C3,
    // Set TXRX pin to 0 in RX and high impedance in idle/TX. 
    HW_REG_OVERRIDE(0x60A8,0x0401),
    // Bluetooth 5: Set IPEAK = 3 and DCDC dither off for TX
    (uint32_t)0x00F388D3,
    (uint32_t)0xFFFFFFFF
};

// Overrides for CMD_BLE5_RADIO_SETUP_PA
uint32_t pOverrides_bleTx20[] =
{
    // override_tx20_placeholder.json
    // TX HighPA power override
    TX20_POWER_OVERRIDE(0x0014475E),
    // The ANADIV radio parameter based on LO divider and front end settings
    (uint32_t)0x01C20703,
    // override_tx20_settings.json
    // Bluetooth 5: Set RTIM offset to 3 for high power PA
    (uint32_t)0x00030783,
    // Bluetooth 5: Set synth mux for high power PA
    (uint32_t)0x010206C3,
    // Set TXRX pin to 0 in RX/TX and high impedance in idle. 
    HW_REG_OVERRIDE(0x60A8,0x0001),
    // Bluetooth 5: Set IPEAK = 7 and DCDC dither off for TX
    (uint32_t)0x00F788D3,
    (uint32_t)0xFFFFFFFF
};




