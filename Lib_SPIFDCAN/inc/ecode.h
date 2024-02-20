/*
 *  ecode.h
 *
 *  API function error code return values
 *
 *  Created on: 7 sept. 2023
 *      Author: rami
 */

#ifndef __SILICON_LABS_ECODE_H__
#define __SILICON_LABS_ECODE_H__

#include <stdint.h>

typedef uint32_t Ecode_t;

#define ECODE_EMDRV_BASE  (0xF0000000U)   ///< Base value for all EMDRV errorcodes.

#define ECODE_OK          (0U)            ///< Generic success return value.

#define ECODE_EMDRV_RTCDRV_BASE      (ECODE_EMDRV_BASE | 0x00001000U)   ///< Base value for RTCDRV error codes.
#define ECODE_EMDRV_SPIDRV_BASE      (ECODE_EMDRV_BASE | 0x00002000U)   ///< Base value for SPIDRV error codes.
#define ECODE_EMDRV_NVM_BASE         (ECODE_EMDRV_BASE | 0x00003000U)   ///< Base value for NVM error codes.
#define ECODE_EMDRV_USTIMER_BASE     (ECODE_EMDRV_BASE | 0x00004000U)   ///< Base value for USTIMER error codes.
#define ECODE_EMDRV_UARTDRV_BASE     (ECODE_EMDRV_BASE | 0x00007000U)   ///< Base value for UARTDRV error codes.
#define ECODE_EMDRV_DMADRV_BASE      (ECODE_EMDRV_BASE | 0x00008000U)   ///< Base value for DMADRV error codes.
#define ECODE_EMDRV_EZRADIODRV_BASE  (ECODE_EMDRV_BASE | 0x00009000U)   ///< Base value for EZRADIODRV error codes.
#define ECODE_EMDRV_TEMPDRV_BASE     (ECODE_EMDRV_BASE | 0x0000D000U)   ///< Base value for TEMPDRV error codes.
#define ECODE_EMDRV_NVM3_BASE        (ECODE_EMDRV_BASE | 0x0000E000U)   ///< Base value for NVM3 error codes.

#endif
