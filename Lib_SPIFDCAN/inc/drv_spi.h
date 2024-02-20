/*
 *  drv_spi.h
 *
 *
 *  Created on: 7 sept. 2023
 *      Author: rami
 */

#ifndef _DRV_SPI_H
#define	_DRV_SPI_H

#include <string.h>

//========================================================//
#ifdef	__cplusplus
extern "C" {
  
#endif
void APP_ReceiveMessage_Tasks(void);
// Index to SPI channel
// Used when multiple MCP2517FD are connected to the same SPI interface, but with different CS    
#define DRV_CANFDSPI_INDEX_0         0
#define DRV_CANFDSPI_INDEX_1         1

//========================================================//
#ifdef	__cplusplus
}
#endif
//========================================================//
#endif	/* _DRV_SPI_H */

