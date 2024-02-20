/*
 * nwt_sc.h
 *
 *  Created on: 24 juil. 2023
 *      Author: gaeta
 */

#ifndef APPLICATION_NWT_SC_H_
#define APPLICATION_NWT_SC_H_

/*********************************************************************
 * INCLUDES
 */
/*********************************************************************
 * CONSTANTS
 */
/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * FUNCTIONS
 */

/*
 * @fn      SC_createTask
 *
 * @brief   Task creation function for Sensor Controller.
 */
extern void SC_createTask(void);

/*
 * @fn      MAGFieldDetFxn
 *
 * @brief   Callback function for the GPIO interrupt on CAP_HALL
 */
extern void MAGFieldDetFxn(uint_least8_t index);


#endif /* APPLICATION_NWT_SC_H_ */
