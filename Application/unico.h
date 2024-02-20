/*
 * unico.h
 *
 *  Created on: 20 juil. 2023
 *      Author: gaeta
 */


#ifndef APPLICATION_UNICO_H_
#define APPLICATION_UNICO_H_

#include <stdint.h>

#ifndef MEMS_UCF_SHARED_TYPES
#define MEMS_UCF_SHARED_TYPES

/** Common data block definition **/
typedef struct {
  uint8_t address;
  uint8_t data;
} ucf_line_t;

#endif /* MEMS_UCF_SHARED_TYPES */
extern ucf_line_t unico_fsm_mlc_conf[236];
extern ucf_line_t fsm_mlc_conf_LP[198];
extern ucf_line_t fsm_mlc_conf_ULP[41];
extern ucf_line_t read_mlc_features[41];
/** Configuration array generated from Unico Tool **/
extern ucf_line_t fsm_mlc_conf_LP_one_feature[190];
extern ucf_line_t fsm_mlc_conf_LP_two_features[196];


#endif /* APPLICATION_UNICO_H_ */
