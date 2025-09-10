/*
 * IR.h
 *
 *  Created on: Sep 1, 2025
 *      Author: nguye
 */

#ifndef IR_H_
#define IR_H_

#include "stdbool.h"

#define IR_NUM_CH        4
#define IR_SAMPLES_PERCH 10
#define IR_BUF_LEN       (IR_NUM_CH * IR_SAMPLES_PERCH)

typedef enum { OKAY, BUSY } IR_status;

extern volatile uint16_t LEFT_IR, FLEFT_IR, FRIGHT_IR, RIGHT_IR;
extern volatile IR_status ir_status;
extern uint16_t sensor_data[IR_BUF_LEN];
extern uint32_t ir_count ;

void EmiterON(void);
void EmiterOFF(void);

uint16_t median_filter(const uint16_t *data, uint8_t channel, uint8_t len);
void ReadIR(ADC_HandleTypeDef* hadc);
bool Wall_Left(void);
bool Wall_Right(void);
bool Wall_Front(void);
bool Check_Start(ADC_HandleTypeDef* hadc);




#endif /* IR_H_ */
