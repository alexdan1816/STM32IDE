/*
 * IR.c
 *
 *  Created on: Sep 1, 2025
 *      Author: nguye
 */
#include "stm32f1xx_hal.h"
#include "main.h"
#include "IR.h"
#include "stdbool.h"
#include "FSM.h"

uint16_t sensor_data[IR_BUF_LEN];
float alpha = 0.2f;

volatile uint16_t LEFT_IR = 0, FLEFT_IR = 0, FRIGHT_IR = 0, RIGHT_IR = 0;
volatile IR_status ir_status = OKAY;


void EmiterON()
{
	HAL_GPIO_WritePin(LEFT_IR_EMIT_GPIO_Port, LEFT_IR_EMIT_Pin, SET);
	HAL_GPIO_WritePin(FLEFT_IR_EMIT_GPIO_Port, FLEFT_IR_EMIT_Pin, SET);
	HAL_GPIO_WritePin(FRIGHT_IR_EMIT_GPIO_Port, FRIGHT_IR_EMIT_Pin, SET);
	HAL_GPIO_WritePin(RIGHT_IR_EMIT_GPIO_Port, RIGHT_IR_EMIT_Pin, SET);
}

void EmiterOFF()
{
	HAL_GPIO_WritePin(LEFT_IR_EMIT_GPIO_Port, LEFT_IR_EMIT_Pin, RESET);
	HAL_GPIO_WritePin(FLEFT_IR_EMIT_GPIO_Port, FLEFT_IR_EMIT_Pin, RESET);
	HAL_GPIO_WritePin(FRIGHT_IR_EMIT_GPIO_Port, FRIGHT_IR_EMIT_Pin, RESET);
	HAL_GPIO_WritePin(RIGHT_IR_EMIT_GPIO_Port, RIGHT_IR_EMIT_Pin, RESET);
}

uint16_t median_filter(const uint16_t *data, uint8_t channel, uint8_t len)
{
    uint16_t array[IR_SAMPLES_PERCH]; // = 10
    for (int i = 0; i < len; i++)
        array[i] = data[i * IR_NUM_CH + channel];

    for (int i = 1; i < len; i++) { // insertion sort
        uint16_t key = array[i];
        int j = i - 1;
        while (j >= 0 && array[j] > key) { array[j+1] = array[j]; j--; }
        array[j+1] = key;
    }
    return (array[len/2] + array[len/2 - 1]) / 2;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    uint16_t raw_L  = median_filter(sensor_data, 0, IR_SAMPLES_PERCH);
    uint16_t raw_FL = median_filter(sensor_data, 1, IR_SAMPLES_PERCH);
    uint16_t raw_FR = median_filter(sensor_data, 2, IR_SAMPLES_PERCH);
    uint16_t raw_R  = median_filter(sensor_data, 3, IR_SAMPLES_PERCH);

    LEFT_IR  = raw_L;
    FLEFT_IR = raw_FL;
    FRIGHT_IR= raw_FR;
    RIGHT_IR = raw_R;

    HAL_ADC_Stop_DMA(hadc);
    EmiterOFF();
    ir_status = OKAY;
    cur_phase = UPDATE_PHR;
}

void ReadIR(ADC_HandleTypeDef* hadc)
{
    if (ir_status == OKAY && cur_phase == SENSOR_PHR) {
        ir_status = BUSY;

        // Bật emitter nếu bạn đo chủ động loại bỏ nền (tùy chiến lược)
        EmiterON();
        // delay ngắn nếu LED cần ổn định (ví dụ vài trăm µs) — tránh HAL_Delay trong ISR

        HAL_ADC_Start_DMA(hadc, (uint32_t*)sensor_data, IR_BUF_LEN);
    }
}

bool Wall_Left()
{
	return (LEFT_IR > 700);
}
bool Wall_Right()
{
	return (RIGHT_IR > 700);
}
bool Wall_Front()
{
	return (FRIGHT_IR >700 && FLEFT_IR > 700);
}


