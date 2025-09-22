/*
 * gyro.c
 *
 *  Created on: Aug 26, 2025
 *      Author: nguye
 */
#include "stm32f1xx_hal.h"
#include <gyro.h>
#include "stdlib.h"
#include "stdbool.h"
#include "FSM.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

uint8_t whoami = 0;
//float buffer[6] = {0};
//float acc_raw[3] = {0};
//float gyro_raw[3] = {0};
//float acc_mg[3] = {0};
//float gyro_dps[3] = {0};

//static bool filt_init = false;
//static float gz_filt = 0.0f;
//float gyro_bias;
//float gz = 0;
//float offset_find;
//uint32_t offset_cout = 0;
//
//static uint32_t last_ms = 0;

//bool gyro_it_done = false;
bool gyro_ready = false;
GyroDS gyro_it_stage = gyro_done;

uint8_t  buffer[6] = {0};
int16_t  gyro_raw_x, gyro_raw_y, gyro_raw_z;
float  gyro_z_dps;
double   gyro_z;
double 	 gyro_z_filt;
float    gyro_z_sum;
double   gyro_z_offset;
uint16_t gyro_z_count;
int8_t   gyro_z_count_ignore;
volatile double   angle = 0;
volatile bool 	 gyro_calib_done = false; // extern
uint32_t prevtime_gyro;
uint32_t delttime_gyro;





void LSM6DS3_Write(uint8_t reg, uint8_t val)
{
    HAL_I2C_Mem_Write(&hi2c1, LSM6DS3_ADDR, reg, 1, &val, 1, 100);
}

uint8_t LSM6DS3_Read(uint8_t reg)
{
    uint8_t val = 0;
    HAL_I2C_Mem_Read(&hi2c1, LSM6DS3_ADDR, reg, 1, &val, 1, 100);
    return val;
}

void LSM6DS3_Init(void)
{
    whoami = LSM6DS3_Read(REG_WHOAMI); // WHOAMI phải ra 0x69

    // Reset chip
    LSM6DS3_Write(REG_CTRL3_C, 0x01);
    HAL_Delay(50);

    // BDU=1, IF_INC=1 để đọc multi-byte
    LSM6DS3_Write(REG_CTRL3_C, 0x44);

    // Bật accel: ODR=104Hz, ±2g, BW=100Hz
    LSM6DS3_Write(REG_CTRL1_XL, 0x40);

    // Bật gyro: ODR=104Hz, 245 dps
    LSM6DS3_Write(REG_CTRL2_G, 0x40);
}

float Gyro_Get_Dps()
{
	switch (gyro_it_stage) {
		case gyro_done:
			HAL_I2C_Mem_Read_IT(&hi2c1, LSM6DS3_ADDR, REG_OUTX_L_G, 1, buffer, 6);
			break;
		case gyro_busy:
			break;
	}
	return gyro_z_dps;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef * hi2c)
{
	if(hi2c->Instance != I2C1) return;

	gyro_raw_x = (buffer[1] << 8 ) | (buffer[0]);
	gyro_raw_y = (buffer[3] << 8 ) | (buffer[2]);
	gyro_raw_z = (buffer[5] << 8 ) | (buffer[4]);

	gyro_z_dps = (float)gyro_raw_z * 0.00875f;
	delttime_gyro = HAL_GetTick() - prevtime_gyro;

	if(cur_phase == GYRO_PHR)
	{
		if(gyro_z_count_ignore < 2)
		{
			gyro_z_count_ignore += 1;
			gyro_it_stage = gyro_done;
			return;
		}
		else
		{
			if(gyro_z_count < 500)
			{
				gyro_z_count += 1;
				gyro_z_sum += gyro_z_dps;
				gyro_it_stage = gyro_done;
				return;
			}
			else if(gyro_z_count == 500)
			{
				gyro_z_offset = gyro_z_sum / gyro_z_count;
				gyro_calib_done = true;
				gyro_it_stage = gyro_done;
				return;
			}
		}
	}
	if(cur_phase == EXECUTE_PHR)
	{
		gyro_z_dps -= gyro_z_offset;
		if(gyro_z_filt == 0)
			gyro_z_filt = gyro_z_dps;
		else
			gyro_z_filt = (1.0f - 0.15f)*gyro_z_filt + 0.15f*gyro_z_dps;

		angle += gyro_z_filt*delttime_gyro/1000;
		gyro_it_stage = gyro_done;
		return;
	}
	gyro_it_stage = gyro_done;
}
void Gyro_Calib()
{
	if(gyro_it_stage == gyro_done)
	{
		HAL_I2C_Mem_Read_IT(&hi2c1, LSM6DS3_ADDR, REG_OUTX_L_G, 1, buffer, 6);
		gyro_it_stage = gyro_busy;
		prevtime_gyro = HAL_GetTick();
	}
	else
		return;
}

void Gyro_Angle_update()
{
	if(gyro_it_stage == gyro_done)
	{
		HAL_I2C_Mem_Read_IT(&hi2c1, LSM6DS3_ADDR, REG_OUTX_L_G, 1, buffer, 6);
		gyro_it_stage = gyro_busy;
		prevtime_gyro = HAL_GetTick();
	}
	return;
}


