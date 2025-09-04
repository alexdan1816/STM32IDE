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


extern I2C_HandleTypeDef hi2c1;


uint8_t whoami = 0;
float acc_raw[3] = {0};
float gyro_raw[3] = {0};
float acc_mg[3] = {0};
float gyro_dps[3] = {0};
float angle = 0;

static bool filt_init = false;
static float gz_filt = 0.0f;
float gyro_bias = -4.03166771;

static uint32_t last_ms = 0;

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

void LSM6DS3_ReadMulti(uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c1, LSM6DS3_ADDR, reg, 1, buf, len, 100);
}

void LSM6DS3_Init(void)
{
    whoami = LSM6DS3_Read(REG_WHOAMI); // WHOAMI phải ra 0x69

    // Reset chip
    LSM6DS3_Write(REG_CTRL3_C, 0x01);
    HAL_Delay(50);

    // Bật accel: ODR=104Hz, ±2g, BW=100Hz
    LSM6DS3_Write(REG_CTRL1_XL, 0x40);

    // Bật gyro: ODR=104Hz, 245 dps
    LSM6DS3_Write(REG_CTRL2_G, 0x40);
}

//void LSM6DS3_ReadAccel(void)
//{
//    uint8_t buf[6];
//    LSM6DS3_ReadMulti(REG_OUTX_L_XL, buf, 6);
//
//    acc_raw[0] = (int16_t)(buf[1] << 8 | buf[0]);
//    acc_raw[1] = (int16_t)(buf[3] << 8 | buf[2]);
//    acc_raw[2] = (int16_t)(buf[5] << 8 | buf[4]);
//
//    // Đổi ra mg (±2g = 0.061 mg/LSB)
//    acc_mg[0] = acc_raw[0] * 0.061f;
//    acc_mg[1] = acc_raw[1] * 0.061f;
//    acc_mg[2] = acc_raw[2] * 0.061f;
//}

float LSM6DS3_ReadGyro(void)
{
    uint8_t buf[6];
    LSM6DS3_ReadMulti(REG_OUTX_L_G, buf, 6);

    gyro_raw[0] = (int16_t)(buf[1] << 8 | buf[0]);
    gyro_raw[1] = (int16_t)(buf[3] << 8 | buf[2]);
    gyro_raw[2] = (int16_t)(buf[5] << 8 | buf[4]);

    // Đổi ra dps (245 dps = 8.75 mdps/LSB = 0.00875 dps/LSB)
    gyro_dps[0] = gyro_raw[0] * 0.00875f;
    gyro_dps[1] = gyro_raw[1] * 0.00875f;
    gyro_dps[2] = gyro_raw[2] * 0.00875f;

    return gyro_dps[2];
}

float Gyro_ReadYawDps(){
	float gz = LSM6DS3_ReadGyro() - gyro_bias;
	if (!filt_init) {
		gz_filt = gz;
		filt_init = true;
		return gz_filt;
	}

	const float alpha = 0.15f;
    gz_filt = (1.0f - alpha)*gz_filt + alpha*gz;
    return gz_filt;            // dps đã lọc
}
void Gyro_UpdateAngle(){
    uint32_t now = HAL_GetTick();
    float dt = (last_ms==0) ? 0.05f : (now - last_ms)/1000.0f;

    float gz_dps = Gyro_ReadYawDps();
    angle += gz_dps * dt;

    if (angle > 180.f)  angle -= 360.f;
    if (angle < -180.f) angle += 360.f;

    last_ms = now;
}
;


