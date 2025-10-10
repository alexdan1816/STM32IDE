/*
 * gyro.h
 *
 *  Created on: Aug 26, 2025
 *      Author: nguye
 */

#ifndef GYRO_H_
#define GYRO_H_

#include "stdbool.h"
// Địa chỉ
#define LSM6DS3_ADDR (0x6B << 1)

// Thanh ghi
#define REG_WHOAMI 0x0F
#define REG_CTRL1_XL 0x10
#define REG_CTRL2_G 0x11
#define REG_CTRL3_C 0x12
#define REG_OUTX_L_G 0x22
#define REG_OUTX_L_XL 0x28

extern uint8_t whoami;
extern float acc_raw[3];
extern float gyro_raw[3];
extern float acc_mg[3];
extern float gyro_dps[3];
extern volatile double angle;

extern float gyro_bias;

extern bool gyro_it_done;
extern bool gyro_ready;

extern double gyro_pid_in;
extern double gyro_pid_out;
extern double gyro_pid_set;
extern double gyro_set_point;

extern volatile bool gyro_calib_done;

typedef enum
{
	gyro_busy,
	gyro_done
} GyroDS; // Data stage of gyro when using interupt,

//--------------------prototype---------------------

void LSM6DS3_Write(uint8_t reg, uint8_t val);
uint8_t LSM6DS3_Read(uint8_t reg);
void LSM6DS3_Init(void);
float Gyro_Get_Dps();
// void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef * hi2c)
void Gyro_Calib();
void Gyro_Angle_update();
// void LSM6DS3_Write(uint8_t reg, uint8_t val);
// uint8_t LSM6DS3_Read(uint8_t reg);
// void LSM6DS3_ReadMulti(uint8_t reg, uint8_t *buf, uint8_t len);
//
// void LSM6DS3_Init(I2C_HandleTypeDef * hi2c);
//
// void LSM6DS3_ReadAccel(void);
//
// float LSM6DS3_ReadGyro(void);
//
////void gyro_Callback();
//
// float Gyro_ReadYawDps();
//
// void Gyro_UpdateAngle();
//
// void Gyro_ResetAngle(void);
//
// bool Gyro_Calibrate();

#endif /* GYRO_H_ */
