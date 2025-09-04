/*
 * motor.c
 *
 *  Created on: Aug 26, 2025
 *      Author: nguye
 */
#include "stm32f1xx_hal.h"
#include <pid.h>

//----------------------initialize
#include "motor.h"
#include "math.h"
#include "gyro.h"
#include "stdlib.h"
#include "stdint.h"

//debug
uint16_t count = 0;
uint16_t countR = 0;
uint16_t countL = 0;
uint32_t SAMPLE_T = 50;


State cur_state = IDLE;

bool already = false;
bool time_out = false;

volatile int16_t encoder_prev_R = 0;
volatile int16_t encoder_prev_L = 0;
volatile int16_t encoder_now_R = 0;
volatile int16_t encoder_now_L = 0;
volatile int16_t prog_R = 0;
volatile int16_t prog_L = 0;

float s_require = 150;
float s_remain = 0;

float st_require = 60;
float st_remain = 0;

float angle_require = 0;
float angle_remain = 0;

float v_stop = 0;
float v_max = 356;
float v_next = 0;

float a_up = 900;
float a_down = 900;

void Motor_Init(Motor *_motor,
                Motor_id id,
                GPIO_TypeDef *_IN1_Port, uint16_t _IN1_Pin,
                GPIO_TypeDef *_IN2_Port, uint16_t _IN2_Pin,
                TIM_HandleTypeDef *htim_pwm, uint32_t Channel,
                TIM_HandleTypeDef *htim_encoder,
                float kp, float ki, float kd)
{
    // Gán ID
    _motor->id = id;

    // Lưu thông tin chân điều khiển chiều
    _motor->IN1_Port = _IN1_Port;
    _motor->IN1_Pin  = _IN1_Pin;
    _motor->IN2_Port = _IN2_Port;
    _motor->IN2_Pin  = _IN2_Pin;

    // PWM
    _motor->htim_pwm = htim_pwm;
    _motor->Channel  = Channel;


    // Encoder
    _motor->htim_encoder = htim_encoder;
    _motor->encoder_count = 0;
    _motor->prev_count    = 0;
    _motor->delta_count   = 0;

    // PID
    _motor->kp = kp;
    _motor->ki = ki;
    _motor->kd = kd;

    // Speed
    _motor->cur_speed    = 0.0;
    _motor->target_speed = 0.0;
    _motor->Pid_output = 0.0;

    // Khởi động PWM
    HAL_TIM_PWM_Start(_motor->htim_pwm, _motor->Channel);

    // Khởi động Encoder
    HAL_TIM_Encoder_Start(_motor->htim_encoder, TIM_CHANNEL_ALL);
}

//----------------------Measurement
void Motor_UpdateSpeed(Motor *_motor, float new);
void Motor_GetSpeed(Motor *_motor)
{
	_motor->encoder_count = __HAL_TIM_GET_COUNTER(_motor->htim_encoder);
	_motor->delta_count = _motor->encoder_count - _motor->prev_count;

	int sign = (_motor->id == LEFT)?- 1: 1;

	_motor->cur_speed = (sign)*( _motor->delta_count/ 570.0) * (60.0 / (SAMPLE_T / 1000.0)); // RPM
	_motor->prev_count = _motor->encoder_count;

}

//----------------------Control
//void Motor_SetDir(Motor *motor)
//{
////	switch (motor->id) {
////		case RIGHT:
//	if(motor->id == RIGHT){
//			if(motor->Pid_output > 0)
//			{
//				HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, SET);
//				HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, RESET);
//			}
//			else if(motor->Pid_output < 0)
//			{
//				HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, SET);
//				HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, RESET);
//			}
//			else
//			{
//				HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, RESET);
//				HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, RESET);
//			}
//			countR++;
//	}
////			break;
////		case LEFT:
//	else if (motor->id == LEFT) {
//			if(motor->pwm > 0)
//			{
//				HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, RESET);
//				HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, SET);
//			}
//			else if(motor->pwm < 0)
//			{
//				HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, RESET);
//				HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, SET);
//			}
//			else
//			{
//				HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, RESET);
//				HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, RESET);
//			}
//			countL++;
//	}
////		default:
////			break;
//}
void PWM_limit(Motor *motor)
{
	if(motor->Pid_output > 400)
	{
		motor->Pid_output = 400;
	}
	else if(motor->Pid_output <0)
	{
		motor->Pid_output = 0;
	}

}
void Motor_SetPwm(Motor *motor)
{
	if(motor->Pid_output > 499)
	{
		motor->Pid_output = 499;
	}
	else if(motor->Pid_output < -499)
	{
		motor->Pid_output = -499;
	}
	if(motor->id == RIGHT)
	{
		if(motor->Pid_output > 0)
		{
			HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, SET);
			HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, RESET);
		}
		else if(motor->Pid_output < 0)
		{
			HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, RESET);
			HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, SET);
		}
		else
		{
			HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, RESET);
			HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, RESET);
		}
		motor->htim_pwm->Instance->CCR2 =(uint32_t)((fabs)(motor->Pid_output) );
		}
	else if (motor->id == LEFT)
	{
		if(motor->Pid_output > 0)
		{
			HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, RESET);
			HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, SET);
		}
		else if(motor->Pid_output < 0)
		{
			HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, SET);
			HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, RESET);
		}
		else
		{
			HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, RESET);
			HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, RESET);
		}
		motor->htim_pwm->Instance->CCR1 =(uint32_t)((fabs)(motor->Pid_output) );

	}


}
void Motor_stop(Motor *motor);
void Motor_SetTarget(Motor*motor, double target)
{
	motor->target_speed = target;
}


//----------------------Advance Control
void Stabilize(Motor *right, Motor *left)
{
	right->target_speed = - 5*angle;
	left->target_speed = 5*angle;
}



void Move_forward(Motor *_motorL, Motor *_motorR)
{
		if(cur_state == IDLE && time_out)
		{
			encoder_prev_R = __HAL_TIM_GET_COUNTER(_motorR->htim_encoder);
			encoder_prev_L = __HAL_TIM_GET_COUNTER(_motorL->htim_encoder);

			cur_state = MOVE;

			s_remain = s_require;

			v_next = a_up*DT;
		}
		if(cur_state == MOVE)
		{
			encoder_now_R = __HAL_TIM_GET_COUNTER(_motorR->htim_encoder);
			encoder_now_L = __HAL_TIM_GET_COUNTER(_motorL->htim_encoder);
			prog_R = encoder_now_R - encoder_prev_R;
			prog_L = encoder_now_L - encoder_prev_L;

			s_remain = s_require - 0.5f*(abs(prog_L) + abs(prog_R))*MM;  // xung còn lại và chuyển sang mm
			if(s_remain < 1)
			{
				cur_state = IDLE;
				v_next = 0;
				Motor_SetTarget(_motorR, (double)v_next);
				Motor_SetTarget(_motorL, (double)v_next);
				time_out = false;
				return;

			}

			double v = 0.5*(_motorL->cur_speed + _motorR->cur_speed)*MMS;
			v_stop = sqrt(2*a_down*s_remain);

			if(v > v_stop)
			{
				v_next = fmax(v - a_down*DT, 0);
			}
			else if(v < fmin(v_max, v_stop))
			{
				v_next = fmin( v + a_up*DT, fmin(v_max, v_stop) );
			}
			else
			{
				v_next = fmin(v_max, v_stop);
			}

		}

		v_next = v_next/MMS;
		Motor_SetTarget(_motorR, (double)v_next);
		Motor_SetTarget(_motorL, (double)v_next);
}
void Move_backwar(Motor *_motorL, Motor *_motorR);
void Move_Left(Motor *_motorL, Motor *_motorR)
{
	if(cur_state == IDLE && time_out)
	{
		encoder_prev_R = __HAL_TIM_GET_COUNTER(_motorR->htim_encoder);
		encoder_prev_L = __HAL_TIM_GET_COUNTER(_motorL->htim_encoder);
		cur_state = TURN_LEFT;
		st_remain = st_require;
		v_next = a_up*DT;
	}
	if(cur_state == TURN_LEFT)
	{
		encoder_now_R = __HAL_TIM_GET_COUNTER(_motorR->htim_encoder);
		encoder_now_L = __HAL_TIM_GET_COUNTER(_motorL->htim_encoder);
		prog_R = encoder_now_R - encoder_prev_R;
		prog_L = encoder_now_L - encoder_prev_L;

		st_remain = st_require - 0.5f*(abs(prog_L) + abs(prog_R))*MM;  // xung còn lại và chuyển sang mm
		if(st_remain < 1)
		{
			cur_state = IDLE;
			v_next = 0;
			Motor_SetTarget(_motorR, (double)v_next);
			Motor_SetTarget(_motorL, (double)v_next);
			time_out = false;
			return;

		}

		double v = 0.5*( - _motorL->cur_speed + _motorR->cur_speed)*MMS;
		v_stop = sqrt(2*a_down*st_remain);

		if(v > v_stop)
		{
			v_next = fmax(v - a_down*DT, 0);
		}
		else if(v < fmin(v_max, v_stop))
		{
			v_next = fmin( v + a_up*DT, fmin(v_max, v_stop) );
		}
		else
		{
			v_next = fmin(v_max, v_stop);
		}

	}

	v_next = v_next/MMS;
	Motor_SetTarget(_motorR, (double)v_next);
	Motor_SetTarget(_motorL, -((double)v_next));

}
void Move_Right(Motor *_motorL, Motor *_motorR);



