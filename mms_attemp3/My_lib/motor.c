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

	bool time_out = false;

	//turn action variables
	volatile int16_t Tencoder_prev_R = 0;
	volatile int16_t Tencoder_prev_L = 0;
	volatile int16_t Tencoder_now_R = 0;
	volatile int16_t Tencoder_now_L = 0;
	volatile int16_t Tprog_R = 0;
	volatile int16_t Tprog_L = 0;
	//move action variavbles
	volatile int16_t Mencoder_prev_R = 0;
	volatile int16_t Mencoder_prev_L = 0;
	volatile int16_t Mencoder_now_R = 0;
	volatile int16_t Mencoder_now_L = 0;
	volatile int16_t Mprog_R = 0;
	volatile int16_t Mprog_L = 0;
	//---------------------Target degree of turn but not accurate ; unit : degree
	double encoder_progress = 0;
	double encoder_output = 0;
	double encoder_target = 0;
	volatile bool begin_flag = false;
	//---------------------Target length of move forward but not accurate; unit : mm
	float s_require = 150;
	float s_remain = 0;
	//*********************Speed profile of move forward but not accurate; unit : rpm
	float v_stop = 0;
	float v_max = 356;
	float v_next = 0;

	float a_up = 1200;
	float a_down = 1200;

	//--------------------State for executing action
	volatile State cur_state ;
	//--------------------State for preCalibration
	volatile State pre_calib_state = IDLE;
	volatile bool calib_value_take = false;
	volatile bool calib_ir_start_flag = false;
	volatile bool calib_done = false;
	volatile bool calib_start = false;
	volatile int8_t calib_turn = 0;
	volatile bool calib_ir_done_flag = false;

 double frightIRsetvalue = 0;
 double fleftIRsetvalue = 0;
 double frightIRoutput = 0;
 double fleftIRoutput = 0;

	uint32_t prevtime = 0;
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
		_motor->progress_count = 0;

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

		_motor->cur_speed = (sign)*( _motor->delta_count/ ENCODER_PPR) * (60.0 / (2 / 1000.0)); // RPM
		_motor->prev_count = _motor->encoder_count;

	}

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
			if(cur_state == MOVE && !begin_flag)
			{
				Mencoder_prev_R = __HAL_TIM_GET_COUNTER(_motorR->htim_encoder);
				Mencoder_prev_L = __HAL_TIM_GET_COUNTER(_motorL->htim_encoder);

				cur_state = MOVE;

				s_remain = s_require;

				v_next = a_up*DT;

				begin_flag = true;

				HAL_GPIO_WritePin(LED_FORWARD_GPIO_Port, LED_FORWARD_Pin, SET);
			}
			else if(cur_state == MOVE && begin_flag)
			{
				Mencoder_now_R = __HAL_TIM_GET_COUNTER(_motorR->htim_encoder);
				Mencoder_now_L = __HAL_TIM_GET_COUNTER(_motorL->htim_encoder);
				Mprog_R = Mencoder_now_R - Mencoder_prev_R;
				Mprog_L = Mencoder_now_L - Mencoder_prev_L;

				s_remain = s_require - 0.5f*(abs(Mprog_L) + abs(Mprog_R))*MM;  // xung còn lại và chuyển sang mm
				if(s_remain < 1)
				{
					v_next = 0;
					Motor_SetTarget(_motorR, (double)v_next);
					Motor_SetTarget(_motorL, (double)v_next);
					_motorL->Pid_output = 0;
					_motorR->Pid_output = 0;
					__HAL_TIM_SET_COUNTER(_motorL->htim_encoder, 0);
					__HAL_TIM_SET_COUNTER(_motorR->htim_encoder, 0);
					HAL_GPIO_WritePin(LED_FORWARD_GPIO_Port, LED_FORWARD_Pin, RESET);
					begin_flag = false;
					cur_state = COOL_DOWN;
					return;
				}
				else{
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

			}
			v_next = v_next/MMS;
			Motor_SetTarget(_motorR, (double)v_next);
			Motor_SetTarget(_motorL, (double)v_next);
	}
	void Move_backward(Motor *_motorL, Motor *_motorR)
	{
		if(cur_state == TURN_BACK && !begin_flag)
		{
			Tencoder_prev_R = __HAL_TIM_GET_COUNTER(_motorR->htim_encoder);
			Tencoder_prev_L = __HAL_TIM_GET_COUNTER(_motorL->htim_encoder);
//			cur_state = TURN_BACK;
			encoder_target   = BACK_DEG;
			encoder_progress = 0;
			encoder_output   = 0;
			begin_flag = true;
			HAL_GPIO_WritePin(LED_BACK_GPIO_Port, LED_BACK_Pin, SET);
			return;
		}
		else if(cur_state == TURN_BACK && begin_flag)
		{
			//READING CURRENT ENCODER
			Tencoder_now_R = __HAL_TIM_GET_COUNTER(_motorR->htim_encoder);
			Tencoder_now_L = __HAL_TIM_GET_COUNTER(_motorL->htim_encoder);
			//READIGN DELTA ENCODER
			Tprog_R = Tencoder_now_R - Tencoder_prev_R;
			Tprog_L = Tencoder_now_L - Tencoder_prev_L;
			//RESET PREVIOUS ENCODER
			Tencoder_prev_L = Tencoder_now_L;
			Tencoder_prev_R = Tencoder_now_R;

			encoder_progress += PULSE_TO_DEG*0.5*(double)(Tprog_L + Tprog_R); // turn from pulse to degree

			if(encoder_target - encoder_progress < BACK_TOLERANCE)
			{
				_motorL->Pid_output = 0;
				_motorR->Pid_output = 0;
	//			Motor_SetPwm(_motorR);
	//			Motor_SetPwm(_motorL);
				__HAL_TIM_SET_COUNTER(_motorL->htim_encoder, 0);
				__HAL_TIM_SET_COUNTER(_motorR->htim_encoder, 0);
				cur_state = COOL_DOWN;
				HAL_GPIO_WritePin(LED_BACK_GPIO_Port, LED_BACK_Pin, RESET);
				begin_flag = false;
				prevtime = HAL_GetTick();
				return;
			}
		}
		_motorL->Pid_output = -(encoder_output * 499)/90;
		_motorR->Pid_output = (encoder_output)*499/90;

	}
	void Move_Left(Motor *_motorL, Motor *_motorR)
	{
				if(cur_state == TURN_LEFT && !begin_flag)
				{
					Tencoder_prev_R = __HAL_TIM_GET_COUNTER(_motorR->htim_encoder);
					Tencoder_prev_L = __HAL_TIM_GET_COUNTER(_motorL->htim_encoder);
					cur_state = TURN_LEFT;
					encoder_target   = TURN_DEG;
					encoder_progress = 0;
					encoder_output   = 0;
					begin_flag = true;
					HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, SET);
					return;
				}
				else if(cur_state == TURN_LEFT && begin_flag)
				{
					//READING CURRENT ENCODER
					Tencoder_now_R = __HAL_TIM_GET_COUNTER(_motorR->htim_encoder);
					Tencoder_now_L = __HAL_TIM_GET_COUNTER(_motorL->htim_encoder);
					//READIGN DELTA ENCODER
					Tprog_R = Tencoder_now_R - Tencoder_prev_R;
					Tprog_L = Tencoder_now_L - Tencoder_prev_L;
					//RESET PREVIOUS ENCODER
					Tencoder_prev_L = Tencoder_now_L;
					Tencoder_prev_R = Tencoder_now_R;

					encoder_progress += PULSE_TO_DEG*0.5*(double)(Tprog_L + Tprog_R); // turn from pulse to degree

					if(encoder_target - encoder_progress < TURN_TOLERANCE)
					{
						_motorL->Pid_output = 0;
						_motorR->Pid_output = 0;
			//			Motor_SetPwm(_motorR);
			//			Motor_SetPwm(_motorL);
						__HAL_TIM_SET_COUNTER(_motorL->htim_encoder, 0);
						__HAL_TIM_SET_COUNTER(_motorR->htim_encoder, 0);
						cur_state = COOL_DOWN;
						HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, RESET);
						begin_flag = false;
						prevtime = HAL_GetTick();
						return;
					}
				}
				_motorL->Pid_output = -(encoder_output * 499)/90;
				_motorR->Pid_output = (encoder_output)*499/90;

		// BẢN ỔN
//		if(cur_state == TURN_LEFT && !begin_flag)
//		{
//			Tencoder_prev_R = __HAL_TIM_GET_COUNTER(_motorR->htim_encoder);
//			Tencoder_prev_L = __HAL_TIM_GET_COUNTER(_motorL->htim_encoder);
//			cur_state = TURN_LEFT;
//			encoder_target   = TURN_DEG;
//			encoder_progress = 0;
//			encoder_output   = 0;
//			begin_flag = true;
//			HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, SET);
//			return;
//		}
//		else if(cur_state == TURN_LEFT && begin_flag)
//		{
//			//READING CURRENT ENCODER
//			Tencoder_now_R = __HAL_TIM_GET_COUNTER(_motorR->htim_encoder);
//			Tencoder_now_L = __HAL_TIM_GET_COUNTER(_motorL->htim_encoder);
//			//READIGN DELTA ENCODER
//			Tprog_R = Tencoder_now_R - Tencoder_prev_R;
//			Tprog_L = Tencoder_now_L - Tencoder_prev_L;
//			//RESET PREVIOUS ENCODER
//			Tencoder_prev_L = Tencoder_now_L;
//			Tencoder_prev_R = Tencoder_now_R;
//
//			encoder_progress += PULSE_TO_DEG*0.5*(double)(Tprog_L + Tprog_R); // turn from pulse to degree
//
//			if(encoder_target - encoder_progress < TURN_TOLERANCE)
//			{
//				_motorL->Pid_output = 0;
//				_motorR->Pid_output = 0;
//	//			Motor_SetPwm(_motorR);
//	//			Motor_SetPwm(_motorL);
//				__HAL_TIM_SET_COUNTER(_motorL->htim_encoder, 0);
//				__HAL_TIM_SET_COUNTER(_motorR->htim_encoder, 0);
//				cur_state = COOL_DOWN;
//				HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, RESET);
//				begin_flag = false;
//				prevtime = HAL_GetTick();
//				return;
//			}
//		}
//		_motorL->Pid_output = -(encoder_output * 499)/90;
//		_motorR->Pid_output = (encoder_output)*499/90;
	}
	void Move_Right(Motor *_motorL, Motor *_motorR)
	{
		if(cur_state == TURN_RIGHT && !begin_flag)
		{
			Tencoder_prev_R = __HAL_TIM_GET_COUNTER(_motorR->htim_encoder);
			Tencoder_prev_L = __HAL_TIM_GET_COUNTER(_motorL->htim_encoder);
			cur_state = TURN_RIGHT;
			encoder_target   = TURN_DEG;
			encoder_progress = 0;
			encoder_output   = 0;
			begin_flag = true;
			HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, SET);
			return;
		}
		else if(cur_state == TURN_RIGHT && begin_flag)
		{
			//READING CURRENT ENCODER
			Tencoder_now_R = __HAL_TIM_GET_COUNTER(_motorR->htim_encoder);
			Tencoder_now_L = __HAL_TIM_GET_COUNTER(_motorL->htim_encoder);
			//READIGN DELTA ENCODER
			Tprog_R = Tencoder_now_R - Tencoder_prev_R;
			Tprog_L = Tencoder_now_L - Tencoder_prev_L;
			//RESET PREVIOUS ENCODER
			Tencoder_prev_L = Tencoder_now_L;
			Tencoder_prev_R = Tencoder_now_R;

			encoder_progress +=  PULSE_TO_DEG*0.5*(double)(- Tprog_L - Tprog_R); // turn from pulse to degree

			if(encoder_target - encoder_progress < TURN_TOLERANCE)
			{
				_motorL->Pid_output = 0;
				_motorR->Pid_output = 0;
	//			Motor_SetPwm(_motorR);
	//			Motor_SetPwm(_motorL);
				__HAL_TIM_SET_COUNTER(_motorL->htim_encoder, 0);
				__HAL_TIM_SET_COUNTER(_motorR->htim_encoder, 0);
				HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, RESET);
				cur_state = COOL_DOWN;
				begin_flag = false;
				prevtime = HAL_GetTick();
				return;
			}
		}
		_motorL->Pid_output = (encoder_output * 499)/90;
		_motorR->Pid_output = -(encoder_output)*499/90;
	}

void Calib_Move(Motor *_motorL, Motor *_motorR)
{

}

