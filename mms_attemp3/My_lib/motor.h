/*
 * motor.h
 *
 *  Created on: Aug 26, 2025
 *      Author: nguye
 */

#ifndef MOTOR_H_
#define MOTOR_H_

//#define SAMPLE_T 50
#include"stdbool.h"
#include"pid.h"

#define MMS 1.7802f // Change from RPM to mm/s
#define MM 0.1874f // Chang from pulse to mm
#define DT 0.05f // DELTA time
#define RADIUS 17 // PIVOT
#define SAMPLE_T 2 // SAMPLE TIME USE FOR VELOCITY CACULATOR

#define PULSE_TO_DEG 0.23
#define TURN_TOLERANCE 8
#define BACK_TOLERANCE 17
#define TURN_DEG 110
#define BACK_DEG 230

#define ENCODER_PPR 570     // pulses per revolution



extern uint32_t prevtime;

typedef enum
{
	LEFT,
	RIGHT
}Motor_id;

typedef enum
{
	CLK_WISE,
	ANTI_CLK_WISE
}Motor_dir;

typedef enum
{
	IDLE,
	MOVE,
	TURN_LEFT,
	TURN_RIGHT,
	TURN_BACK,
	COOL_DOWN
}State;

typedef struct
{
	State _state;

	bool active;

	float a_up;
	float a_down;

	float v_max;
	float v_min;

	double v;


}Motion;

typedef struct
{
    Motor_id id;

    // Điều khiển chiều
    GPIO_TypeDef *IN1_Port;
    uint16_t IN1_Pin;
    GPIO_TypeDef *IN2_Port;
    uint16_t IN2_Pin;

    // Encoder
    TIM_HandleTypeDef *htim_encoder;
    int16_t encoder_count;
    int16_t prev_count;
    int16_t delta_count;
    int16_t progress_count;

    // PID
    float kp;
    float ki;
    float kd;

    // PWM output
    TIM_HandleTypeDef *htim_pwm;
    uint32_t Channel;

    // Tốc độ hiện tại
    double cur_speed;	// input pwwm

    // Tốc độ mục tiêu
    double target_speed; // setpoint pwwm

    // Adjust value from PID
    double Pid_output;


} Motor;



extern double encoder_progress;
extern double encoder_output;
extern double encoder_target;


extern volatile bool begin_flag;
extern volatile State cur_state;

extern volatile State pre_calib_state;
extern volatile bool calib_value_take;
extern volatile bool calib_ir_start_flag;
extern volatile bool calib_ir_done_flag;
extern volatile bool calib_done;
extern volatile bool calib_start;
extern volatile int8_t calib_turn;
extern volatile bool calib_flag;


extern double frightIRsetvalue;
extern double fleftIRsetvalue;
extern double frightIRoutput;
extern double fleftIRoutput;
extern double frightIRin;
extern double fleftIRin;

extern uint16_t count;

//----------------------initialize
void Motor_Init(Motor *_motor,
        		Motor_id id,
                GPIO_TypeDef *_IN1_Port, uint16_t _IN1_Pin,
                GPIO_TypeDef *_IN2_Port, uint16_t _IN2_Pin,
                TIM_HandleTypeDef *htim_pwm, uint32_t Channel,
                TIM_HandleTypeDef *htim_encoder,
				float kp, float ki, float kd);
//----------------------Measurement
void Motor_UpdateSpeed(Motor *_motor, float new);
void Motor_GetSpeed(Motor *motor);

//----------------------Control

//void Motor_SetDir(Motor *motor);
void PWM_limit(Motor *motor);
void Motor_SetPwm(Motor *motor);
void Motor_stop(Motor *motor);
void Motor_SetTarget(Motor*motor, double target);

//----------------------Advance Control


void Move_forward(Motor *_motorL, Motor *_motorR);
void Move_backward(Motor *_motorL, Motor *_motorR);
void Move_Left(Motor *_motorL, Motor *_motorR);
void Move_Right(Motor *_motorL, Motor *_motorR);
void Move_Left_enhanced(Motor *_motorL, Motor*_motorR, float accelerate);

void Calib_Move(Motor *_motorL, Motor *_motorR,ADC_HandleTypeDef* hadc, PID_TypeDef *RIGHT, PID_TypeDef *LEFT);


#endif /* MOTOR_H_ */
