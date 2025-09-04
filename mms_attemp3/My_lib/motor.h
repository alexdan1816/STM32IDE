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
	TURN_BACK
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

// Debug

extern uint16_t count;
extern uint16_t countR;
extern uint16_t countL;
extern uint32_t SAMPLE_T;

extern State cur_state;
extern bool already;
extern bool time_out;

extern volatile int16_t encoder_prev_R ;
extern volatile int16_t encoder_prev_L ;
extern volatile int16_t encoder_now_R;
extern volatile int16_t encoder_now_L;
extern volatile int16_t prog_R ;
extern volatile int16_t prog_L ;

extern float s_require;
extern float s_remain;

extern float angle_require;
extern float angle_remain;

extern float v_stop;
extern float v_max;
extern float v_next;

extern float a_up ;
extern float a_down ;



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
void Stabilize(Motor *right, Motor *left);


void Move_forward(Motor *_motorL, Motor *_motorR);
void Move_backwar(Motor *_motorL, Motor *_motorR);
void Move_Left(Motor *_motorL, Motor *_motorR);
void Move_Right(Motor *_motorL, Motor *_motorR);

void Move_Left_enhanced(Motor *_motorL, Motor*_motorR, float accelerate);


#endif /* MOTOR_H_ */
