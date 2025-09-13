/*
 * FSM.c
 *
 *  Created on: Sep 2, 2025
 *      Author: nguye
 */

#include <FSM.h>
#include "stdbool.h"
#include "main.h"
#include "stdlib.h"


volatile Phase cur_phase = SENSOR_PHR;
uint32_t led_time;
uint32_t buz_time;

void Action_Stack_Init(Action_Stack *a)
{
	for (int i = 0; i < ACTQ_MAX;  i ++)
	{
		a->action[i] = NONE_ACT;
	}
	a->top = -1;
	return;
}


bool Action_Stack_Empy(Action_Stack *a)
{
	if(a->top == -1) 	return true;
	else 				return false;
}
bool Action_Stack_Full(Action_Stack *a)
{
	if(a->top == ACTQ_MAX - 1) 	return true;
	else						return false;
}

void Push_act(Action_Stack *a, Action_type act)
{
	if(!Action_Stack_Full(a))
	{
		a->top += 1;
		a->action[a->top] = act;
		return;
	}
	else if(Action_Stack_Full(a))
		return;
}
Action_type Pop_act(Action_Stack *a)
{
	if(!Action_Stack_Empy(a))
	{
		a->top -=1;
		return a->action[a->top + 1];
	}
	else
		return NONE_ACT;
}

void LED_ON()
{
	if (cur_phase == BEGIN_PHR)
	{
		HAL_GPIO_WritePin(LED_FORWARD_GPIO_Port, LED_FORWARD_Pin, SET);
		HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, SET);
		HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, SET);
		HAL_GPIO_WritePin(LED_BACK_GPIO_Port, LED_BACK_Pin, SET);
	}
	return;
}
void LED_OFF()
{
	if (cur_phase != BEGIN_PHR)
		{
			HAL_GPIO_WritePin(LED_FORWARD_GPIO_Port, LED_FORWARD_Pin, RESET);
			HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, RESET);
			HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, RESET);
			HAL_GPIO_WritePin(LED_BACK_GPIO_Port, LED_BACK_Pin, RESET);
		}
	return;
}
void BUZ_ON()
{
	if(cur_phase == BEGIN_PHR)
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, SET);
	return;
}
void BUZ_OFF()
{
	if(cur_phase != BEGIN_PHR)
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, RESET);
	return;
}

void BUZ_TOG()
{
	if(cur_phase == BEGIN_PHR)
			HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
	return;
}
void LED_TOG()
{
	if(cur_phase == BEGIN_PHR)
	{
				HAL_GPIO_TogglePin(LED_FORWARD_GPIO_Port,LED_FORWARD_Pin);
				HAL_GPIO_TogglePin(LED_BACK_GPIO_Port, LED_BACK_Pin);
				HAL_GPIO_TogglePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin);
				HAL_GPIO_TogglePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin);
	}
			return;
}

