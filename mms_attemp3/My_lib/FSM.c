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

