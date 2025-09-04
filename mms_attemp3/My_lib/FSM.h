/*
 * FSM.h
 *
 *  Created on: Sep 2, 2025
 *      Author: nguye
 */


#ifndef FSM_H_
#define FSM_H_

#include "stdbool.h"
#include "main.h"
#include "stdlib.h"

#define ACTQ_MAX 4 // max action mouse can do


typedef enum
{
	SENSOR_PHR,
	UPDATE_PHR,
	FINDPATH_PHR,
	ALGORITHM_PHR,
	EXECUTE_PHR,
	COMPLETE_PHR
}Phase;

typedef enum
{
	NONE_ACT,
	MOVE_ACT,
	TURN_LEFT_ACT,
	TURN_RIGHT_ACT,
	TURN_BACK_ACT
}Action_type;

typedef struct
{
	Action_type action[ACTQ_MAX];
	int8_t top;
}Action_Stack;


extern Action_Stack action;
extern Phase cur_phase;


bool Action_Stack_Empy(Action_Stack *a);
bool Action_Stack_Full(Action_Stack *a);

void Push_act(Action_Stack *a, Action_type act);
Action_type Execute_act(Action_Stack *a);




#endif /* FSM_H_ */
