/*
 * FLOODFILL.c
 *
 *  Created on: Sep 2, 2025
 *      Author: nguye
 */
#include "main.h"
#include "stdbool.h"
#include "IR.h"
#include "FSM.h"
#include "FLOODFILL.h"
#include "motor.h"

uint32_t check_count_ff = 0;
int16_t min_value = 15;
int8_t calib_stage = 0; // to count time had calibrated

void MazeInitialize(Maze *maze)
{
	for(int i = 0; i < 16;i++)
	{
		for(int j = 0 ; j < 16; j++)
		{
			int dx = 0, dy = 0;
			if (i < 7) dx = 7 - i; else if (i > 8) dx = i - 8;
			if (j < 7) dy = 7 - j; else if (j > 8) dy = j - 8;
			maze->cells[i][j].value = (uint8_t)(dx + dy);
		}
	}
	for(int i = 0; i < 16 ; i++)
	{
		maze->HorizontalWall[0][i] = 1;
		maze->HorizontalWall[16][i] = 1;
		maze->VerticalWall[i][0]  = 1;
		maze->VerticalWall[i][16] = 1;
	}
}
void MazeUpdate(Maze *maze, MousePose *currentPose)
{
	if(cur_phase == UPDATE_PHR)
	{
		switch (currentPose->head) {
			case NORTH:
				maze->HorizontalWall[currentPose->y + 1][currentPose->x] = Wall_Front();
				maze->VerticalWall[currentPose->y][currentPose->x] = Wall_Left() ;
				maze->VerticalWall[currentPose->y][currentPose->x + 1] = Wall_Right() ;
				cur_phase = FINDPATH_PHR;
				break;
			case EAST:
				maze->VerticalWall[currentPose->y][currentPose->x + 1] = Wall_Front();
				maze->HorizontalWall[currentPose->y + 1][currentPose->x] = Wall_Left();
				maze->HorizontalWall[currentPose->y][currentPose->x] = Wall_Right();
				cur_phase = FINDPATH_PHR;
				break;
			case SOUTH:
				maze->HorizontalWall[currentPose->y][currentPose->x] = Wall_Front();
				maze->VerticalWall[currentPose->y][currentPose->x + 1] = Wall_Left() ;
				maze->VerticalWall[currentPose->y][currentPose->x] = Wall_Right() ;
				cur_phase = FINDPATH_PHR;
				break;
			case WEST:
				maze->VerticalWall[currentPose->y][currentPose->x] = Wall_Front();
				maze->HorizontalWall[currentPose->y][currentPose->x] = Wall_Left();
				maze->HorizontalWall[currentPose->y + 1][currentPose->x] = Wall_Right();
				cur_phase = FINDPATH_PHR;
				 break;
			default:
				break;
		}
	}
}

void MazeFloodFill(Maze *maze, Cell_Queue*q, MousePose *mousepose)
{
	if(cur_phase == ALGORITHM1_PHR)
	{
		EnqCellQueue(q, mousepose->x, mousepose->y);
				cur_phase = ALGORITHM2_PHR;
				return;
	}
	if(cur_phase == ALGORITHM2_PHR)
	{
		if(CellqueueEmpty(q))
		{
			cur_phase = FINDPATH_PHR;
		}
		else
		{
			cur_phase = ALGORITHM2_PHR;
		}

		Cell *q1 = DegCellQueue(q);
		int8_t x = q1->x;
		int8_t y = q1->y;

		if(!maze->HorizontalWall[y+1][x])
		{
			if(maze->cells[x][y+1].value + 1 == maze->cells[x][y].value)
			{
				return;
			}
			else
			{
				min_value = maze->cells[x][y+1].value;

			}
		}
		if(!maze->VerticalWall[y][x+1])
		{
			if(maze->cells[x+1][y].value + 1 == maze->cells[x][y].value)
			{
				return;
			}
			else
			{
				min_value = maze->cells[x+1][y].value < min_value ? maze->cells[x+1][y].value : min_value;
			}
		}
		if(!maze->HorizontalWall[y][x])
		{
			if(maze->cells[x][y - 1].value + 1 == maze->cells[x][y].value)
			{
				return;
			}
			else
			{
				min_value = maze->cells[x][y-1].value < min_value ? maze->cells[x][y-1].value : min_value;
			}
		}
		if(!maze->VerticalWall[y][x])
		{
			if(maze->cells[x-1][y].value + 1 == maze->cells[x][y].value)
			{
				return;
			}
			else
			{
				min_value = maze->cells[x-1][y].value < min_value ? maze->cells[x-1][y].value : min_value;
			}
		}
		if(min_value != 0)
		{
			maze->cells[x][y].value = min_value + 1;
			if(!maze->HorizontalWall[y+1][x]) EnqCellQueue(q, x, y + 1);
			if(!maze->VerticalWall[y][x+1]) EnqCellQueue(q, x + 1, y);
			if(!maze->HorizontalWall[y][x]) EnqCellQueue(q, x, y - 1);
			if(!maze->VerticalWall[y][x]) EnqCellQueue(q, x - 1, y);
		}
	}
	if(CellqueueEmpty(q))
	{
		cur_phase = FINDPATH_PHR;
	}
	else
	{
		cur_phase = ALGORITHM2_PHR;
	}
}
bool FindNextCell(Maze *maze, MousePose *mousepose, Action_Stack *action_stack)
{
	if(cur_phase == FINDPATH_PHR)
	{
	int8_t x = mousepose->x;
	int8_t y = mousepose->y;
	int8_t head = mousepose->head;
	switch (head) {
		case NORTH:
			if(!maze->HorizontalWall[y + 1][x])
			{
				if(LegalCell(x, y, x, y + 1, maze))
				{
					Push_act(action_stack, MOVE_ACT);
					return true;
				}
			}
			if(!maze->VerticalWall[y][x+ 1])
			{
				if(LegalCell(x, y, x + 1, y, maze))
				{
					Push_act(action_stack, MOVE_ACT);
					Push_act(action_stack, TURN_RIGHT_ACT);
					return true;
				}
			}
			if(!maze->VerticalWall[y][x])
			{
				if(LegalCell(x, y, x - 1, y, maze))
				{
					Push_act(action_stack, MOVE_ACT);
					Push_act(action_stack, TURN_LEFT_ACT);
					return true;
				}
			}
			if(!maze->HorizontalWall[y][x])
			{
				if(LegalCell(x, y, x, y - 1, maze))
				{
					Push_act(action_stack, TURN_BACK_ACT);
					return true;
				}
			}
			break;
		case EAST:
			if(!maze->HorizontalWall[y + 1][x])
			{
				if(LegalCell(x, y, x, y + 1, maze))
				{
					Push_act(action_stack, MOVE_ACT);
					Push_act(action_stack, TURN_LEFT_ACT);
					return true;
				}
			}
			if(!maze->VerticalWall[y][x + 1])
			{
				if(LegalCell(x, y, x + 1, y, maze))
				{
					Push_act(action_stack, MOVE_ACT);
					return true;
				}
			}
			if(!maze->VerticalWall[y][x])
			{
				if(LegalCell(x, y, x - 1, y, maze))
				{
					Push_act(action_stack, TURN_BACK_ACT);
					return true;
				}
			}
			if(!maze->HorizontalWall[y][x])
			{
				if(LegalCell(x, y, x, y - 1, maze))
				{
					Push_act(action_stack, MOVE_ACT);
					Push_act(action_stack, TURN_RIGHT_ACT);
					return true;
				}
			}
			break;
		case SOUTH:
			if(!maze->HorizontalWall[y + 1][x])
			{
				if(LegalCell(x, y, x, y + 1, maze))
				{
					Push_act(action_stack, TURN_BACK_ACT);
					return true;
				}
			}
			if(!maze->VerticalWall[y][x + 1])
			{
				if(LegalCell(x, y, x + 1, y, maze))
				{
					Push_act(action_stack, MOVE_ACT);
					Push_act(action_stack, TURN_LEFT_ACT);
					return true;
				}
			}
			if(!maze->VerticalWall[y][x])
			{
				if(LegalCell(x, y, x - 1, y, maze))
				{
					Push_act(action_stack, TURN_BACK_ACT);
					Push_act(action_stack, TURN_RIGHT_ACT);
					return true;
				}
			}
			if(!maze->HorizontalWall[y][x])
			{
				if(LegalCell(x, y, x, y - 1, maze))
				{
					Push_act(action_stack, MOVE_ACT);
					return true;
				}
			}
			break;
		case WEST:
			if(!maze->HorizontalWall[y + 1][x])
			{
				if(LegalCell(x, y, x, y + 1, maze))
				{
					Push_act(action_stack, MOVE_ACT);
					Push_act(action_stack, TURN_RIGHT_ACT);
					return true;
				}
			}
			if(!maze->VerticalWall[y][x + 1])
			{
				if(LegalCell(x, y, x + 1, y, maze))
				{
					Push_act(action_stack, TURN_BACK_ACT);
					return true;
				}
			}
			if(!maze->VerticalWall[y][x])
			{
				if(LegalCell(x, y, x - 1, y, maze))
				{
					Push_act(action_stack, MOVE_ACT);
					return true;
				}
			}
			if(!maze->HorizontalWall[y][x])
			{
				if(LegalCell(x, y, x, y - 1, maze))
				{
					Push_act(action_stack, MOVE_ACT);
					Push_act(action_stack, TURN_LEFT_ACT);
					return true;
				}
			}
			break;
	}

	return false;
	}
	return false;
}
bool LegalCell(int8_t curX ,int8_t curY, int8_t nextX, int8_t nextY, Maze *maze)
{
	if(maze->cells[curX][curY].value == maze->cells[nextX][nextY].value + 1)
		return true;
	else
		return false;
}


void PoseInit(MousePose *mousepose, Direction start_dir, int8_t start_x, int8_t start_y)
{
	mousepose->head = start_dir;
	mousepose->x = start_x;
	mousepose->y = start_y;
	return;
}
void PoseUpdate(MousePose *mousepose, Action_type act)
{
	switch (act) {
	case MOVE_ACT:
		switch (mousepose->head) {
		case NORTH:
			mousepose->y += 1;
			break;
		case SOUTH:
			mousepose->y -= 1;
			break;
		case EAST:
			mousepose->x += 1;
			break;
		case WEST:
			mousepose->x -= 1;
			break;
		default:
			break;
		}
		break;
	case TURN_LEFT_ACT:
		switch (mousepose->head) {
		case NORTH:
			mousepose->head = WEST;
			break;
		case SOUTH:
			mousepose->head = EAST;
			break;
		case EAST:
			mousepose->head = NORTH;
			break;
		case WEST:
			mousepose->head = SOUTH;
			break;
		default:
			break;
		}
		break;
	case TURN_RIGHT_ACT:
		switch (mousepose->head) {
		case NORTH:
			mousepose->head = EAST;
			break;
		case SOUTH:
			mousepose->head = WEST;
			break;
		case EAST:
			mousepose->head = SOUTH;
			break;
		case WEST:
			mousepose->head = NORTH;
			break;
		default:
			break;
		}
		break;
	case TURN_BACK_ACT:
		switch (mousepose->head) {
		case NORTH:
			mousepose->head = SOUTH;
			break;
		case SOUTH:
			mousepose->head = NORTH;
			break;
		case EAST:
			mousepose->head = WEST;
			break;
		case WEST:
			mousepose->head = EAST;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
	return;
}

void CellQueueInitialize(Cell_Queue*cellqueue)
{
	cellqueue->head = cellqueue->tail = 0;
	cellqueue->count = 0;
	return;
}
bool CellqueueEmpty(Cell_Queue*cellqueue)
{
	if(cellqueue->count == 0)
		return true;
	else
		return false;
}
bool CellqueueFull(Cell_Queue*cellqueue)
{
	if(cellqueue->count == MAZEMAXCELLS)
		return true;
	else
		return false;
}
Cell *DegCellQueue(Cell_Queue*q)
{
    if (CellqueueEmpty(q)) return NULL;
    Cell* c = &q->cell_list[q->head];
    q->head = (q->head + 1) % MAZEMAXCELLS;
    q->count--;
    return c;
}
void EnqCellQueue(Cell_Queue *q, int8_t x, int8_t y)
{
    if (CellqueueFull(q)) return;
    q->cell_list[q->tail].x = x;
    q->cell_list[q->tail].y = y;
    q->tail = (q->tail + 1) % MAZEMAXCELLS;
    q->count++;
    return;
}

void ExecuteAct(MousePose *m, Action_Stack *s)
{
	if(cur_phase == EXECUTE_PHR)
	{
		if(Action_Stack_Empy(s))
		{
			if(calib_stage == 1)
			{
				cur_phase = CALIB_PHR;
				return;
			}
			check_count_ff++;
			cur_phase = SENSOR_PHR;
			return;
		}
		else
		{
			Action_type a = Pop_act(s);
			switch (a) {
				case MOVE_ACT:
					cur_state  = MOVE;
					break;
				case TURN_LEFT_ACT:
					cur_state = TURN_LEFT;
					break;
				case TURN_RIGHT_ACT:
					cur_state = TURN_RIGHT;
					break;
				case TURN_BACK_ACT:
					cur_state = TURN_BACK;
					break;
				default:
					break;
			}
			PoseUpdate(m, a);
		}
	}
}
bool CheckGoal(MousePose *m, Maze *ma)
{
	if(ma->cells[m->x][m->y].value == 0)
	{
		return true;
	}
	return false;
}

bool CalibCornetExit(MousePose *m, Maze *ma, Action_Stack *as)
{
	int8_t x = m->x;
	int8_t y = m->y;
	switch (m->head) {
		case NORTH:
			if( ma->HorizontalWall[y + 1][x] && (ma->VerticalWall[y][x] || ma->VerticalWall[y][x + 1]) )
			{
				if(ma->VerticalWall[y][x])
				{
					Push_act(as, TURN_LEFT_ACT);
					return true;
				}
				else if(ma->VerticalWall[y][x+1])
				{
					Push_act(as, TURN_RIGHT_ACT);
					return true;
				}
			}
			else
				return false;
			break;
		case EAST:
			if( ma->VerticalWall[y][x + 1] && (ma->HorizontalWall[y + 1][x] || ma->HorizontalWall[y][x]) )
			{
				if(ma->HorizontalWall[y][x])
				{
					Push_act(as, TURN_RIGHT_ACT);
					return true;
				}
				else if(ma->HorizontalWall[y+1][x])
				{
					Push_act(as, TURN_LEFT_ACT);
					return true;
				}
			}
			else
				return false;
			break;
		case SOUTH:
			if( ma->HorizontalWall[y][x] && (ma->VerticalWall[y][x] || ma->VerticalWall[y][x+1]) )
			{
				if(ma->VerticalWall[y][x])
				{
					Push_act(as, TURN_RIGHT_ACT);
					return true;
				}
				else if(ma->VerticalWall[y][x+1])
				{
					Push_act(as, TURN_LEFT_ACT);
					return true;
				}
			}
			else
				return false;
			break;
		case WEST:
			if( ma->VerticalWall[y][x] && (ma->HorizontalWall[y + 1][x] || ma->HorizontalWall[y][x]) )
			{
				if(ma->HorizontalWall[y][x])
				{
					Push_act(as, TURN_LEFT_ACT);
					return true;
				}
				else if(ma->HorizontalWall[y+1][x])
				{
					Push_act(as, TURN_RIGHT_ACT);
					return true;
				}
			}
			else
				return false;
			break;
		default:
			return false;
			break;
	}
	return false;
}

