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


void MazeInitialize(Maze *maze)
{
	for(int i = 0; i < 16;i++)
	{
		for(int j = 0 ; j < 16; j++)
		{
			int d1 = abs(i - 7) + abs(j - 7);
			int d2 = abs(i - 8) + abs(j - 8);
			maze->cells[i][j].value = (uint8_t)((d1 < d2) ? d1 : d2);  // 0..14
			maze->cells[i][j].x = i;
			maze->cells[i][j].y = j;
			maze->cells[i][j].visited = false;
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
				cur_phase = ALGORITHM_PHR;
				break;
			case EAST:
				maze->VerticalWall[currentPose->y][currentPose->x + 1] = Wall_Front();
				maze->HorizontalWall[currentPose->y + 1][currentPose->x] = Wall_Left();
				maze->HorizontalWall[currentPose->y][currentPose->x] = Wall_Right();
				cur_phase = ALGORITHM_PHR;
				break;
			case SOUTH:
				maze->HorizontalWall[currentPose->y][currentPose->x] = Wall_Front();
				maze->VerticalWall[currentPose->y][currentPose->x + 1] = Wall_Left() ;
				maze->VerticalWall[currentPose->y][currentPose->x] = Wall_Right() ;
				cur_phase = ALGORITHM_PHR;
			case WEST:
				maze->VerticalWall[currentPose->y][currentPose->x] = Wall_Front();
				maze->HorizontalWall[currentPose->y][currentPose->x] = Wall_Left();
				maze->HorizontalWall[currentPose->y + 1][currentPose->x] = Wall_Right();
				cur_phase = ALGORITHM_PHR;
			default:
				break;
		}
	}
}
void MazeFloodFill(Maze *maze, Cell_Queue*cellqueue, MousePose *mousepose)
{
	if(cur_phase == ALGORITHM_PHR)
	{

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
			else if(!maze->VerticalWall[y][x+ 1])
			{
				if(LegalCell(x, y, x + 1, y, maze))
				{
					Push_act(action_stack, MOVE_ACT);
					Push_act(action_stack, TURN_RIGHT_ACT);
					return true;
				}
			}
			else if(!maze->VerticalWall[y][x])
			{
				if(LegalCell(x, y, x - 1, y, maze))
				{
					Push_act(action_stack, MOVE_ACT);
					Push_act(action_stack, TURN_LEFT_ACT);
					return true;
				}
			}
			else if(!maze->HorizontalWall[y][x])
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
			else if(!maze->VerticalWall[y][x + 1])
			{
				if(LegalCell(x, y, x + 1, y, maze))
				{
					Push_act(action_stack, MOVE_ACT);
					return true;
				}
			}
			else if(!maze->VerticalWall[y][x])
			{
				if(LegalCell(x, y, x - 1, y, maze))
				{
					Push_act(action_stack, TURN_BACK_ACT);
					return true;
				}
			}
			else if(!maze->HorizontalWall[y][x])
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
			else if(!maze->VerticalWall[y][x + 1])
			{
				if(LegalCell(x, y, x + 1, y, maze))
				{
					Push_act(action_stack, MOVE_ACT);
					Push_act(action_stack, TURN_LEFT_ACT);
					return true;
				}
			}
			else if(!maze->VerticalWall[y][x])
			{
				if(LegalCell(x, y, x - 1, y, maze))
				{
					Push_act(action_stack, TURN_BACK_ACT);
					Push_act(action_stack, TURN_RIGHT_ACT);
					return true;
				}
			}
			else if(!maze->HorizontalWall[y][x])
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
			else if(!maze->VerticalWall[y][x + 1])
			{
				if(LegalCell(x, y, x + 1, y, maze))
				{
					Push_act(action_stack, TURN_BACK_ACT);
					return true;
				}
			}
			else if(!maze->VerticalWall[y][x])
			{
				if(LegalCell(x, y, x - 1, y, maze))
				{
					Push_act(action_stack, MOVE_ACT);
					return true;
				}
			}
			else if(!maze->HorizontalWall[y][x])
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
}
bool LegalCell(int8_t curX ,int8_t curY, int8_t nextX, int8_t nextY, Maze *maze)
{
	if(curX == nextX) // same collum
	{
		return maze->cells[curX][curY].value == maze->cells[nextX][nextY].value + 1;
	}
	else if(curY == nextY)
	{
		return maze->cells[curX][curY].value == maze->cells[nextX][nextY].value + 1;
	}
	else return false;
}
void PoseInit(MousePose *mousepose);
void PoseUpdate(MousePose *mousepose, Direction curren_head, Action_type act);

void CellQueueInitialize(Cell_Queue*cellqueue)
{
	cellqueue->head = cellqueue->tail = 0;
	cellqueue->count = 0;
	return;
}
bool CellqueueEmpty(Cell_Queue*cellqueue)
{
	return (cellqueue->count == 0);
}
bool CellqueueFull(Cell_Queue*cellqueue)
{
	return (cellqueue->count == MAZEMAXCELLS);
}
Cell *DegCellQueue(Cell_Queue*q)
{
	if(CellqueueEmpty(q))
	{
		return NULL;
	}
	q->head = (q->head + 1)%MAZEMAXCELLS;
	q->count--;
	Cell *c = &q->cell_list[q->head - 1];
	return c;
}
void EnqCellQueue(Cell_Queue *q, int8_t x, int8_t y)
{
	if(CellqueueFull(q)) return;
	q->tail = (q->tail + 1) % MAZEMAXCELLS;
	q->cell_list[q->tail].x = x;
	q->cell_list[q->tail].y = y;
	q->count++;
	return;
}


