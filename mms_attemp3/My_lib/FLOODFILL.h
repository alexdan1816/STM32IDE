/*
 * FLOODFILL.h
 *
 *  Created on: Sep 2, 2025
 *      Author: nguye
 */

#ifndef FLOODFILL_H_
#define FLOODFILL_H_

#define MAZESIZE 16
#define MAZEMAXCELLS 256
#define Xstart 0
#define Ystart 0
#define Xgoal 7
#define Ygoal 7
#define HeadStart 1

typedef enum
{
	NONE_DIR,
	NORTH,
	EAST,
	SOUTH,
	WEST
}Direction;

typedef struct
{
	uint8_t value;
	bool visited;
	uint8_t x;
	uint8_t y;
}Cell;

typedef struct
{
	uint8_t VerticalWall [16][17];
	uint8_t HorizontalWall [17][16];
	Cell cells[16][16];
}Maze;

typedef struct
{
	Cell cell_list[MAZEMAXCELLS];
	int16_t head;
	int16_t tail;
	int16_t count;
}Cell_Queue;

typedef struct
{
	int8_t x;
	int8_t y;
	Direction head;
}MousePose;

// map hướng -> vector dịch chuyển 1 ô
extern const int8_t dx[4];
extern const int8_t dy[4];
extern uint32_t check_count_ff;
extern int16_t min_value;


void MazeInitialize(Maze *maze);
void MazeUpdate(Maze *maze, MousePose *currentPose); // Update new cell's wall after sensor phase
void MazeFloodFill(Maze *maze, Cell_Queue*cellstack, MousePose *mousepose); // Floodfilling maze after update phase
bool FindNextCell(Maze *maze, MousePose *mousepose, Action_Stack *action_stack);
bool LegalCell(int8_t curX ,int8_t curY, int8_t nextX, int8_t nextY, Maze *maze);

void PoseInit(MousePose *mousepose, Direction start_dir, int8_t start_x, int8_t start_y);
void PoseUpdate(MousePose *mousepose, Action_type act);

void CellQueueInitialize(Cell_Queue*cellqueue);
bool CellqueueEmpty(Cell_Queue*cellqueue);
bool CellqueueFull(Cell_Queue*cellqueue);
Cell *DegCellQueue(Cell_Queue*q);
void EnqCellQueue(Cell_Queue *q, int8_t x, int8_t y);

void ExecuteAct(MousePose *m ,Action_Stack *s);
bool CheckGoal(MousePose *m, Maze *ma);




#endif /* FLOODFILL_H_ */
