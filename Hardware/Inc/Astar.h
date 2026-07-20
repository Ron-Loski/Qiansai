#ifndef __ASTAR_H_
#define __ASTAR_H_

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define openmax        255U
#define map_width      7U
#define map_heigh      7U
#define closedmax      (map_width * map_heigh)
#define dir_count      8U
#define pathmax        (map_width * map_heigh)
#define obstaclesmax   (map_width * map_heigh)

typedef enum
{
    ASTAR_MODE_FIXED_MAP = 0,
    ASTAR_MODE_USART2 = 1
} AStarMode;

typedef enum
{
    ASTAR_RUN_FAILED = 0,
    ASTAR_RUN_REACHED_TARGET = 1,
    ASTAR_RUN_INTERRUPTED = 2
} AStarRunResult;

typedef struct
{
    int16_t x;
    int16_t y;
} point;

typedef struct
{
    uint8_t on_list;
    int16_t x;
    int16_t y;
    int16_t parent_x;
    int16_t parent_y;
    float g;
    float h;
    float f;
} opennode;

typedef struct
{
    int8_t dx;
    int8_t dy;
    float cost;
} direction;

extern uint8_t g_opencount;
extern uint8_t g_closedcount;
extern uint8_t isobstacle[map_width][map_heigh];
extern int8_t robot_x0, robot_y0, target_x, target_y, robot_x, robot_y;
extern int8_t tempminf_x, tempminf_y;
extern uint16_t g_pathlen;
extern uint16_t pathidx;
extern uint16_t obstacle_count;
extern point g_closedlist[closedmax];
extern point pathnode[pathmax];
extern point g_path[pathmax];
extern point g_obstacles[obstaclesmax];
extern opennode g_openlist[openmax];
extern const direction g_direction[dir_count];

uint8_t isinmap(int16_t x, int16_t y);
uint8_t isinclosed(int16_t x, int16_t y);
uint8_t isinopen(int8_t x, int8_t y);
uint16_t find_openindex(int16_t x, int16_t y);
uint8_t can_diagonal(int16_t x, int16_t y, int16_t dx, int16_t dy);
float distance(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
void insertopen(int16_t x, int16_t y, int16_t parent_x, int16_t parent_y,
                float g, float h, float f);
uint16_t findminfnode(void);
void no_Obstacle(int16_t pointx, int16_t pointy);
void update_obstacle(void);
void expandnode(opennode *node, int8_t start_x, int8_t start_y,
                int8_t target_x, int8_t target_y);

void AStar_ResetNavigation(void);
void AStar_LoadFixedMap(void);
void AStar_LoadRealtimeMap(void);
uint8_t AStar_plan(void);
uint8_t AStar_ExecutePath(void);
AStarRunResult AStar_RunMode(AStarMode mode);

#endif
