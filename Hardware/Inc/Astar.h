#ifndef __ASTAR_H_
#define __ASTAR_H_

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define openmax 255       // OPEN 列表最大节点数量。
#define map_width 100     // 地图宽度最大值。
#define map_heigh 100     // 地图高度最大值。
#define closedmax 400     // CLOSED 列表最大节点数量。
#define dir_count 8       // 每个节点的八个搜索方向。
#define pathmax 200       // 路径最大节点数量。
#define obstaclesmax 200  // 障碍物列表最大数量。

extern uint8_t g_opencount;                         // OPEN 列表当前节点数量。
extern uint8_t g_closedcount;                       // CLOSED 列表当前节点数量。
extern uint8_t isobstacle[map_width][map_heigh];    // 障碍物地图，1 表示有障碍，0 表示无障碍。
extern int8_t robot_x0, robot_y0, target_x, target_y, robot_x, robot_y;
extern int8_t tempminf_x, tempminf_y;
extern uint16_t g_pathlen;
extern uint16_t pathidx;
extern uint16_t obstacle_count;

// 地图节点坐标。
typedef struct {
    int16_t x;
    int16_t y;
} point;

extern point g_closedlist[closedmax];
extern point pathnode[pathmax];
extern point g_path[pathmax];
extern point g_obstacles[obstaclesmax];

// OPEN 列表节点。
typedef struct {
    uint8_t on_list;      // 是否仍在 OPEN 列表中，1 表示在，0 表示已移除。
    int16_t x;
    int16_t y;
    int16_t parent_x;    // 父节点 x 坐标。
    int16_t parent_y;    // 父节点 y 坐标。
    float g;             // 从起点到当前节点的历史代价。
    float h;             // 当前节点到目标点的启发式代价。
    float f;             // 综合代价，通常 f = g + h。
} opennode;

extern opennode g_openlist[openmax];

typedef struct {
    int8_t dx;
    int8_t dy;
    float cost;
} direction;

extern const direction g_direction[dir_count];

uint8_t isinmap(int16_t x, int16_t y);
uint8_t isinclosed(int16_t x, int16_t y);
uint8_t isinopen(int8_t x, int8_t y);
uint16_t find_openindex(int16_t x, int16_t y);
uint8_t can_diagonal(int16_t x, int16_t y, int16_t dx, int16_t dy);
float distance(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
void insertopen(int16_t x, int16_t y, int16_t parent_x, int16_t parent_y, float g, float h, float f);
uint16_t findminfnode(void);
void no_Obstacle(int16_t pointx, int16_t pointy);
void update_obstacle(void);
void expandnode(opennode *node, int8_t start_x, int8_t start_y, int8_t target_x, int8_t target_y);
void AStar_plan(void);

#endif