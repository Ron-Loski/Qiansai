#ifndef __ASTAR_H_
#define __ASTAR_H_

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define openmax 255    //open节点数量得最大值
#define map_width 100  //地图宽度的最大值
#define map_heigh 100  //地图长度的最大值
#define closedmax 400  //closed列表的最大值
#define dir_count 8    //每个节点有八个方向
#define pathmax 200    //最多走200步
#define obstaclesmax   200  
uint8_t g_opencount;   //探索open列表的数量，open列表数量的最大值
uint8_t g_closedcount;  //closed列表的数量
uint8_t isobstacle[map_width][map_heigh];   //判断某个节点是否是障碍物   1是   0不是
int robot_x, robot_y, target_x, target_y;
int16_t curr_x,curr_y;
uint16_t g_pathlen;
uint16_t pathidx;
uint16_t obstacle_count;
//节点坐标
typedef struct{
    int16_t x;
    int16_t y;
}point;
point g_closedlist[closedmax];
point pathnode[pathmax];
point g_path[pathmax];
point g_obstacles[obstaclesmax];
//open列表节点
typedef struct{
    uint8_t on_list;     //是否在open列表中，1表示在，0表示移除
    int16_t x;           
    int16_t y;
    int16_t parent_x;    //父节点x坐标
    int16_t parent_y;    //父节点y坐标
    float g;             //历史代价
    float h;             //预估未来代价
    float f;             //评估函数f
}opennode;
opennode g_openlist[openmax];

typedef struct{
    int8_t dx;
    int8_t dy;
    float cost;
}direction;

const direction g_direction[] = {
    {1,0,1},        //右边的节点
    {1,1,1.414f},   //右上
    {0,1,0},        //上
    {-1,1,1.414f},  //左上
    {-1,0,1},      //左
    {-1,-1,1.414f}, //左下
    {0,-1,1},       //下
    {1,-1,1.414f},  //右下
};    

uint8_t isinmap(int16_t x, int16_t y);
uint8_t isinclosed(int16_t x, int16_t y);
uint8_t isinopen(int16_t x, int16_t y);
uint16_t find_openindex(int16_t x, int16_t y);
uint8_t can_diagonal(int16_t x, int16_t y, int16_t dx, int16_t dy);
float distance(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
void insertopen(int16_t x, int16_t y, int16_t parent_x, int16_t parent_y, float g, float h, float f);
uint16_t findminfnode(void);
void is_Obstacle(int16_t robot_x, int16_t robot_y);
void no_Obstacle(int16_t robot_x, int16_t robot_y);
void expandnode(opennode*node, int16_t start_x, int16_t start_y, int16_t target_x, int16_t target_y);
uint16_t back_path(void);
void AStar_smoothpath(void);
uint16_t AStar_plan(void);
void g_roundobstacle(uint16_t new_x, uint16_t new_y);






















#endif
