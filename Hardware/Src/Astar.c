#include "Astar.h"
#include "Motor.h"
#include "MPU6050.h"
#include "Blue.h"
#include "Button.h"
#include <stdio.h>
#include <string.h>

#define ASTAR_INITIAL_X  3
#define ASTAR_INITIAL_Y  3

uint16_t pathidx = 0;
uint16_t g_pathlen = 0;
uint16_t obstacle_count = 0;
uint8_t g_closedcount = 0;
uint8_t g_opencount = 0;
point g_closedlist[closedmax] = {0};
point pathnode[pathmax] = {0};
point g_path[pathmax] = {0};
point g_obstacles[obstaclesmax] = {0};
opennode g_openlist[openmax] = {0};
int8_t robot_x0 = 3;
int8_t robot_y0 = 3;
int8_t target_x = 5;
int8_t target_y = 5;
int8_t robot_x = 0;
int8_t robot_y = 0;
int8_t tempminf_x = 0;
int8_t tempminf_y = 0;

static uint8_t AStar_ReconstructPath(int16_t end_x, int16_t end_y);
static uint8_t AStar_ExecutePathSteps(uint16_t max_steps);
static uint8_t AStar_ScanCurrentNodeRealtime(void);

static const uint8_t s_fixed_obstacle_map[map_width][map_heigh] =
{
    {1,0,0,1,0,0,0},
    {0,1,1,0,0,0,1},
    {0,0,1,0,0,1,1},
    {0,0,0,0,0,1,1},
    {0,0,1,1,1,0,1},
    {1,0,0,0,1,0,0},
    {1,1,0,0,0,0,0},
};

static uint8_t s_realtime_obstacle_map[map_width][map_heigh] = {0};

const direction g_direction[dir_count]={   
    {0,1,1},        //上
    {1,1,1.414f},   //右上    
    {1,0,1},        //右   
    {1,-1,1.414f},  //右下
    {0,-1,1},       //下
    {-1,-1,1.414f}, //左下
    {-1,0,1},      //左
    {-1,1,1.414f},  //左上
};   

uint8_t isobstacle[map_width][map_heigh] = 
{
    {1,0,0,1,0,0,0},
    {0,1,1,0,0,0,1},
    {0,0,1,0,0,1,1},
    {0,0,0,0,0,1,1},
    {0,0,1,1,1,0,1},
    {1,0,0,0,1,0,0},
    {1,1,0,0,0,0,0},
};

uint8_t isinmap(int16_t x, int16_t y)
{
    return (uint8_t)(x >= 0 && x < (int16_t)map_width &&
                     y >= 0 && y < (int16_t)map_heigh);
}

uint8_t isinclosed(int16_t x, int16_t y)
{
    for (uint8_t i = 0; i <g_closedcount; i++)
    {
        if (g_closedlist[i].x == x && g_closedlist[i].y == y)
        return 1;
    }
    return 0;
}

uint8_t isinopen(int8_t x, int8_t y)
{
    for(uint8_t i = 0; i < g_opencount; i++)
    {
        if(g_openlist[i].on_list && g_openlist[i].x == x && g_openlist[i].y == y)
        return 1;
    }
    return 0;
}

uint16_t find_openindex(int16_t x, int16_t y)
{
    for(uint16_t i=0; i<g_opencount; i++)
    {
        if(g_openlist[i].x == x && g_openlist[i].y == y)
        return i;
    }
    return 0xFFFF;
}

uint8_t can_diagonal(int16_t x, int16_t y, int16_t dx, int16_t dy)      //判断是否可以斜穿
{
    int16_t adjx1 = x + dx;     //x轴方向上相邻的节点1
    int16_t adjy1 = y;          //y轴方向上相邻的节点2
    int16_t adjx2 = x;            
    int16_t adjy2 = y + dy;

    if (isobstacle[adjx1][adjy1] || isobstacle[adjx2][adjy2])
    {return 0;}
    else 
    {return 1;}
}

float distance(int16_t x1, int16_t y1, int16_t x2, int16_t y2)    //两点距离测算函数
{
    float delta_x = x2 - x1;
    float delta_y = y2 - y1;
    return sqrtf((float)(delta_x*delta_x + delta_y*delta_y));
}

void insertopen(int16_t x, int16_t y, int16_t parent_x, int16_t parent_y, float g, float h, float f)    //将新节点添加进open列表
{
    if (g_opencount >= openmax)
    {
        return;
    }

    g_openlist[g_opencount].on_list = 1;
    g_openlist[g_opencount].f = f;
    g_openlist[g_opencount].g = g;
    g_openlist[g_opencount].h = h;
    g_openlist[g_opencount].parent_x = parent_x;
    g_openlist[g_opencount].parent_y = parent_y;
    g_openlist[g_opencount].x = x;
    g_openlist[g_opencount].y = y;

    g_opencount +=1 ;
}

uint16_t findminfnode(void)
{   
    uint16_t minindex = 0xFFFF;
    float min_f = 1e9f;
    uint16_t i;
    for(i=0; i < g_opencount; i++)
    {
       if (g_openlist[i].on_list && g_openlist[i].f < min_f)
           {
            min_f = g_openlist[i].f; 
            minindex = i; 
           }
    }
return minindex;
}

void no_Obstacle(int16_t pointx, int16_t pointy)
{
    isobstacle[pointx][pointy] = 0;
}

void update_obstacle(void)
{
    obstacle_count = 0;
    for(uint16_t y = 0; y < map_heigh; y++)
    {
       for (uint16_t x = 0; x < map_width; x++)
       {
        if ( isobstacle[x][y] == 1 )
        {
        g_obstacles[obstacle_count].x = x;
        g_obstacles[obstacle_count].y = y;
        obstacle_count++;
        }
       }
    }
}

/* 固定障碍物地图模式：只扩展数组中的可通行节点，不转动车辆、不访问USART2。 */
void expandnode(opennode* node, int8_t start_x, int8_t start_y, int8_t target_x, int8_t target_y)
{
    (void)start_x;
    (void)start_y;

    for (uint8_t i = 0; i < dir_count; i++)
    {
        int8_t new_x = node->x + g_direction[i].dx;
        int8_t new_y = node->y + g_direction[i].dy;
        float newG;
        float newH;
        float newF;
        uint16_t open_index;

        printf("方向 %d: 检查节点(%d,%d)\n", i, new_x, new_y);

        if (new_x < 0 || new_x >= map_width || new_y < 0 || new_y >= map_heigh)
        {
            printf("方向 %d: 节点(%d,%d)超出地图，跳过\n", i, new_x, new_y);
            continue;
        }

        if (isobstacle[new_x][new_y] == 1U)
        {
            printf("方向 %d: 固定地图节点(%d,%d)是障碍物，跳过\n", i, new_x, new_y);
            continue;
        }

        if (g_direction[i].dx != 0 && g_direction[i].dy != 0 &&
            !can_diagonal(node->x, node->y, g_direction[i].dx, g_direction[i].dy))
        {
            printf("方向 %d: 节点(%d,%d)存在斜向夹角障碍，跳过\n", i, new_x, new_y);
            continue;
        }

        if (isinclosed(new_x, new_y))
        {
            printf("方向 %d: 节点(%d,%d)已在CLOSED，跳过\n", i, new_x, new_y);
            continue;
        }

        newG = node->g + g_direction[i].cost;
        newH = distance(new_x, new_y, target_x, target_y);
        newF = newG + newH;
        open_index = find_openindex(new_x, new_y);

        if (open_index != 0xFFFF && g_openlist[open_index].on_list)
        {
            if (newG < g_openlist[open_index].g)
            {
                g_openlist[open_index].g = newG;
                g_openlist[open_index].h = newH;
                g_openlist[open_index].f = newF;
                g_openlist[open_index].parent_x = node->x;
                g_openlist[open_index].parent_y = node->y;
                printf("节点(%d,%d)找到更短路径，更新OPEN\n", new_x, new_y);
            }
            continue;
        }

        insertopen(new_x, new_y, node->x, node->y, newG, newH, newF);
        printf("节点(%d,%d)加入OPEN: g=%.3f h=%.3f f=%.3f\r\n",
               new_x, new_y, newG, newH, newF);
    }
}

void AStar_LoadFixedMap(void)
{
    memcpy(isobstacle, s_fixed_obstacle_map, sizeof(isobstacle));

    /* The car's current cell must always remain traversable. */
    if (isinmap(robot_x, robot_y))
    {
        isobstacle[(uint8_t)robot_x][(uint8_t)robot_y] = 0U;
    }
    update_obstacle();
}

void AStar_LoadRealtimeMap(void)
{
    memcpy(isobstacle, s_realtime_obstacle_map, sizeof(isobstacle));

    if (isinmap(robot_x, robot_y))
    {
        isobstacle[(uint8_t)robot_x][(uint8_t)robot_y] = 0U;
        s_realtime_obstacle_map[(uint8_t)robot_x][(uint8_t)robot_y] = 0U;
    }
    update_obstacle();
}

void AStar_ResetNavigation(void)
{
    robot_x0 = ASTAR_INITIAL_X;
    robot_y0 = ASTAR_INITIAL_Y;
    robot_x = ASTAR_INITIAL_X;
    robot_y = ASTAR_INITIAL_Y;
    memset(s_realtime_obstacle_map, 0, sizeof(s_realtime_obstacle_map));
    AStar_LoadFixedMap();
}


static uint8_t AStar_ReconstructPath(int16_t end_x, int16_t end_y)
{
    int16_t current_x = end_x;
    int16_t current_y = end_y;
    uint16_t reverse_length = 0U;

    while (reverse_length < pathmax)
    {
        uint16_t node_index;

        pathnode[reverse_length].x = current_x;
        pathnode[reverse_length].y = current_y;
        reverse_length++;

        if (current_x == robot_x0 && current_y == robot_y0)
        {
            break;
        }

        node_index = find_openindex(current_x, current_y);
        if (node_index == 0xFFFF)
        {
            return 0U;
        }

        current_x = g_openlist[node_index].parent_x;
        current_y = g_openlist[node_index].parent_y;
    }

    if (reverse_length == 0U ||
        pathnode[reverse_length - 1U].x != robot_x0 ||
        pathnode[reverse_length - 1U].y != robot_y0)
    {
        return 0U;
    }

    g_pathlen = reverse_length;
    for (uint16_t i = 0U; i < g_pathlen; i++)
    {
        g_path[i] = pathnode[g_pathlen - 1U - i];
    }

    return 1U;
}

/* 固定地图A*：只规划并输出路径，不在搜索阶段驱动车辆。 */
uint8_t AStar_plan(void)
{
    uint16_t itr_count = 0U;
    uint8_t found = 0U;

    g_opencount = 0;
    g_closedcount = 0;
    g_pathlen = 0U;
    pathidx = 0U;
    memset(g_openlist, 0, sizeof(g_openlist));
    memset(g_closedlist, 0, sizeof(g_closedlist));
    memset(g_path, 0, sizeof(g_path));
    memset(pathnode, 0, sizeof(pathnode));

    if (robot_x0 < 0 || robot_x0 >= map_width || robot_y0 < 0 || robot_y0 >= map_heigh ||
        target_x < 0 || target_x >= map_width || target_y < 0 || target_y >= map_heigh)
    {
        printf("AStar coordinate out of map\r\n");
        return 0U;
    }

    if (isobstacle[robot_x0][robot_y0] == 1 || isobstacle[target_x][target_y] == 1)
    {
        printf("AStar start or target is obstacle\r\n");
        return 0U;
    }

    float h0 = distance(robot_x0, robot_y0,target_x, target_y);
    insertopen(robot_x0, robot_y0, robot_x0, robot_y0, 0.0f, h0, h0);

    while (itr_count < closedmax)
    {
        uint16_t min_index = findminfnode();
        opennode *current_node;

        if (min_index == 0xFFFF)
        {
            break;
        }

        current_node = &g_openlist[min_index];
        tempminf_x = current_node->x;
        tempminf_y = current_node->y;

        if (tempminf_x == target_x && tempminf_y == target_y)
        {
            found = 1U;
            break;
        }

        g_openlist[min_index].on_list = 0;
        if (!isinclosed(tempminf_x, tempminf_y) && g_closedcount < closedmax)
        {
            g_closedlist[g_closedcount].x = tempminf_x;
            g_closedlist[g_closedcount].y = tempminf_y;
            g_closedcount++;
        }

        expandnode(current_node, robot_x0, robot_y0, target_x, target_y);
        itr_count++;
    }

    if (!found)
    {
        printf("AStar no path from (%d,%d) to (%d,%d)\r\n",
               robot_x0, robot_y0, target_x, target_y);
        return 0U;
    }

    if (!AStar_ReconstructPath(target_x, target_y))
    {
        printf("AStar path reconstruction failed\r\n");
        return 0U;
    }

    robot_x = robot_x0;
    robot_y = robot_y0;

    printf("AStar path found, nodes=%u, iterations=%u\r\n",
           g_pathlen, itr_count);
    for (uint16_t i = 0U; i < g_pathlen; i++)
    {
        printf("Path[%u] = (%d,%d)\r\n", i, g_path[i].x, g_path[i].y);
    }

    return 1U;
}

static uint8_t AStar_ExecutePathSteps(uint16_t max_steps)
{
    uint16_t executed_steps = 0U;

    if (g_pathlen == 0U)
    {
        printf("AStar execute failed: path is empty\r\n");
        return 0U;
    }

    if (g_path[0].x != robot_x0 || g_path[0].y != robot_y0)
    {
        printf("AStar execute failed: path start mismatch\r\n");
        return 0U;
    }

    robot_x = (int8_t)g_path[0].x;
    robot_y = (int8_t)g_path[0].y;
    printf("AStar path execution start\r\n");

    for (pathidx = 1U;
         pathidx < g_pathlen && executed_steps < max_steps;
         pathidx++, executed_steps++)
    {
        point previous = g_path[pathidx - 1U];
        point next = g_path[pathidx];

        if (Button_PeekEvents() != BUTTON_EVENT_NONE)
        {
            printf("AStar execution interrupted before step %u\r\n", pathidx);
            return 0U;
        }

        printf("Execute step %u/%u: (%d,%d)->(%d,%d)\r\n",
               pathidx, g_pathlen - 1U,
               previous.x, previous.y, next.x, next.y);

        if (!car_move((uint16_t)previous.x, (uint16_t)previous.y,
                      (uint16_t)next.x, (uint16_t)next.y))
        {
            if ((Button_PeekEvents() & BUTTON_EVENT_DOUBLE) == 0U)
            {
                (void)car_return((int8_t)previous.x, (int8_t)previous.y,
                                 (int8_t)next.x, (int8_t)next.y);
            }
            printf("AStar execution stopped at step %u\r\n", pathidx);
            return 0U;
        }

        /* Forward travel is complete; keep the logical cell correct even if
         * a double click interrupts the following heading restoration. */
        robot_x = (int8_t)next.x;
        robot_y = (int8_t)next.y;
        if (!car_return((int8_t)previous.x, (int8_t)previous.y,
                        (int8_t)next.x, (int8_t)next.y))
        {
            printf("AStar heading restore failed at step %u\r\n", pathidx);
            return 0U;
        }

        if (Button_PeekEvents() != BUTTON_EVENT_NONE)
        {
            printf("AStar execution interrupted after reaching (%d,%d)\r\n",
                   robot_x, robot_y);
            return 0U;
        }
    }

    return 1U;
}

uint8_t AStar_ExecutePath(void)
{
    uint8_t result = AStar_ExecutePathSteps(pathmax);

    if (result && robot_x == target_x && robot_y == target_y)
    {
        printf("AStar path execution reached target (%d,%d)\r\n", robot_x, robot_y);
    }
    return result;
}

static uint8_t AStar_ScanCurrentNodeRealtime(void)
{
    float scan_base_yaw = AngleFilter.Yaw;
    uint8_t last_direction = 0U;
    uint8_t scanned_any = 0U;

    for (uint8_t i = 0U; i < dir_count; i++)
    {
        int16_t scan_x = (int16_t)robot_x + g_direction[i].dx;
        int16_t scan_y = (int16_t)robot_y + g_direction[i].dy;
        float restore_yaw;

        if (!isinmap(scan_x, scan_y))
        {
            continue;
        }

        if (Button_PeekEvents() != BUTTON_EVENT_NONE)
        {
            restore_yaw = (last_direction <= 4U) ? scan_base_yaw
                                                 : scan_base_yaw + 360.0f;
            if ((Button_PeekEvents() & BUTTON_EVENT_DOUBLE) == 0U && scanned_any)
            {
                (void)Motor_TurnToAbsoluteYaw(restore_yaw);
            }
            return 0U;
        }

        if (!car_turn(i, scan_base_yaw))
        {
            restore_yaw = (i <= 4U) ? scan_base_yaw : scan_base_yaw + 360.0f;
            if ((Button_PeekEvents() & BUTTON_EVENT_DOUBLE) == 0U)
            {
                (void)Motor_TurnToAbsoluteYaw(restore_yaw);
            }
            return 0U;
        }

        scanned_any = 1U;
        last_direction = i;
        isobstacle[(uint8_t)scan_x][(uint8_t)scan_y] =
            (uint8_t)(isobstacle_flag != 0U);
        s_realtime_obstacle_map[(uint8_t)scan_x][(uint8_t)scan_y] =
            isobstacle[(uint8_t)scan_x][(uint8_t)scan_y];

        printf("USART2 obstacle: dir=%u cell=(%d,%d) raw=0x%02X obstacle=%u\r\n",
               i, scan_x, scan_y, isobstacle_flag,
               isobstacle[(uint8_t)scan_x][(uint8_t)scan_y]);
    }

    if (scanned_any &&
        !Motor_TurnToAbsoluteYaw((last_direction <= 4U)
                                     ? scan_base_yaw
                                     : scan_base_yaw + 360.0f))
    {
        return 0U;
    }

    isobstacle[(uint8_t)robot_x][(uint8_t)robot_y] = 0U;
    s_realtime_obstacle_map[(uint8_t)robot_x][(uint8_t)robot_y] = 0U;
    update_obstacle();
    return 1U;
}

AStarRunResult AStar_RunMode(AStarMode mode)
{
    if (mode == ASTAR_MODE_FIXED_MAP)
    {
        AStar_LoadFixedMap();
        robot_x0 = robot_x;
        robot_y0 = robot_y;

        if (Button_PeekEvents() != BUTTON_EVENT_NONE)
        {
            return ASTAR_RUN_INTERRUPTED;
        }
        if (!AStar_plan())
        {
            return ASTAR_RUN_FAILED;
        }
        if (!AStar_ExecutePath())
        {
            return (Button_PeekEvents() != BUTTON_EVENT_NONE)
                       ? ASTAR_RUN_INTERRUPTED
                       : ASTAR_RUN_FAILED;
        }
        return (robot_x == target_x && robot_y == target_y)
                   ? ASTAR_RUN_REACHED_TARGET
                   : ASTAR_RUN_FAILED;
    }

    if (mode != ASTAR_MODE_USART2)
    {
        return ASTAR_RUN_FAILED;
    }

    AStar_LoadRealtimeMap();
    while (robot_x != target_x || robot_y != target_y)
    {
        if (Button_PeekEvents() != BUTTON_EVENT_NONE)
        {
            return ASTAR_RUN_INTERRUPTED;
        }

        if (!AStar_ScanCurrentNodeRealtime())
        {
            return (Button_PeekEvents() != BUTTON_EVENT_NONE)
                       ? ASTAR_RUN_INTERRUPTED
                       : ASTAR_RUN_FAILED;
        }

        if (Button_PeekEvents() != BUTTON_EVENT_NONE)
        {
            return ASTAR_RUN_INTERRUPTED;
        }

        robot_x0 = robot_x;
        robot_y0 = robot_y;
        if (!AStar_plan() || g_pathlen < 2U)
        {
            return ASTAR_RUN_FAILED;
        }

        /* Re-plan after every grid so newly received obstacles take effect. */
        if (!AStar_ExecutePathSteps(1U))
        {
            return (Button_PeekEvents() != BUTTON_EVENT_NONE)
                       ? ASTAR_RUN_INTERRUPTED
                       : ASTAR_RUN_FAILED;
        }
    }

    printf("USART2 realtime mode reached target (%d,%d)\r\n", robot_x, robot_y);
    return ASTAR_RUN_REACHED_TARGET;
}
