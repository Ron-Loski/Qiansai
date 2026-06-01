#include "Astar.h"
#include "Motor.h"
#include "usart.h"
uint16_t pathidx = 0;
uint8_t isinmap(int16_t x, int16_t y)
{
    if (x>=0 && x<=map_width && y>=0 && y<=map_heigh)
    return 1;
    else
    return 0;
}

uint8_t isinclosed(int16_t x, int16_t y)
{
    for (uint8_t i = 0; i <=g_closedcount; i++)
    {
        if (g_closedlist[i].x == x && g_closedlist[i].y == y)
        return 1;
    }
    return 0;
}

uint8_t isinopen(int16_t x, int16_t y)
{
    for(uint8_t i = 0; i <= g_opencount; i++)
    {
        if(g_openlist[i].x == x && g_openlist[i].y == y)
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
    return;
    g_openlist[g_opencount].on_list = 1;
    g_openlist[g_opencount].f = f;
    g_openlist[g_opencount].g = g;
    g_openlist[g_opencount].h = h;
    g_openlist[g_opencount].parent_x = parent_x;
    g_openlist[g_opencount].parent_y = parent_y;
    g_openlist[g_opencount].x = x;
    g_openlist[g_opencount].y = y;

    g_opencount ++;
}

uint16_t findminfnode(void)
{   
    uint16_t minindex = 0xFFFF;
    float min_f = 1e9f;
    uint16_t i;
    for(i=0; i<=g_opencount; i++)
    {
       if (g_openlist[i].on_list && g_openlist[i].f < min_f)
           {
            min_f = g_openlist[i].f; 
            minindex = i; 
           }
    }
return minindex;
}

void is_Obstacle(int16_t pointx, int16_t pointy)
{
    isobstacle[pointx][pointy] = 1;
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
       for (uint16_t x = 0; x < map_heigh; x++)
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


void expandnode(opennode*node, int16_t start_x, int16_t start_y, int16_t target_x, int16_t target_y)    //拓展节点
{
    for(uint8_t i =0; i < dir_count; i++)
    {
        int16_t new_x = node->x + g_direction[i].dx;
        int16_t new_y = node->y + g_direction[i].dy;
        if (!isinmap(new_x, new_y)) continue;      //判断是否在地图内
        g_roundobstacle(new_x, new_y);             //写电机转动判断周围8个格子是否是障碍物的代码
        if (isobstacle[new_x][new_y]) continue;    //判断是否是障碍物

        if(g_direction[i].cost >1.0f && !can_diagonal(node->x, node->y, g_direction[i].dx, g_direction[i].dy)) continue;   //判断是否可以斜穿

        if (isinclosed(new_x, new_y)) continue;    //判断是否已经在closed列表内

    float newG = node->g + g_direction[i].cost;
    float newH = node->h + distance(new_x, new_y, target_x, target_y);
    float starttotargetdis = distance(start_x, start_y, target_x, target_y);
    float r = newH;
    float radio = 1.0f + r/starttotargetdis;
    float newF = newG + radio * newH;

    if(isinopen(new_x, new_y))
    {
        uint16_t openindex = find_openindex(new_x, new_y);
        if(newF < g_openlist[openindex].f)
        {
            g_openlist[openindex].f = newF;
            g_openlist[openindex].g = newG;
            g_openlist[openindex].parent_x = node->x;
            g_openlist[openindex].parent_y = node->y;
        }
    }
    else
    {
        insertopen(new_x, new_y, node->x, node->y, newG, newH, newF);
    }
    }
}

uint16_t back_path(void)
{
    if (find_openindex(target_x, target_y) == 0xFFFF)
    return 0;
    int16_t px = target_x;
    int16_t py = target_y;
    uint16_t nodeidx = 0;
    while(px != robot_x && py!= robot_y)
    {
        pathnode[pathidx].x = px;
        pathnode[pathidx].y = py;
        pathidx ++;
        nodeidx = find_openindex(px, py);
        if (nodeidx == 0xFFFF)
        {
            break;
        }
        px = g_openlist[nodeidx].parent_x;
        py = g_openlist[nodeidx].parent_y;
    }
    pathnode[pathidx].x = robot_x;
    pathnode[pathidx].y = robot_y;
    pathidx ++;

    for(uint8_t i = 0; i < pathidx; i++)
    {
        g_path[i].x = pathnode[pathidx - 1 - i].x;
        g_path[i].y = pathnode[pathidx - 1 - i].y;
    }
    return pathidx;
}

void AStar_smoothpath(void)
{
    if (g_pathlen < 3)
    return;
    uint16_t new_len = 0;
    point smoothpath[pathmax];
    smoothpath[new_len++] = g_path[0];
    
    for (uint16_t i = 0; i < pathidx - 1; i++)
       {
        int8_t dx1 = g_path[i].x - g_path[i-1].x;
        int8_t dy1 = g_path[i].y - g_path[i-1].y;
        int8_t dx2 = g_path[i].x - g_path[i+1].x;
        int8_t dy2 = g_path[i].y - g_path[i+1].y;

        if (dx1 != dx2 || dy1 != dy2)
        {
            smoothpath[new_len++] = g_path[i];
        }

        g_pathlen = new_len;
        for (uint16_t i = 0; i < g_pathlen; i++)
        {
            g_path[i] = smoothpath[i];
        }
       }
}

/*   A星代码核心   */
uint16_t AStar_plan(void)
{
    g_opencount = 0;
    g_closedcount = 0;

    if (isobstacle[robot_x][robot_y] == 1 || isobstacle[target_x][target_y] == 1)
    {
        return 0;
    }

    float h0 = distance(robot_x, robot_y,target_x, target_y);
    insertopen(robot_x, robot_y, robot_x, robot_y, 0.0f, h0, h0);

    g_closedlist[g_closedcount].x = robot_x;
    g_closedlist[g_closedcount].y = robot_y;

    g_closedcount++;
    g_openlist[0].on_list = 1;                                                   //把起点放入openm列表
    
    int16_t curr_x = robot_x;
    int16_t curr_y = robot_y;
    //A*主循环 
    uint16_t itr_count = 0;
    while(curr_x != target_x || curr_y != target_y || itr_count <=5000)
    {
        itr_count ++;
        uint16_t min_index = findminfnode();
        if (min_index == 0xFFFF)
        {
            break;                      //若无路径，则跳出来；
        }
        opennode*curnode = &g_openlist[min_index];   //把得到的f最小的节点引出来
        curr_x = curnode->x;                            
        curr_y = curnode->y;
        
        g_closedlist[g_closedcount].x = curr_x;      //引出来后放入closed列表
        g_closedlist[g_closedcount].y = curr_y;
        g_closedcount ++;
        g_openlist->on_list = 0;

        expandnode(curnode, robot_x, robot_y, target_x, target_y);
    }

    if (curr_x != target_x || curr_y != target_y)
    {
        return 0;
    }

    g_pathlen = back_path();
    
    AStar_smoothpath();

    return g_pathlen;
}

//在电机转一个90°时判断此时摄像头内的格子是否是障碍物
void g_roundobstacle(uint16_t new_x, uint16_t new_y)
{
    motor_turn();
    if (g_obstacles == 1)
    {
    is_Obstacle(new_x, new_y);
    }
    else 
    {
        no_Obstacle(new_x, new_y);
    }
}
