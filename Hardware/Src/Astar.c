#include "Astar.h"
#include "Motor.h"
#include "usart.h"

uint16_t pathidx = 0;
uint8_t g_closedcount = 0;
uint8_t g_opencount = 0;
point g_closedlist[closedmax] = {0};
opennode g_openlist[openmax] = {0};
uint8_t isobstacle[map_width][map_heigh] = {0};
int8_t robot_x = 0;
int8_t robot_y = 0;
int8_t tempminf_x = 0;
int8_t tempminf_y = 0;

const direction g_direction[dir_count] = {
    {0, 1, 1},          // 上
    {1, 1, 1.414f},     // 右上
    {1, 0, 1},          // 右
    {1, -1, 1.414f},    // 右下
    {0, -1, 1},         // 下
    {-1, -1, 1.414f},   // 左下
    {-1, 0, 1},         // 左
    {-1, 1, 1.414f},    // 左上
};

// 地图范围判断函数暂未启用，保留接口声明以兼容头文件。
// uint8_t isinmap(int16_t x, int16_t y)
// {
//     if (x >= 0 && x <= map_width && y >= 0 && y <= map_heigh)
//     {
//         return 1;
//     }
//     return 0;
// }

/**
 * @brief 判断指定节点是否已经在 CLOSED 列表中。
 * @param x 节点 x 坐标。
 * @param y 节点 y 坐标。
 * @return 1 表示已在 CLOSED，0 表示不在。
 * @details 遍历 g_closedlist，用于 A* 扩展节点时跳过已经探索过的节点。
 */
uint8_t isinclosed(int16_t x, int16_t y)
{
    for (uint8_t i = 0; i < g_closedcount; i++)
    {
        if (g_closedlist[i].x == x && g_closedlist[i].y == y)
        {
            return 1;
        }
    }
    return 0;
}

/**
 * @brief 判断指定节点是否已经在 OPEN 列表中。
 * @param x 节点 x 坐标。
 * @param y 节点 y 坐标。
 * @return 1 表示已在 OPEN，0 表示不在。
 * @details 遍历 g_openlist，用于避免重复加入待搜索节点。
 */
uint8_t isinopen(int8_t x, int8_t y)
{
    for (uint8_t i = 0; i < g_opencount; i++)
    {
        if (g_openlist[i].x == x && g_openlist[i].y == y)
        {
            return 1;
        }
    }
    return 0;
}

/**
 * @brief 查找指定节点在 OPEN 列表中的索引。
 * @param x 节点 x 坐标。
 * @param y 节点 y 坐标。
 * @return 找到时返回索引，未找到返回 0xFFFF。
 * @details 可用于更新已有 OPEN 节点代价或路径回溯。
 */
uint16_t find_openindex(int16_t x, int16_t y)
{
    for (uint16_t i = 0; i < g_opencount; i++)
    {
        if (g_openlist[i].x == x && g_openlist[i].y == y)
        {
            return i;
        }
    }
    return 0xFFFF;
}

/**
 * @brief 判断从当前节点是否允许斜向穿过到相邻节点。
 * @param x 当前节点 x 坐标。
 * @param y 当前节点 y 坐标。
 * @param dx 斜向移动的 x 增量。
 * @param dy 斜向移动的 y 增量。
 * @return 1 表示允许斜穿，0 表示相邻正交格有障碍物不允许斜穿。
 * @details 检查斜向两侧的正交邻接格，避免路径从两个障碍物夹角中穿过。
 */
uint8_t can_diagonal(int16_t x, int16_t y, int16_t dx, int16_t dy)
{
    int16_t adjx1 = x + dx;    // x 方向相邻节点。
    int16_t adjy1 = y;
    int16_t adjx2 = x;
    int16_t adjy2 = y + dy;    // y 方向相邻节点。

    if (isobstacle[adjx1][adjy1] || isobstacle[adjx2][adjy2])
    {
        return 0;
    }
    return 1;
}

/**
 * @brief 计算两个地图节点之间的欧氏距离。
 * @param x1/y1 起点坐标。
 * @param x2/y2 终点坐标。
 * @return 两点距离，float 类型。
 * @details 用作 A* 中的启发式代价 h。
 */
float distance(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
    float delta_x = x2 - x1;
    float delta_y = y2 - y1;
    return sqrtf((float)(delta_x * delta_x + delta_y * delta_y));
}

/**
 * @brief 向 OPEN 列表插入一个新节点。
 * @param x/y 新节点坐标。
 * @param parent_x/parent_y 父节点坐标。
 * @param g 从起点到当前节点的历史代价。
 * @param h 当前节点到目标点的启发式代价。
 * @param f 综合代价，通常 f=g+h。
 * @return 无。
 * @details 填充 g_openlist[g_opencount] 并递增 g_opencount，供 findminfnode() 后续选择最优节点。
 */
void insertopen(int16_t x, int16_t y, int16_t parent_x, int16_t parent_y, float g, float h, float f)
{
    g_openlist[g_opencount].on_list = 1;
    g_openlist[g_opencount].f = f;
    g_openlist[g_opencount].g = g;
    g_openlist[g_opencount].h = h;
    g_openlist[g_opencount].parent_x = parent_x;
    g_openlist[g_opencount].parent_y = parent_y;
    g_openlist[g_opencount].x = x;
    g_openlist[g_opencount].y = y;

    g_opencount += 1;
}

/**
 * @brief 在 OPEN 列表中寻找 f 值最小的节点。
 * @param 无。
 * @return 最小 f 节点索引；如果没有可用节点返回 0xFFFF。
 * @details AStar_plan() 每轮调用本函数选择下一步扩展的节点。
 */
uint16_t findminfnode(void)
{
    uint16_t minindex = 0xFFFF;
    float min_f = 1e9f;
    uint16_t i;

    for (i = 0; i <= g_opencount; i++)
    {
        if (g_openlist[i].on_list && g_openlist[i].f < min_f)
        {
            min_f = g_openlist[i].f;
            minindex = i;
        }
    }
    return minindex;
}

// 障碍物检测函数暂未启用，当前由 expandnode() 中的 OpenMV 返回值更新 isobstacle。
// void is_Obstacle(int16_t pointx, int16_t pointy, int16_t curx, int16_t cury)
// {
//     car_turn(curx, cury, pointx, pointy);
//     isobstacle[pointx][pointy] = isobstacle_flag;
// }

/**
 * @brief 将指定地图格标记为无障碍。
 * @param pointx 格子 x 坐标。
 * @param pointy 格子 y 坐标。
 * @return 无。
 * @details 直接写 isobstacle[pointx][pointy]=0，用于外部清除障碍标记。
 */
void no_Obstacle(int16_t pointx, int16_t pointy)
{
    isobstacle[pointx][pointy] = 0;
}

/**
 * @brief 根据 isobstacle 地图刷新障碍物列表。
 * @param 无。
 * @return 无。
 * @details 遍历地图，把标记为 1 的格子写入 g_obstacles，并更新 obstacle_count。
 */
void update_obstacle(void)
{
    obstacle_count = 0;
    for (uint16_t y = 0; y < map_heigh; y++)
    {
        for (uint16_t x = 0; x < map_heigh; x++)
        {
            if (isobstacle[x][y] == 1)
            {
                g_obstacles[obstacle_count].x = x;
                g_obstacles[obstacle_count].y = y;
                obstacle_count++;
            }
        }
    }
}

// 旧版 expandnode() 调试代码已停用，保留当前新版实现。

/**
 * @brief 扩展当前节点周围八个方向的候选节点。
 * @param node 当前要扩展的 OPEN 节点指针。
 * @param start_x/start_y 起点坐标，当前实现中主要保留作接口参数。
 * @param target_x/target_y 目标点坐标，用于计算启发式距离 h。
 * @return 无。
 * @details 对八方向逐一计算新节点，跳过 CLOSED/OPEN/已知障碍；调用 car_turn() 让车头朝向该方向，等待 OpenMV 返回障碍物数据，若无障碍则计算 g/h/f 并调用 insertopen() 加入 OPEN 列表。
 * @note car_turn() 内部已接入 Roll 角 PID 闭环，使 Astar 检测方向时可以按角度转到目标朝向。
 */
void expandnode(opennode *node, int8_t start_x, int8_t start_y, int8_t target_x, int8_t target_y)
{
    for (uint8_t i = 0; i < dir_count; i++)
    {
        int8_t new_x = node->x + g_direction[i].dx;
        int8_t new_y = node->y + g_direction[i].dy;

        printf("方向 %d: 新节点(%d,%d)\n", i, new_x, new_y);

        // 1. 判断节点是否已经在 CLOSED 列表中。
        if (isinclosed(new_x, new_y))
        {
            printf("方向 %d: 节点(%d,%d)已在 CLOSED，跳过\n", i, new_x, new_y);
            turnround_right(11.0);
            continue;
        }

        // 2. 判断节点是否已经在 OPEN 列表中。
        if (isinopen(new_x, new_y))
        {
            printf("方向 %d: 节点(%d,%d)已在 OPEN，跳过\n", i, new_x, new_y);
            turnround_right(11.0);
            continue;
        }

        // 3. 判断地图中是否已有障碍物标记。
        if (isobstacle[new_x][new_y])
        {
            printf("方向 %d: 节点(%d,%d)已有障碍标记，跳过\n", i, new_x, new_y);
            turnround_right(11.0);
            continue;
        }

        printf("方向 %d: 新节点(%d,%d)开始检测障碍\n", i, new_x, new_y);

        // 根据方向转向，让传感器或摄像头朝向待检测节点。
        car_turn(i);

        // 4. 等待 OpenMV 返回障碍物检测结果。
        Rx_flag = 0;
        printf("等待 OpenMV 返回...\n");
        while (Rx_flag == 0);
        printf("收到 OpenMV 返回: %d\n", isobstacle_flag);

        // 保存障碍物检测结果。
        isobstacle[new_x][new_y] = isobstacle_flag;

        // 5. 如果检测到障碍物，则跳过该节点。
        if (isobstacle[new_x][new_y] == 1)
        {
            printf("方向 %d: 节点(%d,%d)有障碍，跳过\n", i, new_x, new_y);
            turnround_right(11.0);
            continue;
        }

        // 6. 计算 A* 代价值并加入 OPEN 列表。
        float newG = node->g + g_direction[i].cost;
        float newH = distance(new_x, new_y, target_x, target_y);
        float newF = newG + newH;

        printf("newG=%.1f, newH=%.1f, newF=%.1f\n", newG, newH, newF);

        insertopen(new_x, new_y, node->x, node->y, newG, newH, newF);
        printf("方向 %d: 节点(%d,%d)已加入 OPEN\n", i, new_x, new_y);

        // 7. 回转 11 对应角度，准备检测下一个方向。
        turnround_right(11.0);
    }

    // 扫描完 8 个方向后回到默认朝向。
    printf("8 个方向扫描完成，回正\n");
    turnround_right(11.0);
}

// 路径回溯和平滑函数暂未启用。

/**
 * @brief 执行 A* 路径规划主流程。
 * @param 无。
 * @return 无。
 * @details 初始化 OPEN/CLOSED 列表，将起点加入 OPEN；循环选择 f 最小节点，移动小车到该节点，回正后调用 expandnode() 扩展周围节点。
 * @note car_move() 负责实际移动到下一格，car_return() 负责回正，expandnode() 负责检测周围障碍并继续搜索。
 */
void AStar_plan(void)
{
    g_opencount = 0;
    g_closedcount = 0;

    if (isobstacle[robot_x0][robot_y0] == 1 || isobstacle[target_x][target_y] == 1)
    {
        return;
    }

    float h0 = distance(robot_x0, robot_y0, target_x, target_y);
    insertopen(robot_x0, robot_y0, robot_x, robot_y, 0.0f, h0, h0);

    g_closedlist[g_closedcount].x = robot_x0;
    g_closedlist[g_closedcount].y = robot_y0;

    g_closedcount++;
    g_openlist[0].on_list = 1;    // 将起点加入 OPEN 列表。

    int16_t robot_x = robot_x0;
    int16_t robot_y = robot_y0;
    uint16_t itr_count = 0;

    // A* 主循环。
    while (robot_x != target_x || robot_y != target_y || itr_count <= 5000)
    {
        itr_count++;
        uint16_t min_index = findminfnode();
        if (min_index == 0xFFFF)
        {
            break;    // 没有可扩展节点，路径规划失败。
        }

        opennode *curnode = &g_openlist[min_index];    // 当前 f 值最小的节点。
        tempminf_x = curnode->x;
        tempminf_y = curnode->y;

        g_closedlist[g_closedcount].x = tempminf_x;    // 将当前节点加入 CLOSED 列表。
        g_closedlist[g_closedcount].y = tempminf_y;
        g_closedcount++;
        g_openlist->on_list = 0;

        // 移动小车到当前 f 值最小的节点。
        car_move(robot_x, robot_y, tempminf_x, tempminf_y);
        robot_x = tempminf_x;
        robot_y = tempminf_y;

        // 移动后回正车头，保证下一轮障碍物检测从统一默认朝向开始。
        car_return(g_openlist[min_index].parent_x, g_openlist[min_index].parent_y, robot_x, robot_y);

        expandnode(curnode, robot_x0, robot_y0, target_x, target_y);
    }

    if (tempminf_x != target_x || tempminf_y != target_y)
    {
        return;
    }
}