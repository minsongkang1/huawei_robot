#pragma GCC optimize(2)
#include <bits/stdc++.h>
using namespace std;

const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int N = 211;

struct Robot
{
    int u, v, g_status; // g_status 表示是否拿了物品
    int status;         // 状态
    bool a_step;        // 机器人是否使用过move指令
    int price;          // 机器人拿货物的价值
} robot[robot_num + 1];

struct Berth
{
    int u;
    int v;
    int tsp_time;      // 运输时间
    int ld_speed;      // 装载货物速度
    double goods_num;  // 物品数量
    double goods_cost; // 物品价值

} berth[berth_num + 1];

struct Boat
{
    int pos, status;
    int cur_num; // 当前船的货物数量
} boat[6];

int money, id, boat_capacity;
char ch[N][N];
int goods_time_value[N][N][2];                        // [0] 表示货物距离消失剩余的时间，[1]表示金额
int derx[4] = {0, 0, -1, 1}, dery[4] = {1, -1, 0, 0}; // 方向
vector<pair<int, int>> berth_local_x_y;
int robot_location[N][N];
int global_dist[N][N];
int path_to_berth[N][N];
int temporary_path[N][N];
bool robot_to_one_good[N][N]; // 锁定物品只能被一个机器人选取

int Convert(int x)
{
    if (x == 1)
        return 0;
    else if (x == 0)
        return 1;
    else if (x == 2)
        return 3;
    else
        return 2;
}

// 放货动作原子操作
void Release_goods(int id, int u, int v)
{
    vector<pair<int, int>> vv;
    for (int i = 0; i < berth_local_x_y.size(); i++)
    {
        int summ = abs(u - berth_local_x_y[i].first) + abs(v - berth_local_x_y[i].second);
        vv.push_back({summ, i});
    }
    sort(vv.begin(), vv.end());
    int idxxx = vv[0].second;
    berth[idxxx].goods_cost += robot[id].price;
    berth[idxxx].goods_num++;
    printf("pull %d\n", id);
    robot[id].g_status = 0;
    robot[id].price = 0;
}

// 取货动作原子操作
void get_goods(int id, int u, int v)
{
    robot[id].g_status = 1;
    printf("get %d\n", id);
    robot[id].price = goods_time_value[u][v][1];
    goods_time_value[u][v][1] = goods_time_value[u][v][2] = goods_time_value[u][v][0] = 0;
}

void Initialization()
{
    for (int i = 0; i < n; i++)
        scanf("%s", ch[i]);
    for (int i = 0; i < berth_num; i++)
    {
        int id;
        scanf("%d", &id);
        scanf("%d%d%d%d", &berth[id].u, &berth[id].v, &berth[id].tsp_time, &berth[id].ld_speed);
        berth[id].u += 2, berth[id].v += 1;
        berth_local_x_y.push_back({berth[id].u, berth[id].v});
    }
    memset(path_to_berth, -1, sizeof path_to_berth);
    memset(global_dist, 0x3f, sizeof global_dist);
    priority_queue<pair<int, pair<int, int>>, vector<pair<int, pair<int, int>>>, greater<pair<int, pair<int, int>>>> que;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
        {
            if (ch[i][j] == 'B')
            {
                global_dist[i][j] = 0;
                que.push({0, {i, j}});
            }
        }
    while (que.size())
    {
        auto t = que.top();
        que.pop();
        int dd = t.first, u = t.second.first, v = t.second.second;
        for (int i = 0; i < 4; i++)
        {
            int fu = u + derx[i], fv = v + dery[i];
            if (fu >= 0 && fu < n && fv >= 0 && fv < n && ch[fu][fv] != '*' && ch[fu][fv] != '#')
            {
                if (global_dist[fu][fv] > global_dist[u][v] + 1)
                {
                    global_dist[fu][fv] = global_dist[u][v] + 1;
                    path_to_berth[fu][fv] = Convert(i);
                    que.push({global_dist[fu][fv], {fu, fv}});
                }
            }
        }
    }

    scanf("%d", &boat_capacity);
    char okk[100];
    scanf("%s", okk);
    printf("OK\n");
    fflush(stdout);
}

// 逐渐消失的货物
void down()
{
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (goods_time_value[i][j][0])
            {
                goods_time_value[i][j][0]--;
                if (goods_time_value[i][j][0] == 0)
                {
                    goods_time_value[i][j][1] = 0;
                }
            }
        }
    }
}

int Input()
{
    down();
    memset(robot_to_one_good, 0, sizeof robot_to_one_good);
    memset(robot_location, -1, sizeof robot_location);
    scanf("%d%d", &id, &money);
    int num;
    scanf("%d", &num);
    for (int i = 1; i <= num; i++)
    {
        int u, v, price;
        scanf("%d%d%d", &u, &v, &price);
        goods_time_value[u][v][0] = 1000, goods_time_value[u][v][1] = price;
    }

    for (int i = 0; i < robot_num; i++)
    {
        int sts;
        scanf("%d%d%d%d", &robot[i].g_status, &robot[i].u, &robot[i].v, &sts);
        robot_location[robot[i].u][robot[i].v] = i;
        robot[i].a_step = true;
    }
    for (int i = 0; i < 5; i++)
        scanf("%d%d\n", &boat[i].status, &boat[i].pos);
    char okk[100];
    scanf("%s", okk);
    return id;
}

// 路径预处理
int fway(int robot_id, int u, int v)
{
    int direction = -1;
    while (temporary_path[u][v] != -1)
    {
        direction = temporary_path[u][v];
        int qu = u + derx[direction], qv = v + dery[direction];
        u = qu, v = qv;
    }
    if (direction == -1)
    {
        get_goods(robot_id, u, v);
        return -1;
    }
    return Convert(direction);
}

// 找货
int Find_items(int robot_id, int u, int v)
{
    int distance[N][N];
    memset(distance, 0x3f, sizeof distance);
    distance[u][v] = 0;
    memset(temporary_path, -1, sizeof temporary_path);
    queue<pair<int, pair<int, int>>> que;
    que.push({0, {u, v}});

    while (que.size())
    {
        auto t = que.front();
        que.pop();
        int d = t.first, u = t.second.first, v = t.second.second;
        if (d > 150)
            return -1;
        if (goods_time_value[u][v][0] && distance[u][v] <= goods_time_value[u][v][0] && !robot_to_one_good[u][v])
        {
            robot_to_one_good[u][v] = true;
            return fway(robot_id, u, v);
        }
        for (int i = 3; i >= 0; i--)
        {
            int fu = u + derx[i], fv = v + dery[i];
            if (fu >= 0 && fu < n && fv >= 0 && fv < n && ch[fu][fv] != '*' && ch[fu][fv] != '#')
            {
                if (distance[fu][fv] > distance[u][v] + 1)
                {
                    distance[fu][fv] = distance[u][v] + 1;
                    temporary_path[fu][fv] = Convert(i);
                    que.push({distance[fu][fv], {fu, fv}});
                }
            }
        }
    }
    return -1;
}

// 放货移动动作原子操作
void move_delivery_action(int id, int direction, int x, int y, int fx, int fy)
{
    printf("move %d %d\n", id, direction);
    robot[id].a_step = false;
    robot_location[fx][fy] = id;
    robot_location[x][y] = -1;
    if (global_dist[fx][fy] == 0)
        Release_goods(id, fx, fy);
}
// 机器人去港口
void go_to_port(vector<pair<int, int>> &r_to_goods)
{
    sort(r_to_goods.begin(), r_to_goods.end());
    for (int i = 0; i < r_to_goods.size(); i++)
    {
        int id = r_to_goods[i].second;
        int u = robot[id].u, v = robot[id].v;
        int direction = path_to_berth[u][v];
        if (direction == -1)
            continue;
        if (!robot[id].a_step)
            continue;
        int fu = u + derx[direction], fv = v + dery[direction];
        if (robot_location[fu][fv] == -1)
            move_delivery_action(id, direction, u, v, fu, fv);
        else if (robot[robot_location[fu][fv]].a_step)
        {
            for (int j = 3; j >= 0; j--)
            {
                int qu = fu + derx[j], qv = fv + dery[j];
                if (qu >= 0 && qu < n && qv >= 0 && qv < n && robot_location[qu][qv] == -1 && ch[qu][qv] != '*' && ch[qu][qv] != '#')
                {
                    move_delivery_action(robot_location[fu][fv], j, fu, fv, qu, qv);
                    break;
                }
            }
            if (robot_location[fu][fv] == -1)
                move_delivery_action(id, direction, u, v, fu, fv);
        }
    }
}

// 取货移动动作原子操作
void move_get_action(int id, int direction, int x, int y, int fx, int fy)
{
    printf("move %d %d\n", id, direction);
    robot[id].a_step = false;
    robot_location[fx][fy] = id;
    robot_location[x][y] = -1;
    if (goods_time_value[fx][fy][0])
        get_goods(id, fx, fy);
}
// 机器人去找货物
void robot_goto_goods(vector<int> idle_robot)
{

    for (int i = 0; i < idle_robot.size(); i++)
    {
        int id = idle_robot[i];
        int u = robot[id].u, v = robot[id].v;
        if (!robot[id].a_step)
            continue;
        int direction = Find_items(id, robot[id].u, robot[id].v);
        int fu = robot[id].u + derx[direction], fv = robot[id].v + dery[direction];
        if (direction == -1)
            continue;

        if (robot_location[fu][fv] == -1)
            move_get_action(id, direction, u, v, fu, fv);
        else if (robot[robot_location[fu][fv]].a_step)
        {
            for (int j = 0; j < 4; j++)
            {
                int qu = fu + derx[j], qv = fv + dery[j];
                if (qu >= 0 && qu < n && qv >= 0 && qv < n && robot_location[qu][qv] == -1 && ch[qu][qv] != '*' && ch[qu][qv] != '#')
                {
                    move_get_action(robot_location[fu][fv], j, fu, fv, qu, qv);
                    break;
                }
            }
            if (robot_location[fu][fv] == -1)
                move_get_action(id, direction, u, v, fu, fv);
        }
    }
}

// 机器人调度
void Robot_scheduling()
{
    vector<pair<int, int>> r_to_goods; // 送货机器人
    vector<int> idle_robot;            // 取货机器人
    for (int i = 0; i < robot_num; i++)
    {
        if (global_dist[robot[i].u][robot[i].v] >= 200000)
            continue;
        if (robot[i].g_status == 1)
            r_to_goods.push_back(make_pair(global_dist[robot[i].u][robot[i].v], i));
        else
            idle_robot.push_back(i);
    }
    go_to_port(r_to_goods);
    robot_goto_goods(idle_robot);
}

struct node
{
    double pricesss;
    int id;
    bool operator<(node &t)
    {
        return t.pricesss > pricesss;
    }
} change_berth_cost[10], berth_cost[10];

double pre_ocupation_goods_num[5][berth_num] = {0};
int boat_flag[5] = {0}; // 船提前返回虚拟点标记
void boat_action(int frame)
{
    int max_zhen = 15000;

    // 5艘船  指令:ship到港口   go回到虚拟点
    for (int i = 0; i < 5; i++)
    {
        // 船信息
        int status = boat[i].status;   // 0移动 1正常 2泊位等待
        int pos = boat[i].pos;         // 目标泊位 -1表示虚拟点
        int cur_num = boat[i].cur_num; // 当前装载货物数

        int final_time = frame + berth[boat[i].pos].tsp_time;
        if (final_time == 15000)
        {
            pre_ocupation_goods_num[i][boat[i].pos] = 0; // 走后置0
            printf("go %d\n", i);
            boat[i].status = 0; // 船移动中
            boat[i].pos = -1;   // 目标虚拟点
            boat_flag[i] = 1;
            continue;
        }
        if (boat_flag[i] == 1)
        {
            continue;
        }

        // 10个港口优先级排列
        for (int j = 0; j < berth_num; j++)
        {
            int temp_value = 0;
            for (int k = 0; k < 5; k++)
            { // 五张表合并计算优先级
                temp_value += pre_ocupation_goods_num[k][j];
            }
            // 港口信息
            int tsp_time = berth[j].tsp_time;        // 虚拟点到港口时间
            int ld_speed = berth[j].ld_speed;        // 一帧装载货物数
            double goods_num = berth[j].goods_num;   // 当前港口总货物数
            double goods_cost = berth[j].goods_cost; // 当前港口货物价值总量
            double time_cost = 0;
            if (goods_num - (boat_capacity - boat[i].cur_num) > 0)
            {
                time_cost = ((goods_num / ld_speed) + 0.5 * tsp_time); // 虚拟点来回2*
            }
            else
            {
                time_cost = (((boat_capacity - boat[i].cur_num) / ld_speed) + 0.5 * tsp_time); // 虚拟点来回2*
            }

            berth_cost[j].pricesss = (goods_num - temp_value) / time_cost;
            berth_cost[j].id = j;
        }
        sort(berth_cost, berth_cost + 10);

        double go_threshold = 0.65;

        if (status == 0)
        {
            continue;
        } // 船在移动不处理(多余)

        if (status == 1 && pos == -1)
        { // 船在虚拟点出发到港口
            boat[i].cur_num = 0;
            boat[i].pos = berth_cost[9].id; // 船的目标泊位港口id
            pre_ocupation_goods_num[i][boat[i].pos] += (boat_capacity - boat[i].cur_num);
            printf("ship %d %d\n", i, boat[i].pos);
        }
        if (status == 1 && pos == boat[i].pos)
        { // 船到达任意港口boat[i].pos

            // 一帧装货
            int ld_speed = berth[boat[i].pos].ld_speed; // 当前码头装货速度

            if (cur_num < boat_capacity && berth[boat[i].pos].goods_num > 0)
            { // 船货物未满 且 港口还剩货

                if (berth[boat[i].pos].goods_num - ld_speed < 0 || boat[i].cur_num + ld_speed > boat_capacity)
                { // 港口即将搬空 且 船即将装满
                    double boat_can = boat_capacity - boat[i].cur_num;
                    double berth_leava = berth[boat[i].pos].goods_num;
                    if (boat_can > berth_leava)
                    {
                        berth[boat[i].pos].goods_num = 0;
                        boat[i].cur_num += berth_leava;
                        pre_ocupation_goods_num[i][boat[i].pos] -= berth_leava;
                    }
                    else
                    {
                        boat[i].cur_num = boat_capacity;
                        berth[boat[i].pos].goods_num -= boat_can;
                        pre_ocupation_goods_num[i][boat[i].pos] -= boat_can;
                    }
                }
                else
                {                                             // 正常搬运
                    berth[boat[i].pos].goods_num -= ld_speed; // 港口移除货物
                    boat[i].cur_num += ld_speed;
                    pre_ocupation_goods_num[i][boat[i].pos] -= ld_speed;
                }
                pre_ocupation_goods_num[i][boat[i].pos] -= ld_speed;
            }
            else
            {
                pre_ocupation_goods_num[i][boat[i].pos] = 0; // 走后置0
                if (boat[i].cur_num >= go_threshold * boat_capacity)
                { // 货到了70%，回虚拟点
                    printf("go %d\n", i);
                    continue;
                }
                else
                { // 货物未到70%,寻找下一个港口
                    for (int j = 0; j < berth_num; j++)
                    {
                        double ld_speed = berth[j].ld_speed; // 当前码头装货速度
                        // change:转港时间500帧
                        double temp_value = 0;
                        for (int k = 0; k < 5; k++)
                        { // 五张表合并计算优先级
                            temp_value += pre_ocupation_goods_num[k][j];
                        }
                        // 港口信息
                        double goods_num = berth[j].goods_num;   // 当前港口总货物数
                        double goods_cost = berth[j].goods_cost; // 当前港口货物价值总量
                        double time_cost = 0;
                        double temp_score = 0;
                        double boat_temp = boat_capacity - boat[i].cur_num;
                        if (goods_num - temp_value <= 0)
                        {
                            temp_score = 0;
                            time_cost = -1;
                            change_berth_cost[j].pricesss = 0;
                            change_berth_cost[j].id = j;
                            continue;
                        }
                        if ((goods_num - temp_value) - boat_temp > 0)
                        {
                            temp_score = 500 - 2 * ((goods_num - temp_value) - boat_temp);
                            time_cost = (boat_temp / ld_speed + (0.5 * 500));
                        }
                        else if ((goods_num - temp_value) - boat_temp == 0)
                        {
                            temp_score = 500;
                            time_cost = ((goods_num - temp_value) / ld_speed + (0.5 * 500));
                        }
                        else
                        {
                            temp_score = 500 + 5 * ((goods_num - temp_value) - boat_temp);
                            time_cost = (((goods_num - temp_value) / ld_speed) + (0.5 * 500));
                        }

                        change_berth_cost[j].pricesss = (temp_score) / time_cost;
                        change_berth_cost[j].id = j;
                    }

                    sort(change_berth_cost, change_berth_cost + 10);

                    boat[i].pos = change_berth_cost[9].id; // 船的目标泊位港口id
                    printf("ship %d %d\n", i, boat[i].pos);
                    pre_ocupation_goods_num[i][boat[i].pos] += (boat_capacity - boat[i].cur_num);
                }
            }
        }
    }
}

int main()
{
    Initialization();
    for (int frame = 1; frame <= 15000; frame++)
    {
        Input();
        Robot_scheduling();
        boat_action(frame);
        puts("OK");

        fflush(stdout);
    }
    return 0;
}
