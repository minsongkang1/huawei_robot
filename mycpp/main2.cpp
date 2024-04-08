// 3.22 最终版机器人（增加了ban掉密集港口的功能，助力机器人最后冲刺；修复了机器人走回头路的弊端，冲刺32强）
#pragma GCC optimize(2)
#include <bits/stdc++.h>
using namespace std;

const int n = 200; // 地图维度
const int robot_num = 10;
const int berth_num = 10; // 泊口数量
const int N = 201;        // 地图维度(N,N)
int block[berth_num] = {0};

struct Robot
{
    int x, y, goods; // 坐标(x,y) 拿货状态0/1
    int status;
    int mbx, mby;
    bool have_step;
    int val;
    int last_berth_num = -1;
} robot[robot_num + 1];

struct Berth
{
    int x;
    int y;
    int transport_time;
    int loading_speed; // 装载货物速度
    double goods_num;
    double goods_cost;

} berth[berth_num + 1];

struct Boat
{
    int pos, status, cur_num, pre_pos;
} boat[10];

int money, id, boat_capacity; // 船容积
char ch[N][N];
int berth_local[N][N];
int Goods[N][N][3]; // Good[i][j][0] 表示货物距离消失剩余的时间，[1]表示金额,[2]表示是否被挑选
int dx[4] = {0, 0, -1, 1}, dy[4] = {1, -1, 0, 0};
vector<pair<int, int>> berth_l;
int robot_in_graph[N][N];
int total_money;
int graphdist[N][N], globalpath[N][N];
int select_dist[11][N][N], select_path[11][N][N], dist_num[11];
int tmppath[N][N];
bool rtog[N][N]; // 锁定物品只能被一个机器人选取
int berth_dist[berth_num + 1][berth_num + 1];
int adjust_distnum = 50;
FILE *file;
set<int> ban_berth;
set<int> all_berth = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
// 随机方向,防止避让极端
vector<int> random_num()
{
    vector<int> nums = {0, 1, 2, 3};
    // random_device rd;
    unsigned seed = chrono::system_clock::now().time_since_epoch().count();
    mt19937 g(seed);
    shuffle(nums.begin(), nums.end(), g);
    return nums;
}

// 根据'B'找港口id
int B_find_id(int x, int y)
{
    vector<pair<int, int>> v;
    for (int i = 0; i < berth_l.size(); i++)
    {
        int cnt = abs(x - berth_l[i].first) + abs(y - berth_l[i].second);
        v.push_back(make_pair(cnt, i));
    }
    sort(v.begin(), v.end());
    return v[0].second;
}
// 方向变换
int transfor(int i)
{
    switch (i)
    {
    case 0:
        return 1;
    case 1:
        return 0;
    case 2:
        return 3;
    default:
        return 2;
    }
}

void pull_goods(int id, int x, int y)
{
    int bidx = B_find_id(x, y);
    berth[bidx].goods_cost += robot[id].val;
    total_money += robot[id].val;
    berth[bidx].goods_num++;
    printf("pull %d\n", id);
    robot[id].last_berth_num = bidx;
    robot[id].goods = 0;
    robot[id].val = 0;
}

void get_goods(int id, int x, int y)
{
    robot[id].goods = 1;
    printf("get %d\n", id);
    robot[id].val = Goods[x][y][1];
    Goods[x][y][1] = 0;
    Goods[x][y][2] = 0;
    Goods[x][y][0] = 0;
    // mp.erase({x, y});
}

// 距离预处理
void dist_pre(int id, int graphdist[][N], int globalpath[][N])
{
    queue<pair<int, pair<int, int>>> q;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
        {
            globalpath[i][j] = -1;
            graphdist[i][j] = 0x3f3f3f3f;
            if (id == -1 && ch[i][j] == 'B')
            {
                graphdist[i][j] = 0;
                q.push({0, {i, j}});
            }
        }
    if (id != -1)
    {
        for (int i = berth[id].x - 2, ii = 0; ii < 4; ii++, i++)
        {
            for (int j = berth[id].y - 1, jj = 0; jj < 4; jj++, j++)
            {
                graphdist[i][j] = 0;
                q.push({0, {i, j}});
            }
        }
    }

    while (q.size())
    {
        auto t = q.front();
        q.pop();
        int dd = t.first, x = t.second.first, y = t.second.second;
        if (ch[x][y] == 'B' && id != -1)
        {
            int tid = B_find_id(x, y);
            berth_dist[id][tid] = berth_dist[tid][id] = min(dd, berth_dist[id][tid]);
        }
        for (int i : random_num())
        {
            int px = x + dx[i], py = y + dy[i];
            if (px >= 0 && px < n && py >= 0 && py < n && ch[px][py] != '*' && ch[px][py] != '#')
            {
                if (graphdist[px][py] > graphdist[x][y] + 1)
                {
                    graphdist[px][py] = graphdist[x][y] + 1;
                    globalpath[px][py] = transfor(i);
                    if (id != -1 && graphdist[px][py] < adjust_distnum)
                        dist_num[id]++;
                    q.push({graphdist[px][py], {px, py}});
                }
            }
        }
    }
}

void ban_(int id)
{
    all_berth.erase(id);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (ch[i][j] == '*' || ch[i][j] == '#')
                continue;
            int t = -1, dist = 0x3f3f3f3f;
            for (int x : all_berth)
            {
                if (select_dist[x][i][j] < dist)
                {
                    t = x;
                    dist = select_dist[x][i][j];
                }
            }
            if (t != -1)
            {
                graphdist[i][j] = dist;
                globalpath[i][j] = select_path[t][i][j];
            }
        }
    }
    if (ban_berth.count(id))
        ban_berth.erase(id);
}

void ban_order()
{
    int max_ban = 0;
    for (int cnt = 0; cnt < 5; cnt++)
    {
        int x = 0, y = 0;
        int dist = 0x3f3f3f3f;
        int ban_id = 0;
        for (int i = 0; i < berth_num; i++)
        {
            for (int j = i + 1; j < berth_num; j++)
            {
                if (berth_dist[i][j] < dist)
                {
                    dist = berth_dist[i][j];
                    x = i, y = j;
                }
            }
        }
        if (x == y)
            continue;
        if (dist_num[x] < dist_num[y])
            ban_id = x;
        else
            ban_id = y;
        for (int i = 0; i < berth_num; i++)
        {
            berth_dist[i][ban_id] = berth_dist[ban_id][i] = 0x3f3f3f3f;
        }
        if (dist < 20 && max_ban < 2)
        {
            ban_(ban_id);
            max_ban++;
        }
        else
            ban_berth.insert(ban_id);
    }
}

void Init()
{
    for (int i = 0; i < n; i++)
        scanf("%s", ch[i]);
    for (int i = 0; i < berth_num; i++)
    {
        int id;
        scanf("%d", &id);
        scanf("%d%d%d%d", &berth[id].x, &berth[id].y, &berth[id].transport_time, &berth[id].loading_speed);
        berth[id].x += 2, berth[id].y += 1;
        berth_local[berth[id].x][berth[id].y]++;
        berth_l.push_back({berth[id].x, berth[id].y});
    }
    memset(berth_dist, 0x3f, sizeof berth_dist);
    dist_pre(-1, graphdist, globalpath);
    for (int i = 0; i < berth_num; i++)
        dist_pre(i, select_dist[i], select_path[i]);
    ban_order();
    scanf("%d", &boat_capacity);
    char okk[100];
    scanf("%s", okk);
    printf("OK\n");
    fflush(stdout);
}

// 逐渐消失的货物
void Countdown()
{
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (Goods[i][j][0])
            {
                Goods[i][j][0]--;
                if (Goods[i][j][0] == 0)
                {
                    Goods[i][j][1] = 0;
                }
            }
        }
    }
}

int Input()
{
    Countdown();
    memset(rtog, 0, sizeof rtog);
    memset(robot_in_graph, -1, sizeof robot_in_graph);
    scanf("%d%d", &id, &money);
    int num;
    scanf("%d", &num);
    for (int i = 1; i <= num; i++)
    {
        int x, y, val;
        scanf("%d%d%d", &x, &y, &val);
        Goods[x][y][0] = 1000, Goods[x][y][1] = val;
    }

    for (int i = 0; i < robot_num; i++)
    {
        int sts;
        scanf("%d%d%d%d", &robot[i].goods, &robot[i].x, &robot[i].y, &sts);
        robot_in_graph[robot[i].x][robot[i].y] = i;
        robot[i].have_step = true;
    }
    for (int i = 0; i < 5; i++)
        scanf("%d%d\n", &boat[i].status, &boat[i].pos);
    char okk[100];
    scanf("%s", okk);
    return id;
}

// 路径预处理
int find_path(int robot_id, int x, int y)
{
    int drc = -1;
    while (tmppath[x][y] != -1)
    {
        drc = tmppath[x][y];
        int tx = x + dx[drc], ty = y + dy[drc];
        x = tx, y = ty;
    }
    if (drc == -1)
    {
        get_goods(robot_id, x, y);
        return -1;
    }
    return transfor(drc);
}

int find_best_goods(vector<pair<double, pair<int, int>>> &v_goods, int id)
{
    sort(v_goods.begin(), v_goods.end());
    reverse(v_goods.begin(), v_goods.end());
    int x = -1, y = -1;
    int lsberth = robot[id].last_berth_num, tttx = robot[id].x, ttty = robot[id].y;
    for (int i = 0; i < v_goods.size(); i++)
    {
        x = v_goods[i].second.first, y = v_goods[i].second.second;
        int diss = Goods[x][y][1] / v_goods[i].first - graphdist[x][y] + 0.1;
        if (lsberth == -1 || select_dist[lsberth][x][y] + 8 >= select_dist[lsberth][tttx][ttty] + diss)
        {
            rtog[x][y] = true;
            return find_path(id, x, y);
        }
    }
    x = v_goods[0].second.first, y = v_goods[0].second.second;
    rtog[x][y] = true;
    return find_path(id, x, y);
}
// 找物品
int Robot_find_goods(int robot_id, int x, int y)
{
    int dist[N][N];
    memset(dist, 0x3f, sizeof dist);
    dist[x][y] = 0;
    memset(tmppath, -1, sizeof tmppath);
    queue<pair<int, pair<int, int>>> q;
    q.push({0, {x, y}});
    vector<pair<double, pair<int, int>>> v_goods;
    while (q.size())
    {
        auto t = q.front();
        q.pop();
        int d = t.first, x = t.second.first, y = t.second.second;
        if (d > 150 || v_goods.size() >= 5)
        {
            if (v_goods.size())
            {
                return find_best_goods(v_goods, robot_id);
            }
            else
                return -1;
        }
        int lsberth = robot[robot_id].last_berth_num, tttx = robot[robot_id].x, ttty = robot[robot_id].y;
        if (Goods[x][y][0] && dist[x][y] <= Goods[x][y][0] && !rtog[x][y])
        {
            double cost_g = (double)Goods[x][y][1] / (dist[x][y] + graphdist[x][y]);
            v_goods.push_back({cost_g, {x, y}});
        }
        for (int i = 3; i >= 0; i--)
        {
            int px = x + dx[i], py = y + dy[i];
            if (px >= 0 && px < n && py >= 0 && py < n && ch[px][py] != '*' && ch[px][py] != '#')
            {
                if (dist[px][py] > dist[x][y] + 1)
                {
                    dist[px][py] = dist[x][y] + 1;
                    tmppath[px][py] = transfor(i);
                    q.push({dist[px][py], {px, py}});
                }
            }
        }
    }
    if (v_goods.size())
        return find_best_goods(v_goods, robot_id);

    return -1;
}

void move_pull(int id, int drc, int from_x, int from_y, int to_x, int to_y)
{
    printf("move %d %d\n", id, drc);
    robot[id].have_step = false;
    robot_in_graph[to_x][to_y] = id;
    robot_in_graph[from_x][from_y] = -1;
    if (graphdist[to_x][to_y] == 0)
        pull_goods(id, to_x, to_y);
}

void move_get(int id, int drc, int from_x, int from_y, int to_x, int to_y)
{
    printf("move %d %d\n", id, drc);
    robot[id].have_step = false;
    robot_in_graph[to_x][to_y] = id;
    robot_in_graph[from_x][from_y] = -1;
    if (Goods[to_x][to_y][0])
        get_goods(id, to_x, to_y);
}

bool rot[11] = {false};
bool dfs(int id, int drc, int x, int y, int px, int py)
{
    if (rot[id])
        return false;
    rot[id] = true;
    if (robot_in_graph[px][py] != -1 && robot[robot_in_graph[px][py]].have_step)
    {
        vector<int> ans = random_num();
        for (int j : ans)
        {
            int tx = px + dx[j], ty = py + dy[j];
            if (tx == x && ty == y)
                continue;
            if (tx >= 0 && tx < n && ty >= 0 && ty < n && ch[tx][ty] != '*' && ch[tx][ty] != '#')
            {
                if (dfs(robot_in_graph[px][py], j, px, py, tx, ty))
                {
                    move_get(robot_in_graph[px][py], j, px, py, tx, ty);
                    break;
                }
            }
        }
    }
    if (robot_in_graph[px][py] == -1)
        return true;
    return false;
}

// 机器人去港口
void to_berth(vector<pair<int, int>> &robot_sort)
{
    memset(rot, 0, sizeof rot);
    sort(robot_sort.begin(), robot_sort.end());
    for (int i = 0; i < robot_sort.size(); i++)
    {
        int id = robot_sort[i].second;
        int x = robot[id].x, y = robot[id].y;
        int drc = globalpath[x][y];
        if (drc == -1)
            continue;
        if (!robot[id].have_step)
            continue;
        int px = x + dx[drc], py = y + dy[drc];
        if (dfs(id, drc, x, y, px, py))
            move_pull(id, drc, x, y, px, py);
    }
}

// 机器人去找货物
void to_goods(vector<int> robot_free)
{

    for (int i = 0; i < robot_free.size(); i++)
    {
        int id = robot_free[i];
        int x = robot[id].x, y = robot[id].y;
        if (!robot[id].have_step)
            continue;
        int drc = Robot_find_goods(id, robot[id].x, robot[id].y);

        if (drc == -1)
        {
            for (int i = 0; i < 4; i++)
            {
                int tx = robot[id].x + dx[i], ty = robot[id].y + dy[i];
                if (tx >= 0 && tx < n && ty >= 0 && ty < n && graphdist[tx][ty] > graphdist[x][y] &&
                    robot_in_graph[tx][ty] == -1 && ch[tx][ty] != '*' && ch[tx][ty] != '#')
                {
                    move_get(id, i, x, y, tx, ty);
                    break;
                }
            }
            continue;
        }
        int px = robot[id].x + dx[drc], py = robot[id].y + dy[drc];
        if (robot_in_graph[px][py] == -1)
            move_get(id, drc, x, y, px, py);
        else if (robot[robot_in_graph[px][py]].goods != 1 && robot[robot_in_graph[px][py]].have_step)
        {
            vector<int> ans = random_num();
            for (int j : ans)
            {
                int tx = px + dx[j], ty = py + dy[j];
                if (tx >= 0 && tx < n && ty >= 0 && ty < n && robot_in_graph[tx][ty] == -1 && ch[tx][ty] != '*' && ch[tx][ty] != '#')
                {
                    move_get(robot_in_graph[px][py], j, px, py, tx, ty);
                    break;
                }
            }
            if (robot_in_graph[px][py] == -1)
                move_get(id, drc, x, y, px, py);
        }
    }
}

// 机器人调度
void robot_action()
{
    vector<pair<int, int>> robot_sort;
    vector<int> robot_free;
    for (int i = 0; i < robot_num; i++)
    {
        if (graphdist[robot[i].x][robot[i].y] >= 200000)
            continue;
        if (robot[i].goods == 1)
            robot_sort.push_back(make_pair(graphdist[robot[i].x][robot[i].y], i));
        else
            robot_free.push_back(i);
    }
    to_berth(robot_sort);
    to_goods(robot_free);
}

void ban_action(int zhen)
{
    if (zhen <= 11600)
        return;
    for (int i = 0; i < 10; i++)
    {
        if (berth[i].goods_num == 0 && ban_berth.count(i))
            ban_(i);
    }
}

struct CBC
{
    double daijia;
    int id;
    bool operator<(CBC &t)
    {
        return t.daijia > this->daijia;
    }
} change_berth_cost[10], berth_cost[10];

double pre_ocupation_goods_num[5][berth_num] = {0};
int boat_flag[5] = {0}; // 船提前返回虚拟点标记
void boat_action(int zhen)
{
    int c = 15000;
    double go_threshold = 0.75;
    int deadline = 12000;

    for (int o = 0; o < berth_num; o++)
    {
        int temp_value = 0;
        for (int k = 0; k < 5; k++)
        { // 五张表合并计算优先级
            temp_value += pre_ocupation_goods_num[k][o];
        }
        //   fprintf(file ,"berth_ID:%d\tberth_num:%lf\tberth_temp:%d\n",o,berth[o].goods_num,temp_value);
    }

    // 5艘船  指令:ship到港口   go回到虚拟点
    for (int i = 0; i < 5; i++)
    {
        // 船信息
        int status = boat[i].status;   // 0移动 1正常 2泊位等待
        int pos = boat[i].pos;         // 目标泊位 -1表示虚拟点
        int cur_num = boat[i].cur_num; // 当前装载货物数
        int pre_pos = boat[i].pre_pos;

        //  fprintf(file ,"boadID:%d\tboadStatus:%d\tboad_pos:%d\tboad_curpos:%d\tcur_num:%d\n",i,status,pos,boat[i].pre_pos,cur_num);

        int final_time = 0; // ��0��ԭ�߼���

        int zhen = id;
        if (zhen > deadline)
        { // 12000
            if (boat[i].status == 0 && boat[i].pos == -1 && boat[i].pre_pos != -1)
            { // 船从港口往虚拟点行驶，没钱
                final_time = 0;
            }
            if (boat[i].status == 0 && boat[i].pos != -1 && boat[i].pre_pos == -1)
            { // 船从虚拟点往港口行驶，已经在回了
                final_time = 0;
            }
            if (boat[i].status == 0 && boat[i].pos != -1 && boat[i].pre_pos != -1)
            { // 船在港口之间行驶，有钱
                // fprintf(file, "#change: zhen:%d boat:%d pre_trans_time:%d  trans_time:%d pre_pos:%d pos:%d\n", zhen, i, berth[boat[i].pre_pos].transport_time, berth[boat[i].pos].transport_time,boat[i].pre_pos, boat[i].pos);
                final_time = zhen + berth[boat[i].pre_pos].transport_time; // 计算临界帧
            }
            if (boat[i].status == 1 && boat[i].pos != -1)
            { // 船停靠在港口，有钱
                // fprintf(file, "#to: zhen:%d boat:%d trans_time:%d pre_pos:%d pos:%d\n", zhen, i, berth[boat[i].pos].transport_time, boat[i].pre_pos, boat[i].pos);
                final_time = zhen + berth[boat[i].pos].transport_time;
                if (final_time < 15000)
                { // װ������
                    go_threshold = 0.9;
                }
            }
            if (boat[i].status == 1 && boat[i].pos == -1)
            {                   // 船在虚拟点，无钱
                final_time = 0; // �����棬�Զ�Ѱ�Ҹۿ�
            }

            if (final_time >= 15000)
            {
                //	        	if(boat[i].status == 0 && boat[i].pos!=-1 && boat[i].pre_pos!=-1){
                //	        		fprintf(file, "#ת�۱��ȷ�������� zhen:%d boat:%d pre_time:%d time:%d\n", zhen, i,berth[boat[i].pre_pos].transport_time,berth[boat[i].pos].transport_time);
                //				}
                //	        	if(boat[i].status == 1 && boat[i].pos!=-1){
                //	        		fprintf(file, "#�ڸ۱��ȷ�������� zhen:%d boat:%d pre_time:%d time:%d\n", zhen, i,berth[boat[i].pre_pos].transport_time,berth[boat[i].pos].transport_time);
                //				}
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
            int transport_time = berth[j].transport_time; // 虚拟点到港口时间
            int loading_speed = berth[j].loading_speed;   // 一帧装载货物数
            double goods_num = berth[j].goods_num;        // 当前港口总货物数
            double goods_cost = berth[j].goods_cost;      // 当前港口货物价值总量
            double time_cost = 0;

            if (goods_num - temp_value <= 0)
            {
                berth_cost[j].daijia = 0;
                berth_cost[j].id = j;
            }
            else
            {
                if ((goods_num - temp_value) - (boat_capacity - boat[i].cur_num) > 0)
                {
                    time_cost = (goods_num - temp_value) / loading_speed + 0.3 * transport_time;
                }
                else
                {
                    time_cost = ((boat_capacity - boat[i].cur_num) / loading_speed) + 0.3 * transport_time;
                }
                berth_cost[j].daijia = (goods_num - temp_value) / time_cost;
                berth_cost[j].id = j;
            }

            //            if(goods_num - (boat_capacity - boat[i].cur_num) > 0){
            //                time_cost = ( (goods_num / loading_speed) + 0.5*transport_time); //
            //            }
            //            else{
            //                time_cost = ( ((boat_capacity - boat[i].cur_num) / loading_speed) + 0.5*transport_time); // 虚拟点来回2*
            //            }
            //
            //            berth_cost[j].daijia = (goods_num - temp_value)/ time_cost;
            //            berth_cost[j].id = j;
        }
        sort(berth_cost, berth_cost + 10);

        if (status == 0)
        {
            continue;
        } // 船在移动不处理(多余)

        if (status == 1 && pos == -1)
        { // 船在虚拟点出发到港口
            boat[i].pre_pos = pos;
            boat[i].cur_num = 0;
            boat[i].pos = berth_cost[9].id; // 船的目标泊位港口id
            pre_ocupation_goods_num[i][boat[i].pos] += (boat_capacity - boat[i].cur_num);
            printf("ship %d %d\n", i, boat[i].pos);
        }
        if (status == 1 && pos == boat[i].pos && pos != -1)
        { // 船到达任意港口boat[i].pos

            // 一帧装货
            int loading_speed = berth[boat[i].pos].loading_speed; // 当前码头装货速度
            if (cur_num < boat_capacity && berth[boat[i].pos].goods_num > 0)
            { // 船货物未满 且 港口还剩货

                if (berth[boat[i].pos].goods_num - loading_speed < 0 || boat[i].cur_num + loading_speed > boat_capacity)
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
                {                                                  // 正常搬运
                    berth[boat[i].pos].goods_num -= loading_speed; // 港口移除货物
                    boat[i].cur_num += loading_speed;
                    pre_ocupation_goods_num[i][boat[i].pos] -= loading_speed;
                }
            }
            else
            {
                pre_ocupation_goods_num[i][boat[i].pos] = 0; // 走后置0
                if (zhen >= deadline && boat[i].cur_num >= 0.9 * boat_capacity)
                { // 最后关头多转港少来回
                    printf("go %d\n", i);
                    boat[i].pre_pos = boat[i].pos;
                    continue;
                }
                if (zhen < deadline && boat[i].cur_num >= go_threshold * boat_capacity)
                { // 货到了70%，回虚拟点
                    printf("go %d\n", i);
                    boat[i].pre_pos = boat[i].pos;
                    continue;
                }
                else
                { // 货物未到70%,寻找下一个港口
                    for (int j = 0; j < berth_num; j++)
                    {
                        double loading_speed = berth[j].loading_speed; // 当前码头装货速度
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
                        { // 如果货少直接pass
                            temp_score = 0;
                            time_cost = -1;
                            change_berth_cost[j].daijia = 0;
                            change_berth_cost[j].id = j;
                            continue;
                        }
                        if ((goods_num - temp_value) - boat_temp > 0)
                        {
                            temp_score = 500 - 3 * ((goods_num - temp_value) - boat_temp);
                            time_cost = (boat_temp / loading_speed + (0.5 * 500));
                        }
                        else if ((goods_num - temp_value) - boat_temp == 0)
                        {
                            temp_score = 500;
                            time_cost = ((goods_num - temp_value) / loading_speed + (0.5 * 500));
                        }
                        else
                        {
                            temp_score = 500 + 2 * ((goods_num - temp_value) - boat_temp);
                            time_cost = (((goods_num - temp_value) / loading_speed) + (0.5 * 500));
                        }

                        change_berth_cost[j].daijia = (temp_score) / time_cost;
                        change_berth_cost[j].id = j;
                    }

                    sort(change_berth_cost, change_berth_cost + 10);

                    boat[i].pre_pos = boat[i].pos;
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
    Init();
    file = fopen("outputlog.txt", "w");
    for (int zhen = 1; zhen <= 15000; zhen++)
    {
        Input();
        ban_action(zhen);
        robot_action();
        boat_action(zhen);
        fprintf(file, "zhen: %d robot_money: %d || boat_money: %d\n", id, total_money, money);
        puts("OK");

        fflush(stdout);
    }
    fclose(file);
    return 0;
}
