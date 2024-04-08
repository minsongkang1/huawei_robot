#pragma GCC optimize(2)
#include <bits/stdc++.h>
using namespace std;

const int n = 200; // 地图维度
const int robot_num = 10;
const int berth_num = 10; // 泊口数量
const int N = 201;        // 地图维度(N,N)

struct Robot
{
    int x, y, goods; // 坐标(x,y) 拿货状态0/1
    int status;
    bool have_step;
    int val;
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
    int pos, status, cur_num;
} boat[10];

int money, id, boat_capacity; // 船容积
char ch[N][N];
int berth_local[N][N];
int Goods[N][N][3]; // Good[i][j][0] 表示货物距离消失剩余的时间，[1]表示金额,[2]表示是否被挑选
int dx[4] = {0, 0, -1, 1}, dy[4] = {1, -1, 0, 0};
vector<pair<int, int>> berth_l;
int robot_in_graph[N][N];
int total_money;
int graphdist[N][N];
int globalpath[N][N];
int tmppath[N][N];
bool rtog[N][N]; // 锁定物品只能被一个机器人选取
FILE *file;

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

// 放货动作原子操作
void pull_goods(int id, int x, int y)
{
    vector<pair<int, int>> v;
    for (int i = 0; i < berth_l.size(); i++)
    {
        int cnt = abs(x - berth_l[i].first) + abs(y - berth_l[i].second);
        v.push_back(make_pair(cnt, i));
    }
    sort(v.begin(), v.end());
    int bidx = v[0].second;
    berth[bidx].goods_cost += robot[id].val;
    total_money += robot[id].val;
    berth[bidx].goods_num++;
    printf("pull %d\n", id);
    robot[id].goods = 0;
    robot[id].val = 0;
}

// 取货动作原子操作
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
    memset(globalpath, -1, sizeof globalpath);
    memset(graphdist, 0x3f, sizeof graphdist);

    priority_queue<pair<int, pair<int, int>>, vector<pair<int, pair<int, int>>>, greater<pair<int, pair<int, int>>>> q;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
        {
            if (ch[i][j] == 'B')
            {
                graphdist[i][j] = 0;
                q.push({0, {i, j}});
            }
        }
    while (q.size())
    {
        auto t = q.top();
        q.pop();
        int dd = t.first, x = t.second.first, y = t.second.second;
        for (int i : random_num())
        {
            int px = x + dx[i], py = y + dy[i];
            if (px >= 0 && px < n && py >= 0 && py < n && ch[px][py] != '*' && ch[px][py] != '#')
            {
                if (graphdist[px][py] > graphdist[x][y] + 1)
                {
                    graphdist[px][py] = graphdist[x][y] + 1;
                    globalpath[px][py] = transfor(i);
                    q.push({graphdist[px][py], {px, py}});
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

// 找物品
int Robot_find_goods(int robot_id, int x, int y)
{
    int dist[N][N];
    memset(dist, 0x3f, sizeof dist);
    dist[x][y] = 0;
    memset(tmppath, -1, sizeof tmppath);
    queue<pair<int, pair<int, int>>> q;
    q.push({0, {x, y}});

    while (q.size())
    {
        auto t = q.front();
        q.pop();
        int d = t.first, x = t.second.first, y = t.second.second;
        if (d > 150)
            return -1;
        if (Goods[x][y][0] && dist[x][y] <= Goods[x][y][0] && !rtog[x][y])
        {
            rtog[x][y] = true;
            return find_path(robot_id, x, y);
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
    return -1;
}

// 放货移动动作原子操作
void move_pull(int id, int drc, int from_x, int from_y, int to_x, int to_y)
{
    printf("move %d %d\n", id, drc);
    robot[id].have_step = false;
    robot_in_graph[to_x][to_y] = id;
    robot_in_graph[from_x][from_y] = -1;
    if (graphdist[to_x][to_y] == 0)
        pull_goods(id, to_x, to_y);
}
// 机器人去港口
void to_berth(vector<pair<int, int>> &robot_sort)
{
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
        if (robot_in_graph[px][py] == -1)
            move_pull(id, drc, x, y, px, py);
        else if (robot[robot_in_graph[px][py]].have_step)
        {
            vector<int> ans = random_num();
            for (int j : ans)
            {
                int tx = px + dx[j], ty = py + dy[j];
                if (tx >= 0 && tx < n && ty >= 0 && ty < n && robot_in_graph[tx][ty] == -1 && ch[tx][ty] != '*' && ch[tx][ty] != '#')
                {
                    move_pull(robot_in_graph[px][py], j, px, py, tx, ty);
                    break;
                }
            }
            if (robot_in_graph[px][py] == -1)
                move_pull(id, drc, x, y, px, py);
        }
    }
}

// 取货移动动作原子操作
void move_get(int id, int drc, int from_x, int from_y, int to_x, int to_y)
{
    printf("move %d %d\n", id, drc);
    robot[id].have_step = false;
    robot_in_graph[to_x][to_y] = id;
    robot_in_graph[from_x][from_y] = -1;
    if (Goods[to_x][to_y][0])
        get_goods(id, to_x, to_y);
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
        int px = robot[id].x + dx[drc], py = robot[id].y + dy[drc];
        if (drc == -1)
            continue;

        if (robot_in_graph[px][py] == -1)
            move_get(id, drc, x, y, px, py);
        else if (robot[robot_in_graph[px][py]].have_step)
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
    vector<pair<int, int>> robot_sort; // 送货机器人
    vector<int> robot_free;            // 取货机器人
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
int main()
{
    Init();
    file = fopen("outputlog.txt", "w");
    for (int zhen = 1; zhen <= 15000; zhen++)
    {
        Input();
        robot_action();
        // boat_action(zhen);
        fprintf(file, "zhen: %d robot_money: %d || boat_money: %d\n", id, total_money, money);
        puts("OK");
        fflush(stdout);
    }
    fclose(file);
    return 0;
}
