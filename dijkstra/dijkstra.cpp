//
// Created by ogier on 24-4-3.
//
#include "dijkstra.h"

Dijkstra::Node::Node(double x, double y, float cost, double parent_index)
    : x(x), y(y), cost(cost), parent_index(parent_index) {}

Dijkstra::Dijkstra(const double resolution, const double robotRadius)
    : resolution(resolution), robot_radius(robotRadius) {}

void Dijkstra::setObstacle(vector<double> &ox, vector<double> &oy) {
  for (double i = -10; i < 60; i++) {
    //设置下边界
    ox.push_back(i);
    oy.push_back(-10.0);
  }
  for (double i = -10; i < 60; i++) {
    // 设置右边界
    ox.push_back(60.0);
    oy.push_back(i);
  }
  for (double i = -10; i < 61; i++) {
    // 设置上边界
    ox.push_back(i);
    oy.push_back(60.0);
  }
  for (double i = -10; i < 61; i++) {
    // 设置左边界
    ox.push_back(-10.0);
    oy.push_back(i);
  }
  for (double i = -10; i < 10; i++) {
    // 以下三个设置中间的障碍物
    ox.push_back(i);
    oy.push_back(10);
  }
  for (double i = 0; i < 30; i++) {
    ox.push_back(40.0 - i);
    oy.push_back(30);
  }
  for (double i = 0; i < 15; i++) {
    ox.push_back(60.0 - i);
    oy.push_back(30);
  }
}

// 计算障碍物地图,首先计算地图的边界范围,然后计算地图的长宽,然后初始化地图,遍历地图的每一个点
// ,判断与障碍物距离和机器人半径的关系,如果小于机器人半径则认为是障碍物
void Dijkstra::calObstacleMap(const vector<double> &ox, const vector<double> &oy) {
  min_x = round(*min_element(ox.begin(), ox.end()));
  min_y = round(*min_element(oy.begin(), oy.end()));
  max_x = round(*max_element(ox.begin(), ox.end()));
  max_y = round(*max_element(oy.begin(), oy.end()));

  cout << "min_x:" << min_x << "   min_y:" << min_y << "  max_x:" << max_x << "  max_y:" << max_y << endl;

  x_width = (max_x - min_x) / resolution;
  y_width = (max_y - min_y) / resolution;
  cout << "x_width:" << x_width << "   y_width:" << y_width << endl;

  obstacle_map = vector<vector<bool>>(x_width, vector<bool>(y_width, false));

  // 遍历地图的每一个点，判断与障碍物距离和机器人半径的关系，如果小于机器人半径，则认为是障碍物
  for (double i = 0; i < x_width; ++i) {
    double x = calPosition(i, min_x);
    for (double j = 0; j < y_width; ++j) {
      double y = calPosition(j, min_y);
      for (double k = 0; k < ox.size(); ++k) {
        double d = sqrt(pow(x - ox[k], 2) + pow(y - oy[k], 2));
        if (d <= robot_radius) {
          obstacle_map[i][j] = true;
          break;
        }
      }
    }
  }
}

// 计算栅格二维xy坐标在地图中的位置, index是栅格坐标，minp是地图这个维度的最小值
// 计算得到的是原始地图中的位置，不是栅格上的坐标
// 相当于使用分辨率计算的地图坐标再转回原始地图坐标
double Dijkstra::calPosition(double index, double minp) {
  return index * resolution + minp;
}

//计算移动代价
void Dijkstra::getMotionModel() {
  motion = {{1, 0, 1},
            {0, 1, 1},
            {-1, 0, 1},
            {0, -1, 1},
            {-1, -1, sqrt(2)},
            {-1, 1, sqrt(2)},
            {1, -1, sqrt(2)},
            {1, 1, sqrt(2)}};
}

//计算起点和终点的栅格地图坐标
double Dijkstra::calXyindex(double position, double minp) {
  return round((position - minp) / resolution);
}

//为地图范围内的每一个栅格分配计算一个索引，索引 = y * 宽度 + 这一行的x坐标
double Dijkstra::calIndex(Dijkstra::Node *node) {
  return (node->y - min_y) * x_width + (node->x - min_x);
}

// 判断栅格节点是否合法，边界与障碍物检测
bool Dijkstra::verifyNode(Dijkstra::Node *node) {
  double px = calPosition(node->x, min_x);
  double py = calPosition(node->y, min_y);
  //边界检测
  if (px < min_x || py < min_y || px >= max_x || py >= max_y) {
    return false;
  }
  //障碍物检测
  if (obstacle_map[node->x][node->y]) {
    return false;
  }
  return true;
}

//根据终点节点和已经搜索的节点，反推计算最终路径
pair<vector<double>, vector<double>> Dijkstra::calFinalPath(
    Dijkstra::Node *goal_node, map<double, Dijkstra::Node *> closed_set) {

  //声明存储搜索到轨迹点的容器，首先将终点加入
  vector<double> rx, ry;
  rx.push_back(calPosition(goal_node->x, min_x));
  ry.push_back(calPosition(goal_node->y, min_y));

  double parent_index = goal_node->parent_index;

  //从终点开始，根据父节点索引，反推计算路径
  while (parent_index != -1) {
    Node *node = closed_set[parent_index];
    rx.push_back(calPosition(node->x, min_x));
    ry.push_back(calPosition(node->y, min_y));
    parent_index = node->parent_index;
  }
  return make_pair(rx, ry);
}

pair<vector<double>, vector<double>> Dijkstra::planning(vector<double> start, vector<double> goal) {
  double sx = start[0], sy = start[1];
  double gx = goal[0], gy = goal[1];
  //计算栅格地图二维坐标，初始化cost值为0（因为距离原点为0），父节点索引-1，表示到头了
  Node *start_node = new Node(calXyindex(sx, min_x), calXyindex(sy, min_y), 0.0, -1);
  Node *goal_node = new Node(calXyindex(gx, min_x), calXyindex(gy, min_y), 0.0, -1);

  //初始化两个集合，open_set为待搜索的集合，closed_set为已搜索的集合
  map<double, Node *> open_set, closed_set;
  //计算对应栅格的索引，将起始点加入open集
  open_set[calIndex(start_node)] = start_node;

  Node *current = nullptr;
  //开始进行搜索
  while (true) {
    //初始化cur_id ,cost 为极大值
    double cur_id = numeric_limits<double>::max();
    double cost = numeric_limits<double>::max();
    for (auto iter = open_set.begin(); iter != open_set.end(); ++iter) {
      //遍历open_set中的所有值，寻找cost最小的值
      if (iter->second->cost < cost) {
        cost = iter->second->cost;
        cur_id = iter->first;
      }
    }
    current = open_set[cur_id];
    plotGraph(current);

    //如果找到了终点，跳出循环
    if (abs(current->x - goal_node->x) < EPS && abs(current->y - goal_node->y) < EPS) {
      cout << "Find Goal" << endl;
      goal_node->parent_index = current->parent_index;
      goal_node->cost = current->cost;
      break;
    }
    //如果没有找到终点，将当前节点从open set中删除，加入到closed set中
    auto iter = open_set.find(cur_id);
    open_set.erase(iter);
    closed_set[cur_id] = current;

    for (vector<double> move : motion) {
      //通过模型，探究最小cost节点处周围的点；注意这里将parent_index设置为当前节点的索引
      Node *node = new Node(current->x + move[0], current->y + move[1], current->cost + move[2], cur_id);
      //计算节点的索引，判断在哪个set或者是否合法
      double n_id = calIndex(node);
      //如果在closed_set中，跳过
      if (closed_set.find(n_id) != closed_set.end())
        continue;
      //如果超出边界或者碰到障碍物了,跳过
      if (!verifyNode(node))
        continue;
      //如果open set中没有这个节点,加入open set
      if (open_set.find(n_id) == open_set.end()) {
        open_set[n_id] = node;
      } else {//如果有这个节点，比较cost值，更新,保留cost更小的
        if (open_set[n_id]->cost >= node->cost)
          open_set[n_id] = node;
      }
    }
  }
  return calFinalPath(goal_node, closed_set);
}

void Dijkstra::plotGraph(Dijkstra::Node *current) {
  //计算节点的原始坐标，并绘制
  plt::plot(vector<double>{calPosition(current->x, min_x)},
            vector<double>{calPosition(current->y, min_y)}, "xc");
  plt::pause(0.0000001);
}

void Dijkstra::set(const vector<double> &st,
                   const vector<double> &go,
                   const vector<double> &ox,
                   const vector<double> &oy) {
  Dijkstra::st = st;
  Dijkstra::go = go;
  Dijkstra::ox = ox;
  Dijkstra::oy = oy;
}