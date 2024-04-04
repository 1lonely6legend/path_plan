//
// Created by ogier on 24-4-3.
//
#include "dijkstra.h"

Dijkstra::Node::Node(double x, double y, float cost, double parent_index)
    : x(x), y(y), cost(cost), parent_index(parent_index) {}

Dijkstra::Dijkstra(const double resolution, const double robotRadius)
    : resolution_(resolution), robot_radius_(robotRadius) {}

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
  min_x_ = round(*min_element(ox.begin(), ox.end()));
  min_y_ = round(*min_element(oy.begin(), oy.end()));
  max_x_ = round(*max_element(ox.begin(), ox.end()));
  max_y_ = round(*max_element(oy.begin(), oy.end()));

  cout << "min_x:" << min_x_ << "   min_y:" << min_y_ << "  max_x:" << max_x_ << "  max_y:" << max_y_ << endl;

  x_width_ = (max_x_ - min_x_) / resolution_;
  y_width_ = (max_y_ - min_y_) / resolution_;
  cout << "x_width:" << x_width_ << "   y_width:" << y_width_ << endl;

  obstacle_map_ = vector<vector<bool>>(x_width_, vector<bool>(y_width_, false));

  // 遍历地图的每一个点，判断与障碍物距离和机器人半径的关系，如果小于机器人半径，则认为是障碍物
  for (double i = 0; i < x_width_; ++i) {
    double x = calPosition(i, min_x_);
    for (double j = 0; j < y_width_; ++j) {
      double y = calPosition(j, min_y_);
      for (double k = 0; k < ox.size(); ++k) {
        double d = sqrt(pow(x - ox[k], 2) + pow(y - oy[k], 2));
        if (d <= robot_radius_) {
          obstacle_map_[i][j] = true;
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
  return index * resolution_ + minp;
}

//计算移动代价
void Dijkstra::getMotionModel() {
  motion_ = {{1, 0, 1},
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
  return round((position - minp) / resolution_);
}

//为地图范围内的每一个栅格分配计算一个索引，索引 = y * 宽度 + 这一行的x坐标
double Dijkstra::calIndex(Dijkstra::Node *node) {
  return (node->y - min_y_) * x_width_ + (node->x - min_x_);
}

// 判断栅格节点是否合法，边界与障碍物检测
bool Dijkstra::verifyNode(Dijkstra::Node *node) {
  double px = calPosition(node->x, min_x_);
  double py = calPosition(node->y, min_y_);
  //边界检测
  if (px < min_x_ || py < min_y_ || px >= max_x_ || py >= max_y_) {
    return false;
  }
  //障碍物检测
  if (obstacle_map_[node->x][node->y]) {
    return false;
  }
  return true;
}

//根据终点节点和已经搜索的节点，反推计算最终路径
pair<vector<double>, vector<double>> Dijkstra::calFinalPath(
    Dijkstra::Node *goal_node, map<double, Dijkstra::Node *> closed_set) {

  //声明存储搜索到轨迹点的容器，首先将终点加入
  vector<double> rx, ry;
  rx.push_back(calPosition(goal_node->x, min_x_));
  ry.push_back(calPosition(goal_node->y, min_y_));

  double parent_index = goal_node->parent_index;

  //从终点开始，根据父节点索引，反推计算路径
  while (parent_index != -1) {
    Node *node = closed_set[parent_index];
    rx.push_back(calPosition(node->x, min_x_));
    ry.push_back(calPosition(node->y, min_y_));
    parent_index = node->parent_index;
  }
  return make_pair(rx, ry);
}

pair<vector<double>, vector<double>> Dijkstra::planning() {
  double sx = st_[0], sy = st_[1];
  double gx = go_[0], gy = go_[1];
  //计算栅格地图二维坐标，初始化cost值为0（因为距离原点为0），父节点索引-1，表示到头了
  Node *start_node = new Node(calXyindex(sx, min_x_), calXyindex(sy, min_y_), 0.0, -1);
  Node *goal_node = new Node(calXyindex(gx, min_x_), calXyindex(gy, min_y_), 0.0, -1);

  //best——costs存储，索引对应节点，找到的最小值
  map<double, double> best_costs;
  //closed_set存放已经被遍历过的节点
  map<double, Node *> closed_set;

  //定义一个比较函数，用于优先队列的排序，需要是struct类型因为要重载()运算符
  struct cmp {
    bool operator()(Node *left, Node *right) { return left->cost > right->cost; }
  };
  priority_queue<Node *, vector<Node *>, cmp> open_minHeap;

  //将起点加入到open set,并更新best_costs
  open_minHeap.push(start_node);
  best_costs[calIndex(start_node)] = start_node->cost;

  //开始进行搜索
  while (!open_minHeap.empty()) {
    //由于使用小顶堆，所以每次取出的都是cost最小的节点
    Node *current_node = open_minHeap.top();
    open_minHeap.pop();
    double current_node_index = calIndex(current_node);

    //如果该节点已经被遍历过，则跳过
    if (closed_set.find(current_node_index) != closed_set.end()) {
      continue;
    }
    //如果该节点之前已经被遍历过，且cost更小，则跳过
    if (best_costs.find(current_node_index) != best_costs.end()
        && best_costs[current_node_index] < current_node->cost) {
      continue;
    }
    //将该节点加入到closed_set中,表示已经遍历过,并绘制
    closed_set[current_node_index] = current_node;
    plotGraph(current_node);

    // 若找到了目标结点，则退出循环
    if (abs(current_node->x - goal_node->x) < EPS && abs(current_node->y - goal_node->y) < EPS) {
      cout << "Find goal" << endl;
      //需要将终点的父节点索引和cost值更新
      goal_node->parent_index = current_node->parent_index;
      goal_node->cost = current_node->cost;
      break;
    }

    for (auto move : motion_) {
      //遍历cost值最小的当前节点的周围节点
      Node *neighbor = new Node(current_node->x + move[0],
                                current_node->y + move[1],
                                current_node->cost + move[2],
                                current_node_index);
      double neighbor_index = calIndex(neighbor);
      //如果该节点不合法，跳过
      if (!verifyNode(neighbor))
        continue;
      //如果该节点已经被遍历过，跳过
      if(closed_set.find(neighbor_index) != closed_set.end())
        continue;
      //如果该节点的cost更小，则更新best_costs,并加入到open set
      if (best_costs.find(neighbor_index) == best_costs.end() || best_costs[neighbor_index] > neighbor->cost) {
        best_costs[neighbor_index] = neighbor->cost;
        open_minHeap.push(neighbor);
      }
    }
  }
  return calFinalPath(goal_node, closed_set);
}

void Dijkstra::plotGraph(Dijkstra::Node *current) {
  //计算节点的原始坐标，并绘制, 形状为x,颜色为c(青色)
  plt::plot(vector<double>{calPosition(current->x, min_x_)},
            vector<double>{calPosition(current->y, min_y_)}, "xc");
  plt::pause(0.0000001);
}

void Dijkstra::set(const vector<double> &st,
                   const vector<double> &go,
                   const vector<double> &ox,
                   const vector<double> &oy) {
  Dijkstra::st_ = st;
  Dijkstra::go_ = go;
  Dijkstra::ox_ = ox;
  Dijkstra::oy_ = oy;
}