//
// Created by ogier on 24-4-4.
//

#include "astar.h"

Astar::Node::Node(double x, double y, float cost, double parent_index)
    : x(x), y(y), cost(cost), parent_index(parent_index) {}

Astar::Astar(const double resolution, const double robotRadius)
    : resolution_(resolution), robot_radius_(robotRadius) {
  heuristic_ = HEURISTIC;
  heuristic_weight_ = HEURISTIC_WEIGHT;
}

// 计算障碍物地图,首先计算地图的边界范围,然后计算地图的长宽,然后初始化地图,遍历地图的每一个点
// ,判断与障碍物距离和机器人半径的关系,如果小于机器人半径则认为是障碍物
void Astar::calObstacleMap(const vector<double> &ox, const vector<double> &oy) {
  min_x_ = round(*min_element(ox.begin(), ox.end()));
  min_y_ = round(*min_element(oy.begin(), oy.end()));
  max_x_ = round(*max_element(ox.begin(), ox.end()));
  max_y_ = round(*max_element(oy.begin(), oy.end()));

  cout << "min_x:" << min_x_ << "   min_y:" << min_y_ << "  max_x:" << max_x_ << "  max_y:" << max_y_ << endl;

  x_width_ = (max_x_ - min_x_) / resolution_;
  y_width_ = (max_y_ - min_y_) / resolution_;
  cout << "x_width:" << x_width_ << "   y_width:" << y_width_ << endl;

  obstacle_map_ = vector<vector<bool >>(x_width_, vector<bool>(y_width_, false));

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
double Astar::calPosition(double index, double minp) {
  return index * resolution_ + minp;
}

//计算移动代价
void Astar::getMotionModel() {
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
double Astar::calXyindex(double position, double minp) {
  return round((position - minp) / resolution_);
}

//为地图范围内的每一个栅格分配计算一个索引，索引 = y * 宽度 + 这一行的x坐标
double Astar::calIndex(Astar::Node *node) {
  return (node->y - min_y_) * x_width_ + (node->x - min_x_);
}

//计算启发式函数的值，根据私有变量heuristic_的值，选择曼哈顿距离或者欧几里得距离
double Astar::calHeuristic(Astar::Node *node) {
  double gx = calXyindex(go_[0], min_x_);
  double gy = calXyindex(go_[1], min_y_);

  if (heuristic_ == 1)
    return abs(node->x - gx) + abs(node->y - gy) * heuristic_weight_;
  else if (heuristic_ == 2) {
    return sqrt(pow(node->x - gx, 2) + pow(node->y - gy, 2)) * heuristic_weight_;
  } else //指定启发式函数不合法，直接报错
    throw invalid_argument("heuristic_ is not valid");
}

// 判断栅格节点是否合法，边界与障碍物检测
bool Astar::verifyNode(Astar::Node *node) {
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
pair<vector<double>, vector<double>> Astar::calFinalPath(
    Astar::Node *goal_node, map<double, Astar::Node *> closed_set) {

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

pair<vector<double>, vector<double>> Astar::planning() {
  double sx = st_[0], sy = st_[1];
  double gx = go_[0], gy = go_[1];
  //计算栅格地图二维坐标，初始化cost值为0（因为距离原点为0），父节点索引-1，表示到头了
  Node *start_node = new Node(calXyindex(sx, min_x_), calXyindex(sy, min_y_), 0.0, -1);
  Node *goal_node = new Node(calXyindex(gx, min_x_), calXyindex(gy, min_y_), 0.0, -1);

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
      double cost_current = iter->second->cost + calHeuristic(iter->second);
      if (cost_current < cost) {
        cost = cost_current;
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

    for (vector<double> move : motion_) {
      //通过模型，探究最小cost节点处周围的点；注意这里将parent_index设置为当前节点的索引
      //别在这加启发式函数的值，否则每次遍历邻居节点,如果恰好是同一个节点,会导致启发式函数值不断增加
      Node *neighbor = new Node(current->x + move[0],
                                current->y + move[1],
                                current->cost + move[2],
                                cur_id);
      //计算节点的索引，判断在哪个set或者是否合法
      double n_id = calIndex(neighbor);
      //如果在closed_set中，跳过
      if (closed_set.find(n_id) != closed_set.end())
        continue;
      //如果超出边界或者碰到障碍物了,跳过
      if (!verifyNode(neighbor))
        continue;
      //如果open set中没有这个节点,加入open set
      if (open_set.find(n_id) == open_set.end()) {
        open_set[n_id] = neighbor;
      } else {//如果有这个节点，比较cost值，更新,保留cost更小的
        if (open_set[n_id]->cost >= neighbor->cost)
          open_set[n_id] = neighbor;
      }
    }
  }
  return calFinalPath(goal_node, closed_set);
}

void Astar::plotGraph(Astar::Node *current) {
  //计算节点的原始坐标，并绘制, 形状为x,颜色为c(青色)
  plt::plot(vector<double>{calPosition(current->x, min_x_)},
            vector<double>{calPosition(current->y, min_y_)}, "xc");
  plt::pause(PAUSE_TIME);
}

void Astar::set(const vector<double> &st,
                const vector<double> &go,
                const vector<double> &ox,
                const vector<double> &oy) {
  Astar::st_ = st;
  Astar::go_ = go;
  Astar::ox_ = ox;
  Astar::oy_ = oy;
}

void Astar::setObstacle1(vector<double> &ox, vector<double> &oy) {
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

void Astar::setObstacle2(vector<double> &ox, vector<double> &oy) {
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
  for (double i = 0; i < 20; i++) {
    ox.push_back(30);
    oy.push_back(30 - i);
  }
}

void Astar::setObstacle3(vector<double> &ox, vector<double> &oy) {
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
  for (double i = 0; i < 20; i++) {
    ox.push_back(30);
    oy.push_back(30 - i);
  }
  for (double i = 0; i < 20; i++) {
    ox.push_back(30 + i);
    oy.push_back(30);
  }
}