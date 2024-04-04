//
// Created by ogier on 24-4-4.
//
#include "astar.h"
#include "config.h"
int main() {
  //设置起点和终点,栅格大小,机器人半径,障碍物坐标
  vector<double> start{START_X, START_Y}, goal{GOAL_X, GOAL_Y};
  double robot_radius = ROBOT_RADIUS;
  vector<double> ox, oy;

  //初始化astar类,设置障碍物信息,设置起点,目标点,障碍物的(x,y),在地图上生成障碍物,创建移动代价
  Astar astar(GRID_SIZE, ROBOT_RADIUS);
  //将障碍物信息存在oxoy中，具体设定是在函数循环中每次手动指定
  astar.setObstacle(ox, oy);
  // 设定起点和终点，障碍物信息
  astar.set(start, goal, ox, oy);
  // 在地图上生成障碍物
  astar.calObstacleMap(ox, oy);
  // 获得移动代价
  astar.getMotionModel();

  //绘制地图,.k表示黑色点,ob表示蓝色圆点,or表示红色圆点,true表示显示网格
  // 绘制了障碍物，起点，终点
  plt::plot(ox, oy, ".k");
  plt::plot(vector<double>{start[0]}, vector<double>{start[1]}, "ob");
  plt::plot(vector<double>{goal[0]}, vector<double>{goal[1]}, "or");
  //开启显示网格
  plt::grid(false);

  //规划路径,-r表示红色线，绘制路径,进行规划，返回路径
  pair<vector<double>, vector<double>> xy = astar.planning();
  plt::plot(xy.first, xy.second, "-r");

  //保存图片,dijkstra_demo.png,显示图片
  const char *filename = "./astar_demo.png";
  cout << "Saving result to " << filename << endl;
  plt::save(filename);
  plt::show();

  return 0;
}