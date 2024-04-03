//
// Created by ogier on 24-4-3.
//
#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <iostream>
//#include <Eigen/Dense>
#include <vector>
#include <map>
#include <cmath>
#include <stdlib.h>
#include <time.h>
#include "../matplotlibcpp.h"

#define EPS 1e-4
#define PI 3.14159265354

namespace plt = matplotlibcpp;
using namespace std;
//using namespace Eigen;

class Dijkstra {
 public:
  //栅格地图中点信息，与没有经过resolution处理过的地图区分
  struct Node {
    Node(double x, double y, float cost, double parent_index);
    double x;
    double y;
    float cost;
    double parent_index;
  };
  //默认构造函数
  Dijkstra(const double resolution, const double robotRadius);
  void setObstacle(vector<double> &ox,vector<double> &oy);
  void calObstacleMap(const vector<double> &ox, const vector<double> &oy);

  double calPosition(double index, double minp);
  void getMotionModel();
  double calXyindex(double position, double minp);
  double calIndex(Node *node);
  bool verifyNode(Node *node);

  pair<vector<double>, vector<double>> calFinalPath(Node *goal_node, map<double, Node *> closed_set);

  pair<vector<double>, vector<double>> planning();

  void plotGraph(Node *current);

  void set(const vector<double> &st, const vector<double> &go, const vector<double> &ox, const vector<double> &oy);

 private:
  double resolution_;
  double robot_radius_;
  //地图xy的范围
  double min_x_, min_y_, max_x_, max_y_;
  //xy方向上栅格的个数
  double x_width_, y_width_;
  vector<vector<bool>> obstacle_map_;
  vector<vector<double>> motion_;
  vector<double> st_, go_;
  vector<double> ox_, oy_;
};

#endif //DIJKSTRA_H
