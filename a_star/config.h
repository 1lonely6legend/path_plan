//
// Created by ogier on 24-4-4.
//

#ifndef PATH_PLAN_DIJKSTRA_CONFIG_H_
#define PATH_PLAN_DIJKSTRA_CONFIG_H_

//定义起点终点
#define START_X -5
#define START_Y -5
#define GOAL_X 50
#define GOAL_Y 50

//定义栅格的大小,即地图的分辨率
#define GRID_SIZE 2.0

//定义机器人的半径
#define ROBOT_RADIUS 1.0

//定义使用的启发式函数，1代表曼哈顿距离，2代表欧几里德距离
//在这个版本astar中，由于可以八联通，所以使用曼哈顿距离不能保证最短路径
#define HEURISTIC 2

//定义启发式函数的倍率
#define HEURISTIC_WEIGHT 10

//定义动画每次拓展节点的时间间隔
#define PAUSE_TIME 0.01

//定义使用的地图
#define MAP "map3"

#endif //PATH_PLAN_DIJKSTRA_CONFIG_H_
