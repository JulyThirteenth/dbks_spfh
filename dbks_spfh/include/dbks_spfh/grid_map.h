#pragma once

#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <iostream>
#include <glog/logging.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include "car.h"
#include "params.h"
#include "utility.h"

class GridMap
{
public:
    GridMap(bool isSimulation = true);
    inline void transformFromCarToGrid(double x0, double y0,
                                       double &x1, double &y1)
    {
        x1 = x0 / resolution + 0.5 * width;
        y1 = y0 / resolution + 0.5 * height;
    }
    inline void transformFromGridToCar(double x0, double y0,
                                       double &x1, double &y1)
    {
        x1 = (x0 - 0.5 * width) * resolution;
        y1 = (y0 - 0.5 * height) * resolution;
    }
    void transformToGrid(double x0, double y0, double theta0,
                         double &x1, double &y1, double &theta1);
    void transformFromGrid(double x0, double y0, double theta0,
                           double &x1, double &y1, double &theta1);
    void transformToGrid(Pose &p0, Pose &p1)
    {
        transformToGrid(p0.x, p0.y, p0.t, p1.x, p1.y, p1.t);
    }
    void transformFromGrid(Pose &p0, Pose &p1)
    {
        transformFromGrid(p0.x, p0.y, p0.t, p1.x, p1.y, p1.t);
    }
    inline bool isInternal(const double x, const double y, const double theta)
    {
        double row = x / resolution;
        double col = y / resolution;
        return (row >= 0 && row < width) && (col >= 0 && col < height) &&
               (theta / FLAGS_DeltaHeadingRad >= 0 && theta / FLAGS_DeltaHeadingRad < FLAGS_Headings);
    }
    bool isPassable(const double x, const double y, const double theta,
                    const double swelling) const;
    Point getNearestObstaclePoint(const Point &curr,
                                  const double thresholdDistance) const;
    void updateBinMap(const nav_msgs::OccupancyGridConstPtr &map);
    void updateAstar2dLookup(int x, int y);

public:
    // binMap[i][j]==1,表示点(i,j)右上角的栅格为障碍物
    std::vector<std::vector<bool>> binMap;
    std::vector<std::vector<int>> astar2dLookup;
    int width;
    int height;
    double resolution;
    // GridMap有效标志，true:已更新地图， false:未更新地图
    bool isValidMap;
    // GridMap在全局坐标系下的坐标
    Pose origin;
    // 仿真有效标志，true:仿真，false:实车
    bool simulationFlag;
};

#endif