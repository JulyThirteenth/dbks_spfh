#pragma once

#ifndef CAR_H
#define CAR_H

#include <vector>
#include "params.h"
#include "utility.h"

class Car
{
public:
    //#######
    // 1，0，2
    // 0，0，0
    // 0，0，0
    // 4，0，3
    //#######
    static void getCarVertices(const double x, const double y, const double theta, // car corrdinates
                               std::vector<Point> &vertices,
                               const double gridMapResolution,
                               const double swelling);
    static void getCarOccupyGrids(const double x, const double y, const double theta,
                                  std::vector<intPoint> &ret,
                                  const double gridMapResolution,
                                  const double swelling);
};

#endif