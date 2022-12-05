#pragma once

#ifndef SMOOTH_PATH_H
#define SMOOTH_PATH_H

#include "grid_map.h"
#include "path.h"

void smoothPath(const Path &path, const GridMap &gridMap);
void interpolate(const double resolution);

#endif