#pragma once

#ifndef SMOOTH_PATH_H
#define SMOOTH_PATH_H

#include "grid_map.h"
#include "path.h"
#include "visualize.h"

void smoothPath(Path &path, GridMap &gridMap, Visualization &visualization);
void interpolate(const double resolution);

#endif