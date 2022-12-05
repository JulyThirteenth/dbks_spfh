#pragma once

#ifndef SEARCH_PATH_H
#define SEARCH_PATH_H

#include <queue>
#include <memory>
#include <iostream>
#include "node.h"
#include "path.h"
#include "params.h"
#include "utility.h"
#include "grid_map.h"
#include "visualize.h"

typedef std::vector<std::shared_ptr<Node>> NodePV;
typedef std::vector<std::vector<std::vector<NodeP>>> NodePC;
typedef std::priority_queue<NodeP, NodePV, NodeCompareByCost> NodePQ;

Path searchPath(Pose &start, Pose &end,
              GridMap &configurationSpace,
              Visualization &visualization);

#endif