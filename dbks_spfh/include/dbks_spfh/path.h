#pragma once

#ifndef PATH_H
#define PATH_H

#include "node.h"
#include "utility.h"
#include "reeds_shepp.h"
#include "dubins.h"
#include <vector>

class Path
{
public:
    Path(){};
    void trace_path(const NodeP &pred, const NodeP &next, bool shooting = true);
    void total_path();
public:
    std::vector<Node> forward_path;
    std::vector<Node> shooting_path;
    std::vector<Node> reverse_path;
    std::vector<Node> initial_path;
    std::vector<Node> smooth_path;
};

#endif