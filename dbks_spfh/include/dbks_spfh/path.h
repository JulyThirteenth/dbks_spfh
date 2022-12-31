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
    // low quality codes, waiting for improvment.
    void print_forward_path();
    void print_shooting_path();
    void print_reverse_path();
    void print_initial_path();
    void print_smooth_path();
public:
    // 存储前向搜索路径
    std::vector<Node> forward_path;
    // 存储dubins or rs shooting路径
    std::vector<Node> shooting_path;
    // 存储反向搜索路径
    std::vector<Node> reverse_path;
    // 存储初始化路径
    std::vector<Node> initial_path;
    // 存储优化后路径
    std::vector<Node> smooth_path;
};

#endif