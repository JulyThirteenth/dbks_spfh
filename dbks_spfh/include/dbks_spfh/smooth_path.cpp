#include "smooth_path.h"

void smoothPath(const Path &path, const GridMap &gridMap)
{
    double maxIterations = FLAGS_MaxSmoothIterations;
    double ratioOfSmooth = FLAGS_RatioOfSmoothness;
    double ratioOfObs = FLAGS_RatioOfObstacle;
    double stepSize = FLAGS_SmoothStepSize;
    bool smoothTermToggle = fabs(ratioOfSmooth) > 1e-5;
    bool obsTermToggle = fabs(ratioOfObs) > 1e-5;
    //当前的总代价、平滑代价、障碍物代价
    double totalCost = 0, smoothnessCost = 0, obsCost = 0;
    //前一次迭代的总代价、平滑代价、障碍物代价、图代价
    double preTotalCost = totalCost, preSmoothnessCost = smoothnessCost, preObsCost = obsCost;
    // 路径节点总数
    const int pathLength = path.full_path.size();
    //记录需要锚定的点的下标，true表示优化时锚定
    std::vector<bool> anchorIndex(pathLength, false);
    //记录迭代次数
    int iterations = 0;
    //优化退出标志
	bool notQuit = true;
}