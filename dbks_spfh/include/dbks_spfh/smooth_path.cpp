#include "smooth_path.h"

double smoothnessTarget(const Point &predP, const Point &currP, const Point &nextP)
{
    Point temp = (predP - currP) - (currP - nextP);
    return temp.sqlength();
}
Point smoothnessTerm(const Point &predP2, const Point &predP1, const Point &currP,
                     const Point &nextP1, const Point &nextP2)
{
    Point res = (predP2 - 4 * predP1 + 6 * currP - 4 * nextP1 + nextP2);
    double len = res.length();
    // 模规范化为1
    res = (len != 0) ? res / len : res;
    return res;
}
double obstacleTarget(const Point &currP, const Point &obsP)
{
    double obsMaxDist = FLAGS_ThresholdDistance;
    Point delta = currP - obsP;
    double obsMinDist = delta.length();
    double res = (obsMinDist < obsMaxDist) ? pow(obsMaxDist - obsMinDist, 2) : 0;
    return res;
}
Point obstacleTerm(const Point &currP, const Point &obsP)
{
    Point res;
    double obsMaxDist = FLAGS_ThresholdDistance;
    Point delta = currP - obsP;
    double obsMinDist = delta.length();
    // 距离过小时才计算梯度，否则梯度为零
    if (obsMinDist < obsMaxDist)
    {
        // 防止除以0
        if (obsMinDist < 1e-5f)
        {
            obsMinDist = 1e-5f;
        }
        res = (obsMinDist - obsMaxDist) * delta / obsMinDist;
    }
    double len = res.length();
    // 模规范化为1
    res = (len != 0) ? res / len : res;
    return res;
}
void smoothPath(Path &path, GridMap &gridMap, Visualization &visualization)
{
    double maxIterations = FLAGS_MaxSmoothIterations;
    double ratioOfSmoothness = FLAGS_RatioOfSmoothness;
    double ratioOfObstacle = FLAGS_RatioOfObstacle;
    double stepSize = FLAGS_SmoothStepSize;
    bool smoothnessTermToggle = fabs(ratioOfSmoothness) > 1e-5;
    bool obstacleTermToggle = fabs(ratioOfObstacle) > 1e-5;
    // 当前的总代价、平滑代价、障碍物代价
    double totalCost = 0, smoothnessCost = 0, obstacleCost = 0;
    // 前一次迭代的总代价、平滑代价、障碍物代价
    double preTotalCost = totalCost, preSmoothnessCost = smoothnessCost, preObstacleCost = obstacleCost;
    // 路径节点总数
    const int pathLength = path.initial_path.size();
    // 记录需要锚定的点的下标，true表示优化时锚定
    std::vector<bool> anchorIndex(pathLength, false);
    // 记录迭代次数
    int iterations = 0;
    // 记录优化次数
    int smoothTimes = 0;
    // 优化退出标志
    bool notQuit = true;
    // Path unsmoothedPath = path;
    std::vector<Node> &smoothedPath = path.smooth_path;
    if (smoothnessTermToggle == false && obstacleTermToggle == false)
    {
        notQuit = false;
    }
    while (notQuit)
    {
        smoothedPath = path.initial_path;
        smoothTimes++;
        iterations = 0;
        totalCost = 0, smoothnessCost = 0, obstacleCost = 0;
        // 优化的起始索引点，从第二个点开始优化,最后三个点不优化（实际是两个，因为最后添加了一个目标点）
        int beginIndex = 2, endIndex = pathLength - 3;
        // 轨迹优化
        while (iterations < maxIterations)
        {
            preTotalCost = totalCost;
            preSmoothnessCost = smoothnessCost;
            preObstacleCost = obstacleCost;
            totalCost = 0;
            smoothnessCost = 0;
            obstacleCost = 0;
            for (int i = beginIndex; i < endIndex; ++i)
            {
                // 如果该点是锚点，则不参与优化
                if (anchorIndex[i])
                {
                    continue;
                }
                // 确保优化同向轨迹点
                if (smoothedPath[i - 2].isReverse() != smoothedPath[i - 1].isReverse() ||
                    smoothedPath[i - 1].isReverse() != smoothedPath[i].isReverse() ||
                    smoothedPath[i].isReverse() != smoothedPath[i + 1].isReverse() ||
                    smoothedPath[i + 1].isReverse() != smoothedPath[i + 2].isReverse())
                {
                    continue;
                }
                if (i - 3 < 0 || smoothedPath[i - 3].isReverse() != smoothedPath[i].isReverse() ||
                    i + 3 >= pathLength || smoothedPath[i + 3].isReverse() != smoothedPath[i].isReverse())
                {
                    continue;
                }
                Point predP2(smoothedPath[i - 2].x, smoothedPath[i - 2].y);
                Point predP1(smoothedPath[i - 1].x, smoothedPath[i - 1].y);
                Point currP(smoothedPath[i].x, smoothedPath[i].y);
                Point nextP1(smoothedPath[i + 1].x, smoothedPath[i + 1].y);
                Point nextP2(smoothedPath[i + 2].x, smoothedPath[i + 2].y);
                Point predCarP2;
                Point predCarP1;
                Point currCarP;
                Point nextCarP1;
                Point nextCarP2;
                gridMap.transformFromGridToCar(predP1.x, predP1.y, predCarP1.x, predCarP1.y);
                gridMap.transformFromGridToCar(predP2.x, predP2.y, predCarP2.x, predCarP2.y);
                gridMap.transformFromGridToCar(currP.x, currP.y, currCarP.x, currCarP.y);
                gridMap.transformFromGridToCar(nextP1.x, nextP1.y, nextCarP1.x, nextCarP1.y);
                gridMap.transformFromGridToCar(nextP2.x, nextP2.y, nextCarP2.x, nextCarP2.y);
                // 梯度方向
                Point gradP;
                if (smoothnessTermToggle)
                {
                    smoothnessCost += ratioOfSmoothness * smoothnessTarget(predCarP1, currCarP, nextCarP1);
                    gradP = gradP - stepSize * smoothnessTerm(predCarP2, predCarP1, currCarP, nextCarP1, nextCarP2);
                }
                if (obstacleTermToggle)
                {
                    Point obsP = gridMap.getNearestObstaclePoint(currP, FLAGS_ThresholdDistance);
                    Point obsCarP;
                    gridMap.transformFromGridToCar(obsP.x, obsP.y, obsCarP.x, obsCarP.y);
                    obstacleCost += ratioOfObstacle * obstacleTarget(currCarP, obsCarP);
                    gradP = gradP - stepSize * obstacleTerm(currCarP, obsCarP);
                }
                totalCost = smoothnessCost + obstacleCost;
                // 更新path
                currCarP = currCarP + gradP;
                gridMap.transformFromCarToGrid(currCarP.x, currCarP.y, currP.x, currP.y);
                smoothedPath[i].x = currP.x;
                smoothedPath[i].y = currP.y;
            }
            iterations++;
            // 如果本次值大于上次值，则步长过大，应该减少步长
            if (totalCost - preTotalCost > 0)
            {
                stepSize = stepSize * 0.98;
                // myinfo << "stepSize: " << stepSize;
            }
            // 提前终止
            if (fabs(totalCost - preTotalCost) < 1e-6)
            {
                break;
            }
        }
        // 计算角度
        for (int i = beginIndex; i < endIndex; ++i)
        {
            if (anchorIndex[i])
            {
                continue;
            }
            // 三点同向
            if (smoothedPath[i - 1].isReverse() == smoothedPath[i].isReverse() &&
                smoothedPath[i + 1].isReverse() == smoothedPath[i].isReverse())
            {
                // 如果是前向行驶
                if (false == smoothedPath[i].isReverse())
                {
                    smoothedPath[i].t = atan2(smoothedPath[i + 1].y - smoothedPath[i - 1].y,
                                                        smoothedPath[i + 1].x - smoothedPath[i - 1].x);
                }
                // 如果是逆向行驶
                else
                {
                    smoothedPath[i].t = atan2(smoothedPath[i - 1].y - smoothedPath[i + 1].y,
                                                        smoothedPath[i - 1].x - smoothedPath[i + 1].x);
                }
            }
            else
            {
                if (smoothedPath[i].isReverse() == smoothedPath[i - 1].isReverse())
                {
                    // 前向行驶
                    if (false == smoothedPath[i].isReverse())
                    {
                        smoothedPath[i].t = atan2(smoothedPath[i].y - smoothedPath[i - 1].y,
                                                            smoothedPath[i].x - smoothedPath[i - 1].x);
                    }
                    // 逆向行驶
                    else
                    {
                        smoothedPath[i].t = atan2(smoothedPath[i - 1].y - smoothedPath[i].y,
                                                            smoothedPath[i - 1].x - smoothedPath[i].x);
                    }
                }
                else if (smoothedPath[i].isReverse() == smoothedPath[i + 1].isReverse())
                {
                    // 前向行驶
                    if (false == smoothedPath[i].isReverse())
                    {
                        smoothedPath[i].t = atan2(smoothedPath[i + 1].y - smoothedPath[i].y,
                                                            smoothedPath[i + 1].x - smoothedPath[i].x);
                    }
                    // 逆向行驶
                    else
                    {
                        smoothedPath[i].t = atan2(smoothedPath[i].y - smoothedPath[i + 1].y,
                                                            smoothedPath[i].x - smoothedPath[i + 1].x);
                    }
                }
            }
        }
        // 默认可以退出
        notQuit = false;
        // 碰撞检查
        double swelling = FLAGS_MinSwelling;
        double currX, currY, currT;
        for (int i = 0; i < pathLength; ++i)
        {
            //! 原始路径点均为后轴
            // 如果优化后该点有障碍物（锚点也有可能有障碍物，因为角度改变）
            currX = smoothedPath[i].x;
            currY = smoothedPath[i].y;
            currT = smoothedPath[i].t;
            if (false == anchorIndex[i] &&
                false == gridMap.isPassable(currX, currY, currT, swelling))
            {
                anchorIndex[i] = true;
                myinfo << "第 " << i << " 个点不通过, 迭代 ： " << iterations << ", 优化 ： " << smoothTimes;
                // 优化的轨迹有障碍物，则不可以退出
                notQuit = true;
                break;
            }
        }
    }
    if(FLAGS_Visualization)
    {
        visualization.visualize_smooth_path(path);
    }
    myinfo << "一共优化了" << smoothTimes << "次";
    myinfo << "最后优化第" << iterations << "次迭代终止";
    myinfo << "上次总代价:" << preTotalCost << "，本次总代价:" << totalCost;
    myinfo << "上次平滑代价:" << preSmoothnessCost << "，本次平滑代价:" << smoothnessCost;
    myinfo << "上次障碍物代价:" << preObstacleCost << "，本次障碍物代价:" << obstacleCost;
    myinfo << "stepSize:" << stepSize << std::endl;
}
void interpolate(const double resolution)
{
}