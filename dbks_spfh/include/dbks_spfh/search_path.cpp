#include "search_path.h"

void initNodeCube(NodePC &cube, double width, double height)
{
    myinfo << "initNodeCube......";
    int depth = FLAGS_Headings;
    cube.resize(width);
    for (int i = 0; i < width; i++)
    {
        cube[i].resize(height);
        for (int j = 0; j < height; j++)
        {
            cube[i][j].resize(depth);
            for (int k = 0; k < depth; k++)
            {
                cube[i][j][k] = std::make_shared<Node>();
            }
        }
    }
    myinfo << "Finish initNodeCube!";
}
void resetNodeCube(NodePC &cube, double width, double height)
{
    myinfo << "resetNodeCube......";
    int depth = FLAGS_Headings;
    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {
            for (int k = 0; k < depth; k++)
            {
                cube[i][j][k]->reset();
            }
        }
    }
    myinfo << "Finish resetNodeCube!";
}
void clearQue(NodePQ &queue)
{
    while (!queue.empty())
    {
        queue.pop();
    }
}
bool dubinsShooting(NodeP &start, NodeP &end,
                    GridMap &configurationSpace,
                    double swelling)
{
    double s[3] = {start->x, start->y, start->t};
    double e[3] = {end->x, end->y, end->t};
    DubinsPath dubinsPath;
    dubins_shortest_path(&dubinsPath, s, e, FLAGS_CarMinTurningRadius);
    std::vector<std::vector<double>> path;
    double sample_length = FLAGS_ExpansionVelocity * 1.0;
    double length = dubins_path_length(&dubinsPath);
    double now[3];
    for (int l = sample_length; l < length; l += sample_length)
    {
        dubins_path_sample(&dubinsPath, l, now);
        if (!configurationSpace.isPassable(now[0], now[1], now[2], swelling))
        {
            return false;
        }
    }
    myinfo << "dubins shoot successfully!";
    return true;
}
bool reedsSheppShooting(NodeP &start, NodeP &end,
                        GridMap &configurationSpace,
                        double swelling)
{
    double s[3] = {start->x, start->y, start->t};
    double e[3] = {end->x, end->y, end->t};
    ReedsSheppStateSpace r(FLAGS_CarMinTurningRadius);
    double sample_length = FLAGS_ExpansionVelocity * 1.0;
    std::vector<std::vector<double>> path;
    path = r.xingshensample(s, e, sample_length);
    for (int i = 0; i < path.size(); i++)
    {
        if (!configurationSpace.isPassable(path[i][0], path[i][1], path[i][2], swelling))
        {
            return false;
        }
    }
    myinfo << "reedsShepp shoot successfully!";
    return true;
}
bool shooting(NodeP &start, NodeP &end,
              GridMap &configurationSpace,
              double swelling)
{
    int random = rand() % 10 + 1;
    double dx = std::abs(start->x - end->x) / random;
    double dy = std::abs(start->y - end->y) / random;
    double distance = sqrt(pow(dx, 2) + pow(dy, 2));
    if (distance > FLAGS_MaxShootingDistance)
        return false;
    if (FLAGS_CarReverse)
        return reedsSheppShooting(start, end, configurationSpace, swelling);
    else
        return dubinsShooting(start, end, configurationSpace, swelling);
}
//###################################################
// DBSK algorithm
// start & end are in world corrdinate
//###################################################
Path searchPath(Pose &start, Pose &end,
                GridMap &configurationSpace,
                Visualization &visualization)
{
    Path res;
    Index startId, endId;
    double width = configurationSpace.width;
    double height = configurationSpace.height;
    double resolution = configurationSpace.resolution;
    {
        myinfo << "In World, start: "
               << start.x << "," << start.y << "," << start.t;
        configurationSpace.transformToGrid(start, start);
        myinfo << "In Grid, start: "
               << start.x << "," << start.y << "," << start.t;
        startId = {(int)(start.x / resolution),
                   (int)(start.y / resolution),
                   (int)(start.t / FLAGS_DeltaHeadingRad)};
        myinfo << "In Grid, startId: "
               << startId.x << "," << startId.y << "," << startId.t;
        myinfo << "In World, end: "
               << end.x << "," << end.y << "," << end.t;
        configurationSpace.transformToGrid(end, end);
        myinfo << "In Grid, end: "
               << end.x << "," << end.y << "," << end.t;
        endId = {(int)(end.x / resolution),
                 (int)(end.y / resolution),
                 (int)(end.t / FLAGS_DeltaHeadingRad)};
        myinfo << "In Grid, endId: "
               << endId.x << "," << endId.y << "," << endId.t;
    }
    double swelling = FLAGS_MaxSwelling;       //膨胀参数
    int maxIterations = FLAGS_MaxIterations;   //最大迭代次数
    int directions = FLAGS_CarReverse ? 7 : 3; //扩展方向，只向前走只有向前3个方向, 可以倒车则有7个方向(包括Nop)
    NodePQ startQue, endQue;                   //优先队列
    NodePC cube;                               //内部节点地图
    initNodeCube(cube, width, height);         //初始化内部节点地图
    while (swelling >= FLAGS_MinSwelling)
    {
        myinfo << "current swelling: " << swelling;
        myinfo << "current maxIterations: " << maxIterations;
        if (FLAGS_Visualization)
        {
            visualization.clear();
            visualization.visaulize_start_and_end(start, end, swelling);
        }
        NodeP startNode = cube[startId.x][startId.y][startId.t];
        {
            startNode->x = start.x;
            startNode->y = start.y;
            startNode->t = start.t;
        }
        NodeP endNode = cube[endId.x][endId.y][endId.t];
        {
            endNode->x = end.x;
            endNode->y = end.y;
            endNode->t = end.t;
        }
        SearchType searchWay = (SearchType)FLAGS_SearchWay; //搜索方式
        if (searchWay == BothWay || searchWay == ForwardWay)
        {
            configurationSpace.updateAstar2dLookup((int)end.x, (int)end.y);
            startNode->updateGCost(true);
            startNode->updateHCost(true, endNode, configurationSpace);
            startNode->updateICost(endNode, configurationSpace);
            startQue.push(startNode);
            startNode->state = ForwardOpen;
        }
        if (searchWay == BothWay || searchWay == ReverseWay)
        {
            endNode->updateGCost(false);
            endNode->updateHCost(false, startNode, configurationSpace);
            endNode->updateICost(startNode, configurationSpace);
            endQue.push(endNode);
            endNode->state = ReverseOpen;
        }
        int iterations = 0;       //记录迭代次数
        NodeP currS = startNode;  //当前开始节点
        NodeP currE = endNode;    //当前终止节点
        while ((searchWay == BothWay && !startQue.empty() && !endQue.empty()) ||
               (searchWay == ForwardWay && !startQue.empty()) ||
               (searchWay == ReverseWay && !endQue.empty()))
        {
            if ((++iterations) > maxIterations)
                break;
            // myinfo << "currS-> " << *currS;
            // myinfo << "currE-> " << *currE;
            if (searchWay == BothWay || searchWay == ForwardWay)
            {
                myinfo << "--------Forward finding of DBKS--------";
                NodeP curr = startQue.top();
                startQue.pop();
                curr->state = ForwardClose;
                myinfo << "curr from start -> " << *curr;
                if (iterations % FLAGS_ShiftIterations == 0)
                {
                    currS = curr;
                }
                if (FLAGS_Visualization)
                    visualization.visualize_expansion_nodes(curr);
                if (shooting(curr, currE, configurationSpace, swelling))
                {
                    res.trace_path(curr, currE);
                    if (FLAGS_Visualization)
                        visualization.visualize_initial_path(res);
                    return res;
                }
                NodeP currC = std::make_shared<Node>();
                for (int dir = 0; dir < directions; dir++)
                {
                    if ((Direction)dir == Nop)
                    {
                        // myinfo << "Nop";
                        continue;
                    }
                    curr->kinematicExpansion((Direction)dir, currC);
                    if (!configurationSpace.isInternal(currC->x, currC->y, currC->t))
                    {
                        // myinfo << "Not in map";
                        continue;
                    }
                    if (!configurationSpace.isPassable(currC->x, currC->y, currC->t, swelling))
                    {
                        // myinfo << "Not passable";
                        continue;
                    }
                    if (*currC == *currE)
                    {
                        myinfo << "connect the curr with currE successfully!";
                        res.trace_path(curr, currE, false);
                        if (FLAGS_Visualization)
                            visualization.visualize_initial_path(res);
                        return res;
                    }
                    Index poseId = {(int)(currC->x / resolution),
                                    (int)(currC->y / resolution),
                                    (int)(currC->t / FLAGS_DeltaHeadingRad)};
                    // myinfo << "poseId: " << poseId;
                    NodeP temp = cube[poseId.x][poseId.y][poseId.t];
                    switch (temp->state)
                    {
                    case UnDiscover:
                    {
                        *temp = *currC; //位置和方向
                        temp->pred = curr;
                        temp->updateGCost(true);
                        temp->updateHCost(true, currE, configurationSpace);
                        temp->updateICost(currE, configurationSpace);
                        temp->state = ForwardOpen;
                        startQue.push(temp);
                        break;
                    }
                    case ForwardOpen:
                    {
                        currC->pred = curr;
                        currC->updateGCost(true);
                        currC->updateHCost(true, currE, configurationSpace);
                        currC->updateICost(currE, configurationSpace);
                        if (currC->getCost() < temp->getCost())
                        {
                            *temp = *currC; //位置、方向、pred和Cost
                            startQue.push(temp);
                        }
                        break;
                    }
                    case ForwardClose:
                        break;
                    case ReverseOpen:
                        break;
                    case ReverseClose:
                        break;
                    }
                    myinfo << "child of curr from start -> " << *temp;
                }
            }
            if (searchWay == BothWay || searchWay == ReverseWay)
            {
                myinfo << "--------Reverse finding of DBKS--------";
                NodeP curr = endQue.top();
                endQue.pop();
                curr->state = ReverseClose;
                myinfo << "curr from end -> " << *curr;
                if (iterations % FLAGS_ShiftIterations == 0)
                {
                    currE = curr;
                }
                if (FLAGS_Visualization)
                    visualization.visualize_expansion_nodes(curr);
                if (shooting(currS, curr, configurationSpace, swelling))
                {
                    res.trace_path(currS, curr);
                    if (FLAGS_Visualization)
                        visualization.visualize_initial_path(res);
                    return res;
                }
                NodeP currC = std::make_shared<Node>();
                for (int dir = 0; dir < directions; dir++)
                {
                    if ((Direction)dir == Nop)
                    {
                        // myinfo << "Nop";
                        continue;
                    }
                    curr->kinematicExpansion((Direction)dir, currC);
                    if (!configurationSpace.isInternal(currC->x, currC->y, currC->t))
                    {
                        // myinfo << "Not in map";
                        continue;
                    }
                    if (!configurationSpace.isPassable(currC->x, currC->y, currC->t, swelling))
                    {
                        // myinfo << "Not passable";
                        continue;
                    }
                    if (*currC == *currS)
                    {
                        myinfo << "connect the curr with currS successfully!";
                        res.trace_path(currS, curr, false);
                        if (FLAGS_Visualization)
                            visualization.visualize_initial_path(res);
                        return res;
                    }
                    Index poseId = {(int)(currC->x / resolution),
                                    (int)(currC->y / resolution),
                                    (int)(currC->t / FLAGS_DeltaHeadingRad)};
                    // myinfo << "poseId: " << poseId;
                    NodeP temp = cube[poseId.x][poseId.y][poseId.t];
                    switch (temp->state)
                    {
                    case UnDiscover:
                    {
                        *temp = *currC; //位置和方向
                        temp->pred = curr;
                        temp->updateGCost(false);
                        temp->updateHCost(false, currS, configurationSpace);
                        temp->updateICost(currS, configurationSpace);
                        temp->state = ReverseOpen;
                        endQue.push(temp);
                        break;
                    }
                    case ForwardOpen:
                        break;
                    case ForwardClose:
                        break;
                    case ReverseOpen:
                    {
                        currC->pred = curr;
                        currC->updateGCost(false);
                        currC->updateHCost(false, currS, configurationSpace);
                        currC->updateICost(currS, configurationSpace);
                        if (currC->getCost() < temp->getCost())
                        {
                            *temp = *currC; //位置、方向、pred和Cost
                            endQue.push(temp);
                        }
                        break;
                    }
                    case ReverseClose:
                        break;
                    }
                    myinfo << "child of curr from end -> " << *temp;
                }
            }
        }
        swelling -= 0.05;
        maxIterations += 2000;
        clearQue(startQue);
        clearQue(endQue);
        resetNodeCube(cube, width, height);
    }
    myinfo << "Cannot find the path of no collision!";
    return res;
}