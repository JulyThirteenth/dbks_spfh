#include "node.h"

void Node::kinematicExpansion(const Direction &dir, std::shared_ptr<Node> &node)
{
    if (node == nullptr)
        myfatal << "Node pointer of expansion can`t be nullptr";
    node->reset();
    node->dir = dir;
    double v = FLAGS_ExpansionVelocity;
    double fi = toRad(FLAGS_ExpansionDeviationAngle);
    switch (dir)
    {
    case LeftForward:
        v *= 1;
        fi *= 1;
        break;
    case Forward:
        v *= 1;
        fi *= 0;
        break;
    case RightForWard:
        v *= 1;
        fi *= -1;
        break;
    case RightBack:
        v *= -1; // a
        fi *= -1;
        break;
    case Back:
        v *= -1;
        fi *= 0;
        break;
    case LeftBack:
        v *= -1;
        fi *= 1;
        break;
    }
    if (fi == 0)
    {
        node->t = this->t;
        node->x = this->x + v * cos(this->t);
        node->y = this->y + v * sin(this->t);
    }
    else
    {
        double temp = v * tan(fi) / FLAGS_CarWheelbase;
        node->t = this->t + temp;
        node->x = this->x + v / temp * (sin(node->t) - sin(this->t));
        node->y = this->y + v / temp * (cos(this->t) - cos(node->t));
    }
}
void Node::updateGCost(bool isForwardExpansion)
{
    if (pred == nullptr)
        return;
    double penaltyTurning = 0.;   //转方向盘代价
    double penaltyReversing = 1.; //逆向行驶代价
    double penaltyCOD = 0.;       //改变行驶方向代价
    if (dir != pred->dir)
        penaltyTurning = FLAGS_PenaltyTurning;
    if (dir > 3 && isForwardExpansion == true)
        penaltyReversing = FLAGS_PenaltyReversing;
    if (dir < 3 && isForwardExpansion == false)
        penaltyReversing = FLAGS_PenaltyReversing;
    if ((dir < 3 && pred->dir > 3) || (dir > 3 && pred->dir < 3))
        penaltyCOD = FLAGS_PenaltyCOD;
    g = pred->g + (penaltyTurning + penaltyReversing + penaltyCOD) * FLAGS_ExpansionVelocity * 1.;
}
void Node::updateHCost(bool isForwardExpansion,
                       const std::shared_ptr<Node> &goal,
                       const GridMap &searchSpace)
{
    double dubinsCost = 0.;
    double reedsSheppCost = 0.;
    double astar2dLookupCost = 0.;
    // double astar2dLookupOffset = 0.;
    double start[3] = {x, y, t};
    double end[3] = {goal->x, goal->y, goal->t};
    if (FLAGS_CarReverse)
    {
        ReedsSheppStateSpace r(FLAGS_CarMinTurningRadius);
        reedsSheppCost = r.distance(start, end);
        // myinfo << "reedsSheppCost: " << reedsSheppCost;
    }
    else
    {
        DubinsPath temp;
        dubins_shortest_path(&temp, start, end, FLAGS_CarMinTurningRadius);
        dubinsCost = dubins_path_length(&temp);
        // myinfo << "dubinsCost: " << dubinsCost;
    }
    if (isForwardExpansion)
    {
        astar2dLookupCost = static_cast<double>(searchSpace.astar2dLookup[int(x)][int(y)]) *
                            searchSpace.resolution;
    }
    // myinfo << "astar2dLookupCost: " << astar2dLookupCost;
    h = (dubinsCost + reedsSheppCost + astar2dLookupCost) * 0.5;
    // myinfo << "hCost: " << h;
}
void Node::updateICost(const std::shared_ptr<Node> &goal, const GridMap &searchSpace)
{
    Point curr = {goal->x, goal->y};
    Point nearObstPoint = searchSpace.getNearestObstaclePoint(curr, FLAGS_ThresholdDistance);
    double cosineSimilarity = (x * goal->x + y * goal->y + t * goal->t) /
                              (sqrt(pow(x, 2) + pow(y, 2) + pow(t, 2)) *
                               sqrt(pow(goal->x, 2) + pow(goal->y, 2) + pow(goal->t, 2)));
    double obstacleDistance = sqrt(pow(nearObstPoint.x - x, 2) + pow(nearObstPoint.y - y, 2));
    obstacleDistance = obstacleDistance > FLAGS_ThresholdDistance ? FLAGS_ThresholdDistance : obstacleDistance;
    double obsFreeDegree = obstacleDistance / FLAGS_ThresholdDistance;
    // myinfo << "cosineSimilarity: " << cosineSimilarity;
    // myinfo << "obsFreeDegree: " << obsFreeDegree;
    i = (cosineSimilarity - obsFreeDegree + 1) * FLAGS_ExpansionVelocity * 1.;
    // myinfo << "iCost: " << i;
}