#include "path.h"

void Path::trace_path(const NodeP &pred, const NodeP &next, bool shooting /* = true*/)
{
    NodeP p = pred;
    NodeP n = next;
    Node node;
    forward_path.clear();
    shooting_path.clear();
    reverse_path.clear();
    while (p != nullptr)
    {
        node = *p;
        forward_path.push_back(node);
        p = p->pred;
    }
    std::reverse(forward_path.begin(), forward_path.end());
    if (shooting)
    {
        double s[3] = {pred->x, pred->y, pred->t};
        double e[3] = {next->x, next->y, next->t};
        if (FLAGS_CarReverse)
        {
            ReedsSheppStateSpace r(FLAGS_CarMinTurningRadius);
            double sample_length = FLAGS_ExpansionVelocity * 1.0;
            std::vector<std::vector<double>> path;
            path = r.xingshensample(s, e, sample_length);
            for (int i = 0; i < path.size(); i++)
            {
                node.x = path[i][0];
                node.y = path[i][1];
                node.t = path[i][2];
                shooting_path.push_back(node);
            }
        }
        else
        {
            DubinsPath dubinsPath;
            dubins_shortest_path(&dubinsPath, s, e, FLAGS_CarMinTurningRadius);
            std::vector<std::vector<double>> path;
            double sample_length = FLAGS_ExpansionVelocity * 1.0;
            double length = dubins_path_length(&dubinsPath);
            double now[3];
            for (int l = sample_length; l < length; l += sample_length)
            {
                dubins_path_sample(&dubinsPath, l, now);
                node.x = now[0];
                node.y = now[1];
                node.t = now[2];
                shooting_path.push_back(node);
            }
        }
    }
    while (n != nullptr)
    {
        node = *n;
        reverse_path.push_back(node);
        n = n->pred;
    }
    myinfo << "Forward Path Size: " << forward_path.size() << ", "
           << "Shooting Path Size: " << shooting_path.size() << ", "
           << "Reverse Path Size: " << reverse_path.size();
}
void Path::total_path()
{
    if (!full_path.empty())
        full_path.clear();
    for (auto path_node : forward_path)
    {
        full_path.push_back(path_node);
    }
    for (auto path_node : shooting_path)
    {
        full_path.push_back(path_node);
    }
    for (auto path_node : reverse_path)
    {
        full_path.push_back(path_node);
    }
    myinfo << "Total Path Size: " << full_path.size();
}