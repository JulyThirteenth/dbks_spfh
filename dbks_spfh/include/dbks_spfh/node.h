#pragma once

#ifndef NODE_H
#define NODE_H

#include <memory>
#include <math.h>
#include <algorithm>
#include "params.h"
#include "utility.h"
#include "grid_map.h"
#include "reeds_shepp.h"
#include "dubins.h"

enum State
{
    UnDiscover,
    ForwardOpen,
    ReverseOpen,
    ForwardClose,
    ReverseClose
};
enum Direction
{
    LeftForward,
    Forward,
    RightForWard,
    Nop,
    RightBack,
    Back,
    LeftBack
};
class Node
{
public:
    Node(double x = 0, double y = 0, double t = 0, double g = 0, double h = 0, double i = 0,
         Direction dir = Nop, State state = UnDiscover, const std::shared_ptr<Node> pred = nullptr)
    {
        this->x = x;
        this->y = y;
        this->t = t;
        this->g = g;
        this->h = h;
        this->i = i;
        this->dir = dir;
        this->pred = pred;
        this->state = state;
    }
    inline double getCost()
    {
        return g + FLAGS_RatioOfH * h + FLAGS_RatioOfI * i;
    };
    inline bool operator==(const Node &rhs) const
    {
        double distance = 0.5 * FLAGS_ExpansionVelocity;
        double deltaRad = 3.0 * FLAGS_DeltaHeadingRad;
        double deltaNegRad = 2.0 * M_PI - deltaRad;
        return (sqrt(pow(this->x - rhs.x, 2) + pow(this->y - rhs.y, 2)) < distance &&
                ((std::abs(this->t - rhs.t) <= deltaRad) || (std::abs(this->t - rhs.t) >= deltaNegRad)));
    }
    inline Node &operator=(const Node &rhs)
    {
        if (this != &rhs)
        {
            this->x = rhs.x;
            this->y = rhs.y;
            this->t = rhs.t;
            this->g = rhs.g;
            this->h = rhs.h;
            this->i = rhs.i;
            this->dir = rhs.dir;
            this->pred = rhs.pred;
            this->state = rhs.state;
        }
        return *this;
    }
    inline void reset()
    {
        this->x = this->y = this->t = 0.;
        this->g = this->h = this->i = 0.;
        this->dir = Nop;
        this->pred = nullptr;
        this->state = UnDiscover;
    }
    void kinematicExpansion(const Direction &dir, std::shared_ptr<Node> &node);
    void updateGCost(bool isForwardExpansion);
    void updateHCost(bool isForwardExpansion,
                     const std::shared_ptr<Node> &goal,
                     const GridMap &searchSpace);
    void updateICost(const std::shared_ptr<Node> &goal, const GridMap &searchSpace);

public:
    double x;
    double y;
    double t;
    double g;
    double h;
    double i;
    State state;
    Direction dir;
    std::shared_ptr<Node> pred;
};
typedef std::shared_ptr<Node> NodeP;
struct NodeCompareByCost
{
    bool operator()(const std::shared_ptr<Node> &lhs, const std::shared_ptr<Node> &rhs)
    {
        return lhs->getCost() > rhs->getCost();
    }
};
static std::ostream &operator<<(std::ostream &os, const Node n)
{
    std::string state;
    switch (n.state)
    {
    case UnDiscover:
        state = "UnDiscover";
        break;
    case ForwardOpen:
        state = "ForwardOpen";
        break;
    case ForwardClose:
        state = "ForwardClose";
        break;
    case ReverseOpen:
        state = "ReverseOpen";
        break;
    case ReverseClose:
        state = "ReverseClose";
        break;
    }
    os << "state: " << state << " "
       << "corrd: " << n.x << ", " << n.y << ", " << n.t << " "
       << "cost: " << n.g << ", " << n.h << ", " << n.i;
    return os;
}
#endif
