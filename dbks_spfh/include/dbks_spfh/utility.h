#pragma once

#ifndef UTILITY_H
#define UTILITY_H

#include <math.h>
#include <cfloat>
#include <vector>
#include <iostream>
#include <algorithm>
#include <glog/logging.h>

// ###################################################
// functions of logging
// ###################################################
#define myinfo LOG(INFO)
#define mywarn LOG(WARNING)
#define myerror LOG(ERROR)
#define myfatal LOG(FATAL)

// ###################################################
// functions of angle handling
// ###################################################
inline double normalizeHeadingDeg(double t)
{
    if ((int)t <= 0 || (int)t >= 360)
    {
        if (t < -0.1)
            t += 360.0;
        else if ((int)t >= 360)
            t -= 360.0;
        else
            t = 0;
    }
    return t;
}
inline double normalizeHeadingRad(double t)
{
    if (t < 0)
    {
        t = t - 2.0 * M_PI * (int)(t / (2.0 * M_PI));
        return 2.0 * M_PI + t;
    }
    return t - 2.0 * M_PI * (int)(t / (2.0 * M_PI));
}
inline double toDeg(double t)
{
    return normalizeHeadingRad(t) * 180.0 / M_PI;
}

inline double toRad(double t)
{
    return normalizeHeadingRad(t / 180.0 * M_PI);
}

// ###################################################
//  A simple template point in struct
// ###################################################
template <typename T>
struct TemplatePoint
{
    T x, y;
    TemplatePoint(T x_, T y_) : x(x_), y(y_) {}
    TemplatePoint() : x(0), y(0) {}
    bool operator==(const struct TemplatePoint<T> &rhs) const
    {
        return (this->x == rhs.x) && (this->y == rhs.y);
    }
    // a method to multiply a point by a scalar
    inline TemplatePoint<T> operator*(const double k) const { return TemplatePoint<T>(x * k, y * k); }
    // a method to divide a point by a scalar
    inline TemplatePoint<T> operator/(const double k) const { return TemplatePoint<T>(x / k, y / k); }
    // a method to add a point to a point
    inline TemplatePoint<T> operator+(const TemplatePoint<T> &b) const { return TemplatePoint<T>(x + b.x, y + b.y); }
    // a method to subtract a point from a point
    inline TemplatePoint<T> operator-(const TemplatePoint<T> &b) const { return TemplatePoint<T>(x - b.x, y - b.y); }
    // a method to negate a point
    inline TemplatePoint<T> operator-() const { return TemplatePoint<T>(-x, -y); }
    /// a method to calculate the length of the point vector
    double length() const { return std::sqrt(std::pow(x, 2) + std::pow(y, 2)); }
    /// a method to calculate the length of the point vector
    double sqlength() const { return x * x + y * y; }
    /// a method to calculate the dot product of two point vectors
    double dot(TemplatePoint<T> b) const { return x * b.x + y * b.y; }
    ///a method that returns the orthogonal complement of two point vectors
    inline TemplatePoint<T> ort(TemplatePoint<T> b) const
    {
      TemplatePoint<T> a(this->x, this->y);
      TemplatePoint<T> c;
      // multiply b by the dot product of this and b then divide it by b's length
      c = a - b * a.dot(b) / b.sqlength();
      return c;
    }
};
typedef struct TemplatePoint<double> Point;
typedef struct TemplatePoint<int> intPoint;
template <typename T>
static std::ostream &operator<<(std::ostream &os, struct TemplatePoint<T> &p)
{
    os << "(" << p.x << ", " << p.y << ")";
    return os;
}
template <typename T>
inline TemplatePoint<T> operator*(double k, const TemplatePoint<T> &b)
{
    return (b * k);
}
// ###################################################
//  A simple template point in struct
// ###################################################
template <typename T1, typename T2>
struct Templatepoint
{
    T1 x, y;
    T2 t;
    Templatepoint(T1 x_, T1 y_, T2 t_) : x(x_), y(y_), t(t_) {}
    Templatepoint() : x(0), y(0), t(0) {}
    struct Templatepoint<T1, T2> &operator=(const struct Templatepoint<T1, T2> &vec)
    {
        if (this != &vec)
        {
            this->x = vec.x;
            this->y = vec.y;
            this->t = vec.t;
        }
        return *this;
    }
};
typedef struct Templatepoint<double, double> Pose;
typedef struct Templatepoint<int, int> Index;
template <typename T1, typename T2>
static std::ostream &operator<<(std::ostream &os, struct Templatepoint<T1, T2> &v)
{
    os << v.x << ", " << v.y << ", " << v.t;
    return os;
}

// ###################################################
//  fill polygon on grids
// ###################################################
typedef struct edge
{
    int ymax;
    double x;
    double dx;
    struct edge *next;
} Edge;
typedef Edge **EdgeTable;
typedef Edge *ActiveEdgeTable;
void fillPolygon(const std::vector<Point> &vertices, double resolution, std::vector<intPoint> &polygArea);

// ###################################################
//  line traversal on grids
// ###################################################
void gridTraversal(const Point &start, const Point &goal, const double resolution, std::vector<intPoint> &visitedGrid);

#endif