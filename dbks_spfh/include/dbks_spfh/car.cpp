#include "car.h"

//#######
// 1，0，2
// 0，0，0
// 0，0，0
// 4，0，3
//#######
void Car::getCarVertices(const double x, const double y, const double theta, // map corrdinates
                    std::vector<Point> &vertices,
                    const double gridMapResolution,
                    const double swelling)
{
    Point vertex;
    double left = (FLAGS_CarLeft + swelling);
    double right = (FLAGS_CarRight + swelling);
    double front = (FLAGS_CarFront + swelling);
    double back = (FLAGS_CarBack + swelling);
    vertex.x = front * sin(M_PI_2 - theta) - left * cos(M_PI_2 - theta) + x;
    vertex.y = front * cos(M_PI_2 - theta) + left * sin(M_PI_2 - theta) + y;
    // myinfo << "1: " << vertex;
    vertices.push_back(vertex);
    vertex.x = front * sin(M_PI_2 - theta) + right * cos(M_PI_2 - theta) + x;
    vertex.y = front * cos(M_PI_2 - theta) - right * sin(M_PI_2 - theta) + y;
    // myinfo << "2: " << vertex;
    vertices.push_back(vertex);
    vertex.x = -back * sin(M_PI_2 - theta) + right * cos(M_PI_2 - theta) + x;
    vertex.y = -back * cos(M_PI_2 - theta) - right * sin(M_PI_2 - theta) + y;
    // myinfo << "3: " << vertex;
    vertices.push_back(vertex);
    vertex.x = -back * sin(M_PI_2 - theta) - left * cos(M_PI_2 - theta) + x;
    vertex.y = -back * cos(M_PI_2 - theta) + left * sin(M_PI_2 - theta) + y;
    // myinfo << "4: " << vertex;
    vertices.push_back(vertex);
}

void Car::getCarOccupyGrids(const double x, const double y, const double theta,
                       std::vector<intPoint> &ret,
                       const double gridMapResolution,
                       const double swelling)
{
    if (!ret.empty())
        ret.clear();
    std::vector<Point> vertices;
    getCarVertices(x, y, theta, vertices, gridMapResolution, swelling);
    fillPolygon(vertices, gridMapResolution, ret);
}