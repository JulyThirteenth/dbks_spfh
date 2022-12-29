#include "grid_map.h"

GridMap::GridMap(bool isSimulation /*= true*/)
{
    simulationFlag = isSimulation;
    isValidMap = false;
}
void GridMap::transformToGrid(double x0, double y0, double theta0,
                              double &x1, double &y1, double &theta1)
{
    if (simulationFlag)
    {
        x1 = x0;
        y1 = y0;
        theta1 = normalizeHeadingRad(theta0); // warning: not normalized
        return;
    }
    else
    {
        x0 -= origin.x;
        y0 -= origin.y;
        double angle = origin.t - M_PI / 2;
        x1 = x0 * cos(angle) + y0 * sin(angle);
        y1 = y0 * cos(angle) - x0 * sin(angle);
        transformFromCarToGrid(x1, y1, x1, y1);
        theta1 = normalizeHeadingRad(theta0 - angle);
    }
}
void GridMap::transformFromGrid(double x0, double y0, double theta0,
                                double &x1, double &y1, double &theta1)
{
    if (simulationFlag)
    {
        x1 = x0;
        y1 = y0;
        theta1 = normalizeHeadingRad(theta0); // warning: not normalized
        return;
    }
    else
    {
        transformFromGridToCar(x0, y0, x0, y0);
        double angle = origin.t - M_PI / 2;
        x1 = x0 * cos(angle) - y0 * sin(angle) + origin.x;
        y1 = y0 * cos(angle) + x0 * sin(angle) + origin.y;
        theta1 = normalizeHeadingRad(theta0 + angle);
    }
}
bool GridMap::isPassable(const double x, const double y, const double theta,
                         const double swelling) const
{
    std::vector<intPoint> carArea;
    Car::getCarOccupyGrids(x, y, theta, carArea, resolution, swelling);
    // myinfo << "carArea Occupygrids: " << carArea.size();
    for (auto grid : carArea)
    {
        if (grid.x < 0 || grid.x >= width || grid.y < 0 || grid.y >= height)
            continue;
        if (binMap[grid.x][grid.y])
        {
            // myinfo << "Collision!";
            return false;
        }
    }
    // myinfo << "No Collision!";
    return true;
}
Point GridMap::getNearestObstaclePoint(const Point &curr,
                                       const double thresholdDistance) const
{
    double x = curr.x;
    double y = curr.y;
    int floorX = floor(x);
    int floorY = floor(y);
    int minSearchX = floor(x - thresholdDistance / resolution) - 1;
    int maxSearchX = floor(x + thresholdDistance / resolution) + 1;
    int minSearchY = floor(y - thresholdDistance / resolution) - 1;
    int maxSearchY = floor(y + thresholdDistance / resolution) + 1;
    double currObsDis = 100000.0;
    double miniObsDis = 100000.0;
    Point obsPoint = {0., 0.};
    Point minObsPoint = {x + currObsDis, y + currObsDis};
    for (int indexX = minSearchX; indexX <= maxSearchX; indexX++)
    {
        for (int indexY = minSearchY; indexY <= maxSearchY; indexY++)
        {
            if ((indexX != floorX || indexY != floorY) &&
                indexX >= 0 && indexX < width &&
                indexY >= 0 && indexY < height &&
                binMap[indexX][indexY])
            {
                if (indexX == floorX)
                {
                    if (indexY > floorY)
                        obsPoint = {x, static_cast<double>(indexY)};
                    else
                        obsPoint = {x, static_cast<double>(indexY + 1)};
                }
                else if (indexY == floorY)
                {
                    if (indexX > floorX)
                        obsPoint = {static_cast<double>(indexX), y};
                    else
                        obsPoint = {static_cast<double>(indexX + 1), y};
                }
                else
                {
                    if (indexX > floorX && indexY > floorY)
                        obsPoint = {static_cast<double>(indexX), static_cast<double>(indexY)};
                    else if (indexX > floorX && indexY < floorY)
                        obsPoint = {static_cast<double>(indexX), static_cast<double>(indexY + 1)};
                    else if (indexX < floorX && indexY < floorY)
                        obsPoint = {static_cast<double>(indexX + 1), static_cast<double>(indexY + 1)};
                    else if (indexX < floorX && indexY > floorY)
                        obsPoint = {static_cast<double>(indexX + 1), static_cast<double>(indexY)};
                }
            }
            currObsDis = sqrt(pow(curr.x - obsPoint.x, 2) + pow(curr.y - obsPoint.y, 2));
            if (currObsDis < miniObsDis)
            {
                minObsPoint = obsPoint;
                miniObsDis = currObsDis;
            }
        }
    }
    return minObsPoint;
}
void GridMap::updateBinMap(const nav_msgs::OccupancyGridConstPtr &map)
{
    if (map == nullptr)
    {
        this->isValidMap = false;
        return;
    }
    width = map->info.width;
    height = map->info.height;
    resolution = map->info.resolution;
    binMap.resize(width);
    for (int i = 0; i < width; i++)
    {
        binMap[i].resize(height);
    }
    for (int x = 0; x < width; x++)
    {
        for (int y = 0; y < height; y++)
        {
            binMap[x][y] = (map->data[y * width + x] != 0);
        }
    }
    astar2dLookup.resize(width);
    for (int i = 0; i < width; i++)
    {
        astar2dLookup[i].resize(height);
    }
    this->isValidMap = true;
}
//###################################################
//每个点到目标点（x,y）的距离（考虑障碍物不考虑车辆约束）
//###################################################
void GridMap::updateAstar2dLookup(int x, int y)
{
    int *cor = new int[2 * (width + height) * 2 * 2];
    short *status = new short[width * height];
    memset(status, 0x00, sizeof(short) * width * height);
    int heapIndex = 0;
    int len[2] = {0, 0};        //内层和外层节点个数
    astar2dLookup[x][y] = 0;    //终点到自身的距离为0
    status[x * height + y] = 1; //终点状态由0变为1，代表已经探索过
    cor[2 * heapIndex + 0] = x;
    cor[2 * heapIndex + 1] = y;
    len[heapIndex]++; // len[0]变为1
    //双层循环，内循环里面的一次循环是将上次拓展的其中一个节点进行拓展
    do
    {
        while (len[heapIndex] > 0)
        {
            int l = len[heapIndex] - 1;
            int i = cor[l * 2 * 2 + heapIndex * 2 + 0];
            int j = cor[l * 2 * 2 + heapIndex * 2 + 1];
            int iup = std::min(i + 1, width - 1);
            int idown = std::max(i - 1, 0);
            int jup = std::min(j + 1, height - 1);
            int jdown = std::max(j - 1, 0);
            if (status[iup * height + j] == 0 && binMap[iup][j] == false)
            {
                astar2dLookup[iup][j] = astar2dLookup[i][j] + 1;
                status[iup * height + j] = 1;
                cor[len[1 - heapIndex] * 2 * 2 + (1 - heapIndex) * 2 + 0] = iup;
                cor[len[1 - heapIndex] * 2 * 2 + (1 - heapIndex) * 2 + 1] = j;
                len[1 - heapIndex]++;
            }
            if (status[idown * height + j] == 0 && binMap[idown][j] == 0)
            {
                astar2dLookup[idown][j] = astar2dLookup[i][j] + 1;
                status[idown * height + j] = 1;
                cor[len[1 - heapIndex] * 2 * 2 + (1 - heapIndex) * 2 + 0] = idown;
                cor[len[1 - heapIndex] * 2 * 2 + (1 - heapIndex) * 2 + 1] = j;
                len[1 - heapIndex]++;
            }
            if (status[i * height + jup] == 0 && binMap[i][jup] == 0)
            {
                astar2dLookup[i][jup] = astar2dLookup[i][j] + 1;
                status[i * height + jup] = 1;
                cor[len[1 - heapIndex] * 2 * 2 + (1 - heapIndex) * 2 + 0] = i;
                cor[len[1 - heapIndex] * 2 * 2 + (1 - heapIndex) * 2 + 1] = jup;
                len[1 - heapIndex]++;
            }
            if (status[i * height + jdown] == 0 && binMap[i][jdown] == 0)
            {
                astar2dLookup[i][jdown] = astar2dLookup[i][j] + 1;
                status[i * height + jdown] = 1;
                cor[len[1 - heapIndex] * 2 * 2 + (1 - heapIndex) * 2 + 0] = i;
                cor[len[1 - heapIndex] * 2 * 2 + (1 - heapIndex) * 2 + 1] = jdown;
                len[1 - heapIndex]++;
            }
            len[heapIndex]--;
        }
        heapIndex = 1 - heapIndex;
    } while (len[heapIndex] > 0);
    delete[] cor;
    delete[] status;
    cor = nullptr;
    status = nullptr;
}