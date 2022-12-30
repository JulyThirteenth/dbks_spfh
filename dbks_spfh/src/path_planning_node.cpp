#include <tf/tf.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "../include/dbks_spfh/reeds_shepp.h"
#include "../include/dbks_spfh/dubins.h"
#include "../include/dbks_spfh/grid_map.h"
#include "../include/dbks_spfh/search_path.h"
#include "../include/dbks_spfh/smooth_path.h"
#include "../include/dbks_spfh/visualize.h"
#include <vector>
#include <iostream>
#include <glog/logging.h>

class PathPlanning
{
public:
    PathPlanning();
    void run();

private:
    void callbackOfSubStart(const geometry_msgs::PoseWithCovarianceStampedConstPtr &start);
    void callbackOfSubEnd(const geometry_msgs::PoseStampedConstPtr &end);
    void callbackOfSubMap(const nav_msgs::OccupancyGridConstPtr &map);

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_start;
    ros::Subscriber sub_end;
    ros::Subscriber sub_map;
    double m_start[3];
    double m_end[3];
    bool is_valid_start;
    bool is_valid_end;
    GridMap grid_map;
    Visualization visualizaiton;
};
PathPlanning::PathPlanning()
{
    sub_start = nh.subscribe("/initialpose", 1, &PathPlanning::callbackOfSubStart, this);
    sub_end = nh.subscribe("/move_base_simple/goal", 1, &PathPlanning::callbackOfSubEnd, this);
    sub_map = nh.subscribe("/map", 1, &PathPlanning::callbackOfSubMap, this);
    is_valid_start = is_valid_end = false;
}
void PathPlanning::callbackOfSubStart(const geometry_msgs::PoseWithCovarianceStampedConstPtr &start)
{
    if (!is_valid_start)
    {
        m_start[0] = start->pose.pose.position.x;
        m_start[1] = start->pose.pose.position.y;
        m_start[2] = tf::getYaw(start->pose.pose.orientation);
        LOG(INFO) << "start:" << m_start[0] << "," << m_start[1] << "," << m_start[2];
        is_valid_start = true;
        return;
    }
    LOG(WARNING) << "There is a start already!";
}
void PathPlanning::callbackOfSubEnd(const geometry_msgs::PoseStampedConstPtr &end)
{
    if (!is_valid_end)
    {
        m_end[0] = end->pose.position.x;
        m_end[1] = end->pose.position.y;
        m_end[2] = tf::getYaw(end->pose.orientation);
        LOG(INFO) << "end:" << m_end[0] << "," << m_end[1] << "," << m_end[2];
        is_valid_end = true;
        return;
    }
    LOG(WARNING) << "There is a end already!";
}
void PathPlanning::callbackOfSubMap(const nav_msgs::OccupancyGridConstPtr &map)
{
    grid_map.updateBinMap(map);
}
void PathPlanning::run()
{
    if (is_valid_start && is_valid_end)
    {
        Pose start = {m_start[0], m_start[1], m_start[2]};
        Pose end = {m_end[0], m_end[1], m_end[2]};
        Path path = searchPath(start, end, grid_map, visualizaiton);
        smoothPath(path, grid_map, visualizaiton);
        is_valid_start = is_valid_end = false;
    }
}

int main(int argc, char **argv)
{
    FLAGS_alsologtostderr = true;
    FLAGS_log_dir = "/home/shaw/Desktop/dev/catkin_ws/src/dbks_spfh/log";
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "path_planning_node");
    PathPlanning pp;
    while (ros::ok)
    {
        pp.run();
        ros::spinOnce();
    }
    return 0;
}