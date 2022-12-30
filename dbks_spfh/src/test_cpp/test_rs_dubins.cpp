#include <tf/tf.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "../include/dbks_spfh/dubins.h"
#include "../include/dbks_spfh/reeds_shepp.h"
#include "../include/dbks_spfh/grid_map.h"

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
    ros::Publisher pub_dubins_path;
    ros::Publisher pub_rs_path;
    double m_start[3];
    double m_end[3];
    bool is_valid_start;
    bool is_valid_end;
    GridMap grid_map;
};
PathPlanning::PathPlanning()
{
    sub_start = nh.subscribe("/initialpose", 1, &PathPlanning::callbackOfSubStart, this);
    sub_end = nh.subscribe("/move_base_simple/goal", 1, &PathPlanning::callbackOfSubEnd, this);
    sub_map = nh.subscribe("/map", 1, &PathPlanning::callbackOfSubMap, this);
    pub_dubins_path = nh.advertise<nav_msgs::Path>("/dubins_path", 1, true);
    pub_rs_path = nh.advertise<nav_msgs::Path>("/rs_path", 1, true);
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
int dubinsTransform(double q[3], double x, void *user_data)
{
    std::vector<double> user_data_node;
    user_data_node.push_back(q[0]);
    user_data_node.push_back(q[1]);
    user_data_node.push_back(q[2]);
    std::vector<std::vector<double>> *dubins_path =
        (std::vector<std::vector<double>> *)user_data;
    dubins_path->push_back(user_data_node);
    return 0;
}
void PathPlanning::run()
{
    if (is_valid_start && is_valid_end)
    {
        nav_msgs::Path visual_path;
        visual_path.header.frame_id = "map";
        geometry_msgs::PoseStamped visual_path_node;
        visual_path_node.header.frame_id = "map";
        visual_path.poses.clear();
        ReedsSheppStateSpace r(5.);
        std::vector<std::vector<double>> rs_path = r.xingshensample(m_start, m_end, 0.1);
        for (auto it : rs_path)
        {
            visual_path_node.pose.position.x = it[0];
            visual_path_node.pose.position.y = it[1];
            visual_path_node.pose.orientation = tf::createQuaternionMsgFromYaw(it[3]);
            visual_path.poses.push_back(visual_path_node);
        }
        int last_idx = rs_path.size() - 1;
        LOG(INFO) << "rs path start: " << rs_path[0][0] << "," << rs_path[0][1] << "," << rs_path[0][2];
        LOG(INFO) << "rs path end: " << rs_path[last_idx][0] << "," << rs_path[last_idx][1] << "," << rs_path[last_idx][2];
        pub_rs_path.publish(visual_path);
        visual_path.poses.clear();
        DubinsPath temp;
        double minimal_turning_radius = 5.;
        dubins_shortest_path(&temp, m_start, m_end, minimal_turning_radius);
        std::vector<std::vector<double>> dubins_path;
        double sample_length = 0.1;
        dubins_path_sample_many(&temp, sample_length, dubinsTransform, &dubins_path);
        for (auto it : dubins_path)
        {
            visual_path_node.pose.position.x = it[0];
            visual_path_node.pose.position.y = it[1];
            visual_path_node.pose.orientation = tf::createQuaternionMsgFromYaw(it[3]);
            visual_path.poses.push_back(visual_path_node);
        }
        last_idx = dubins_path.size() - 1;
        LOG(INFO) << "dubins path start: " << dubins_path[0][0] << "," << dubins_path[0][1] << "," << dubins_path[0][2];
        LOG(INFO) << "dubins path end: " << dubins_path[last_idx][0] << "," << dubins_path[last_idx][1] << "," << dubins_path[last_idx][2];
        pub_dubins_path.publish(visual_path);
        is_valid_start = is_valid_end = false;
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_rs_dubins");

    PathPlanning pp;
    while (ros::ok)
    {
        pp.run();
        ros::spinOnce();
    }

    return 0;
}