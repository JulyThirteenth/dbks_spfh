#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include "../include/dbks_spfh/params.h"
#include "../include/dbks_spfh/grid_map.h"

ros::Publisher pub_polygon;
GridMap grid_map;
bool is_valid_map = false;
void handle_with_map(const nav_msgs::OccupancyGridConstPtr &map)
{
    grid_map.updateBinMap(map);
    is_valid_map = true;
}
void handle_with_pose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose)
{
    if (is_valid_map)
    {
        std::vector<Point> vertices;
        Car::getCarVertices(pose->pose.pose.position.x, pose->pose.pose.position.y,
                            tf::getYaw(pose->pose.pose.orientation),
                            vertices, grid_map.resolution, FLAGS_MaxSwelling);
        geometry_msgs::PolygonStamped car_polygon;
        car_polygon.header.frame_id = "map";
        car_polygon.header.stamp = ros::Time::now();
        geometry_msgs::Point32 vertex;
        for (auto it = vertices.begin(); it != vertices.end(); it++)
        {
            vertex.x = it->x;
            vertex.y = it->y;
            car_polygon.polygon.points.push_back(vertex);
        }
        pub_polygon.publish(car_polygon);
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_grid_map");
    ros::NodeHandle nh;
    pub_polygon = nh.advertise<geometry_msgs::PolygonStamped>("/car", 1, true);
    ros::Subscriber sub_map = nh.subscribe("/map", 1, handle_with_map);
    ros::Subscriber sub_pose = nh.subscribe("/initialpose", 1, handle_with_pose);
    ros::spin();
    return 0;
}