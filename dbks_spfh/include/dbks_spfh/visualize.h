#pragma once

#ifndef VISUALIZE_H
#define VISUALIZE_H

#include <tf/tf.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "node.h"
#include "path.h"
#include "params.h"
#include "utility.h"
#include <math.h>

class Visualization
{
public:
    Visualization();
    void clear();
    void visaulize_start_and_end(const Pose &start, const Pose &end, const double swelling);
    void visualize_expansion_nodes(const std::shared_ptr<Node> &node);
    void visualize_initial_path(Path &path);
    void visualize_smooth_path(Path &path);
    
private:
    ros::NodeHandle nh;
    ros::Publisher pub_start_car;
    ros::Publisher pub_end_car;
    ros::Publisher pub_forward_nodes;
    ros::Publisher pub_reverse_nodes;
    ros::Publisher pub_initial_path;
    ros::Publisher pub_smooth_path;
    visualization_msgs::Marker start_car;
    visualization_msgs::Marker end_car;
    geometry_msgs::PoseArray forward_nodes;
    geometry_msgs::PoseArray reverse_nodes;
    nav_msgs::Path initial_path;
    nav_msgs::Path smooth_path;
};

#endif