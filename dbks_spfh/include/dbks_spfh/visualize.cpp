#include "visualize.h"

Visualization::Visualization()
{
    {
        start_car.header.frame_id = "/map";
        start_car.ns = "startCar";
        start_car.lifetime = ros::Duration(0.);
        start_car.frame_locked = true;
        start_car.type = visualization_msgs::Marker::LINE_STRIP;
        start_car.action = visualization_msgs::Marker::ADD;
        start_car.color.r = 1.0;
        start_car.color.g = 0.0;
        start_car.color.b = 0.0;
        start_car.color.a = 0.5;
        start_car.pose.orientation.w = 1.0;
        end_car.header.frame_id = "/map";
        end_car.ns = "endCar";
        end_car.lifetime = ros::Duration(0.);
        end_car.frame_locked = true;
        end_car.type = visualization_msgs::Marker::LINE_STRIP;
        end_car.action = visualization_msgs::Marker::ADD;
        end_car.color.r = 0.0;
        end_car.color.g = 1.0;
        end_car.color.b = 0.0;
        end_car.color.a = 0.5;
        end_car.pose.orientation.w = 1.0;
    }
    forward_nodes.header.frame_id = "/map";
    reverse_nodes.header.frame_id = "/map";
    initial_path.header.frame_id = "/map";
    smooth_path.header.frame_id = "/map";
    pub_start_car = nh.advertise<visualization_msgs::Marker>("/start_car", 1, true);
    pub_end_car = nh.advertise<visualization_msgs::Marker>("/end_car", 1, true);
    pub_forward_nodes = nh.advertise<geometry_msgs::PoseArray>("/farward_nodes", 1, true);
    pub_reverse_nodes = nh.advertise<geometry_msgs::PoseArray>("/reverse_nodes", 1, true);
    pub_initial_path = nh.advertise<nav_msgs::Path>("/initial_path", 1, true);
    pub_smooth_path = nh.advertise<nav_msgs::Path>("/smooth_path", 1, true);
}
void Visualization::clear()
{
    start_car.points.clear();
    end_car.points.clear();
    pub_start_car.publish(start_car);
    pub_end_car.publish(end_car);
    forward_nodes.poses.clear();
    reverse_nodes.poses.clear();
    pub_forward_nodes.publish(forward_nodes);
    pub_reverse_nodes.publish(reverse_nodes);
    initial_path.poses.clear();
    pub_initial_path.publish(initial_path);
    smooth_path.poses.clear();
    pub_smooth_path.publish(smooth_path);
}
void Visualization::visaulize_start_and_end(const Pose &start, const Pose &end, const double swelling)
{
    double left = (FLAGS_CarLeft + swelling);
    double right = (FLAGS_CarRight + swelling);
    double front = (FLAGS_CarFront + swelling);
    double back = (FLAGS_CarBack + swelling);
    start_car.header.stamp = ros::Time::now();
    geometry_msgs::Point start_point;
    start_point.x = start.x + front * sin(M_PI_2 - start.t);
    start_point.y = start.y + front * cos(M_PI_2 - start.t);
    start_car.points.push_back(start_point);
    start_point.x = start.x - back * sin(M_PI_2 - start.t);
    start_point.y = start.y - back * cos(M_PI_2 - start.t);
    start_car.points.push_back(start_point);
    start_car.scale.x = left + right;
    pub_start_car.publish(start_car);
    end_car.header.stamp = ros::Time::now();
    geometry_msgs::Point end_point;
    end_point.x = end.x + front * sin(M_PI_2 - end.t);
    end_point.y = end.y + front * cos(M_PI_2 - end.t);
    end_car.points.push_back(end_point);
    end_point.x = end.x - back * sin(M_PI_2 - end.t);
    end_point.y = end.y - back * cos(M_PI_2 - end.t);
    end_car.points.push_back(end_point);
    end_car.scale.x = left + right;
    pub_end_car.publish(end_car);
}
void Visualization::visualize_expansion_nodes(const std::shared_ptr<Node> &node)
{
    geometry_msgs::Pose pose;
    pose.position.x = node->x;
    pose.position.y = node->y;
    // FORWARD
    if (node->dir < 3)
    {
        forward_nodes.header.stamp = ros::Time::now();
        pose.orientation = tf::createQuaternionMsgFromYaw(node->t);
        forward_nodes.poses.push_back(pose);
        forward_nodes.header.stamp = ros::Time::now();
        // PUBLISH THE POSEARRAY
        pub_forward_nodes.publish(forward_nodes);
    }
    // REVERSE
    else
    {
        reverse_nodes.header.stamp = ros::Time::now();
        pose.orientation = tf::createQuaternionMsgFromYaw(node->t + M_PI);
        reverse_nodes.poses.push_back(pose);
        reverse_nodes.header.stamp = ros::Time::now();
        // PUBLISH THE POSEARRAY
        pub_reverse_nodes.publish(reverse_nodes);
    }
}
void Visualization::visualize_initial_path(Path &path)
{
    geometry_msgs::PoseStamped pose_stamped;
    initial_path.header.stamp = ros::Time::now();
    path.total_path();
    for(auto path_node: path.full_path)
    {
        pose_stamped.pose.position.x = path_node.x;
        pose_stamped.pose.position.y = path_node.y;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(path_node.t);
        initial_path.poses.push_back(pose_stamped); 
    }
    pub_initial_path.publish(initial_path);
}
void Visualization::visualize_smooth_path(Path &path)
{
    geometry_msgs::PoseStamped pose_stamped;
    smooth_path.header.stamp = ros::Time::now();
    path.total_path();
    for(auto path_node: path.full_path)
    {
        pose_stamped.pose.position.x = path_node.x;
        pose_stamped.pose.position.y = path_node.y;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(path_node.t);
        smooth_path.poses.push_back(pose_stamped); 
    }
    pub_smooth_path.publish(smooth_path);
}