/*!
 * \waypoint_gen.cpp
 * \brief waypoint_gen (...).
 *
 * Waypoint generation (...).
 * 
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * 
 * \date June 20, 2020
 */

#include "waypoint_gen/waypoint_gen.h"

WaypointGeneration::WaypointGeneration(ros::NodeHandle & nh)
    : nh_(nh)
{
    // Subscriber
    subOdom = nh_.subscribe("dead_reckoning/odometry", 1000, &WaypointGeneration::odometryCallback, this);
    subOdomTruth = nh_.subscribe("odometry/truth", 1000, &WaypointGeneration::odometryTruthCallback, this);

    // Publisher
    pubGoalPose = nh_.advertise<geometry_msgs::Pose>("navigation/goal_pos", 1000);

    if (!nh_.hasParam("/waypoints/")) 
    {
        ROS_INFO("No waypoints found.");
    }
    else
    {
        ROS_INFO("Waypoints loaded.");
    }

    nh_.getParam("/waypoints/x", x_);
    nh_.getParam("/waypoints/y", y_);

    ROS_INFO_STREAM("x size: " << x_.size());
    ROS_INFO_STREAM("y size: " << y_.size());
}

void WaypointGeneration::odometryTruthCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    localPos_curr_ = msg->pose.pose;
    localVel_curr_ = msg->twist.twist;
    ROS_INFO_STREAM("(x,y)=("<<localPos_curr_.position.x <<","<<localPos_curr_.position.y<<")");
    startedOdom = true;
}

void WaypointGeneration::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    localPos_curr_ = msg->pose.pose;
    localVel_curr_ = msg->twist.twist;
    // ROS_INFO_STREAM("odometry"<<localPos_curr_);
}


void WaypointGeneration::setupPathStart()
{
    double min_distance = 200;

    double ex, ey, distance;

    for (int i = 0; i < x_.size(); i++)
    {
        // ROS_INFO_STREAM("x: " << localPos_curr_.position.x);
        // ROS_INFO_STREAM("y: " << localPos_curr_.position.y);
        ex = x_[i] - localPos_curr_.position.x;
        ey = y_[i] - localPos_curr_.position.y;

        distance = std::hypot(ex, ey);
        // ROS_INFO_STREAM("Distance: " << distance);
        if (distance < min_distance)
        {
            min_distance = distance;
            counter_ = i;
            ROS_INFO_STREAM("Distance: " << min_distance);
        }
    }

    ROS_INFO_STREAM("Starting from index: " << counter_);

    goalPos_.position.x = x_[counter_];
    goalPos_.position.y = y_[counter_];
    goalPos_.position.z = 0;
    goalPos_.orientation.w = 0.0;
    goalPos_.orientation.x = 0.0;
    goalPos_.orientation.y = 0.0;
    goalPos_.orientation.z = 0.0;

    // ROS_INFO_STREAM("New goal pose" << goalPos_);
    pubGoalPose.publish(goalPos_);
}

void WaypointGeneration::generateWaypoint()
{
    double ex, ey, distance;

    ex = x_[counter_] - localPos_curr_.position.x;
    ey = y_[counter_] - localPos_curr_.position.y;

    distance = std::hypot(ex, ey);
    
    if (distance < 1.0) 
    {
        (++counter_ > x_.size())? counter_ = 0 : counter_;

        goalPos_.position.x = x_[counter_];
        goalPos_.position.y = y_[counter_];

        ROS_INFO_STREAM("New goal pose" << goalPos_);
        pubGoalPose.publish(goalPos_);
    }
}

/*!
 * \brief Creates and runs the waypoint_gen node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_gen");
    ros::NodeHandle nh("");
    
    ros::Rate rate(10);
    
    ROS_INFO("Waypoint Gen Node initializing...");
    WaypointGeneration waypoint_gen(nh);

    while(!waypoint_gen.startedOdom) 
    {
        ros::spinOnce();
        rate.sleep();
    }

    waypoint_gen.setupPathStart();

    while(ros::ok()) 
    {
        waypoint_gen.generateWaypoint();
        ros::spinOnce();
        rate.sleep();
    }
                
    return 0;
}