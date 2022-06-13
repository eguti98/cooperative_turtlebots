/*!
 * \waypoint_gen.h
 * \brief waypoint_gen (...).
 *
 * Waypoint generation (...).
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * \author Matteo De Petrillo, WVU - madepetrillo@mix.wvu.edu
 * \date June 01, 2020
 */

#ifndef WAYPOINT_GEN_H
#define WAYPOINT_GEN_H

// Include cpp important headers
#include <math.h>
#include <stdio.h> 
#include <chrono>
#include <thread>
#include <termios.h>
#include <vector>


// ROS headers
#include <ros/ros.h>
#include <tf/tf.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// Custom message includes. Auto-generated from msg/ directory.
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>


class WaypointGeneration
{
public:
    WaypointGeneration(ros::NodeHandle & nh);

    void generateWaypoint();
    void setupPathStart();

    bool startedOdom = false;

private:
    // Node Handle
    ros::NodeHandle & nh_;

    // Subscriber
    ros::Subscriber subOdom;
    ros::Subscriber subOdomTruth;

    // Publisher
    ros::Publisher pubGoalPose;

    geometry_msgs::Pose localPos_curr_;
    geometry_msgs::Twist localVel_curr_;
    geometry_msgs::Pose goalPos_;
    
    void odometryTruthCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

    std::vector<double> x_, y_;
    int counter_ = 0;

};


#endif // WAYPOINT_GEN_H