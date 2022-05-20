#include <ros/ros.h>
#include <math.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/NavSatStatus.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#define PI 3.14159265359

// Declare sensor data variables
double posx= 0.0;
double posy= 0.0;
double posz= 0.0;
double linvelx=0.0;
double linvely=0.0;
double linvelz=0.0;
double angvelx=0.0;
double angvely=0.0;
double angvelz=0.0;
double orientx=0.0;
double orienty=0.0;
double orientz=0.0;
double orientw=1.0;

double posxgps =0.0;
// double posdrx= 0.0;
// double posdry= 0.0;
// double posdrz= 0.0;
// double linveldrx=0.0;
// double linveldry=0.0;
// double linveldrz=0.0;
// double angveldrx=0.0;
// double angveldry=0.0;
// double angveldrz=0.0;
// double orientdrx=0.0;
// double orientdry=0.0;
// double orientdrz=0.0;
// double orientdrw=0.0;

std::string odometry_drone_name;

// Declare sensor callback functions
// void odomdrCallback(const visualization_msgs::Marker::ConstPtr& msg);
void odomtruthCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
void gpsiris0Callback(const nav_msgs::Odometry::ConstPtr& msg);
void gpsiris1Callback(const nav_msgs::Odometry::ConstPtr& msg);
void gpsiris2Callback(const nav_msgs::Odometry::ConstPtr& msg);

int main(int argc, char** argv)
{
  // Declare variables that are read from params in yaml
  float loop_rate; // Hz
  // std::string odometry_dr_topic_name;
  std::string odometry_truth_in_topic_name;
  std::string odometry_truth_out_topic_name;
  // std::string odometry_err_topic_name;
  std::string odometry_frame_id;
  std::string odometry_child_frame_id;
  // Initialize ROS
  std::string node_name = "truth_node";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;

  // Read params from yaml file
    if(ros::param::get(node_name+"/loop_rate",loop_rate)==false)
    {
      ROS_FATAL("No parameter 'loop_rate' specified");
      ros::shutdown();
      exit(1);
    }
    if(ros::param::get(node_name+"/odometry_drone_name",odometry_drone_name)==false)
    {
      ROS_FATAL("No parameter 'odometry_drone_name' specified");
      ros::shutdown();
      exit(1);
    }
    // if(ros::param::get(node_name+"/odometry_dr_topic_name",odometry_dr_topic_name)==false)
    // {
    //   ROS_FATAL("No parameter 'odometry_dr_topic_name' specified");
    //   ros::shutdown();
    //   exit(1);
    // }
    if(ros::param::get(node_name+"/odometry_truth_in_topic_name",odometry_truth_in_topic_name)==false)
    {
      ROS_FATAL("No parameter 'odometry_truth_in_topic_name' specified");
      ros::shutdown();
      exit(1);
    }
    if(ros::param::get(node_name+"/odometry_truth_out_topic_name",odometry_truth_out_topic_name)==false)
    {
      ROS_FATAL("No parameter 'odometry_truth_out_topic_name' specified");
      ros::shutdown();
      exit(1);
    }
    // if(ros::param::get(node_name+"/odometry_err_topic_name",odometry_err_topic_name)==false)
    // {
    //   ROS_FATAL("No parameter 'odometry_err_topic_name' specified");
    //   ros::shutdown();
    //   exit(1);
    // }
    if(ros::param::get(node_name+"/odometry_frame_id",odometry_frame_id)==false)
    {
      ROS_FATAL("No parameter 'odometry_frame_id' specified");
      ros::shutdown();
      exit(1);
    }
    if(ros::param::get(node_name+"/odometry_child_frame_id",odometry_child_frame_id)==false)
    {
      ROS_FATAL("No parameter 'odometry_child_frame_id' specified");
      ros::shutdown();
      exit(1);
    }

  // Initialize publishers and subscribers
  // ros::Subscriber odom_dr_sub = nh.subscribe(odometry_dr_topic_name, 1, odomdrCallback);
  ros::Subscriber odom_truth_sub =nh.subscribe(odometry_truth_in_topic_name, 1, odomtruthCallback);
  ros::Subscriber gps_iris_0 =nh.subscribe("/uav0/mavros/global_position/local", 1, gpsiris0Callback);
  ros::Subscriber gps_iris_1 =nh.subscribe("/uav1/mavros/global_position/local", 1, gpsiris1Callback);
  ros::Subscriber gps_iris_2 =nh.subscribe("/uav2/mavros/global_position/local", 1, gpsiris2Callback);

  // ros::Publisher odom_err_pub = nh.advertise<nav_msgs::Odometry>(odometry_err_topic_name, 1);
  ros::Publisher odom_truth_pub = nh.advertise<nav_msgs::Odometry>(odometry_truth_out_topic_name, 1);
  ros::Publisher gps_pub = nh.advertise<nav_msgs::Odometry>("gps_data", 1);
  // Initialize states
  // nav_msgs::Odometry odom_err_msg; // Initializes to all zero, by default
  nav_msgs::Odometry odom_truth_msg; // Initializes to all zero, by default
  nav_msgs::Odometry gps_msg; // Initializes to all zero, by default
  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformListener tf_listener_;

  // Loop and compute states by integrating yaw rate to get heading and averaging encoders to get distance
  ros::Rate rate(loop_rate);
  uint32_t seq = 0;
  while(ros::ok())
  {
    //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = odometry_frame_id;
    odom_trans.child_frame_id = odometry_child_frame_id;
    odom_trans.transform.translation.x = posx;
    odom_trans.transform.translation.y = posy;
    odom_trans.transform.translation.z = posz;
    odom_trans.transform.rotation.x =orientx;
    odom_trans.transform.rotation.y =orienty;
    odom_trans.transform.rotation.z =orientz;
    odom_trans.transform.rotation.w =orientw;
    odom_broadcaster.sendTransform(odom_trans);
// Pose
    // Add delta distances to odom message
    odom_truth_msg.pose.pose.position.x = posx;
    odom_truth_msg.pose.pose.position.y = posy;
    odom_truth_msg.pose.pose.position.z = posz;
    // Compute odom message quaternion from yaw angle
    odom_truth_msg.pose.pose.orientation.x = orientx;
    odom_truth_msg.pose.pose.orientation.y = orienty;
    odom_truth_msg.pose.pose.orientation.z = orientz;
    odom_truth_msg.pose.pose.orientation.w = orientw;

    // Velocity (twist)
    odom_truth_msg.twist.twist.linear.x = linvelx;
    odom_truth_msg.twist.twist.linear.y = linvely;
    odom_truth_msg.twist.twist.linear.z = linvelz;

    odom_truth_msg.twist.twist.angular.x = angvelx;
    odom_truth_msg.twist.twist.angular.y = angvely;
    odom_truth_msg.twist.twist.angular.z = angvelz;

    // Header and frame info
    odom_truth_msg.header.seq = seq;
    odom_truth_msg.header.stamp = ros::Time::now();
    odom_truth_msg.header.frame_id = odometry_frame_id;
    odom_truth_msg.child_frame_id = odometry_child_frame_id;
    odom_truth_pub.publish(odom_truth_msg);
    gps_msg.pose.pose.position.x = posxgps;
    gps_pub.publish(gps_msg);
///////////////////////////////////////////////////////////////////////
    // odom_err_msg.pose.pose.position.x = posx-posdrx;
    // odom_err_msg.pose.pose.position.y = posy-posdry;
    // odom_err_msg.pose.pose.position.z = posz-posdrz;
    // Compute odom message quaternion from yaw angle
    // odom_err_msg.pose.pose.orientation.x =orientx-orientdrx;
    // odom_err_msg.pose.pose.orientation.y =orienty-orientdry;
    // odom_err_msg.poseose.pose.orientation.z =orientz-orientdrz;
    // odom_err_msg.pose.pose.orientation.w =orientw-orientdrw;
    // Velocity (twist)y
    // odom_err_msg.twist.twist.linear.x = linvelx-linveldrx;
    // odom_err_msg.twist.twist.linear.y = linvely-linveldry;
    // odom_err_msg.twist.twist.linear.z = linvelz-linveldrz;
    // odom_err_msg.twist.twist.angular.x = angvelx-angveldrx;
    // odom_err_msg.twist.twist.angular.y = angvely-angveldry;
    // odom_err_msg.twist.twist.angular.z = angvelz-angveldrz;
    // Header and frame info
    // odom_err_msg.header.seq = seq;
    // odom_err_msg.header.stamp = ros::Time::now();
    // odom_err_msg.header.frame_id = odometry_frame_id;
    // odom_err_msg.child_frame_id = odometry_child_frame_id;
    // odom_err_pub.publish(odom_err_msg);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}


double deltaCounts(double counts, double counts_prev)
{
  double temp_counts = (double)counts;
  double temp_counts_prev = (double)counts_prev;
  double temp_delta_counts = temp_counts - temp_counts_prev;
  return (double)temp_delta_counts;
}

void odomtruthCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  //get drone name
  int drone_idx = -1;
  for(int i=0; i<msg->name.size(); i++) {
    if(msg->name[i] == odometry_drone_name) {
      drone_idx = i;
      break;
    }
  }

  if(drone_idx == -1) {
    ROS_ERROR("odomtruthCallback: Must define drone name in parameters!");
  }

  posx=msg->pose[drone_idx].position.x;
  posy=msg->pose[drone_idx].position.y;
  posz=msg->pose[drone_idx].position.z;
  linvelx=msg->twist[drone_idx].linear.x;
  linvely=msg->twist[drone_idx].linear.y;
  linvelz=msg->twist[drone_idx].linear.z;
  angvelx=msg->twist[drone_idx].angular.x;
  angvely=msg->twist[drone_idx].angular.y;
  angvelz=msg->twist[drone_idx].angular.z;
  orientx=msg->pose[drone_idx].orientation.x;
  orienty=msg->pose[drone_idx].orientation.y;
  orientz=msg->pose[drone_idx].orientation.z;
  orientw=msg->pose[drone_idx].orientation.w;

  // posx_delta=deltaCounts(posx,posx_prev);
  // posy_delta=deltaCounts(posy,posy_prev);
  // posz_delta=deltaCounts(posz,posz_prev);
  // linvelx_delta=deltaCounts(linvelx,linvelx_prev);
  // linvely_delta=deltaCounts(linvely,linvely_prev);
  // angvelz_delta=deltaCounts(angvelz,angvelz_prev);
}

void gpsiris0Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  posxgps  = msg->pose.pose.position.x;
}
void gpsiris1Callback(const nav_msgs::Odometry::ConstPtr& msg)
{

}
void gpsiris2Callback(const nav_msgs::Odometry::ConstPtr& msg)
{

}

// void odomdrCallback(const visualization_msgs::Marker::ConstPtr& msg)
// {
//   posdrx= msg->pose.position.x;
//   posdry= msg->pose.position.y;
//   posdrz= msg->pose.position.z;
//   // linveldrx=msg->twist.twist.linear.x;
//   // linveldry=msg->twist.twist.linear.y;
//   // linveldrz=msg->twist.twist.linear.z;
//   // angveldrx=msg->twist.twist.angular.x;
//   // angveldry=msg->twist.twist.angular.y;
//   // angveldrz=msg->twist.twist.angular.z;
//   orientdrx=msg->pose.orientation.x;
//   orientdry=msg->pose.orientation.y;
//   orientdrz=msg->pose.orientation.z;
//   orientdrw=msg->pose.orientation.w;
// }
