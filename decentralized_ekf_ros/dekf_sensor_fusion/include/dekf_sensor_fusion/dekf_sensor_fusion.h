#ifndef DEKF_SENSOR_FUSION_H
#define DEKF_SENSOR_FUSION_H

#include "ros/ros.h"
#include <tf/tf.h>
#include "dekf_sensor_fusion/SrvCov.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <eigen3/Eigen/Dense>
#include <time.h>
#include <cmath>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include "dekf_sensor_fusion/globalCovariance.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

using namespace Eigen;
class DekfSensorFusion
{
public:
  DekfSensorFusion(ros::NodeHandle &);
  void SendCovariance();
  void relativeUpdate();
  void initialization();
  void publishRange_();
  // std::string node_name;
  std::string robot_name;
  std::string robot_id_received;
  Eigen::Matrix <double, 15, 1>  state_received;
  Eigen::Matrix <double, 15, 1>  state_sent;
  Eigen::Matrix <double, 15, 1>  err_state_received;
  Eigen::Matrix <double, 15, 1>  err_state_sent;
  Eigen::Matrix <double, 6, 1>  true_position0;
  Eigen::Matrix <double, 6, 1>  true_position1;
  Eigen::Matrix <double, 6, 1>  true_position2;
  Eigen::Matrix <double, 2, 1>  zupt_command_0;
  Eigen::Matrix <double, 2, 1>  zupt_command_1;
  Eigen::Matrix <double, 2, 1>  zupt_command_2;
  Eigen::Matrix <double, 3, 1>  odom_command_0;
  Eigen::Matrix <double, 3, 1>  odom_command_1;
  Eigen::Matrix <double, 3, 1>  odom_command_2;
  Eigen::Matrix <double, 15, 1>  error_state_updated;
  Eigen::Matrix <double, 2, 1>  _range;
  Eigen::Matrix <double, 2, 1>  range_to_drone;
  // Eigen::Matrix <double, 15, 15>  range_cov;
  Eigen::Matrix <double, 15, 1> state1;
  Eigen::Matrix <double, 15, 1> state2;
  Eigen::Matrix <double, 30, 1> states;
  Eigen::Matrix <double, 15, 1> err_state1;
  Eigen::Matrix <double, 15, 1> err_state2;
  Eigen::Matrix <double, 30, 1> err_states;
  Eigen::Matrix <double, 15, 15> P_d1;
  Eigen::Matrix <double, 15, 15> P_d12;
  Eigen::Matrix <double, 15, 15> P_d2;
  Eigen::Matrix <double, 15, 15> P_d21;
  Eigen::Matrix <double, 15, 15> P_corr;
  Eigen::Matrix <double, 15, 15> P_corr2;
  Eigen::Matrix <double, 15, 15> sigma_ij;
  Eigen::Matrix <double, 15, 15> sigma_ji;
  Eigen::Matrix <double, 15, 1> _error_states;
  typedef Eigen::Matrix<double, 15, 1> Vector15;
  typedef Eigen::Matrix<double, 3, 1> Vector3;
  Vector3d V_old;
  Vector3d Pos_old;

  bool initializer;
  bool truths_0;
  bool truths_1;
  bool truths_2;
  bool stop_propation;
  bool relative_update_done;
  bool gps_update_done;

  double res_range;
  double range_update;
  double error_im;

  Matrix3d eye3=Eigen::Matrix3d::Identity();
  Matrix3d zeros3=Eigen::Matrix3d::Zero(3,3);
  // ros::ServiceClient dekf_sensor_fusion_client;
  tf::TransformBroadcaster odom_broadcaster_;
private:
  ros::NodeHandle &nh_;
  ros::ServiceClient dekf_sensor_fusion_client_1;
  ros::ServiceClient dekf_sensor_fusion_client_2;
  ros::ServiceServer dekf_sensor_fusion_service;
  bool calculation(dekf_sensor_fusion::SrvCov::Request &req, dekf_sensor_fusion::SrvCov::Response &res);
//   // ros::Publisher pubOdom_, ...
//   // ros::Subscriber subVO_, subImu_, ...
  ros::Subscriber sub_imu; // Subscribe to IMU data
  ros::Subscriber sub_vo; // Subscribe to Visual Odometry TODO: Are we going to use RealSense VO or our VO?
  ros::Subscriber sub_GPS; // Subscribe to GPS data
  ros::Subscriber sub_Range; //Subscribe to Range data
  ros::Subscriber sub_Altimeter; //Subscribe to Altimeter data
  ros::Subscriber sub_Bearing; //Subscribe to Bearing data
  ros::Subscriber true_drone0; //Subscribes to truth position from drone0
  ros::Subscriber true_drone1; //Subscribes to truth position from drone1
  ros::Subscriber true_drone2; //Subscribes to truth position from drone2
  ros::Subscriber vel_command_tb0;  //Subscribes to velocity command from turtlebot 0
  ros::Subscriber vel_command_tb1;  //Subscribes to velocity command from turtlebot 1
  ros::Subscriber vel_command_tb2;  //Subscribes to velocity command from turtlebot 2
  ros::Subscriber odom_tb0;  //Subscribes to velocity command from turtlebot 0
  ros::Subscriber odom_tb1;  //Subscribes to velocity command from turtlebot 1
  ros::Subscriber odom_tb2;  //Subscribes to velocity command from turtlebot 2
  ros::Publisher pubOdom_;
  ros::Publisher pubRange_;
  // ros::Publisher pubResidual_;

  geometry_msgs::Pose pose_;
  geometry_msgs::Twist bias_;
  geometry_msgs::Point32 twist_;
  geometry_msgs::Pose err_pose_;
  geometry_msgs::Twist err_bias_;
  geometry_msgs::Point32 err_twist_;
  std_msgs::String robot_id;

  Eigen::Matrix <double, 15, 15> P_;
  Eigen::Matrix <double, 15, 1> _x;

  Eigen::Matrix <double, 15, 15> _P;
  Eigen::Matrix <double, 15, 15> _P_init;
  Eigen::Matrix <double, 15, 15> gps_UP;
  Eigen::Matrix <double, 15, 15> _Q_ins;
  Eigen::Matrix <double, 45, 45> _globalP;
  Eigen::Matrix <double, 6, 15> H_gps;
  Eigen::Matrix <double, 6, 6> R_gps;
  Eigen::Matrix <double, 1, 1> R_range;
  Eigen::Matrix <double, 1, 30> H_range;
  Eigen::Matrix<double, 2, 2> R_holo;
  Eigen::Matrix<double, 1, 1> R_holoS;
  Eigen::Matrix<double, 15, 2> K_holo;
  Eigen::Matrix<double, 15, 1> K_holoS;
  Eigen::Matrix<double, 2, 15> H_holo;
  Eigen::Matrix<double, 1, 15> H_holoS;
  Eigen::Matrix<double, 2, 1> z_holo;
  Eigen::Matrix<double, 1, 1> z_holoS;
  Eigen::Matrix<double, 3, 3> R_zupt;
  Eigen::Matrix<double, 3, 3> R_zaru;
  Eigen::Matrix<double, 6, 6> R_zero;
  Eigen::Matrix<double, 15, 3> K_zupt;
  Eigen::Matrix<double, 15, 3> K_zaru;
  Eigen::Matrix<double, 15, 6> K_zero;
  Eigen::Matrix<double, 3, 15> H_zupt;
  Eigen::Matrix<double, 3, 15> H_zaru;
  Eigen::Matrix<double, 6, 15> H_zero;


  // VectorXd state1(15,1);
  // VectorXd state2(15,1);
  // VectorXd states(30,1);
  // MatrixXd P_d1(15,15);
  // MatrixXd P_d12(15,15);
  // MatrixXd P_d2(15,15);
  // MatrixXd P_d21(15,15);
  // MatrixXd P_corr(15,15);
  // MatrixXd P_corr2(15,15);


  typedef Eigen::Matrix <double, 6, 1> Vector6d;

  double r;
  double roll_curr_;
  double pitch_curr_;
  double yaw_curr_;
  sensor_msgs::Imu imu_raw_;
  // double _range;

  double h_range;
  double error;
  // double range_to_drone;

  double degradation;
  double random_gps;

  double _dt;
  double _t;
  ros::Time _time;
  Vector3d _imu_gyro;
  Vector3d _imu_acce;
  Matrix3d _Cnb;
  VectorXd _attitude;
  Vector3d _vel;
  Vector3d _pos;
  Vector3d ba;
  Vector3d bg;
  Vector3d gps_Uatt;
  Vector3d gps_Uvel;
  Vector3d gps_Upos;

  Vector3d gps_pos;
  Vector3d gps_vel;
  Vector3d z_gps_pos;
  Vector3d z_gps_vel;
  Vector6d z_gps;
  VectorXd _dx;



  float _posP;
  // MatrixXd _P;
  // MatrixXd _Q_ins;
  int _insUpdate;
  Matrix3d _euler2dcmV(double phi, double theta, double psi);
  Matrix3d _euler2dcm(Vector3d eulVec);
  Vector4d _dcm2qua(Matrix3d Cnb);
  Matrix3d _qua2dcm(Vector4d qua);
  Vector3d _dcm2euler(Matrix3d dcm);
  Matrix3d _skewsym(Vector3d vec);
  // void NonHolonomic(const DekfSensorFusion::Vector3 vel,const DekfSensorFusion::Vector3 att,const DekfSensorFusion::Vector3 pos,  const DekfSensorFusion::Vector15 _x, const Eigen::Matrix <double, 15, 15> _P, DekfSensorFusion::Vector3 imu_gyro) ;
  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void voCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void gpsCallback(const nav_msgs::Odometry::ConstPtr& msg);       // TODO: Fill this with GPS message type based on what sensor used.
  // void rangeCallback(const sensor_msgs::Range::ConstPtr &msg);
  void true_drone0Callback(const nav_msgs::Odometry::ConstPtr& msg);
  void true_drone1Callback(const nav_msgs::Odometry::ConstPtr& msg);
  void true_drone2Callback(const nav_msgs::Odometry::ConstPtr& msg);
  void vel_command_tb0Callback(const geometry_msgs::Twist::ConstPtr& msg);
  void vel_command_tb1Callback(const geometry_msgs::Twist::ConstPtr& msg);
  void vel_command_tb2Callback(const geometry_msgs::Twist::ConstPtr& msg);
  void odom_tb0Callback(const nav_msgs::Odometry::ConstPtr& msg);
  void odom_tb1Callback(const nav_msgs::Odometry::ConstPtr& msg);
  void odom_tb2Callback(const nav_msgs::Odometry::ConstPtr& msg);

  void altimeterCallback(); // TODO: Fill this with altimeter message type based on what sensor used.
  void bearingCallback();   // TODO: Fill this with GPS message type based on what sensor used
  void publishOdom_();
  // void publishRange_();
  // void publishResidual_();
  void zeroUpdate();
  void nonHolonomicUpdate();
  void calculateProcessNoiseINS();
  // void relativeUpdate();
};

#endif
