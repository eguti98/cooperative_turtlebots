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

using namespace Eigen;
class DekfSensorFusion
{
public:
  DekfSensorFusion(ros::NodeHandle &);
  void SendCovariance();
  void relativeUpdate();
  void initialization();
  // std::string node_name;
  std::string robot_name;
  Eigen::Matrix <double, 9, 1>  state_received;
  Eigen::Matrix <double, 9, 1>  state_sent;
  Eigen::Matrix <double, 6, 1>  true_position1;
  Eigen::Matrix <double, 6, 1>  true_position2;
  Eigen::Matrix <double, 9, 1>  range_est;
  // Eigen::Matrix <double, 9, 9>  range_cov;
  Eigen::Matrix <double, 9, 1> state1;
  Eigen::Matrix <double, 9, 1> state2;
  Eigen::Matrix <double, 18, 1> states;
  Eigen::Matrix <double, 9, 9> P_d1;
  Eigen::Matrix <double, 9, 9> P_d12;
  Eigen::Matrix <double, 9, 9> P_d2;
  Eigen::Matrix <double, 9, 9> P_d21;
  Eigen::Matrix <double, 9, 9> P_corr;
  Eigen::Matrix <double, 9, 9> P_corr2;
  Eigen::Matrix <double, 9, 9> sigma_ij;
  Eigen::Matrix <double, 9, 9> sigma_ji;

  bool initializer;
  bool truths_1;
  bool truths_2;
  bool relative_update_done;
  bool gps_update_done;
  double _range;
  double res_range;
  // ros::ServiceClient dekf_sensor_fusion_client;
private:
  ros::NodeHandle &nh_;
  ros::ServiceClient dekf_sensor_fusion_client;
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
  ros::Subscriber true_drone1; //Subscribes to truth position from drone1
  ros::Subscriber true_drone2; //Subscribes to truth position from drone2

  ros::Publisher pubOdom_;
  ros::Publisher pubRange_;
  ros::Publisher pubResidual_;

  geometry_msgs::Pose pose_;
  geometry_msgs::Twist twist_;
  Eigen::Matrix <double, 9, 9> P_;
  Eigen::Matrix <double, 9, 1> _x;
  Eigen::Matrix <double, 9, 9> _P;
  Eigen::Matrix <double, 9, 9> _P_init;
  Eigen::Matrix <double, 9, 9> gps_UP;
  Eigen::Matrix <double, 9, 9> _Q_ins;
  Eigen::Matrix <double, 18, 18> _globalP;
  Eigen::Matrix <double, 6, 9> H_gps;
  Eigen::Matrix <double, 6, 6> R_gps;
  Eigen::Matrix <double, 1, 1> R_range;
  Eigen::Matrix <double, 1, 18> H_range;
  // Eigen::Matrix <double, 6, 3> G; 

  // VectorXd state1(9,1);
  // VectorXd state2(9,1);
  // VectorXd states(18,1);
  // MatrixXd P_d1(9,9);
  // MatrixXd P_d12(9,9);
  // MatrixXd P_d2(9,9);
  // MatrixXd P_d21(9,9);
  // MatrixXd P_corr(9,9);
  // MatrixXd P_corr2(9,9);


  typedef Eigen::Matrix <double, 6, 1> Vector6d;

  double r;
  double roll_curr_;
  double pitch_curr_;
  double yaw_curr_;
  sensor_msgs::Imu imu_raw_;
  // double _range;

  double h_range;
  double error;
  double range_to_drone;

  double _dt;
  double _t;
  ros::Time _time;
  Vector3d _imu_gyro;
  Vector3d _imu_acce;
  Matrix3d _Cnb;
  VectorXd _attitude;
  Vector3d _vel;
  Vector3d _pos;
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
  void _euler2dcmV();
  Matrix3d _euler2dcm(Vector3d eulVec);
  Vector4d _dcm2qua(Matrix3d Cnb);
  Matrix3d _qua2dcm(Vector4d qua);
  Vector3d _dcm2euler(Matrix3d dcm);
  Matrix3d _skewsym(Vector3d vec);

  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void voCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void gpsCallback(const nav_msgs::Odometry::ConstPtr& msg);       // TODO: Fill this with GPS message type based on what sensor used.
  // void rangeCallback(const sensor_msgs::Range::ConstPtr &msg);
  void true_drone1Callback(const nav_msgs::Odometry::ConstPtr& msg);
  void true_drone2Callback(const nav_msgs::Odometry::ConstPtr& msg);
  void altimeterCallback(); // TODO: Fill this with altimeter message type based on what sensor used.
  void bearingCallback();   // TODO: Fill this with GPS message type based on what sensor used
  void publishOdom_();
  void publishRange_();
  void publishResidual_();
  // void relativeUpdate();
};

#endif
