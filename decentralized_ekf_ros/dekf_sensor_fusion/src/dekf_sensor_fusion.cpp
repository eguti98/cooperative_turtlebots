#include "dekf_sensor_fusion/dekf_sensor_fusion.h"

DekfSensorFusion::DekfSensorFusion(ros::NodeHandle &nh) : nh_(nh)
{
  if (ros::param::get("robot_name", robot_name) == false)
  {
    ROS_FATAL("No parameter 'robot_name' specified");
    ros::shutdown();
    exit(1);
  }
  std::string node_name = "dekf_sensor_fusion";

  dekf_sensor_fusion_service =
  nh.advertiseService("covariance_srv", &DekfSensorFusion::calculation, this);
  if (robot_name=="tb3_0") {
    dekf_sensor_fusion_client_1 =
    nh.serviceClient<dekf_sensor_fusion::SrvCov>("/tb3_1/covariance_srv");
    dekf_sensor_fusion_client_2 =
    nh.serviceClient<dekf_sensor_fusion::SrvCov>("/tb3_2/covariance_srv");
  }
  else if (robot_name=="tb3_1") {
    dekf_sensor_fusion_client_1 =
    nh.serviceClient<dekf_sensor_fusion::SrvCov>("/tb3_0/covariance_srv");
    dekf_sensor_fusion_client_2 =
    nh.serviceClient<dekf_sensor_fusion::SrvCov>("/tb3_2/covariance_srv");
  }
  else if (robot_name=="tb3_2") {
    dekf_sensor_fusion_client_1 =
    nh.serviceClient<dekf_sensor_fusion::SrvCov>("/tb3_0/covariance_srv");
    dekf_sensor_fusion_client_2 =
    nh.serviceClient<dekf_sensor_fusion::SrvCov>("/tb3_1/covariance_srv");
  }

  pubOdom_ = nh_.advertise<nav_msgs::Odometry>(
      "localization/odometry/sensor_fusion", 1);
  pubRange_ = nh_.advertise<std_msgs::Float64MultiArray>("range",1);
  // pubResidual_ = nh_.advertise<std_msgs::Float64>("residual",1);

  Vector3d eul;
  eul<< 0.0,0.0,0.0;
  _Cnb = _euler2dcm(eul);
  _vel << 0,0,0;
  ba << 0,0,0;  // TODO: Check values later
  bg << 0,0,0;  // TODO: Check values later

  if (robot_name=="tb3_0") {
    _pos << 0,0,0;
  }
  else if (robot_name=="tb3_1") {
    _pos << 0,0,0;
  }
  else if (robot_name=="tb3_2") {
    _pos << 0,0,0;
  }


  _attitude = _dcm2euler(_Cnb.transpose());

  _x << _attitude(0),_attitude(1),_attitude(2),_vel(0),_vel(1),_vel(2),_pos(0),_pos(1),_pos(2),ba(0),ba(1),ba(2),bg(0),bg(1),bg(2);
  _error_states = Eigen::VectorXd::Zero(15);
  Eigen::VectorXd _P_initVal(15);
  _P_initVal << 0.01,0.01,0.01,0.1,0.1,0.1,0.5,0.5,0.5,0,0,0,0,0,0;
  _P = _P_initVal.asDiagonal();
  _Q_ins = _P;
  _globalP = MatrixXd::Zero(45,45);
  _globalP.block<15,15>(0,0) = _P;
  _globalP.block<15,15>(15,15) = _P;
  _globalP.block<15,15>(30,30) = _P;

  H_gps = MatrixXd::Zero(6,15);
  // H_gps.bottomRightCorner(6,6) = MatrixXd::Zero(6,6)-MatrixXd::Identity(6,6);
  H_gps.block<6,6>(0,3) = MatrixXd::Zero(6,6)-MatrixXd::Identity(6,6);

  Eigen::VectorXd R_gpsVal(6);
  R_gpsVal << std::pow(0.1,2), std::pow(0.1,2), std::pow(0.1,2), std::pow(0.4,2), std::pow(0.4,2), std::pow(0.4,2);
  R_gps = R_gpsVal.asDiagonal();

  H_wo = MatrixXd::Zero(3,15);
  H_wo.block<3,3>(0,3) = MatrixXd::Zero(3,3)-MatrixXd::Identity(3,3);

  Eigen::VectorXd R_woVal(3);
  R_woVal << std::pow(0.01,2), std::pow(0.01,2), std::pow(0.001,2);
  R_wo = R_woVal.asDiagonal();


  R_range << 0.2*0.2; // TODO: Value?
  initializer = 0;
  truths_0 = 0;
  truths_1 = 0;
  truths_2 = 0;

  relative_update_done = 0;
  gps_update_done = 0;
  wo_update_done = 0;


  R_zero << std::pow(0.01,2),0,0,0,0,0,
            0,std::pow(0.01,2),0,0,0,0,
            0,0,std::pow(0.0025,2),0,0,0,
            0,0,0,std::pow(0.1,2),0,0,
            0,0,0,0,std::pow(0.1,2),0,
            0,0,0,0,0,std::pow(1.0,2);

  // NON HOLONONOMIC R VALUES
  R_holo << 0.05,0,
            0,0.1;

  H_zupt << 0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,
            0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0;

  H_zaru << 0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,
            0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1;

  H_zero << 0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,
            0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,
            0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,
            0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0;


  sub_imu = nh.subscribe("imu", 10, &DekfSensorFusion::imuCallback, this);
  sub_GPS = nh.subscribe("gps", 1, &DekfSensorFusion::gpsCallback,this); ///uav0/mavros/global_position/local // this is not the specific gps topic
  sub_wo= nh.subscribe("deadreckoning", 1, &DekfSensorFusion::woCallback,this);

  true_drone0 = nh.subscribe("/tb3_0/truth", 1, &DekfSensorFusion::true_drone0Callback, this);
  true_drone1 = nh.subscribe("/tb3_1/truth", 1, &DekfSensorFusion::true_drone1Callback, this);
  true_drone2 = nh.subscribe("/tb3_2/truth", 1, &DekfSensorFusion::true_drone2Callback, this);

  vel_command_tb0 = nh.subscribe("/tb3_0/cmd_vel", 1, &DekfSensorFusion::vel_command_tb0Callback, this);
  vel_command_tb1 = nh.subscribe("/tb3_1/cmd_vel", 1, &DekfSensorFusion::vel_command_tb1Callback, this);
  vel_command_tb2 = nh.subscribe("/tb3_2/cmd_vel", 1, &DekfSensorFusion::vel_command_tb2Callback, this);
}

//
// IMU Prediction
//
void DekfSensorFusion::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{

  if (initializer == 1) {
    // if (stop_propation == 0) {
      if (_time.toSec() < msg->header.stamp.toSec()) {
        _time = msg->header.stamp;
      }

      if (_t) {
        _dt = _time.toSec() - _t;
        if(abs(_dt) > 0.5){
          _dt = 0.01;
        }
      } else {
        _dt = 0.01;
      }
      _t = _time.toSec();
      relative_update_done = 0;
      double p = msg->angular_velocity.x-bg(0);
      double q = msg->angular_velocity.y-bg(1);
      double r = msg->angular_velocity.z-bg(2);
      // omega_ib = p,q,r
      double ax= msg->linear_acceleration.x-ba(0);
      double ay= msg->linear_acceleration.y-ba(1);
      double az= msg->linear_acceleration.z + 9.81-ba(2);
      // // f_ib= ax,ay,az

      _imu_gyro << p, q, r;
      _imu_acce << ax,ay,az;

      Matrix3d Omega_ib;
      Omega_ib << 0, -r, q, r, 0, -p, -q, p, 0;
      Matrix3d I;
      I.setIdentity();

      Matrix3d CnbMinus = _euler2dcmV(_attitude(0),_attitude(1),_attitude(2));
      Matrix3d CbnMinus;
      CbnMinus =CnbMinus.transpose();

      Matrix3d Cbn;
      Cbn = CbnMinus * (I + Omega_ib * _dt); // ignore Earth rate and corriolis terms
      _attitude = _dcm2euler(Cbn);
      std::cout << "yaw" <<'\n'<< _attitude(2)<<'\n';
      std::cout << "yawX" <<'\n'<< _x[2]<<'\n';
      Vector3d V_n_ib;
      V_n_ib=0.5*(Cbn+CbnMinus)*_imu_acce*_dt;

      Vector3d grav_;
      grav_ << 0,0,-9.81;
      V_old=_vel+V_n_ib+(grav_)*_dt;

      Pos_old=_pos+(V_old+_vel)*_dt/2.0;

      MatrixXd I15(15, 15);
      I15.setIdentity();
      MatrixXd STM(15,15);
      // starting at (i,j), block of size (p,q),
      // matrix.block(i,j,p,q)  dynamic size block expression
      // matrix.block<p,q>(i,j) fixed size block expression

      Eigen::Matrix <double, 15, 15> F = Eigen::MatrixXd::Zero(15,15);
      F.block<3,3>(3,0) = _skewsym(-Cbn*_imu_acce);
      F.block<3,3>(3,9) = Cbn;
      F.block<3,3>(0,12) = Cbn;
      MatrixXd I3(3, 3);
      I3.setIdentity();
      F.block<3,3>(6,3) = I3;
      STM = I15+F*_dt;

      _error_states= STM*_error_states; //State

      // START - CALCULATE Q
      // calculateProcessNoiseINS();
      // END - CALCULATE Q
      _P = STM * _P * STM.transpose() + _Q_ins; // Covariance Matrix


// #define WHEEL_RADIUS                    0.033     // meter
// wheel_seperation_ = 0.287;
// turning_radius_   = 0.1435;
// robot_radius_     = 0.220;

        // _attitude=Att_old;
        _vel=V_old;
        _pos=Pos_old;

      //Update Global P
      if (robot_name=="tb3_0") {
        _globalP.block<15,15>(0,0) = _P;
        _globalP.block<15,15>(0,15) = STM*_globalP.block<15,15>(0,15);
        _globalP.block<15,15>(0,30) = STM*_globalP.block<15,15>(0,30);
        error_im = sqrt(pow((true_position0(0)-_x(6)),2)+pow((true_position0(1)-_x(7)),2)+pow((true_position0(2)-_x(8)),2));
      }
      else if (robot_name=="tb3_1") {
        _globalP.block<15,15>(15,15) = _P;
        _globalP.block<15,15>(15,0) = STM*_globalP.block<15,15>(15,0);
        _globalP.block<15,15>(15,30) = STM*_globalP.block<15,15>(15,30);
        error_im = sqrt(pow((true_position1(0)-_x(6)),2)+pow((true_position1(1)-_x(7)),2)+pow((true_position1(2)-_x(8)),2));
      }
      else if (robot_name=="tb3_2") {
        _globalP.block<15,15>(30,30) = _P;
        _globalP.block<15,15>(30,0) = STM*_globalP.block<15,15>(30,0);
        _globalP.block<15,15>(30,15) = STM*_globalP.block<15,15>(30,15);
        error_im = sqrt(pow((true_position2(0)-_x(6)),2)+pow((true_position2(1)-_x(7)),2)+pow((true_position2(2)-_x(8)),2));
      }


      ba(0)=ba(0)+_error_states(9);
      ba(1)=ba(1)+_error_states(10);
      ba(2)=ba(2)+_error_states(11);
      bg(0)=bg(0)+_error_states(12);
      bg(1)=bg(1)+_error_states(13);
      bg(2)=bg(2)+_error_states(14);
    }
      // START - Zero Update (ZUPT & ZARU)
  if (robot_name=="tb3_0") {
    // std::cout << "Linear Velocity Command: " << zupt_command_0(0)<< '\n';
    // std::cout << "Angular Velocity Command: " << zupt_command_0(1)<< '\n';
    if (zupt_command_0(0)==0 && zupt_command_0(1)==0) {
      // zeroUpdate();
      // nonHolonomicUpdate();
      // // ROS_INFO("Zero Update Done");
    }
  }
  else if (robot_name=="tb3_1") {
    // std::cout << "Linear Velocity Command: " << zupt_command_1(0)<< '\n';
    // std::cout << "Angular Velocity Command: " << zupt_command_1(1)<< '\n';
    if (zupt_command_1(0)==0 && zupt_command_1(1)==0) {
      // zeroUpdate();
      // nonHolonomicUpdate();
      // ROS_INFO("Zero Update Done");
    }
  }
  else if (robot_name=="tb3_2") {
    // std::cout << "Linear Velocity Command: " << zupt_command_2(0)<< '\n';
    // std::cout << "Angular Velocity Command: " << zupt_command_2(1)<< '\n';
    if (zupt_command_2(0)==0 && zupt_command_2(1)==0) {
      // zeroUpdate();
      // nonHolonomicUpdate();
      // ROS_INFO("Zero Update Done");
    }
  }
// nonHolonomicUpdate();
      _x << _attitude(0),_attitude(1),_attitude(2),_vel(0),_vel(1),_vel(2),_pos(0),_pos(1),_pos(2),ba(0),ba(1),ba(2),bg(0),bg(1),bg(2);
      _error_states.segment(9,6)<<Eigen::VectorXd::Zero(6);

    // }

    publishOdom_();
  }
  // else {return;}

// GPS Update
//
void DekfSensorFusion::gpsCallback(const nav_msgs::Odometry::ConstPtr &msg)
{

  if (initializer == 1) {
    Matrix3d Cnb = _euler2dcmV(_attitude(0),_attitude(1),_attitude(2));
    Matrix3d Cbn = Cnb.transpose();
    MatrixXd K_gps(15,6);

    gps_pos[0] = msg->pose.pose.position.x;
    gps_pos[1] = msg->pose.pose.position.y;
    gps_pos[2] = msg->pose.pose.position.z;

    gps_vel[0] = msg->twist.twist.linear.x;
    gps_vel[1] = msg->twist.twist.linear.y;
    gps_vel[2] = msg->twist.twist.linear.z;

    z_gps_pos= gps_pos-_pos;
    z_gps_vel= gps_vel-_vel;
    // z_gps_pos= gps_pos-Pos_old;
    // z_gps_vel= gps_vel-V_old;
    K_gps = _P * H_gps.transpose() * (H_gps * _P * H_gps.transpose() + R_gps).inverse();

    z_gps <<  z_gps_vel[0],z_gps_vel[1],z_gps_vel[2],z_gps_pos[0],z_gps_pos[1],z_gps_pos[2];
    _error_states = _error_states + K_gps * (z_gps  - H_gps * _error_states);

    _attitude = _dcm2euler((Eigen::MatrixXd::Identity(3,3)- _skewsym(_error_states.segment(0,3)))*Cbn);
    _vel = _vel-_error_states.segment(3,3);
    _pos = _pos-_error_states.segment(6,3);
    // V_old = V_old-_error_states.segment(3,3);
    // Pos_old = Pos_old -_error_states.segment(6,3);

    _error_states.segment(0,9)<<Eigen::VectorXd::Zero(9);

    _P=(Eigen::MatrixXd::Identity(15,15) - K_gps * H_gps) * _P * ( Eigen::MatrixXd::Identity(15,15) - K_gps * H_gps ).transpose() + K_gps * R_gps * K_gps.transpose();


    if (robot_name=="tb3_0") {
      _globalP.block<15,15>(0,0) = _P;
      _globalP.block<15,15>(0,15) = (Eigen::MatrixXd::Identity(15,15) - K_gps * H_gps) * _globalP.block<15,15>(0,15) * ( Eigen::MatrixXd::Identity(15,15) - K_gps * H_gps ).transpose() + K_gps * R_gps * K_gps.transpose();
      _globalP.block<15,15>(0,30) = (Eigen::MatrixXd::Identity(15,15) - K_gps * H_gps) * _globalP.block<15,15>(0,30) * ( Eigen::MatrixXd::Identity(15,15) - K_gps * H_gps ).transpose() + K_gps * R_gps * K_gps.transpose();
    }
    else if (robot_name=="tb3_1") {
      _globalP.block<15,15>(15,15) = _P;
      _globalP.block<15,15>(15,0) = (Eigen::MatrixXd::Identity(15,15) - K_gps * H_gps) * _globalP.block<15,15>(15,0) * ( Eigen::MatrixXd::Identity(15,15) - K_gps * H_gps ).transpose() + K_gps * R_gps * K_gps.transpose();
      _globalP.block<15,15>(15,30) = (Eigen::MatrixXd::Identity(15,15) - K_gps * H_gps) * _globalP.block<15,15>(15,30) * ( Eigen::MatrixXd::Identity(15,15) - K_gps * H_gps ).transpose() + K_gps * R_gps * K_gps.transpose();
    }
    else if (robot_name=="tb3_2") {
      _globalP.block<15,15>(30,30) = _P;
      _globalP.block<15,15>(30,0) = (Eigen::MatrixXd::Identity(15,15) - K_gps * H_gps) * _globalP.block<15,15>(30,0) * ( Eigen::MatrixXd::Identity(15,15) - K_gps * H_gps ).transpose() + K_gps * R_gps * K_gps.transpose();
      _globalP.block<15,15>(30,15) = (Eigen::MatrixXd::Identity(15,15) - K_gps * H_gps) * _globalP.block<15,15>(30,15) * ( Eigen::MatrixXd::Identity(15,15) - K_gps * H_gps ).transpose() + K_gps * R_gps * K_gps.transpose();
    }
    gps_update_done = 1;

    // publishOdom_();
    // publishRange_();
    // publishResidual_();
  }
}
//
// VO Update
//
void DekfSensorFusion::voCallback(const nav_msgs::Odometry::ConstPtr &msg)
{}

void DekfSensorFusion::woCallback(const nav_msgs::Odometry::ConstPtr &msg)
{

    if (initializer == 1) {
      Matrix3d Cnb = _euler2dcmV(_attitude(0),_attitude(1),_attitude(2));
      Matrix3d Cbn = Cnb.transpose();
      MatrixXd K_wo(15,3);

      // gps_pos[0] = msg->pose.pose.position.x;
      // gps_pos[1] = msg->pose.pose.position.y;
      // gps_pos[2] = msg->pose.pose.position.z;


      wo_vel[0] = msg->twist.twist.linear.x;
      wo_vel[1] = msg->twist.twist.linear.y; //0
      wo_vel[2] = msg->twist.twist.linear.z; //0
      // std::cout << "----------------" << '\n';
      // std::cout << "WO VEL"<<'\n' << wo_vel<<'\n';
      // wo_vel=Cbn*wo_vel;
      // std::cout << "WO VELAfter Transform" <<'\n'<< wo_vel<<'\n';
      // std::cout << "VEL" <<'\n'<< _vel<<'\n';
      // std::cout << "yaw" <<'\n'<< _attitude(2)<<'\n';
      // std::cout << "yawX" <<'\n'<< _x[2]<<'\n';
      // gps_vel[2] = msg->twist.twist.linear.z;

      // z_gps_pos= gps_pos-_pos;
      z_wo_vel= wo_vel-_vel;
      // z_gps_pos= gps_pos-Pos_old;
      // z_gps_vel= gps_vel-V_old;
      K_wo = _P * H_wo.transpose() * (H_wo * _P * H_wo.transpose() + R_wo).inverse();

      z_wo <<  z_wo_vel[0],z_wo_vel[1],wo_vel[2];
      _error_states = _error_states + K_wo * (z_wo  - H_wo* _error_states);
// std::cout << "POS before removal" <<'\n'<< _pos<<'\n';
      _attitude = _dcm2euler((Eigen::MatrixXd::Identity(3,3)- _skewsym(_error_states.segment(0,3)))*Cbn);
      _vel = _vel-_error_states.segment(3,3);
      _pos = _pos-_error_states.segment(6,3);
      // V_old = V_old-_error_states.segment(3,3);
      // Pos_old = Pos_old -_error_states.segment(6,3);
      // std::cout << "VEL after removal" <<'\n'<< _vel<<'\n';
      // std::cout << "Real VEL" <<'\n'<< true_position1.segment(6,3)<<'\n';
      // std::cout << "yaw after removal" <<'\n'<< _attitude(2)<<'\n';
      // std::cout << "yawX after removal" <<'\n'<< _x[2]<<'\n';
      // std::cout << "POS after removal" <<'\n'<< _pos<<'\n';
      _error_states.segment(0,9)<<Eigen::VectorXd::Zero(9);

      _P=(Eigen::MatrixXd::Identity(15,15) - K_wo * H_wo) * _P * ( Eigen::MatrixXd::Identity(15,15) - K_wo * H_wo ).transpose() + K_wo * R_wo * K_wo.transpose();


      if (robot_name=="tb3_0") {
        _globalP.block<15,15>(0,0) = _P;
        _globalP.block<15,15>(0,15) = (Eigen::MatrixXd::Identity(15,15) - K_wo * H_wo) * _globalP.block<15,15>(0,15) * ( Eigen::MatrixXd::Identity(15,15) - K_wo * H_wo ).transpose() + K_wo * R_wo * K_wo.transpose();
      }
      else if (robot_name=="tb3_1") {
        _globalP.block<15,15>(15,15) = _P;
        _globalP.block<15,15>(15,0) = (Eigen::MatrixXd::Identity(15,15) - K_wo * H_wo) * _globalP.block<15,15>(15,0) * ( Eigen::MatrixXd::Identity(15,15) - K_wo * H_wo ).transpose() + K_wo * R_wo * K_wo.transpose();
      }
      wo_update_done = 1;

      // publishOdom_();
      // publishRange_();
      // publishResidual_();
    }
}

// Relative Update
//
void DekfSensorFusion::relativeUpdate()
{

  MatrixXd covariances(30,30);

  if (robot_name=="tb3_0")
  {
    if (robot_id_received=="tb3_1")
    {
      P_d1 = _globalP.block<15,15>(0,0);        //Sigma_ii
      P_d12 = _globalP.block<15,15>(0,15);      //Sigma_ij
      P_d21 = _globalP.block<15,15>(15,0);      //Sigma_ji
      P_d2 = _globalP.block<15,15>(15,15);      //Sigma_jj
      P_corr = P_d12 * P_d21.transpose();


      P_corr2 = P_corr.transpose();

      covariances.block<15,15>(0,0) = P_d1;
      covariances.block<15,15>(0,15) = P_corr;
      covariances.block<15,15>(15,0) = P_corr2;
      covariances.block<15,15>(15,15) = P_d2;

      state1 = _x;                             // Total state
      err_state1=_error_states;                // Error state
      state2 = state_received;                 // Total state received
      err_state2=err_state_received;           // Error state received


      range_update = _range(0);
    }
    else if (robot_id_received=="tb3_2")
    {
      P_d1 = _globalP.block<15,15>(0,0);        //Sigma_ii
      P_d12 = _globalP.block<15,15>(0,30);      //Sigma_ij
      P_d21 = _globalP.block<15,15>(30,0);      //Sigma_ji
      P_d2 = _globalP.block<15,15>(30,30);      //Sigma_jj
      P_corr = P_d12 * P_d21.transpose();
      P_corr2 = P_corr.transpose();

      covariances.block<15,15>(0,0) = P_d1;
      covariances.block<15,15>(0,15) = P_corr;
      covariances.block<15,15>(15,0) = P_corr2;
      covariances.block<15,15>(15,15) = P_d2;

      state1 = _x;                             // Total state
      err_state1 = _error_states;              // Error state
      state2 = state_received;                 // Total state received
      err_state2 = err_state_received;         // Error state received

      range_update = _range(1);
    }
  }
  else if (robot_name=="tb3_1")
  {
    if (robot_id_received=="tb3_0")
    {
      P_d1 = _globalP.block<15,15>(0,0);        //Sigma_ii
      P_d12 = _globalP.block<15,15>(0,15);      //Sigma_ij
      P_d21 = _globalP.block<15,15>(15,0);      //Sigma_ji
      P_d2 = _globalP.block<15,15>(15,15);      //Sigma_jj
      P_corr = P_d21 * P_d12.transpose();

      P_corr2 = P_corr.transpose();

      covariances.block<15,15>(0,0) = P_d1;
      covariances.block<15,15>(15,0) = P_corr;
      covariances.block<15,15>(0,15) = P_corr2;
      covariances.block<15,15>(15,15) = P_d2;

      state1 = state_received;                   // Total state
      err_state1 = err_state_received;           // Error state
      state2 = _x;                               // Total state received
      err_state2 = _error_states;                // Error state received

      range_update = _range(0);
    }
    else if (robot_id_received=="tb3_2")
    {
      P_d1 = _globalP.block<15,15>(15,15);        //Sigma_ii
      P_d12 = _globalP.block<15,15>(15,30);       //Sigma_ij
      P_d21 = _globalP.block<15,15>(30,15);       //Sigma_ji
      P_d2 = _globalP.block<15,15>(30,30);        //Sigma_jj
      P_corr = P_d12 * P_d21.transpose();
      P_corr2 = P_corr.transpose();

      covariances.block<15,15>(0,0) = P_d1;
      covariances.block<15,15>(0,15) = P_corr;
      covariances.block<15,15>(15,0) = P_corr2;
      covariances.block<15,15>(15,15) = P_d2;

      state1 = _x;                                // Total state
      err_state1 = _error_states;                 // Error state
      state2 = state_received;                    // Total state received
      err_state2 = err_state_received;            // Error state received

      range_update = _range(1);
    }
  }
  else if (robot_name=="tb3_2")
  {
    if (robot_id_received=="tb3_0")
    {
      P_d1 = _globalP.block<15,15>(0,0);        //Sigma_ii
      P_d12 = _globalP.block<15,15>(0,30);      //Sigma_ij
      P_d21 = _globalP.block<15,15>(30,0);      //Sigma_ji
      P_d2 = _globalP.block<15,15>(30,30);      //Sigma_jj
      P_corr = P_d21 * P_d12.transpose();
      P_corr2 = P_corr.transpose();

      covariances.block<15,15>(0,0) = P_d1;
      covariances.block<15,15>(15,0) = P_corr;
      covariances.block<15,15>(0,15) = P_corr2;
      covariances.block<15,15>(15,15) = P_d2;

      state1 = state_received;                   // Total state
      err_state1 = err_state_received;           // Error state
      state2 = _x;                               // Total state received
      err_state2 = _error_states;                // Error state received

      range_update = _range(0);
    }
    else if (robot_id_received=="tb3_1")
    {
      P_d1 = _globalP.block<15,15>(15,15);       //Sigma_ii
      P_d12 = _globalP.block<15,15>(15,30);      //Sigma_ij
      P_d21 = _globalP.block<15,15>(30,15);      //Sigma_ji
      P_d2 = _globalP.block<15,15>(30,30);       //Sigma_jj
      P_corr = P_d12 * P_d21.transpose();
      P_corr2 = P_corr.transpose();

      covariances.block<15,15>(0,0) = P_d1;
      covariances.block<15,15>(0,15) = P_corr;
      covariances.block<15,15>(15,0) = P_corr2;
      covariances.block<15,15>(15,15) = P_d2;

      state1 = state_received;                   // Total state
      err_state1 = err_state_received;           // Error state
      state2 = _x;                               // Total state received
      err_state2 = _error_states;                // Error state received

      range_update = _range(1);
    }
  }

  states << state1(0),state1(1),state1(2),state1(3),state1(4),state1(5),state1(6),state1(7),state1(8),state1(9),state1(10),state1(11),state1(12),state1(13),state1(14),state2(0),state2(1),state2(2),state2(3),state2(4),state2(5),state2(6),state2(7),state2(8),state2(9),state2(10),state2(11),state2(12),state2(13),state2(14);
  err_states << err_state1(0),err_state1(1),err_state1(2),err_state1(3),err_state1(4),err_state1(5),err_state1(6),err_state1(7),err_state1(8),err_state1(9),err_state1(10),err_state1(11),err_state1(12),err_state1(13),err_state1(14),err_state2(0),err_state2(1),err_state2(2),err_state2(3),err_state2(4),err_state2(5),err_state2(6),err_state2(7),err_state2(8),err_state2(9),err_state2(10),err_state2(11),err_state2(12),err_state2(13),err_state2(14);

  h_range = sqrt(pow((states(21)-states(6)),2)+pow((states(22)-states(7)),2)+pow((states(23)-states(8)),2));

  H_range << 0,0,0,0,0,0,
             -(states(21)-states(6)) / h_range,
             -(states(22)-states(7)) / h_range,
             -(states(23)-states(8)) / h_range,
             0,0,0,0,0,0,
             0,0,0,0,0,0,
            (states(21)-states(6)) / h_range,
            (states(22)-states(7)) / h_range,
            (states(23)-states(8)) / h_range,
            0,0,0,0,0,0;

  MatrixXd S(1, 1);
  S = H_range * covariances * H_range.transpose() + R_range;

  MatrixXd K_range(30, 1);
  K_range = covariances * H_range.transpose() * S.inverse();

  res_range = range_update - h_range;

  if (abs(res_range) > 500.0)
  {
      // return;
      ROS_ERROR("Relative Update Ignored");
      ROS_INFO_STREAM("Range "<< robot_id_received << ": " << range_update);
      /* Perform Zupt? */
  }
  else
  {

      MatrixXd I30(30,30);
      I30.setIdentity();

      err_states = err_states + K_range*res_range;

      covariances = (I30 - K_range*H_range)*covariances;
      // covariances = (I30 - K_range*H_range)*covariances*(I30 - K_range*H_range).inverse() + K_range*R_range*K_range.transpose();

    if (robot_name=="tb3_0")
    {
      if (robot_id_received=="tb3_1")
      {

        error_state_updated = err_states.segment(0,15);

        Matrix3d Cnb = _euler2dcmV(state1(0),state1(1),state1(2));
        Matrix3d Cbn = Cnb.transpose();

        _attitude = _dcm2euler((Eigen::MatrixXd::Identity(3,3)+ _skewsym(error_state_updated.segment(0,3)))*Cbn);
        _vel(0) = state1(3)+ error_state_updated(3);
        _vel(1) = state1(4)+ error_state_updated(4);
        _vel(2) = state1(5)+ error_state_updated(5);
        _pos(0) = state1(6)+ error_state_updated(6);
        _pos(1) = state1(7)+ error_state_updated(7);
        _pos(2) = state1(8)+ error_state_updated(8);
        ba(0) =  error_state_updated(9);
        ba(1) =  error_state_updated(10);
        ba(2) =  error_state_updated(11);
        bg(0) =  error_state_updated(12);
        bg(1) =  error_state_updated(13);
        bg(2) =  error_state_updated(14);
        err_states.segment(0,9)<<Eigen::VectorXd::Zero(9);

        _error_states<<err_states.segment(0,15);

        _P = covariances.block<15,15>(0,0);
        _globalP.block<15,15>(0,0) = covariances.block<15,15>(0,0);
        _globalP.block<15,15>(0,15) = Eigen::MatrixXd::Identity(15,15);
        // _globalP.block<15,15>(15,0) = covariances.block<15,15>(15,0);
        // _globalP.block<15,15>(15,15) = covariances.block<15,15>(15,15);
        // _globalP.block<15,15>(0,30) = covariances.block<15,15>(0,0)*P_d1.inverse()*_globalP.block<15,15>(0,30);

        relative_update_done = 1;
        ROS_WARN("Relative Update Done");
        ROS_INFO_STREAM("Range tb1: " << range_update);
        // error = sqrt(pow((true_position1(0)-_x(6)),2)+pow((true_position1(1)-_x(7)),2)+pow((true_position1(2)-_x(8)),2));
        // ROS_INFO("Error: %.4f",error);

      }
      else if (robot_id_received=="tb3_2")
      {

        error_state_updated = err_states.segment(0,15);

        Matrix3d Cnb = _euler2dcmV(state1(0),state1(1),state1(2));
        Matrix3d Cbn = Cnb.transpose();

        _attitude = _dcm2euler((Eigen::MatrixXd::Identity(3,3)+ _skewsym(error_state_updated.segment(0,3)))*Cbn);
        _vel(0) = state1(3)+ error_state_updated(3);
        _vel(1) = state1(4)+ error_state_updated(4);
        _vel(2) = state1(5)+ error_state_updated(5);
        _pos(0) = state1(6)+ error_state_updated(6);
        _pos(1) = state1(7)+ error_state_updated(7);
        _pos(2) = state1(8)+ error_state_updated(8);
        ba(0) =  error_state_updated(9);
        ba(1) =  error_state_updated(10);
        ba(2) =  error_state_updated(11);
        bg(0) =  error_state_updated(12);
        bg(1) =  error_state_updated(13);
        bg(2) =  error_state_updated(14);
        err_states.segment(0,9)<<Eigen::VectorXd::Zero(9);

        _error_states<<err_states.segment(0,15);

        _P = covariances.block<15,15>(0,0);
        _globalP.block<15,15>(0,0) = covariances.block<15,15>(0,0);
        _globalP.block<15,15>(0,30) = Eigen::MatrixXd::Identity(15,15);
        // _globalP.block<15,15>(30,0) = covariances.block<15,15>(15,0);
        // _globalP.block<15,15>(30,30) = covariances.block<15,15>(15,15);
        // _globalP.block<15,15>(0,15) = covariances.block<15,15>(0,0)*P_d1.inverse()*_globalP.block<15,15>(0,15);

        relative_update_done = 1;
        ROS_WARN("Relative Update Done");
        ROS_INFO_STREAM("Range tb2: " << range_update);
        // error = sqrt(pow((true_position1(0)-_x(6)),2)+pow((true_position1(1)-_x(7)),2)+pow((true_position1(2)-_x(8)),2));
        // ROS_INFO("Error: %.4f",error);

      }
    }
    else if (robot_name=="tb3_1")
    {
      if (robot_id_received=="tb3_0")
      {

        error_state_updated = err_states.segment(15,15);

        Matrix3d Cnb = _euler2dcmV(state2(0),state2(1),state2(2));
        Matrix3d Cbn = Cnb.transpose();

        _attitude = _dcm2euler((Eigen::MatrixXd::Identity(3,3)+ _skewsym(error_state_updated.segment(0,3)))*Cbn);
        _vel(0) = state2(3)+ error_state_updated(3);
        _vel(1) = state2(4)+ error_state_updated(4);
        _vel(2) = state2(5)+ error_state_updated(5);
        _pos(0) = state2(6)+ error_state_updated(6);
        _pos(1) = state2(7)+ error_state_updated(7);
        _pos(2) = state2(8)+ error_state_updated(8);
        ba(0) =  error_state_updated(9);
        ba(1) =  error_state_updated(10);
        ba(2) =  error_state_updated(11);
        bg(0) =  error_state_updated(12);
        bg(1) =  error_state_updated(13);
        bg(2) =  error_state_updated(14);
        err_states.segment(15,9)<<Eigen::VectorXd::Zero(9);

        _error_states<<err_states.segment(15,15);

        _P = covariances.block<15,15>(15,15);
        // _globalP.block<15,15>(0,0) = covariances.block<15,15>(0,0);
        // _globalP.block<15,15>(0,15) = Eigen::MatrixXd::Identity(15,15);
        _globalP.block<15,15>(15,0) = covariances.block<15,15>(15,0);
        _globalP.block<15,15>(15,15) = covariances.block<15,15>(15,15);
        // _globalP.block<15,15>(15,30) = covariances.block<15,15>(15,15)*P_d2.inverse()*_globalP.block<15,15>(15,30);

        relative_update_done = 1;
        ROS_WARN("Relative Update Done");
        ROS_INFO_STREAM("Range tb0: " << range_update);
        // error = sqrt(pow((true_position1(0)-_x(6)),2)+pow((true_position1(1)-_x(7)),2)+pow((true_position1(2)-_x(8)),2));
        // ROS_INFO("Error: %.4f",error);

      }
      else if (robot_id_received=="tb3_2")
      {

        error_state_updated = err_states.segment(0,15);

        Matrix3d Cnb = _euler2dcmV(state1(0),state1(1),state1(2));
        Matrix3d Cbn = Cnb.transpose();

        _attitude = _dcm2euler((Eigen::MatrixXd::Identity(3,3)+ _skewsym(error_state_updated.segment(0,3)))*Cbn);
        _vel(0) = state1(3)+ error_state_updated(3);
        _vel(1) = state1(4)+ error_state_updated(4);
        _vel(2) = state1(5)+ error_state_updated(5);
        _pos(0) = state1(6)+ error_state_updated(6);
        _pos(1) = state1(7)+ error_state_updated(7);
        _pos(2) = state1(8)+ error_state_updated(8);
        ba(0) =  error_state_updated(9);
        ba(1) =  error_state_updated(10);
        ba(2) =  error_state_updated(11);
        bg(0) =  error_state_updated(12);
        bg(1) =  error_state_updated(13);
        bg(2) =  error_state_updated(14);
        err_states.segment(0,9)<<Eigen::VectorXd::Zero(9);

        _error_states<<err_states.segment(0,15);

        _P = covariances.block<15,15>(0,0);
        _globalP.block<15,15>(15,15) = covariances.block<15,15>(0,0);
        _globalP.block<15,15>(15,30) = Eigen::MatrixXd::Identity(15,15);
        // _globalP.block<15,15>(30,15) = covariances.block<15,15>(15,0);
        // _globalP.block<15,15>(30,30) = covariances.block<15,15>(15,15);
        // _globalP.block<15,15>(15,0) = covariances.block<15,15>(0,0)*P_d1.inverse()*_globalP.block<15,15>(15,0);

        relative_update_done = 1;
        ROS_WARN("Relative Update Done");
        ROS_INFO_STREAM("Range tb2: " << range_update);
        // error = sqrt(pow((true_position1(0)-_x(6)),2)+pow((true_position1(1)-_x(7)),2)+pow((true_position1(2)-_x(8)),2));
        // ROS_INFO("Error: %.4f",error);

      }
    }
    else if (robot_name=="tb3_2")
    {
      if (robot_id_received=="tb3_0")
      {

        error_state_updated = err_states.segment(15,15);

        Matrix3d Cnb = _euler2dcmV(state2(0),state2(1),state2(2));
        Matrix3d Cbn = Cnb.transpose();

        _attitude = _dcm2euler((Eigen::MatrixXd::Identity(3,3)+ _skewsym(error_state_updated.segment(0,3)))*Cbn);
        _vel(0) = state2(3)+ error_state_updated(3);
        _vel(1) = state2(4)+ error_state_updated(4);
        _vel(2) = state2(5)+ error_state_updated(5);
        _pos(0) = state2(6)+ error_state_updated(6);
        _pos(1) = state2(7)+ error_state_updated(7);
        _pos(2) = state2(8)+ error_state_updated(8);
        ba(0) =  error_state_updated(9);
        ba(1) =  error_state_updated(10);
        ba(2) =  error_state_updated(11);
        bg(0) =  error_state_updated(12);
        bg(1) =  error_state_updated(13);
        bg(2) =  error_state_updated(14);
        err_states.segment(15,9)<<Eigen::VectorXd::Zero(9);

        _error_states<<err_states.segment(15,15);

        _P = covariances.block<15,15>(15,15);
        // _globalP.block<15,15>(0,0) = covariances.block<15,15>(0,0);
        // _globalP.block<15,15>(0,30) = Eigen::MatrixXd::Identity(15,15);
        _globalP.block<15,15>(30,0) = covariances.block<15,15>(15,0);
        _globalP.block<15,15>(30,30) = covariances.block<15,15>(15,15);
        // _globalP.block<15,15>(30,15) = covariances.block<15,15>(15,15)*P_d2.inverse()*_globalP.block<15,15>(30,15);

        relative_update_done = 1;
        ROS_WARN("Relative Update Done");
        ROS_INFO_STREAM("Range tb0: " << range_update);
        // error = sqrt(pow((true_position1(0)-_x(6)),2)+pow((true_position1(1)-_x(7)),2)+pow((true_position1(2)-_x(8)),2));
        // ROS_INFO("Error: %.4f",error);
      }
      else if (robot_id_received=="tb3_1")
      {
        error_state_updated = err_states.segment(15,15);

        Matrix3d Cnb = _euler2dcmV(state2(0),state2(1),state2(2));
        Matrix3d Cbn = Cnb.transpose();

        _attitude = _dcm2euler((Eigen::MatrixXd::Identity(3,3)+ _skewsym(error_state_updated.segment(0,3)))*Cbn);
        _vel(0) = state2(3)+ error_state_updated(3);
        _vel(1) = state2(4)+ error_state_updated(4);
        _vel(2) = state2(5)+ error_state_updated(5);
        _pos(0) = state2(6)+ error_state_updated(6);
        _pos(1) = state2(7)+ error_state_updated(7);
        _pos(2) = state2(8)+ error_state_updated(8);
        ba(0) =  error_state_updated(9);
        ba(1) =  error_state_updated(10);
        ba(2) =  error_state_updated(11);
        bg(0) =  error_state_updated(12);
        bg(1) =  error_state_updated(13);
        bg(2) =  error_state_updated(14);
        err_states.segment(15,9)<<Eigen::VectorXd::Zero(9);

        _error_states<<err_states.segment(15,15);

        _P = covariances.block<15,15>(15,15);
        // _globalP.block<15,15>(15,15) = covariances.block<15,15>(0,0);
        // _globalP.block<15,15>(15,30) = Eigen::MatrixXd::Identity(15,15);
        _globalP.block<15,15>(30,15) = covariances.block<15,15>(15,0);
        _globalP.block<15,15>(30,30) = covariances.block<15,15>(15,15);
        // _globalP.block<15,15>(30,0) = covariances.block<15,15>(15,15)*P_d2.inverse()*_globalP.block<15,15>(30,0);

        relative_update_done = 1;
        ROS_WARN("Relative Update Done");
        ROS_INFO_STREAM("Range tb1: " << range_update);
        // error = sqrt(pow((true_position1(0)-_x(6)),2)+pow((true_position1(1)-_x(7)),2)+pow((true_position1(2)-_x(8)),2));
        // ROS_INFO("Error: %.4f",error);
      }
    }
  }
}
//
// SERVER
//
bool DekfSensorFusion::calculation(dekf_sensor_fusion::SrvCov::Request &req , dekf_sensor_fusion::SrvCov::Response &res)
{

  state_received << req.poscov.pose.orientation.x,req.poscov.pose.orientation.y,req.poscov.pose.orientation.z,
                 req.poscov.twist.x,req.poscov.twist.y,req.poscov.twist.z,
                 req.poscov.pose.position.x,req.poscov.pose.position.y,req.poscov.pose.position.z,
                 req.poscov.bias.linear.x,req.poscov.bias.linear.y,req.poscov.bias.linear.z,
                 req.poscov.bias.angular.x,req.poscov.bias.angular.y,req.poscov.bias.angular.z;

  err_state_received << req.poscov.err_pose.orientation.x,req.poscov.err_pose.orientation.y,req.poscov.err_pose.orientation.z,
                    req.poscov.err_twist.x,req.poscov.err_twist.y,req.poscov.err_twist.z,
                    req.poscov.err_pose.position.x,req.poscov.err_pose.position.y,req.poscov.err_pose.position.z,
                    req.poscov.err_bias.linear.x,req.poscov.err_bias.linear.y,req.poscov.err_bias.linear.z,
                    req.poscov.err_bias.angular.x,req.poscov.err_bias.angular.y,req.poscov.err_bias.angular.z;

  robot_id_received = req.poscov.robot_id.data;

  MatrixXd receivedCov(15,45);
  int count=0;
  for (int i = 0; i < 15; i++) {
    for (int j = 0; j < 45; j++) {
      receivedCov(i,j) = req.poscov.globalCov[count];
      count++;
    }
  }


  if (initializer == 1) {
    if (robot_name=="tb3_0") {
      if (robot_id_received == "tb3_1" && _range(0)<5) {
        _globalP.block<15,45>(15,0) << receivedCov;
        // ROS_WARN("Relative Update tb1");
        // ROS_INFO("Range tb1: %.4f",_range(0));
        relativeUpdate();
      }
      if(robot_id_received == "tb3_2" && _range(1)<5) {
        _globalP.block<15,45>(30,0) << receivedCov;
        // ROS_WARN("Relative Update tb2");
        // ROS_INFO("Range tb2: %.4f",_range(1));
        relativeUpdate();
      }
    }
    else if (robot_name=="tb3_1") {
      if (robot_id_received == "tb3_0" && _range(0)<5) {
        _globalP.block<15,45>(0,0) << receivedCov;
        // ROS_WARN("Relative Update tb0");
        // ROS_INFO("Range tb0: %.4f",_range(0));
        relativeUpdate();
      }
      if(robot_id_received == "tb3_2" && _range(1)<5) {
        _globalP.block<15,45>(30,0) << receivedCov;
        // ROS_WARN("Relative Update tb2");
        // ROS_INFO("Range tb2: %.4f",_range(1));
        relativeUpdate();
      }
    }
    else if (robot_name=="tb3_2") {
      if (robot_id_received == "tb3_0" && _range(0)<5) {
        _globalP.block<15,45>(0,0) << receivedCov;
        // ROS_WARN("Relative Update tb0");
        // ROS_INFO("Range tb0: %.4f",_range(0));
        relativeUpdate();
      }
      if(robot_id_received == "tb3_1" && _range(1)<5) {
        _globalP.block<15,45>(15,0) << receivedCov;
        // ROS_WARN("Relative Update tb1");
        // ROS_INFO("Range tb1: %.4f",_range(1));
        relativeUpdate();
      }
    }

  }
  return true;
}
//
// CLIENT
//
void DekfSensorFusion::SendCovariance()
{

  dekf_sensor_fusion::SrvCov srv_cov_share;

  pose_.orientation.x = _x[0];
  pose_.orientation.y = _x[1];
  pose_.orientation.z = _x[2];
  twist_.x =            _x[3];
  twist_.y =            _x[4];
  twist_.z =            _x[5];
  pose_.position.x =    _x[6];
  pose_.position.y =    _x[7];
  pose_.position.z =    _x[8];
  bias_.linear.x =      _x[9];
  bias_.linear.y =      _x[10];
  bias_.linear.z =      _x[11];
  bias_.angular.x =     _x[12];
  bias_.angular.y =     _x[13];
  bias_.angular.z =     _x[14];
  err_pose_.orientation.x = _error_states[0];
  err_pose_.orientation.y = _error_states[1];
  err_pose_.orientation.z = _error_states[2];
  err_twist_.x =            _error_states[3];
  err_twist_.y =            _error_states[4];
  err_twist_.z =            _error_states[5];
  err_pose_.position.x =    _error_states[6];
  err_pose_.position.y =    _error_states[7];
  err_pose_.position.z =    _error_states[8];
  err_bias_.linear.x =      _error_states[9];
  err_bias_.linear.y =      _error_states[10];
  err_bias_.linear.z =      _error_states[11];
  err_bias_.angular.x =     _error_states[12];
  err_bias_.angular.y =     _error_states[13];
  err_bias_.angular.z =     _error_states[14];

  srv_cov_share.request.poscov.pose = pose_;
  srv_cov_share.request.poscov.twist = twist_;
  srv_cov_share.request.poscov.bias = bias_;
  srv_cov_share.request.poscov.err_pose = err_pose_;
  srv_cov_share.request.poscov.err_twist = err_twist_;
  srv_cov_share.request.poscov.err_bias = err_bias_;

  srv_cov_share.request.poscov.robot_id.data = robot_name;

  state_sent << pose_.orientation.x,pose_.orientation.y,pose_.orientation.z,twist_.x,twist_.y,twist_.z,pose_.position.x,pose_.position.y,pose_.position.z,bias_.linear.x,bias_.linear.y,bias_.linear.z,bias_.angular.x,bias_.angular.y,bias_.angular.z;
  err_state_sent << err_pose_.orientation.x,err_pose_.orientation.y,err_pose_.orientation.z,err_twist_.x,err_twist_.y,err_twist_.z,err_pose_.position.x,err_pose_.position.y,err_pose_.position.z,err_bias_.linear.x,err_bias_.linear.y,err_bias_.linear.z,err_bias_.angular.x,err_bias_.angular.y,err_bias_.angular.z;

  MatrixXd sender(15,45);
  std::vector<double> senderV(675);

  if (robot_name=="tb3_0") {
    sender = _globalP.block<15,45>(0,0);
  }
  else if (robot_name=="tb3_1") {
    sender = _globalP.block<15,45>(15,0);
  }
  else if (robot_name=="tb3_2") {
    sender = _globalP.block<15,45>(30,0);
  }

  int count=0;
  for (int i = 0; i < 15; i++) {
    for (int j = 0; j < 45; j++) {
      senderV[count]  = sender(i,j);
      count++;
    }
  }

  srv_cov_share.request.poscov.globalCov = senderV;

  if(!dekf_sensor_fusion_client_1.call(srv_cov_share) && _range(0) < 5)
  {

      if (robot_name=="tb3_0") {
        ROS_ERROR("Failed to call Service tb1");
        ROS_INFO("Range tb1: %.4f",_range(0));
      }
      else if (robot_name=="tb3_1") {
        ROS_ERROR("Failed to call Service tb0");
        ROS_INFO("Range tb0: %.4f",_range(0));
      }
      else if (robot_name=="tb3_2") {
        ROS_ERROR("Failed to call Service tb0");
        ROS_INFO("Range tb0: %.4f",_range(0));
      }
  }

  if(!dekf_sensor_fusion_client_2.call(srv_cov_share) && _range(1) < 5)
  {

      if (robot_name=="tb3_0") {
        ROS_ERROR("Failed to call Service tb2");
        ROS_INFO("Range tb2: %.4f",_range(1));
      }
      else if (robot_name=="tb3_1") {
        ROS_ERROR("Failed to call Service tb2");
        ROS_INFO("Range tb2: %.4f",_range(1));
      }
      else if (robot_name=="tb3_2") {
        ROS_ERROR("Failed to call Service tb1");
        ROS_INFO("Range tb1: %.4f",_range(1));
      }
  }

  if (robot_name=="tb3_0") {
    if (_range(0)>5) {
      ROS_ERROR("Out of Range tb1");
      ROS_INFO("Range tb1: %.4f",_range(0));
    }
    else if (_range(1)>5) {
      ROS_ERROR("Out of Range tb2");
      ROS_INFO("Range tb2: %.4f",_range(1));
    }
  }
  else if (robot_name=="tb3_1") {
    if (_range(0)>5) {
      ROS_ERROR("Out of Range tb0");
      ROS_INFO("Range tb0: %.4f",_range(0));
    }
    else if (_range(1)>5) {
      ROS_ERROR("Out of Range tb2");
      ROS_INFO("Range tb2: %.4f",_range(1));
    }
  }
  else if (robot_name=="tb3_2") {
    if (_range(0)>5) {
      ROS_ERROR("Out of Range tb0");
      ROS_INFO("Range tb0: %.4f",_range(0));
    }
    else if (_range(1)>5) {
      ROS_ERROR("Out of Range tb1");
      ROS_INFO("Range tb1: %.4f",_range(1));
    }
  }

}
//
// INITIALIZATION FORMULAS
//
void DekfSensorFusion::true_drone0Callback(const nav_msgs::Odometry::ConstPtr &msg)
{

  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double z = msg->pose.pose.position.z;
  double orientx = msg->pose.pose.orientation.x;
  double orienty = msg->pose.pose.orientation.y;
  double orientz = msg->pose.pose.orientation.z;
  double orientw = msg->pose.pose.orientation.w;
  double velx = msg->twist.twist.linear.x;
  double vely = msg->twist.twist.linear.y; //0
  double velz = msg->twist.twist.linear.z; //0
  tf::Quaternion q(orientx,orienty,orientz,orientw);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  true_position0 << x,y,z,roll,pitch,yaw,velx,vely,velz;
  truths_0 = 1;

}
void DekfSensorFusion::true_drone1Callback(const nav_msgs::Odometry::ConstPtr &msg)
{

  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double z = msg->pose.pose.position.z;
  double orientx = msg->pose.pose.orientation.x;
  double orienty = msg->pose.pose.orientation.y;
  double orientz = msg->pose.pose.orientation.z;
  double orientw = msg->pose.pose.orientation.w;
  double velx = msg->twist.twist.linear.x;
  double vely = msg->twist.twist.linear.y; //0
  double velz = msg->twist.twist.linear.z; //0
  tf::Quaternion q(orientx,orienty,orientz,orientw);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  true_position1 << x,y,z,roll,pitch,yaw,velx,vely,velz;
  truths_1 = 1;


}
void DekfSensorFusion::true_drone2Callback(const nav_msgs::Odometry::ConstPtr &msg)
{

  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double z = msg->pose.pose.position.z;
  double orientx = msg->pose.pose.orientation.x;
  double orienty = msg->pose.pose.orientation.y;
  double orientz = msg->pose.pose.orientation.z;
  double orientw = msg->pose.pose.orientation.w;
  double velx = msg->twist.twist.linear.x;
  double vely = msg->twist.twist.linear.y; //0
  double velz = msg->twist.twist.linear.z; //0
  tf::Quaternion q(orientx,orienty,orientz,orientw);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  true_position2 << x,y,z,roll,pitch,yaw,velx,vely,velz;
  truths_2 = 1;

}
void DekfSensorFusion::vel_command_tb0Callback(const geometry_msgs::Twist::ConstPtr &msg)
{

  double linear = msg->linear.x;
  double angular = msg->angular.z;

  zupt_command_0 << linear,angular;

}
void DekfSensorFusion::vel_command_tb1Callback(const geometry_msgs::Twist::ConstPtr &msg)
{

  double linear = msg->linear.x;
  double angular = msg->angular.z;

  zupt_command_1 << linear,angular;

}
void DekfSensorFusion::vel_command_tb2Callback(const geometry_msgs::Twist::ConstPtr &msg)
{

  double linear = msg->linear.x;
  double angular = msg->angular.z;

  zupt_command_2 << linear,angular;

}
void DekfSensorFusion::initialization()
{

if (truths_0==1 && truths_1==1 && truths_2==1) {
  if (robot_name == "tb3_0") {
    _pos << true_position0(0),true_position0(1),true_position0(2);
    _attitude << true_position0(3),true_position0(4),true_position0(5);
    _x << _attitude(0),_attitude(1),_attitude(2),_vel(0),_vel(1),_vel(2),_pos(0),_pos(1),_pos(2),ba(0),ba(1),ba(2),bg(0),bg(1),bg(2);
    initializer = 1;
  }
  else if (robot_name == "tb3_1") {
    _pos << true_position1(0),true_position1(1),true_position1(2);
    _attitude << true_position1(3),true_position1(4),true_position1(5);
    _x << _attitude(0),_attitude(1),_attitude(2),_vel(0),_vel(1),_vel(2),_pos(0),_pos(1),_pos(2),ba(0),ba(1),ba(2),bg(0),bg(1),bg(2);
    initializer = 1;
  }
  else if (robot_name == "tb3_2") {
    _pos << true_position2(0),true_position2(1),true_position2(2);
    _attitude << true_position2(3),true_position2(4),true_position2(5);
    _x << _attitude(0),_attitude(1),_attitude(2),_vel(0),_vel(1),_vel(2),_pos(0),_pos(1),_pos(2),ba(0),ba(1),ba(2),bg(0),bg(1),bg(2);
    initializer = 1;
  }
}

}
//
// PROPAGATION STATE FORMULAS
//
Matrix3d DekfSensorFusion::_euler2dcmV(double phi, double theta, double psi)
{
  // from Titterton and Weston & adapted from MS Braasch Matlab toolbox
  double ph = phi;
  double th = theta;
  double ps = psi;

  double cps = cos(ps);
  double sps = sin(ps);
  double cth = cos(th);
  double sth = sin(th);
  double cph = cos(ph);
  double sph = sin(ph);

  Matrix3d C1;
  C1 << cps, sps, 0, -sps, cps, 0, 0, 0, 1;

  Matrix3d C2;
  C2 << cth, 0, -sth, 0, 1, 0, sth, 0, cth;

  Matrix3d C3;
  C3 << 1, 0, 0, 0, cph, sph, 0, -sph, cph;

  Matrix3d DCMnb = C3 * C2 * C1;
return DCMnb;
}
Matrix3d DekfSensorFusion::_euler2dcm(Vector3d eulVec)
{

  // from Titterton and Weston & adapted from MS Braasch Matlab toolbox
  double ph = eulVec(0);
  double th = eulVec(1);
  double ps = eulVec(2);

  double cps = cos(ps);
  double sps = sin(ps);
  double cth = cos(th);
  double sth = sin(th);
  double cph = cos(ph);
  double sph = sin(ph);

  Matrix3d C1;
  C1 << cps, sps, 0, -sps, cps, 0, 0, 0, 1.0;

  Matrix3d C2;
  C2 << cth, 0, -sth, 0, 1.0, 0, sth, 0, cth;

  Matrix3d C3;
  C3 << 1.0, 0, 0, 0, cph, sph, 0, -sph, cph;

  return C3 * C2 * C1;

}

  tf::Matrix3x3 DekfSensorFusion::_euler2dcmTF(tf::Vector3 attVec)
{
  // from Titterton and Weston & adapted from MS Braasch Matlab toolbox
  // double ph = attVec(0);
  // double th = attVec(1);
  // double ps = attVec(2);
  //
  // double cps = cos(ps);
  // double sps = sin(ps);
  // double cth = cos(th);
  // double sth = sin(th);
  // double cph = cos(ph);
  // double sph = sin(ph);

  tf::Matrix3x3 C1(cos(attVec[2]), sin(attVec[2]), 0.0, -sin(attVec[2]), cos(attVec[2]), 0.0, 0.0, 0.0, 1.0);
  // cps=cos(attVec(2))
  // sps=sin(attVec(2))
  // cth=cos(attVec(1))
  // sth=sin(attVec(1))
  // cph=cos(attVec(0))
  // sph=sin(attVec(0))

  tf::Matrix3x3 C2(cos(attVec[1]), 0.0, -sin(attVec[1]), 0.0, 1.0, 0.0, sin(attVec[1]), 0.0, cos(attVec[1]));

  tf::Matrix3x3 C3(1.0, 0.0, 0.0, 0.0, cos(attVec[0]), sin(attVec[0]), 0.0, -sin(attVec[0]), cos(attVec[0]));

  tf::Matrix3x3 DCMnb = C3 * C2 * C1;
return DCMnb;
}

Vector3d DekfSensorFusion::_dcm2euler(Matrix3d Cbn)
{

  // from Titterton and Weston & adapted from MS Braasch Matlab toolbox

  Vector3d eulVec;
  eulVec(0) = atan2(Cbn(2, 1), Cbn(2, 2));
  eulVec(1) = asin(-Cbn(2, 0));
  eulVec(2) = atan2(Cbn(1, 0), Cbn(0, 0));
  for (int i = 0; i < 3; i++) {
    if (std::isnan(eulVec(i))) {
      eulVec = ArrayXd::Zero(3);
    }
  }
  return eulVec;

}
Vector4d DekfSensorFusion::_dcm2qua(Matrix3d DCMbn)
{

  Vector4d qua;

  if ((1.0 + DCMbn(0, 0) + DCMbn(1, 1) + DCMbn(2, 2)) < 0) {
    qua(0) = 1;
    qua(1) = 0;
    qua(2) = 0;
    qua(3) = 0;

  } else {
    qua(0) = 0.5 * sqrt(1.0 + DCMbn(0, 0) + DCMbn(1, 1) + DCMbn(2, 2));

    double tmp = 1.0 / (4.0 * qua(0));

    qua(1) = tmp * (DCMbn(2, 1) - DCMbn(1, 2));
    qua(2) = tmp * (DCMbn(0, 2) - DCMbn(2, 0));
    qua(3) = tmp * (DCMbn(1, 0) - DCMbn(0, 1));

  }
  return qua;

}
Matrix3d DekfSensorFusion::_qua2dcm(Vector4d qua)
{

  Matrix3d DCM;

  DCM(0, 0) =
      qua(0) * qua(0) + qua(1) * qua(1) - qua(2) * qua(2) - qua(3) * qua(3);
  DCM(0, 1) = 2.0 * (qua(1) * qua(2) - qua(0) * qua(3));
  DCM(0, 2) = 2.0 * (qua(1) * qua(3) + qua(0) * qua(2));
  DCM(1, 0) = 2.0 * (qua(1) * qua(2) + qua(0) * qua(3));
  DCM(1, 1) =
      qua(0) * qua(0) - qua(1) * qua(1) + qua(2) * qua(2) - qua(3) * qua(3);
  DCM(1, 2) = 2.0 * (qua(2) * qua(3) - qua(0) * qua(1));
  DCM(2, 0) = 2.0 * (qua(1) * qua(3) - qua(0) * qua(2));
  DCM(2, 1) = 2.0 * (qua(2) * qua(3) + qua(0) * qua(1));
  DCM(2, 2) = qua(0) * qua(0) - qua(1) * qua(1) - qua(2) * qua(2) + qua(3) * qua(3);
  return DCM;

}
Matrix3d DekfSensorFusion::_skewsym(Vector3d vec)
{

  Matrix3d skewMat;

  skewMat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;

  return skewMat;

}
//
// PSEUDOMEASUREMENT UPDATES FORMULAS
//
void DekfSensorFusion::zeroUpdate()
{
   //check again for velocity old vs velocity -as all states
Vector3d z_zaru;
Vector3d z_zupt;
Matrix3d Cnb = _euler2dcmV(_attitude(0),_attitude(1),_attitude(2));
// std::cout << "ZUPT" << '\n' << _P <<'\n';

        z_zaru = -_imu_gyro.transpose();
        z_zupt = -V_old;
        Eigen::Matrix<double, 6, 1> z_zero;
        z_zero.segment(0,3) <<z_zaru;
        z_zero.segment(3,3) <<z_zupt;
        K_zero = _P * H_zero.transpose() * (H_zero * _P * H_zero.transpose() + R_zero).inverse();
        _error_states = _error_states + K_zero * (z_zero  - (H_zero * _error_states));
        _attitude = _dcm2euler((Eigen::MatrixXd::Identity(3,3)- _skewsym(_error_states.segment(0,3)))*Cnb.transpose());
        V_old = V_old-_error_states.segment(3,3);
        Pos_old = Pos_old -_error_states.segment(6,3);
        _error_states.segment(0,9)<<Eigen::VectorXd::Zero(9);

        _P=(Eigen::MatrixXd::Identity(15,15) - K_zero * H_zero) * _P * ( Eigen::MatrixXd::Identity(15,15) - K_zero * H_zero ).transpose() + K_zero * R_zero * K_zero.transpose();
//END - ZERO UPDATES
// std::cout << "vel inside zupt" << '\n' << V_old <<'\n';

return;
}
void DekfSensorFusion::nonHolonomicUpdate()
{
Matrix3d Cnb = _euler2dcmV(_attitude(0),_attitude(1),_attitude(2));
Vector3d lf2b(0.0, 0.0, 0.272);
z_holo.row(0) = -eye3.row(1)*(Cnb*V_old-_skewsym(_imu_gyro)*lf2b); //z31
z_holo.row(1) = -eye3.row(2)*(Cnb*V_old-_skewsym(_imu_gyro)*lf2b); //z41
H_holo.row(0) << zeros3.row(0), -eye3.row(1)*Cnb, zeros3.row(0), zeros3.row(0), zeros3.row(0); //h32
H_holo.row(1) << zeros3.row(0), -eye3.row(2)*Cnb, zeros3.row(0), zeros3.row(0), zeros3.row(0); //h42
        if (abs(_imu_gyro[2]>0.1)) {
          R_holoS << 0.05;
          z_holoS << -eye3.row(2)*(Cnb*V_old-_skewsym(_imu_gyro)*lf2b);
          H_holoS << zeros3.row(0), -eye3.row(2)*Cnb, zeros3.row(0), zeros3.row(0), zeros3.row(0);
          double tempVar1= H_holoS * _P * H_holoS.transpose();
          double tempVar2= 1/(tempVar1+0.05);
          Vector15 tempvar3= _P * H_holoS.transpose();
          Vector15 tempVar4= tempvar3*tempVar2;
           K_holoS=tempVar4;
           _error_states = _error_states + K_holoS* (z_holoS  - (H_holoS * _error_states));
           _attitude = _dcm2euler((Eigen::MatrixXd::Identity(3,3)- _skewsym(_error_states.segment(0,3)))*Cnb.transpose());
           V_old = V_old-_error_states.segment(3,3);
           Pos_old = Pos_old -_error_states.segment(6,3);
           _P=(Eigen::MatrixXd::Identity(15,15) - K_holoS * H_holoS) * _P* ( Eigen::MatrixXd::Identity(15,15) - K_holoS * H_holoS).transpose() + K_holoS * R_holoS * K_holoS.transpose();
        }
        else {
          K_holo = _P * H_holo.transpose() * (H_holo * _P * H_holo.transpose() + R_holo).inverse();
          _error_states = _error_states + K_holoS* (z_holoS  - (H_holoS * _error_states));
          _attitude = _dcm2euler((Eigen::MatrixXd::Identity(3,3)- _skewsym(_error_states.segment(0,3)))*Cnb.transpose());
          V_old = V_old-_error_states.segment(3,3);
          Pos_old = Pos_old -_error_states.segment(6,3);
          _error_states.segment(0,9)<<Eigen::VectorXd::Zero(9);
          _P=(Eigen::MatrixXd::Identity(15,15) - K_holoS * H_holoS) * _P* ( Eigen::MatrixXd::Identity(15,15) - K_holoS * H_holoS).transpose() + K_holoS * R_holoS * K_holoS.transpose();
  }


return;
}
void DekfSensorFusion::calculateProcessNoiseINS()
{
  // double sig_gyro_inRun = 1.6*3.14/180/3600; //rad/s -- standard deviation of the gyro dynamic biases
  // double sig_ARW = 10*(3.14/180)*sqrt(3600)/3600;; //rad -- standard deviation of the noise on the gyro angular rate measurement 10-0.2
  //
  // double sig_accel_inRun = (3.2e-5)*9.81; // m/s -- standard deviation of the accelerometer dynamic biases
  // double sig_VRW = 10*sqrt(3600)/3600; //m/s -- standard deviation of the noise on the accelerometer specific force measurement 10.
  double sig_gyro_inRun = 0; //0.0000008; //rad/s -- standard deviation of the gyro dynamic biases
  double sig_ARW = 2.0e-4; //rad -- standard deviation of the noise on the gyro angular rate measurement 10-0.2

  double sig_accel_inRun = 0;//0.001; // m/s -- standard deviation of the accelerometer dynamic biases
  double sig_VRW = 1.7e-2; //m/s -- standard deviation of the noise on the accelerometer specific force measurement 10.

  //following 14.2.6 of Groves
  double Srg= pow(sig_ARW,2)*_dt; // PSD of the gyro noise
  double Sra= pow(sig_VRW,2)*_dt; // PSD of the acce noise
  double Sbad=pow(sig_accel_inRun,2); // accelerometer bias variation PSD
  double Sbgd=pow(sig_gyro_inRun,2); // gyro bias variation PSD

  //Simplified -> eq 14.82 pg 592

      Eigen::Matrix <double, 15, 15> Q(15,15);
      // Q<<Srg*_dt*Eigen::Matrix3d::Identity(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),
      // Eigen::Matrix3d::Zero(3,3),Sra*_dt*Eigen::Matrix3d::Identity(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),
      // Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),
      // Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Sbad*_dt*Eigen::Matrix3d::Identity(3,3),Eigen::Matrix3d::Zero(3,3),
      // Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Sbgd*_dt*Eigen::Matrix3d::Identity(3,3);

      Q<<Srg*Eigen::Matrix3d::Identity(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),
      Eigen::Matrix3d::Zero(3,3),Sra*Eigen::Matrix3d::Identity(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),
      Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),
      Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Sbad*Eigen::Matrix3d::Identity(3,3),Eigen::Matrix3d::Zero(3,3),
      Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Sbgd*Eigen::Matrix3d::Identity(3,3);

      _Q_ins=Q;

}
//
// PUBLISHERS
//
void DekfSensorFusion::publishOdom_()
{
  nav_msgs::Odometry updatedOdom;

  updatedOdom.header.stamp = ros::Time::now();
  updatedOdom.header.frame_id = "odom";
  updatedOdom.child_frame_id = "base_footprint";

  // tf::Vector3 attVec(_x[0],_x[1],_x[2]);
  // tf::Matrix3x3 Rnb_ = _euler2dcmTF(attVec);
  // Rbn_=Rnb_.transpose();
  // tf::Quaternion qup;
  // qup.normalize();
  // Rbn_.getRotation(qup);



  updatedOdom.pose.pose.position.x = _x[6];
  updatedOdom.pose.pose.position.y = _x[7];
  updatedOdom.pose.pose.position.z = _x[8];

  // updatedOdom.pose.pose.orientation.x = qup.x();
  // updatedOdom.pose.pose.orientation.y = qup.y();
  // updatedOdom.pose.pose.orientation.z = qup.z();
  // updatedOdom.pose.pose.orientation.w = qup.w();
  // put body_vel


  // Matrix3d Cnb = _euler2dcmV(_x[0],_x[1],_x[2]);
  // _x.segment(3,3) = Cnb.transpose() * _x.segment(3,3);


  updatedOdom.twist.twist.linear.x = _x[3];
  updatedOdom.twist.twist.linear.y = _x[4];
  updatedOdom.twist.twist.linear.z = _x[5];

  updatedOdom.pose.covariance[0] = _x[6]+3*sqrt(_P(6, 6));
  updatedOdom.pose.covariance[1] = _x[6]-3*sqrt(_P(6, 6));
  updatedOdom.pose.covariance[2] = _x[7]+3*sqrt(_P(7, 7));
  updatedOdom.pose.covariance[3] = _x[7]-3*sqrt(_P(7, 7));

  updatedOdom.pose.covariance[4] = _x[6]-true_position1(0);
  updatedOdom.pose.covariance[5] = _x[7]-true_position1(1);
  updatedOdom.pose.covariance[6] = _x[8]-true_position1(2);

  updatedOdom.pose.covariance[7] = _x[6]-true_position2(0);
  updatedOdom.pose.covariance[8] = _x[7]-true_position2(1);
  updatedOdom.pose.covariance[9] = _x[8]-true_position2(2);

  updatedOdom.pose.covariance[10] = _x[9];
  updatedOdom.pose.covariance[11] = _x[10];
  updatedOdom.pose.covariance[12] = _x[11];
  updatedOdom.pose.covariance[13] = _x[12];
  updatedOdom.pose.covariance[14] = _x[13];
  updatedOdom.pose.covariance[15] = _x[14];

  updatedOdom.pose.covariance[16] = _x[0]; //roll
  updatedOdom.pose.covariance[17] = _x[1]; //pitch
  updatedOdom.pose.covariance[18] = _x[2]; //yaw

  updatedOdom.pose.covariance[19] = true_position1(3); //roll ,,;
  updatedOdom.pose.covariance[20] = true_position1(4); //pitch
  updatedOdom.pose.covariance[21] = true_position1(5); //yaw
  pubOdom_.publish(updatedOdom);

  // geometry_msgs::TransformStamped odom_trans;
  // odom_trans.header.stamp = updatedOdom.header.stamp;
  // odom_trans.header.frame_id = updatedOdom.header.frame_id;
  // odom_trans.child_frame_id = updatedOdom.child_frame_id;
  // odom_trans.transform.translation.x = updatedOdom.pose.pose.position.x;
  // odom_trans.transform.translation.y = updatedOdom.pose.pose.position.y;
  // odom_trans.transform.translation.z = updatedOdom.pose.pose.position.z;
  // odom_trans.transform.rotation = updatedOdom.pose.pose.orientation;
  // // odom_trans.transform.rotation.y = updatedOdom.pose.pose.orientation.y;
  // // odom_trans.transform.rotation.z = updatedOdom.pose.pose.orientation.z;
  // // odom_trans.transform.rotation.w = updatedOdom.pose.pose.orientation.w;
  // odom_broadcaster_.sendTransform(odom_trans);


}
void DekfSensorFusion::publishRange_()
{
  if (robot_name=="tb3_0") {
    _range(0) = sqrt(pow(true_position0(0)-true_position1(0),2)+pow(true_position0(1)-true_position1(1),2)+pow(true_position0(2)-true_position1(2),2));
    _range(1) = sqrt(pow(true_position0(0)-true_position2(0),2)+pow(true_position0(1)-true_position2(1),2)+pow(true_position0(2)-true_position2(2),2));
  }
  else if (robot_name=="tb3_1") {
    _range(0) = sqrt(pow(true_position1(0)-true_position0(0),2)+pow(true_position1(1)-true_position0(1),2)+pow(true_position1(2)-true_position0(2),2));
    _range(1) = sqrt(pow(true_position1(0)-true_position2(0),2)+pow(true_position1(1)-true_position2(1),2)+pow(true_position1(2)-true_position2(2),2));
  }
  else if (robot_name=="tb3_2") {
    _range(0) = sqrt(pow(true_position2(0)-true_position0(0),2)+pow(true_position2(1)-true_position0(1),2)+pow(true_position2(2)-true_position0(2),2));
    _range(1) = sqrt(pow(true_position2(0)-true_position1(0),2)+pow(true_position2(1)-true_position1(1),2)+pow(true_position2(2)-true_position1(2),2));
  }

  std::vector<double> _rangeV(2);
  _rangeV[0]  = _range(0);
  _rangeV[1]  = _range(1);
  std_msgs::Float64MultiArray range_to_drone;
  range_to_drone.data = _rangeV;
  pubRange_.publish(range_to_drone);

}
//
// MAIN:
//
int main(int argc, char **argv)
{
  ros::init(argc, argv, "dekf_sensor_fusion");
  ros::NodeHandle nh("");

  ROS_INFO("ready service server");
  ros::Rate rate(1); //TODO make sure if it affects the AsyncSpinner or not
  ros::AsyncSpinner spinner(0);
  spinner.start();
  DekfSensorFusion dekf_sensor_fusion(nh);
  int counter = 0;

  while (ros::ok())
  {

    ROS_INFO("----------------");
    if (dekf_sensor_fusion.initializer == 0 && dekf_sensor_fusion.truths_0 == 1 && dekf_sensor_fusion.truths_1 == 1 && dekf_sensor_fusion.truths_2 == 1) {
      dekf_sensor_fusion.initialization();
    }
    if (dekf_sensor_fusion.initializer == 1)
    {
      dekf_sensor_fusion.publishRange_();
      if (dekf_sensor_fusion._range(0) < 5.0 || dekf_sensor_fusion._range(1) < 5.0)
      {
        dekf_sensor_fusion.SendCovariance();
      }
      else
      {
        if (dekf_sensor_fusion.robot_name=="tb3_0")
        {
          ROS_ERROR_STREAM("Out of Range tb1");
          ROS_INFO("Range tb1: %.4f",dekf_sensor_fusion._range(0));
          ROS_ERROR_STREAM("Out of Range tb2");
          ROS_INFO("Range tb2: %.4f",dekf_sensor_fusion._range(1));
        }
        else if (dekf_sensor_fusion.robot_name=="tb3_1")
        {
          ROS_ERROR_STREAM("Out of Range tb0");
          ROS_INFO("Range tb0: %.4f",dekf_sensor_fusion._range(0));
          ROS_ERROR_STREAM("Out of Range tb2");
          ROS_INFO("Range tb2: %.4f",dekf_sensor_fusion._range(1));
        }
        else if (dekf_sensor_fusion.robot_name=="tb3_2")
        {
          ROS_ERROR_STREAM("Out of Range tb0");
          ROS_INFO("Range tb0: %.4f",dekf_sensor_fusion._range(0));
          ROS_ERROR_STREAM("Out of Range tb1");
          ROS_INFO("Range tb1: %.4f",dekf_sensor_fusion._range(1));
        }
      }
    }
    // ros::spinOnce();
    rate.sleep(); //TODO make sure if it affects the AsyncSpinner or not
    counter++;

  }
  return 0;
}
