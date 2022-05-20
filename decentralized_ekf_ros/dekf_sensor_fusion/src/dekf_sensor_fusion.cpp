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
    dekf_sensor_fusion_client =
    nh.serviceClient<dekf_sensor_fusion::SrvCov>("/tb3_1/covariance_srv");
  }
  else if (robot_name=="tb3_1") {
    dekf_sensor_fusion_client =
    nh.serviceClient<dekf_sensor_fusion::SrvCov>("/tb3_0/covariance_srv");
  }

  pubOdom_ = nh_.advertise<nav_msgs::Odometry>(
      "localization/odometry/sensor_fusion", 1);
  pubRange_ = nh_.advertise<std_msgs::Float64>("range",1);
  pubResidual_ = nh_.advertise<std_msgs::Float64>("residual",1);

  Vector3d eul;
  eul<< 0.0,0.0,0.0;
  _Cnb = _euler2dcm(eul);
  _vel << 0,0,0;
  ba << 2e-11,3e-11,5.7e-11;  // TODO: Check values later
  bg << -3.64e-05,0.000142,1.51e-05;  // TODO: Check values later

  if (robot_name=="tb3_0") {
    _pos << 0,0,0;
  }
  else if (robot_name=="tb3_1") {
    _pos << 0,0,0;
  }

  _attitude = _dcm2euler(_Cnb.transpose());

  _x << _attitude(0),_attitude(1),_attitude(2),_vel(0),_vel(1),_vel(2),_pos(0),_pos(1),_pos(2),ba(0),ba(1),ba(2),bg(0),bg(1),bg(2);

  Eigen::VectorXd _P_initVal(15);
  _P_initVal << 0.01,0.01,0.01,0.1,0.1,0.1,0.5,0.5,0.5,0,0,0,0,0,0;
  _P = _P_initVal.asDiagonal();
  // sigma_ij=MatrixXd::Zero(9, 9);
  // sigma_ji=_P;
  _Q_ins = _P;
  _globalP = MatrixXd::Zero(30, 30);
  _globalP.block<15,15>(0,0) = _P;
  _globalP.block<15,15>(15,15) = _P;


  H_gps = MatrixXd::Zero(6,15);
  // H_gps.bottomRightCorner(6,6) = MatrixXd::Zero(6,6)-MatrixXd::Identity(6,6);
  H_gps.block<6,6>(0,5) = MatrixXd::Zero(6,6)-MatrixXd::Identity(6,6);

  Eigen::VectorXd R_gpsVal(6);
  R_gpsVal << std::pow(0.1,2), std::pow(0.1,2), std::pow(0.1,2), std::pow(0.4,2), std::pow(0.4,2), std::pow(0.4,2);
  R_gps = R_gpsVal.asDiagonal();

  R_range << 0.2*0.2; // TODO: Value?
  initializer = 0;
  truths_1 = 0;
  truths_2 = 0;
  relative_update_done = 0;
  gps_update_done = 0;
  _range= 100; //TODO if the range is not given it starts with '0'

  sub_imu = nh.subscribe("imu", 10, &DekfSensorFusion::imuCallback, this);
  sub_GPS = nh.subscribe("gps", 1, &DekfSensorFusion::gpsCallback,this); ///uav0/mavros/global_position/local // this is not the specific gps topic
  true_drone1 = nh.subscribe("/tb3_0/truth", 1, &DekfSensorFusion::true_drone1Callback, this);
  true_drone2 = nh.subscribe("/tb3_1/truth", 1, &DekfSensorFusion::true_drone2Callback, this);

}

// True position for drone 1
void DekfSensorFusion::true_drone1Callback(const nav_msgs::Odometry::ConstPtr &msg)
{

  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double z = msg->pose.pose.position.z;
  double orientx = msg->pose.pose.orientation.x;
  double orienty = msg->pose.pose.orientation.y;
  double orientz = msg->pose.pose.orientation.z;
  double orientw = msg->pose.pose.orientation.w;
  tf::Quaternion q(orientx,orienty,orientz,orientw);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  true_position1 << x,y,z;
  truths_1 = 1;

}

// True position for drone 2
void DekfSensorFusion::true_drone2Callback(const nav_msgs::Odometry::ConstPtr &msg)
{

  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double z = msg->pose.pose.position.z;
  double orientx = msg->pose.pose.orientation.x;
  double orienty = msg->pose.pose.orientation.y;
  double orientz = msg->pose.pose.orientation.z;
  double orientw = msg->pose.pose.orientation.w;
  tf::Quaternion q(orientx,orienty,orientz,orientw);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  true_position2 << x,y,z;
  truths_2 = 1;

}

// Initilization for position
void DekfSensorFusion::initialization()
{

if (truths_1==1 && truths_2==1) {
  if (robot_name == "tb3_0") {
    _pos << true_position1(0),true_position1(1),true_position1(2);
    initializer = 1;
  }
  else if (robot_name == "tb3_1") {
    _pos << true_position2(0),true_position2(1),true_position2(2);
    initializer = 1;
  }
}
}

// IMU Prediction
void DekfSensorFusion::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{

  if (initializer == 1) {
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

    double p = msg->angular_velocity.x-bg(0);
    double q = -msg->angular_velocity.y-bg(1);
    double r = -msg->angular_velocity.z-bg(2);
    double ax= msg->linear_acceleration.x-ba(0);
    double ay= msg->linear_acceleration.y-ba(1);
    double az= msg->linear_acceleration.z + 9.81-ba(2);
    _imu_gyro << p, q, r; // TODO: Add bg
    _imu_acce << ax,ay,az; // TODO: Add ba

    Matrix3d Omega_ib;
    Omega_ib << 0, -r, q, r, 0, -p, -q, p, 0;
    Matrix3d I;
    I.setIdentity();
    // Cbn_old =_Cnb.transpose();
    // Cbn_new= Cbn_old * (I + Omega_ib * _dt)
    // Cbn_old=Cbn_new;
    // _attitude = _dcm2euler(Cbn_new);
    Matrix3d Cbn;
    Cbn = _Cnb.transpose() * (I + Omega_ib * _dt); // ignore Earth rate and corriolis terms
    // std::cout << "_dt" << '\n' << _dt << '\n' ;
    //     std::cout << "_Cnb TT" << '\n' << _Cnb.transpose() << '\n' ;
    //         std::cout << "_Cnb" << '\n' << _Cnb << '\n' ;
    //                     std::cout << "Omega_ib" << '\n' << Omega_ib << '\n' ;
    _Cnb = Cbn.transpose();
    _attitude = _dcm2euler(_Cnb.transpose());

    // std::cout << "_attitude: " << '\n' << _attitude << '\n';
    // std::cout << "Ba: " << '\n' << ba << '\n';
    // std::cout << "Bg: " << '\n' << bg << '\n';


    Vector3d V_n_ib;
    V_n_ib=0.5*(Cbn+_Cnb.transpose())*_imu_acce*_dt;

    Vector3d grav_;
    grav_ << 0,0,-9.81;
    Vector3d V_old;
    V_old=_vel+V_n_ib+(grav_)*_dt;
    // std::cout << "_vel: " << '\n' << _vel << '\n';
    // std::cout << "V_old: " << '\n' << V_old << '\n';
    Vector3d Pos_old;
    Pos_old=_pos+(V_old+_vel)*_dt/2.0;
    // std::cout << "_pos: " << '\n' << _pos << '\n';
    // std::cout << "Pos_old: " << '\n' << Pos_old << '\n';
    _vel=V_old;
    _pos=Pos_old;

    if (relative_update_done == 1) {  // TODO: Check
      relative_update_done = 0;
      if (gps_update_done) {
        _attitude(0) = gps_Uatt(0);
        _attitude(1) = gps_Uatt(1);
        _attitude(2)= gps_Uatt(2);
        _vel(0) = gps_Uvel(0);
        _vel(1) = gps_Uvel(1);
        _vel(2)= gps_Uvel(2);
        _pos(0)=gps_Upos(0);
        _pos(1) =gps_Upos(1);
        _pos(2)=gps_Upos(2);
        _P=gps_UP;
        gps_update_done =0;
      }
    }

    MatrixXd I15(15, 15);
    I15.setIdentity();
    MatrixXd STM;
    // starting at (i,j), block of size (p,q),
    // matrix.block(i,j,p,q)  dynamic size block expression
    // matrix.block<p,q>(i,j) fixed size block expression
    Eigen::Matrix <double, 15, 15> F = Eigen::MatrixXd::Zero(15,15);
    F.block<3,3>(3,0) = _skewsym(-_Cnb.transpose()*_imu_acce);
    // F.block<3,3>(3,9) = _Cnb.transpose();
    // F.block<3,3>(0,12) = _Cnb.transpose();

    MatrixXd I3(3, 3);
    I3.setIdentity();
    F.block<3,3>(6,3) = I3;

    STM = I15+F*_dt;

    _x << _attitude(0),_attitude(1),_attitude(2),_vel(0),_vel(1),_vel(2),_pos(0),_pos(1),_pos(2),ba(0),ba(1),ba(2),bg(0),bg(1),bg(2);
    // std::cout << "/* IMU STATE */" << '\n' << _x << '\n';
    // publishOdom_();
    // publishRange_();
    // publishResidual_();
    _x= STM*_x; //State
    // std::cout << "/* IMU STATE after STM */" << '\n' << _x << '\n';
    _P = STM * _P * STM.transpose() + _Q_ins; // Covariance Matrix
    // std::cout << "Local P IN MOTION UPDATE" << '\n' << _P <<'\n';
    //Update Global P

    if (robot_name=="tb3_0") {
      _globalP.block<15,15>(0,0) = _P;
      _globalP.block<15,15>(0,15) = STM*_globalP.block<15,15>(0,15);
      // sigma_ij=STM*sigma_ij; //TODO
      // _globalP.block(0,9,9,9) = sigma_ij;
    }
    else if (robot_name=="tb3_1") {
      _globalP.block<15,15>(15,15) = _P;
      _globalP.block<15,15>(15,0) = STM*_globalP.block<15,15>(15,0);
      // sigma_ij=STM*sigma_ij; //TODO
      // _globalP.block(9,0,9,9) = sigma_ij;
    }

    // ba(0)=ba(0)+_x(9);
    // ba(1)=ba(1)+_x(10);
    // ba(2)=ba(2)+_x(11);
    // bg(0)=bg(0)+_x(12);
    // bg(1)=bg(1)+_x(13);
    // bg(2)=bg(2)+_x(14);

    // std::cout << "Sates: " << '\n' << _x << '\n';
    // std::cout << "Ba: " << '\n' << ba << '\n';
    // std::cout << "Bg: " << '\n' << bg << '\n';

    _x.segment(9,6)<<Eigen::VectorXd::Zero(6);

    publishOdom_();
    publishRange_();

  }

}

// GPS Update
void DekfSensorFusion::gpsCallback(const nav_msgs::Odometry::ConstPtr &msg)
{

  if (initializer == 1) {
    Matrix3d Cnb = _euler2dcm(_attitude);
    Matrix3d Cbn = Cnb.transpose();
    MatrixXd K_gps(15,6);

    // if (robot_name=="tb3_0") {
    //   gps_pos[0] = msg->pose.pose.position.x+1;
    //   gps_pos[1] = msg->pose.pose.position.y;
    // }
    // else if (robot_name=="tb3_1") {
    //   gps_pos[0] = msg->pose.pose.position.x;
    //   gps_pos[1] = msg->pose.pose.position.y+1;
    // }
    gps_pos[0] = msg->pose.pose.position.x;
    gps_pos[1] = msg->pose.pose.position.y;
    gps_pos[2] = msg->pose.pose.position.z;

    gps_vel[0] = msg->twist.twist.linear.x;
    gps_vel[1] = msg->twist.twist.linear.y;
    gps_vel[2] = msg->twist.twist.linear.z;

    z_gps_pos= -gps_pos;
    z_gps_vel= -gps_vel;
    K_gps = _P * H_gps.transpose() * (H_gps * _P * H_gps.transpose() + R_gps).inverse();

    z_gps <<  z_gps_vel[0],z_gps_vel[1],z_gps_vel[2],z_gps_pos[0],z_gps_pos[1],z_gps_pos[2];
    _x = _x + K_gps * (z_gps  - H_gps * _x);

    _attitude = _dcm2euler((Eigen::MatrixXd::Identity(3,3)- _skewsym(_x.segment(0,3)))*Cbn);
    _vel = _x.segment(3,3);
    _pos = _x.segment(6,3);

    _P=(Eigen::MatrixXd::Identity(15,15) - K_gps * H_gps) * _P * ( Eigen::MatrixXd::Identity(15,15) - K_gps * H_gps ).transpose() + K_gps * R_gps * K_gps.transpose();


    if (robot_name=="tb3_0") {
      _globalP.block<15,15>(0,0) = _P;
      _globalP.block<15,15>(0,15) = (Eigen::MatrixXd::Identity(15,15) - K_gps * H_gps) * _globalP.block<15,15>(0,15) * ( Eigen::MatrixXd::Identity(15,15) - K_gps * H_gps ).transpose() + K_gps * R_gps * K_gps.transpose();
      // sigma_ij=(Eigen::MatrixXd::Identity(9,9)-K_gps*H_gps)*sigma_ij; //TODO
      // _globalP.block(0,9,9,9) = sigma_ij;
    }
    else if (robot_name=="tb3_1") {
      _globalP.block<15,15>(15,15) = _P;
      _globalP.block<15,15>(15,0) = (Eigen::MatrixXd::Identity(15,15) - K_gps * H_gps) * _globalP.block<15,15>(15,0) * ( Eigen::MatrixXd::Identity(15,15) - K_gps * H_gps ).transpose() + K_gps * R_gps * K_gps.transpose();

      // sigma_ij=(Eigen::MatrixXd::Identity(9,9)-K_gps*H_gps)*sigma_ij; //TODO
      // _globalP.block(9,0,9,9) = sigma_ij;
    }


    gps_Uatt=_attitude;
    gps_Uvel=_vel ;
    gps_Upos=_pos;
    gps_UP=_P;

    // if (robot_name=="uav1") {
    // _globalP.block(0,0,9,9) =gps_UP;
    // }
    // else if(robot_name=="uav2") {
    // _globalP.block(9,9,9,9) =gps_UP;
    // }


    // std::cout << "GPS P" << '\n' << gps_UP<< '\n';
    // std::cout << "GPS State" << '\n' << _x << '\n';
    gps_update_done = 1;

    // publishOdom_();
    publishRange_();
    publishResidual_();
  }
}

// VO Update
void DekfSensorFusion::voCallback(const nav_msgs::Odometry::ConstPtr &msg)
{}

// Relative Update
void DekfSensorFusion::relativeUpdate()
{

    MatrixXd covariances(30,30);
    P_d1 = _globalP.block<15,15>(0,0);  //Sigma_ii
    P_d12 = _globalP.block<15,15>(0,15); //Sigma_ij
    P_d21 = _globalP.block<15,15>(15,0); //Sigma_ij.transpose() or Sigma_ji ?
    P_d2 = _globalP.block<15,15>(15,15);  //Sigma_jj

    if (robot_name=="tb3_0") {

      P_corr = P_d12 * P_d21.transpose();
      // state1 = _x; // TODO: Use sent State?
      state1 = state_sent; // Use the state that has been sent
      // std::cout << "State_Sent in Simulate Range" <<'\n'<< state_sent <<'\n';
      state2 = state_received;
      P_corr2 = P_corr.transpose();

      covariances.block<15,15>(0,0) = P_d1;
      // std::cout << "/* P_d1 */" << '\n' << P_d1 <<'\n';
      covariances.block<15,15>(0,15) = P_corr;
      // std::cout << "/* P_corr */" << '\n' << P_corr <<'\n';
      covariances.block<15,15>(15,0) = P_corr2;
      // std::cout << "/* P_corr2 */" << '\n' << P_corr2 <<'\n';
      covariances.block<15,15>(15,15) = P_d2;
      // std::cout << "/* P_d2 */" << '\n' << P_d2 <<'\n';

    }
    else if (robot_name=="tb3_1") {
      P_corr = P_d21 * P_d12.transpose();
      state1 = state_received;
      // state2 =_x; //
      state2 = state_sent; // Use the state that has been sent.
      // std::cout << "State_Sent in Simulate Range" <<'\n'<< state_sent <<'\n';
      P_corr2 = P_corr.transpose();

      covariances.block<15,15>(0,0) = P_d1;
      // std::cout << "/* P_d1 */" << '\n' << P_d1 <<'\n';
      covariances.block<15,15>(0,15) = P_corr2;
      // std::cout << "/* P_corr */" << '\n' << P_corr <<'\n';
      covariances.block<15,15>(15,0) = P_corr;
      // std::cout << "/* P_corr2 */" << '\n' << P_corr2 <<'\n';
      covariances.block<15,15>(15,15) = P_d2;
      // std::cout << "/* P_d2 */" << '\n' << P_d2 <<'\n';
    }

    states << state1(0),state1(1),state1(2),state1(3),state1(4),state1(5),state1(6),state1(7),state1(8),state1(9),state1(10),state1(11),state1(12),state1(13),state1(14),state2(0),state2(1),state2(2),state2(3),state2(4),state2(5),state2(6),state2(7),state2(8),state2(9),state2(10),state2(11),state2(12),state2(13),state2(14);

    // std::cout << "Global P" << '\n' << _globalP <<'\n';
    // std::cout << "Relative P" << '\n' << covariances <<'\n';

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

    res_range = _range - h_range;

    if (abs(res_range) > 500.0) {
      return;

      /* Perform Zupt? */
    }
    else {
      MatrixXd I30(30,30);
      I30.setIdentity();

      states = states + K_range*(_range - H_range*states);
      // std::cout << "states1" <<'\n'<< states + K_range*res_range <<'\n';
      // std::cout << "states2" <<'\n'<< states + K_range*(_range - H_range*states) <<'\n';

      // states = states + K_range*res_range;

      covariances = (I30 - K_range*H_range)*covariances;

          if (robot_name=="tb3_0") {


            // _x = states.segment(0,15);
            range_est = states.segment(0,15);
            Matrix3d Cnb = _euler2dcm(range_est.segment(0,3));
            Matrix3d Cbn = Cnb.transpose();

            // _attitude(0) = (0);
            // _attitude(1) = range_est(1);
            // _attitude(2) = range_est(2);
            _attitude = _dcm2euler((Eigen::MatrixXd::Identity(3,3)- _skewsym(range_est.segment(0,3)))*Cbn);
            _vel(0) = range_est(3);
            _vel(1) = range_est(4);
            _vel(2) = range_est(5);
            _pos(0) = range_est(6);
            _pos(1) = range_est(7);
            _pos(2) = range_est(8);
            ba(0) =  range_est(9);
            ba(1) =  range_est(10);
            ba(2) =  range_est(11);
            bg(0) =  range_est(12);
            bg(1) =  range_est(13);
            bg(2) =  range_est(14);

            _x << _attitude,_vel,_pos,ba,bg;

            _globalP.block<15,15>(0,0) = covariances.block<15,15>(0,0);
            _globalP.block<15,15>(0,15) = Eigen::MatrixXd::Identity(15,15);
            // _globalP.block<15,15>(0,15) = covariances.block<15,15>(0,15);
            _globalP.block<15,15>(15,0) = (covariances.block<15,15>(0,15)).transpose();
            _globalP.block<15,15>(15,15) = covariances.block<15,15>(15,15);

            _P = covariances.block<15,15>(0,0);


            relative_update_done = 1;
            ROS_WARN("Relative Update Done - Range: %.4f",_range);

            error = sqrt(pow((true_position1(0)-_x(6)),2)+pow((true_position1(1)-_x(7)),2)+pow((true_position1(2)-_x(8)),2));
            ROS_INFO("Error = %.4f",error);
          }
          else if (robot_name=="tb3_1") {


            // _x = states.segment(15,15);
            range_est = states.segment(15,15);
            Matrix3d Cnb = _euler2dcm(range_est.segment(0,3));
            Matrix3d Cbn = Cnb.transpose();
            _attitude = _dcm2euler((Eigen::MatrixXd::Identity(3,3)- _skewsym(range_est.segment(0,3)))*Cbn);
            _vel(0) = range_est(3);
            _vel(1) = range_est(4);
            _vel(2) = range_est(5);
            _pos(0) = range_est(6);
            _pos(1) = range_est(7);
            _pos(2) = range_est(8);
            ba(0) =  range_est(9);
            ba(1) =  range_est(10);
            ba(2) =  range_est(11);
            bg(0) =  range_est(12);
            bg(1) =  range_est(13);
            bg(2) =  range_est(14);

            _x << _attitude,_vel,_pos,ba,bg;

            _globalP.block<15,15>(0,0) = covariances.block<15,15>(0,0);
            _globalP.block<15,15>(15,0) = Eigen::MatrixXd::Identity(15,15);
            // _globalP.block<15,15>(15,0) = covariances.block<15,15>(15,0);
            _globalP.block<15,15>(0,15) = (covariances.block<15,15>(15,0)).transpose();
            _globalP.block<15,15>(15,15) = covariances.block<15,15>(15,15);

            _P = covariances.block<15,15>(15,15);


            relative_update_done = 1;
            ROS_WARN("Relative Update Done - Range: %.4f",_range);

            error = sqrt(pow((true_position2(0)-_x(6)),2)+pow((true_position2(1)-_x(7)),2)+pow((true_position2(2)-_x(8)),2));
            ROS_INFO("Error = %.4f",error);
          }
    }

    publishRange_();
    // publishOdom_();
}

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

  srv_cov_share.request.poscov.pose = pose_;
  srv_cov_share.request.poscov.twist = twist_;
  srv_cov_share.request.poscov.bias = bias_;

  state_sent << pose_.orientation.x,pose_.orientation.y,pose_.orientation.z,twist_.x,twist_.y,twist_.z,pose_.position.x,pose_.position.y,pose_.position.z,bias_.linear.x,bias_.linear.y,bias_.linear.z,bias_.angular.x,bias_.angular.y,bias_.angular.z;
  // state_sent =_x;

  MatrixXd sender(15,30);
  std::vector<double> senderV(450);

  if (robot_name=="tb3_0") {
    sender = _globalP.block<15,30>(0,0);
    // std::cout << "sender in send covariance" << '\n' << sender << '\n';
  }
  else if (robot_name=="tb3_1") {
    sender = _globalP.block<15,30>(15,0);
    // std::cout << "sender in send covariance" << '\n' << sender << '\n';
  }
// ROS_INFO("SENT P in send covariance");

  int count=0;
    for (int i = 0; i < 15; i++) {
      for (int j = 0; j < 30; j++) {
        senderV[count]  = sender(i,j);
        count++;
      }
    }

  srv_cov_share.request.poscov.globalCov = senderV;

  if(!dekf_sensor_fusion_client.call(srv_cov_share))
  {
ROS_ERROR("Failed to call service SrvCov");
if (robot_name=="tb3_0") {
  error = sqrt(pow((true_position1(0)-_x(6)),2)+pow((true_position1(1)-_x(7)),2)+pow((true_position1(2)-_x(8)),2));
  ROS_INFO("Error = %.4f",error);
  // std::cout << "State:" << '\n' << _x << '\n';
  // std::cout << "Bias a:" << '\n' << ba << '\n';
  // std::cout << "Bias g:" << '\n' << bg << '\n';
}
else if (robot_name=="tb3_1") {
  error = sqrt(pow((true_position2(0)-_x(6)),2)+pow((true_position2(1)-_x(7)),2)+pow((true_position2(2)-_x(8)),2));
  ROS_INFO("Error = %.4f",error);
  // std::cout << "State:" << '\n' << _x << '\n';
  // std::cout << "Bias a:" << '\n' << ba << '\n';
  // std::cout << "Bias g:" << '\n' << bg << '\n';
}
  }
}

bool DekfSensorFusion::calculation(dekf_sensor_fusion::SrvCov::Request &req , dekf_sensor_fusion::SrvCov::Response &res)
{

  state_received << req.poscov.pose.orientation.x,req.poscov.pose.orientation.y,req.poscov.pose.orientation.z,
                 req.poscov.twist.x,req.poscov.twist.y,req.poscov.twist.z,
                 req.poscov.pose.position.x,req.poscov.pose.position.y,req.poscov.pose.position.z,
                 req.poscov.bias.linear.x,req.poscov.bias.linear.y,req.poscov.bias.linear.z,
                 req.poscov.bias.angular.x,req.poscov.bias.angular.y,req.poscov.bias.angular.z;
                 // state_sent =_x;

  // pose_=req.poscov.pose;
  // res.result=pose_.position.x + pose_.position.y + pose_.position.z;
  // res.success= true;

  MatrixXd receivedCov(15,30);
  int count=0;
  for (int i = 0; i < 15; i++) {
    for (int j = 0; j < 30; j++) {
      receivedCov(i,j) = req.poscov.globalCov[count];
      count++;
    }
  }


  if (robot_name=="tb3_0") {
  _globalP.block<15,30>(15,0) << receivedCov;
  // std::cout << "receiver in calculation" << '\n' << receivedCov << '\n';

// std::cout << "State_Received" <<'\n'<< state_received <<'\n';
  // std::cout << "Local P IN RELATIVE UPDATE" << '\n' << _P <<'\n';
  // ROS_INFO("RECEIVED P");
  // std::cout << "Received E_jj of Robot2" << '\n' << receivedCov.block<15,15>(0,15) <<'\n';
  // std::cout << "Received sigma_ji of Robot2" << '\n' << receivedCov.block<15,15>(0,0) <<'\n';
  // ROS_INFO("RECEIVED P in calculation");
  // std::cout << "RECEIVED P" << '\n' << receivedCov <<'\n';
  // std::cout << "Global P" << '\n' << _globalP <<'\n';

  }
  else if (robot_name=="tb3_1") {
  _globalP.block<15,30>(0,0) << receivedCov;

  // std::cout << "receiver in calculation" << '\n' << receivedCov << '\n';
  // std::cout << "State_Received" <<'\n'<< state_received <<'\n';
  // std::cout << "Local P IN RELATIVE UPDATE" << '\n' << _P <<'\n';
  // ROS_INFO("RECEIVED P");
  // std::cout << "Received E_ii of Robot2" << '\n' << receivedCov.block<15,15>(0,15) <<'\n';
  // std::cout << "Received sigma_ij of Robot2" << '\n' << receivedCov.block<15,15>(0,0) <<'\n';
  // ROS_INFO("RECEIVED P in calculation");

  // std::cout << "RECEIVED P" << '\n' << receivedCov <<'\n';
  // std::cout << "Global P" << '\n' << _globalP <<'\n';
  }

  if (initializer == 1) {
    relativeUpdate();
  }
  return true;
}
// TOOLS
void DekfSensorFusion::_euler2dcmV()
{
  // from Titterton and Weston & adapted from MS Braasch Matlab toolbox
  double ph = _attitude(0);
  double th = _attitude(1);
  double ps = _attitude(2);

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

  _Cnb = C3 * C2 * C1;

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
void DekfSensorFusion::publishOdom_()
{
  nav_msgs::Odometry updatedOdom;

  updatedOdom.header.stamp = ros::Time::now();
  updatedOdom.header.frame_id = "map";
  // updatedOdom.child_frame_id = odometry_child_frame_id;

  // tf::Quaternion qup;
  // qup.normalize();
  // Rbn_.getRotation(qup);
  //
  // double roll, pitch, yaw;
  // Rbn_.getRPY(roll, pitch, yaw);
  // ROS_INFO("RPY in WO %f  %f _x.segment(3,3)
  // _pos
  // updatedOdom.pose.pose.position.x = _pos(0);
  // updatedOdom.pose.pose.position.y = _pos(1);
  // updatedOdom.pose.pose.position.z = _pos(2);

  updatedOdom.pose.pose.position.x = _x[6];
  updatedOdom.pose.pose.position.y = _x[7];
  updatedOdom.pose.pose.position.z = _x[8];

  // _x.segment(3,3)

  // updatedOdom.pose.pose.orientation.x = qup.x();
  // updatedOdom.pose.pose.orientation.y = qup.y();
  // updatedOdom.pose.pose.orientation.z = qup.z();
  // updatedOdom.pose.pose.orientation.w = qup.w();
  // put body_vel

  // tf::Vector3 v_body_ekf;
  // tf::Vector3 v_nav_ekf(x_[3], x_[4], x_[5]);
  // v_body_ekf = Rbn_.transpose() * v_nav_ekf;
  // _vel
  // updatedOdom.twist.twist.linear.x = _vel(0);
  // updatedOdom.twist.twist.linear.y = _vel(1);
  // updatedOdom.twist.twist.linear.z = _vel(2);

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

  pubOdom_.publish(updatedOdom);

}
void DekfSensorFusion::publishRange_()
{
if (initializer == 1 )
{
  _range = sqrt(pow(true_position1(0)-true_position2(0),2)+pow(true_position1(1)-true_position2(1),2)+pow(true_position1(2)-true_position2(2),2));
  std_msgs::Float64 range_to_drone;
  range_to_drone.data = _range;
  // ROS_INFO("RANGE %.2f \n", _range);
  pubRange_.publish(range_to_drone);
}
}
void DekfSensorFusion::publishResidual_()
{

  std_msgs::Float64 residual_robot;
  residual_robot.data = res_range;
  pubResidual_.publish(residual_robot);
}
// MAIN:
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
  std::string robot_name;
  while (ros::ok())
  {
    if (dekf_sensor_fusion.initializer == 0 && dekf_sensor_fusion.truths_1 == 1 && dekf_sensor_fusion.truths_2 == 1) {
      dekf_sensor_fusion.initialization();
    }
    if (dekf_sensor_fusion.initializer == 1){
      if (dekf_sensor_fusion._range < 5.0) {
          dekf_sensor_fusion.SendCovariance();
        }
        else{
          ROS_ERROR_STREAM("[" << dekf_sensor_fusion.robot_name << "] Out of Range: " << dekf_sensor_fusion._range); // TODO: WHen GPS is turned off for one drone, it still sends pose/cov no matter what the range is. Investigate that.

        }
      }
    // ros::spinOnce();
    rate.sleep(); //TODO make sure if it affects the AsyncSpinner or not
    counter++;
  }
  return 0;
}
