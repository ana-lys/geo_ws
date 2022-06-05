/****************************************************************************
 *
 *   Copyright (c) 2018-2021 Jaeyoung Lim. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Geometric Controller
 *
 * Geometric controller
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "geometric_controller/geometric_controller.h"
#include "geometric_controller/UTM.h"
using namespace Eigen;
using namespace std;
// Constructor
geometricCtrl::geometricCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      fail_detec_(false),
      ctrl_enable_(true),
      landing_commanded_(false),
      feedthrough_enable_(false),
      node_state(WAITING_FOR_HOME_POSE) {
  referenceSub_ =
      nh_.subscribe("reference/setpoint", 1, &geometricCtrl::targetCallback, this, ros::TransportHints().tcpNoDelay());
  flatreferenceSub_ = nh_.subscribe("reference/flatsetpoint", 1, &geometricCtrl::flattargetCallback, this,
                                    ros::TransportHints().tcpNoDelay());
  yawreferenceSub_ =
      nh_.subscribe("reference/yaw", 1, &geometricCtrl::yawtargetCallback, this, ros::TransportHints().tcpNoDelay());
  multiDOFJointSub_ = nh_.subscribe("command/trajectory", 1, &geometricCtrl::multiDOFJointCallback, this,
                                    ros::TransportHints().tcpNoDelay());
  mavstateSub_ =
      nh_.subscribe("mavros/state", 1, &geometricCtrl::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
  mavposeSub_ = nh_.subscribe("mavros/local_position/pose", 1, &geometricCtrl::mavposeCallback, this,
                              ros::TransportHints().tcpNoDelay());
  mavtwistSub_ = nh_.subscribe("mavros/local_position/velocity_local", 1, &geometricCtrl::mavtwistCallback, this,
                               ros::TransportHints().tcpNoDelay()); 
  ctrltriggerServ_ = nh_.advertiseService("trigger_rlcontroller", &geometricCtrl::ctrltriggerCallback, this);
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &geometricCtrl::cmdloopCallback,
                                   this); 
  // commandTimer = nh_.createTimer(ros::Duration(0.01), &geometricCtrl::commandTimerCallback,
  //                                  this);
   // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &geometricCtrl::statusloopCallback,
                                      this);  // Define timer for constant loop rate
  gpsrawSub_= nh_.subscribe("mavros/global_position/global", 1, &geometricCtrl::gpsrawCallback, this,
                              ros::TransportHints().tcpNoDelay());
  global_poseSub_= nh_.subscribe("mavros/global_position/local", 1, &geometricCtrl::globalCallback, this,
                               ros::TransportHints().tcpNoDelay());
  angularVelPub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("command/bodyrate_command", 1);
  referencePosePub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference/pose", 1);
  target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  posehistoryPub_ = nh_.advertise<nav_msgs::Path>("geometric_controller/path", 10);
  systemstatusPub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("mavros/companion_process/status", 1);
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  land_service_ = nh_.advertiseService("land", &geometricCtrl::landCallback, this);
  imuSub_ = nh_.subscribe("/mavros/imu/data",1,&geometricCtrl::imuCallback, this,
                              ros::TransportHints().tcpNoDelay());
  nh_private_.param<string>("mavname", mav_name_, "iris");
  nh_private_.param<int>("ctrl_mode", ctrl_mode_, RPG_CONTROLLER);
  nh_private_.param<bool>("enable_sim", sim_enable_, true);
  nh_private_.param<bool>("velocity_yaw", velocity_yaw_, false);
  nh_private_.param<double>("max_acc", max_fb_acc_, 9.0);
  nh_private_.param<double>("yaw_heading", mavYaw_, 0.0);
  nh_private_.param<double>("drag_dx", dx_, 0.7);
  nh_private_.param<double>("drag_dy", dy_, 0.7);//0.7
  nh_private_.param<double>("drag_dz", dz_, 0.0);
  nh_private_.param<double>("attctrl_constant", attctrl_tau_, 0.1);
  nh_private_.param<double>("normalizedthrust_constant", norm_thrust_const_, 0.05);  // 1 / max acceleration
  nh_private_.param<double>("normalizedthrust_offset", norm_thrust_offset_, 0.05);    // 1 / max acceleration
  nh_private_.param<double>("Kp_x", Kpos_x_, 5.0);
  nh_private_.param<double>("Kp_y", Kpos_y_, 5.0);
  nh_private_.param<double>("Kp_z", Kpos_z_, 8.0);
  nh_private_.param<double>("Kv_x", Kvel_x_, 1);
  nh_private_.param<double>("Kv_y", Kvel_y_, 1);
  nh_private_.param<double>("Kv_z", Kvel_z_, 3);
  nh_private_.param<int>("posehistory_window", posehistory_window_, 200);
  nh_private_.param<double>("init_pos_x", initTargetPos_x_, 0.0);
  nh_private_.param<double>("init_pos_y", initTargetPos_y_, 0.0);
  nh_private_.param<double>("init_pos_z", initTargetPos_z_, 2.0);

  targetPos_ << initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;  // Initial Position
  targetVel_ << 0.0, 0.0, 0.0;
  mavPos_ << 0.0, 0.0, 0.0;
  globalPos_ << 0.0 ,0.0, 0.0;
  globalVel_ << 0.0 ,0.0, 0.0;
  mavVel_ << 0.0, 0.0, 0.0;
  Imu_base << 0.0, 0.0 ,0.0;
  g_ << 0.0, 0.0, -9.81;
  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
  gps_enable = 0 ;
  D_ << dx_, dy_, dz_;
  tau << tau_x, tau_y, tau_z;
}
geometricCtrl::~geometricCtrl() {
  // Destructor
}
void geometricCtrl::gpsrawCallback(const sensor_msgs::NavSatFix &msg){
  if(!gps_home_init){
    gps_home_init = true;
     gps_home(0) = msg.latitude;
     gps_home(1) = msg.longitude;
     gps_home(2) = msg.altitude ; 
    LatLonToUTMXY(gps_home(0),gps_home(1),32,UTM_HOME_X,UTM_HOME_Y);
  }
 gpsraw(0) = msg.latitude;
 gpsraw(1) = msg.longitude;
 gpsraw(2) = msg.altitude ;
 LatLonToUTMXY(gpsraw(0),gpsraw(1),32,UTM_X,UTM_Y);
 gps_pos(0) = UTM_X-UTM_HOME_X;
 gps_pos(1) = UTM_Y-UTM_HOME_Y;

 ROS_INFO_STREAM("you are at "<<std::setprecision(20)<<"X:"<<UTM_X-UTM_HOME_X<<"  Y:"<<UTM_Y-UTM_HOME_Y);
}
void geometricCtrl::imuCallback(const sensor_msgs::Imu &msg){
Imu_base(0) =msg.linear_acceleration.x;
Imu_base(1) =msg.linear_acceleration.y;
Imu_base(2) =msg.linear_acceleration.z;
Imu_ang_vel(0)=msg.angular_velocity.x;
Imu_ang_vel(1)=msg.angular_velocity.y;
Imu_ang_vel(2)=msg.angular_velocity.z;
ROS_INFO("IMU");
}
void geometricCtrl::targetCallback(const geometry_msgs::TwistStamped &msg) {
  reference_request_last_ = reference_request_now_;
  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ = toEigen(msg.twist.angular);
  targetVel_ = toEigen(msg.twist.linear);

  if (reference_request_dt_ > 0)
    targetAcc_ = (targetVel_ - targetVel_prev_) / reference_request_dt_;
  else
    targetAcc_ = Eigen::Vector3d::Zero();
    ROS_INFO("Target callback");
}
void geometricCtrl::globalCallback(const nav_msgs::Odometry &msg){
    globalPos_ = toEigen(msg.pose.pose.position);
    globalVel_ = toEigen(msg.twist.twist.linear);
    globalAtt_(0) = msg.pose.pose.orientation.w;
    globalAtt_(1) = msg.pose.pose.orientation.w;
    globalAtt_(2) = msg.pose.pose.orientation.w;
   globalAtt_(3) = msg.pose.pose.orientation.w;
   ROS_INFO("GLOBAL callback");
}
void geometricCtrl::flattargetCallback(const controller_msgs::FlatTarget &msg) {
  reference_request_last_ = reference_request_now_;

  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ = toEigen(msg.position);
  targetVel_ = toEigen(msg.velocity);

  if (msg.type_mask == 1) {
    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = toEigen(msg.jerk);
    targetSnap_ = Eigen::Vector3d::Zero();

  } else if (msg.type_mask == 2) {
    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();

  } else if (msg.type_mask == 4) {
    targetAcc_ = Eigen::Vector3d::Zero();
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();

  } else {
    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = toEigen(msg.jerk);
    targetSnap_ = toEigen(msg.snap);
  }
  ROS_INFO("Flat");
}

void geometricCtrl::yawtargetCallback(const std_msgs::Float32 &msg) {
  if (!velocity_yaw_) mavYaw_ = double(msg.data);
   ROS_INFO("Yaw");
}

void geometricCtrl::multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg) {
  trajectory_msgs::MultiDOFJointTrajectoryPoint pt = msg.points[0];
  reference_request_last_ = reference_request_now_;

  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ << pt.transforms[0].translation.x, pt.transforms[0].translation.y, pt.transforms[0].translation.z;
  targetVel_ << pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z;

  targetAcc_ << pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z;
  targetJerk_ = Eigen::Vector3d::Zero();
  targetSnap_ = Eigen::Vector3d::Zero();

  if (!velocity_yaw_) {
    Eigen::Quaterniond q(pt.transforms[0].rotation.w, pt.transforms[0].rotation.x, pt.transforms[0].rotation.y,
                         pt.transforms[0].rotation.z);
    Eigen::Vector3d rpy = Eigen::Matrix3d(q).eulerAngles(0, 1, 2);  // RPY
    mavYaw_ = rpy(2);
  }
   ROS_INFO("DOF");
}

void geometricCtrl::mavposeCallback(const geometry_msgs::PoseStamped &msg) {
  if (!received_home_pose) {
    received_home_pose = true;
    home_pose_ = msg.pose;
    ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
  }
  mavPos_ = toEigen(msg.pose.position);
  gps_pos(2) = msg.pose.position.z;
  mavAtt_(0) = msg.pose.orientation.w;
  mavAtt_(1) = msg.pose.orientation.x;
  mavAtt_(2) = msg.pose.orientation.y;
  mavAtt_(3) = msg.pose.orientation.z;
   ROS_INFO("POSE");
}

void geometricCtrl::mavtwistCallback(const geometry_msgs::TwistStamped &msg) {
  mavVel_ = toEigen(msg.twist.linear);
  mavRate_ = toEigen(msg.twist.angular);
     ROS_INFO("TWIST");
}

bool geometricCtrl::landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
  node_state = LANDING;
  return true;
   ROS_INFO("LAND");
}

void geometricCtrl::cmdloopCallback(const ros::TimerEvent &event) {
   ROS_INFO("LOOP");
  switch (node_state) {
    case WAITING_FOR_HOME_POSE:
      waitForPredicate(&received_home_pose, "Waiting for home pose...");
      ROS_INFO("Got pose! Drone Ready to be armed.");
      node_state = MISSION_EXECUTION;
      start_control=ros::Time::now();
      creates();
      break;

    case MISSION_EXECUTION: {
     
      Eigen::Vector3d desired_acc;
      if (feedthrough_enable_) {
        desired_acc = targetAcc_;
      } else {
        desired_acc = controlPosition(targetPos_, targetVel_, targetAcc_ ,accel_fb);
      }
      ROS_INFO_STREAM(cmdBodyRate_<< " "<< q_des);
      computeBodyRateCmd(cmdBodyRate_, desired_acc ,accel_fb);
      pubReferencePose(targetPos_, q_des);
      ROS_INFO_STREAM(cmdBodyRate_<< " "<< q_des);
      pubRateCommands(cmdBodyRate_, q_des);
      appendPoseHistory();
      pubPoseHistory();
      updates((ros::Time::now()-start_control).toSec(),pos_error(0),pos_error(1),pos_error(2),accel_fb(0),accel_fb(1),accel_fb(2),Imu_accel(0),Imu_accel(1),Imu_accel(2),targetAcc_(0),targetAcc_(1),targetAcc_(2),targetVel_(0),targetVel_(1),targetVel_(2),mavVel_(0),mavVel_(1),mavVel_(2),normalized_thrust);
      break;
    }

    case LANDING: {
      geometry_msgs::PoseStamped landingmsg;
      landingmsg.header.stamp = ros::Time::now();
      landingmsg.pose = home_pose_;
      landingmsg.pose.position.z = landingmsg.pose.position.z + 1.0;
      target_pose_pub_.publish(landingmsg);
      node_state = LANDED;
      ros::spinOnce();
      break;
    }
    case LANDED:
      ROS_INFO("Landed. Please set to position control and disarm.");
      cmdloop_timer_.stop();
      break;
  }
}

void geometricCtrl::mavstateCallback(const mavros_msgs::State::ConstPtr &msg) { current_state_ = *msg; 
 ROS_INFO("STATE");}

void geometricCtrl::statusloopCallback(const ros::TimerEvent &event) {
   ROS_INFO("STATUS");
  if (sim_enable_) {
    // Enable OFFBoard mode and arm automatically
    // This will only run if the vehicle is simulated
    arm_cmd_.request.value = true;
    offb_set_mode_.request.custom_mode = "OFFBOARD";
    if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
      if (set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent) {
        ROS_INFO("Offboard enabled");
      }
      last_request_ = ros::Time::now();
    } else {
      if (!current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
        if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success) {
          ROS_INFO("Vehicle armed");
        }
        last_request_ = ros::Time::now();
      }
    }
  }
  pubSystemStatus();
  
}

void geometricCtrl::pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude) {
  geometry_msgs::PoseStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.pose.position.x = target_position(0);
  msg.pose.position.y = target_position(1);
  msg.pose.position.z = target_position(2);
  msg.pose.orientation.w = target_attitude(0);
  msg.pose.orientation.x = target_attitude(1);
  msg.pose.orientation.y = target_attitude(2);
  msg.pose.orientation.z = target_attitude(3);
  referencePosePub_.publish(msg);
}

void geometricCtrl::pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude) {
  mavros_msgs::AttitudeTarget msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.body_rate.x = cmd(0);
  msg.body_rate.y = cmd(1);
  msg.body_rate.z = cmd(2);
  msg.type_mask = 128;  // Ignore orientation messages
  msg.orientation.w = target_attitude(0);
  msg.orientation.x = target_attitude(1);
  msg.orientation.y = target_attitude(2);
  msg.orientation.z = target_attitude(3);
  msg.thrust = cmd(3);

  angularVelPub_.publish(msg);
}

void geometricCtrl::pubPoseHistory() {
  nav_msgs::Path msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.poses = posehistory_vector_;

  posehistoryPub_.publish(msg);
}

void geometricCtrl::pubSystemStatus() {
  mavros_msgs::CompanionProcessStatus msg;

  msg.header.stamp = ros::Time::now();
  msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
  msg.state = (int)companion_state_;

  systemstatusPub_.publish(msg);
}

void geometricCtrl::appendPoseHistory() {
  posehistory_vector_.insert(posehistory_vector_.begin(), vector3d2PoseStampedMsg(mavPos_, mavAtt_));
  if (posehistory_vector_.size() > posehistory_window_) {
    posehistory_vector_.pop_back();
  }
}

geometry_msgs::PoseStamped geometricCtrl::vector3d2PoseStampedMsg(Eigen::Vector3d &position,
                                                                  Eigen::Vector4d &orientation) {
  geometry_msgs::PoseStamped encode_msg;
  encode_msg.header.stamp = ros::Time::now();
  encode_msg.header.frame_id = "map";
  encode_msg.pose.orientation.w = orientation(0);
  encode_msg.pose.orientation.x = orientation(1);
  encode_msg.pose.orientation.y = orientation(2);
  encode_msg.pose.orientation.z = orientation(3);
  encode_msg.pose.position.x = position(0);
  encode_msg.pose.position.y = position(1);
  encode_msg.pose.position.z = position(2);
  return encode_msg;
}

Eigen::Vector3d geometricCtrl::controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                               const Eigen::Vector3d &target_acc, Eigen::Vector3d &accel_fb) {
  /// Compute BodyRate commands using differential flatness
  /// Controller based on Faessler 2017
  const Eigen::Vector3d a_ref = target_acc;
  if (velocity_yaw_) {
    mavYaw_ = getVelocityYaw(mavVel_);
  }
  
  const Eigen::Vector4d q_ref = acc2quaternion(a_ref - g_, mavYaw_);
  const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);

   pos_error = mavPos_ - target_pos;
   vel_error = mavVel_ - target_vel;
  if(gps_enable==1 ){
  const Eigen::Vector3d pos_error = globalPos_ - target_pos;
  //ROS_INFO_STREAM("error "<< pos_error);
  }
  // Position Controller
  const Eigen::Vector3d a_fb = poscontroller(pos_error, vel_error);
  accel_fb = a_fb;
  // Rotor Drag compensation
  const Eigen::Vector3d a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * target_vel;  // Rotor drag

  // Reference acceleration
  const Eigen::Vector3d a_des = a_fb + a_ref - a_rd - g_;

  return a_des;
}

void geometricCtrl::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &a_des, const Eigen::Vector3d &accel_fb) {
  // Reference attitude
  q_des = acc2quaternion(a_des, mavYaw_);
  // Choose which kind of attitude controller you are running
  bool jerk_enabled = false;
  if (!jerk_enabled) {
    if (ctrl_mode_ == ERROR_GEOMETRIC) {
      bodyrate_cmd = geometric_attcontroller(q_des, a_des, mavAtt_); 
    } else if(ctrl_mode_ == ERROR_QUATERNION) {
      bodyrate_cmd = attcontroller(q_des, a_des, mavAtt_);  // Calculate BodyRate
    }
     else if(ctrl_mode_ == RPG_CONTROLLER){ 
      Eigen::Vector3d ang_vel_ref = rpgcontroller( targetAcc_, mavAtt_,drag_accel) ;
      Eigen::Vector3d desire_accel = computeDesiredAccel(accel_fb ,targetAcc_, drag_accel);
      //ROS_INFO_STREAM("accfb" <<accel_fb);
      Eigen::Quaterniond desired_attitude = computeDesiredAttitude(desire_accel, mavYaw_,vec2quat(mavAtt_));
      Eigen::Vector3d feedback_bodyrates = computeFeedBackControlBodyrates(
      desired_attitude, mavAtt_ ,desire_accel,emergency_status);
      // ROS_INFO_STREAM("feedback"<<feedback_bodyrates);
      normalized_thrust = computeNormalizedThrust(desire_accel, mavAtt_);
      // ROS_INFO_STREAM("thrust"<<normalized_thrust);
      bodyrate_cmd.head(3) = ang_vel_ref + feedback_bodyrates;
      bodyrate_cmd(3) = normalized_thrust;
     }
  } else {
    bodyrate_cmd = jerkcontroller(targetJerk_, a_des, q_des, mavAtt_);
  }
}

Eigen::Quaterniond geometricCtrl::computeDesiredAttitude(
     Eigen::Vector3d& desired_acceleration, const double reference_heading,
    const Eigen::Quaterniond& attitude_estimate) {
  const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
      Eigen::AngleAxisd(reference_heading, Eigen::Vector3d::UnitZ()));

  // Compute desired orientation
  const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();
  Imu_accel = attitude_estimate * Imu_base;
  // desired_acceleration+=(targetAcc_- Imu_accel)*0.1;
  Eigen::Vector3d z_B;
  if (almostZero(desired_acceleration.norm())) {
    // In case of free fall we keep the thrust direction to be the estimated one
    // This only works assuming that we are in this condition for a very short
    // time (otherwise attitude drifts)
    z_B = attitude_estimate * Eigen::Vector3d::UnitZ();
  } else {
    z_B = desired_acceleration.normalized();
  }
  //ROS_INFO_STREAM("IMU ACCEL "<< Imu_accel);
  const Eigen::Vector3d x_B_prototype = y_C.cross(z_B);
  const Eigen::Vector3d x_B =
      computeRobustBodyXAxis(x_B_prototype, x_C, y_C, attitude_estimate);

  const Eigen::Vector3d y_B = (z_B.cross(x_B)).normalized();

  // From the computed desired body axes we can now compose a desired attitude
  const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());

  const Eigen::Quaterniond desired_attitude(R_W_B);

  return desired_attitude;
}
Eigen::Vector3d geometricCtrl::computeDesiredAccel(Eigen::Vector3d accel_fb ,Eigen::Vector3d target_ac,Eigen::Vector3d drag){

Eigen::Vector3d raw_acc  = accel_fb + target_ac - drag ;
//  ROS_INFO_STREAM("drag accel "<< drag);
Eigen::Vector3d desired_acc = raw_acc - g_;
// ROS_INFO_STREAM("raw_acc"<<raw_acc <<"desire_acc"<<desired_acc);
if(desired_acc(2)>drone_max_ver_acc)desired_acc(2) = drone_max_ver_acc;
if(desired_acc.norm()> drone_max_accel*0.85){
  // ROS_INFO_STREAM("max_ac_sqr"<<drone_max_accel*drone_max_accel*0.725<<"des2_sqr"<<desired_acc(2)*desired_acc(2));
  double xy_acc = sqrt(drone_max_accel*drone_max_accel*0.725 - desired_acc(2)*desired_acc(2));
  Eigen::Vector3d xy ; xy(0)= desired_acc(0) ; xy(1)= desired_acc(1) ; xy(2) =0;
  xy = xy / xy.norm() * xy_acc ;
  desired_acc(0) =xy(0);desired_acc(1) =xy(1);
}
return desired_acc;
}

double geometricCtrl::computeNormalizedThrust(const Eigen::Vector3d &desire_accel,const Eigen::Vector4d &current_att){
Eigen::Quaterniond orientation = vec2quat(current_att);
const Eigen::Vector3d body_z_axis = orientation * Eigen::Vector3d::UnitZ();
  
  double normalized_thrust = desire_accel.dot(body_z_axis) - k_thrust_horz * (pow (mavVel_(0), 2.0) +
                                pow(mavVel_(1), 2.0)) ;
 
  normalized_thrust = std::max(0.0,std::min(1.0, norm_thrust_const_ * normalized_thrust + norm_thrust_offset_));
 return normalized_thrust;
}
Eigen::Vector3d geometricCtrl::poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error) {
  Eigen::Vector3d a_fb =
      Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error;  // feedforward term for trajectory error

  if (a_fb.norm() > max_fb_acc_)
    a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;  // Clip acceleration if reference is too large

  return a_fb;
}
Eigen::Vector3d geometricCtrl::emergency_feedback_accel  () {
  Eigen::Vector3d a_fb =
      Kpos_.asDiagonal() * (mavPos_- emergency_pos )+ Kvel_.asDiagonal() * mavVel_;  // feedforward term for trajectory error

  if (a_fb.norm() > max_fb_acc_)
    a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;  // Clip acceleration if reference is too large

  return a_fb;
}
Eigen::Vector3d geometricCtrl::computeFeedBackControlBodyrates(
    const Eigen::Quaterniond& desired_attitude,
    const Eigen::Vector4d& attitude_estimate , Eigen::Vector3d& des_acc , bool& emergency ) {
  // Compute the error quaternion
  Eigen::Quaterniond att_est = vec2quat(attitude_estimate);
  Eigen::Vector3d bodyrates;
  Eigen::Quaterniond q_e;
  // if (!emergency_status){
   q_e =att_est.inverse() *  desired_attitude ;

  if (q_e.w() <= 0) {
    bodyrates.x() =-2.0 * krp * q_e.x();
    bodyrates.y() =-2.0 * krp * q_e.y();
    bodyrates.z() =-2.0 * kyaw * q_e.z();
  } else {
    bodyrates.x() = +2.0 * krp * q_e.x();
    bodyrates.y() = +2.0 * krp * q_e.y();
    bodyrates.z() = +2.0 * kyaw * q_e.z();
  }
  
  return bodyrates;
}
Eigen::Vector4d geometricCtrl::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw) {
  Eigen::Vector4d quat;
  Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
  Eigen::Matrix3d rotmat;

  proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;
  zb_des = vector_acc / vector_acc.norm();
  yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
  xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();
  rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
  quat = rot2Quaternion(rotmat);
  return quat;
}

Eigen::Vector4d geometricCtrl::attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc,
                                             Eigen::Vector4d &curr_att) {
  // Geometric attitude controller
  // Attitude error is defined as in Brescianini, Dario, Markus Hehn, and Raffaello D'Andrea. Nonlinear quadrocopter
  // attitude control: Technical report. ETH Zurich, 2013.

  Eigen::Vector4d ratecmd;

  const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
  const Eigen::Vector4d q_inv = inverse.asDiagonal() * curr_att;
  const Eigen::Vector4d qe = quatMultiplication(q_inv, ref_att);
  ratecmd(0) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);
  ratecmd(1) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);
  ratecmd(2) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3);
  const Eigen::Matrix3d rotmat = quat2RotMatrix(mavAtt_);
  const Eigen::Vector3d zb = rotmat.col(2);
  ratecmd(3) =
      std::max(0.0, std::min(1.0, norm_thrust_const_ * ref_acc.dot(zb) + norm_thrust_offset_));  // Calculate thrust

  return ratecmd;
}

Eigen::Vector4d geometricCtrl::jerkcontroller(const Eigen::Vector3d &ref_jerk, const Eigen::Vector3d &ref_acc,
                                              Eigen::Vector4d &ref_att, Eigen::Vector4d &curr_att) {
  // Jerk feedforward control
  // Based on: Lopez, Brett Thomas. Low-latency trajectory planning for high-speed navigation in unknown environments.
  // Diss. Massachusetts Institute of Technology, 2016.
  // Feedforward control from Lopez(2016)

  double dt_ = 0.01;
  // Numerical differentiation to calculate jerk_fb
  const Eigen::Vector3d jerk_fb = (ref_acc - last_ref_acc_) / dt_;
  const Eigen::Vector3d jerk_des = ref_jerk + jerk_fb;
  const Eigen::Matrix3d R = quat2RotMatrix(curr_att);
  const Eigen::Vector3d zb = R.col(2);

  const Eigen::Vector3d jerk_vector =
      jerk_des / ref_acc.norm() - ref_acc * ref_acc.dot(jerk_des) / std::pow(ref_acc.norm(), 3);
  const Eigen::Vector4d jerk_vector4d(0.0, jerk_vector(0), jerk_vector(1), jerk_vector(2));

  Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
  const Eigen::Vector4d q_inv = inverse.asDiagonal() * curr_att;
  const Eigen::Vector4d qd = quatMultiplication(q_inv, ref_att);

  const Eigen::Vector4d qd_star(qd(0), -qd(1), -qd(2), -qd(3));

  const Eigen::Vector4d ratecmd_pre = quatMultiplication(quatMultiplication(qd_star, jerk_vector4d), qd);

  Eigen::Vector4d ratecmd;
  ratecmd(0) = ratecmd_pre(2);  // TODO: Are the coordinate systems consistent?
  ratecmd(1) = (-1.0) * ratecmd_pre(1);
  ratecmd(2) = 0.0;
  ratecmd(3) =
      std::max(0.0, std::min(1.0, norm_thrust_const_ * ref_acc.dot(zb) + norm_thrust_offset_));  // Calculate thrust
  last_ref_acc_ = ref_acc;
  return ratecmd;
}

Eigen::Vector4d geometricCtrl::geometric_attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc,
                                                       Eigen::Vector4d &curr_att) {
  // Geometric attitude controller
  // Attitude error is defined as in Lee, Taeyoung, Melvin Leok, and N. Harris McClamroch. "Geometric tracking control
  // of a quadrotor UAV on SE (3)." 49th IEEE conference on decision and control (CDC). IEEE, 2010.
  // The original paper inputs moment commands, but for offboard control, angular rate commands are sent

  Eigen::Vector4d ratecmd;
  Eigen::Matrix3d rotmat;    // Rotation matrix of current attitude
  Eigen::Matrix3d rotmat_d;  // Rotation matrix of desired attitude
  Eigen::Vector3d error_att;

  rotmat = quat2RotMatrix(curr_att);
  rotmat_d = quat2RotMatrix(ref_att);

  error_att = 0.5 * matrix_hat_inv(rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat_d);
  ratecmd.head(3) = (2.0 / attctrl_tau_) * error_att;
  rotmat = quat2RotMatrix(mavAtt_);
  const Eigen::Vector3d zb = rotmat.col(2);
  ratecmd(3) =
      std::max(0.0, std::min(1.0, norm_thrust_const_ * ref_acc.dot(zb) + norm_thrust_offset_));  // Calculate thrust

  return ratecmd;
}
Eigen::Vector3d geometricCtrl::computeRobustBodyXAxis(
    const Eigen::Vector3d& x_B_prototype, const Eigen::Vector3d& x_C,
    const Eigen::Vector3d& y_C,
    const Eigen::Quaterniond& attitude_estimate){
  Eigen::Vector3d x_B = x_B_prototype;

  if (almostZero(x_B.norm())) {
    // if cross(y_C, z_B) == 0, they are collinear =>
    // every x_B lies automatically in the x_C - z_C plane

    // Project estimated body x-axis into the x_C - z_C plane
    const Eigen::Vector3d x_B_estimated =
        attitude_estimate * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d x_B_projected =
        x_B_estimated - (x_B_estimated.dot(y_C)) * y_C;
    if (almostZero(x_B_projected.norm())) {
      // Not too much intelligent stuff we can do in this case but it should
      // basically never occur
      x_B = x_C;
    } else {
      x_B = x_B_projected.normalized();
    }
  } else {
    x_B.normalize();
  }
  // if the quad is upside down, x_B will point in the "opposite" direction
  // of x_C => flip x_B (unfortunately also not the solution for our problems)
  //  if (x_B.dot(x_C) < 0.0)
  //  {
  //    x_B = -x_B;
  //  }
  return x_B;
}


Eigen::Vector3d geometricCtrl::rpgcontroller( const Eigen::Vector3d &ref_acc,
                                                       Eigen::Vector4d &curr_att, Eigen::Vector3d &drag_accel) {
  Eigen::Vector3d ratecmd;
  //Eigen::Vector3d euler = quat2RotMatrix(curr_att).eulerAngles(0, 1, 2);
  controller_dt = ((ros::Time::now()-last_yaw_ref_time).toSec()) ;
  double yaw_ref_rate = (mavYaw_-last_yaw_ref)/ controller_dt ;
  last_yaw_ref = mavYaw_;
  last_yaw_ref_time = ros::Time::now();
  // Getting ref_att that drag applied
  Eigen::Quaterniond q_heading = Eigen::Quaterniond(
      Eigen::AngleAxisd( mavYaw_, Eigen::Vector3d::UnitZ()));
  Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
  Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();
  Eigen::Quaterniond orientation = vec2quat(curr_att);
  
  Eigen::Vector3d alpha =
      ref_acc - g_ + dx_ * targetVel_;
  Eigen::Vector3d beta =
      ref_acc - g_ + dy_ * targetVel_;
  Eigen::Vector3d gamma =
      ref_acc - g_ + dz_ * targetVel_;
  Eigen::Vector3d x_B_prototype = y_C.cross(alpha);
  Eigen::Vector3d x_B = computeRobustBodyXAxis(
      x_B_prototype, x_C, y_C, orientation);

  Eigen::Vector3d y_B = beta.cross(x_B);
  if (almostZero(y_B.norm())) {
    Eigen::Vector3d z_B_estimated =
        orientation * Eigen::Vector3d::UnitZ();
    y_B = z_B_estimated.cross(x_B);
    if (almostZero(y_B.norm())) {
      y_B = y_C;
    } else {
      y_B.normalize();
    }
  } else {
    y_B.normalize();
  }

  Eigen::Vector3d z_B = x_B.cross(y_B);
  Eigen::Matrix3d R_W_B_ref(
      (Eigen::Matrix3d() << x_B, y_B, z_B).finished());

  Eigen::Quaterniond orientation_cmd = Eigen::Quaterniond(R_W_B_ref);
  double collective_thrust = z_B.dot(gamma);

    // Rotor drag matrix
  Eigen::Matrix3d D = Eigen::Vector3d(dx_, dy_, dz_).asDiagonal();

  // Reference body rates and angular accelerations
  double B1 = collective_thrust -
                    (dz_ - dx_) * z_B.dot(targetVel_);
  double C1 = -(dx_ - dy_) * y_B.dot(targetVel_);
  double D1 = x_B.dot(targetJerk_) +
                    dx_ * x_B.dot(targetAcc_);
  double A2 = collective_thrust +
                    (dy_ - dz_) * z_B.dot(targetVel_);
  double C2 = (dx_ - dy_) * x_B.dot(targetVel_);
  double D2 = -y_B.dot(targetJerk_) -
                    dy_ * y_B.dot(targetAcc_);
  double B3 = -y_C.dot(z_B);
  double C3 = (y_C.cross(z_B)).norm();
  double D3 = yaw_ref_rate * x_C.dot(x_B);

  double denominator = B1 * C3 - B3 * C1;
  if (almostZero(denominator)) { 
    ratecmd = Eigen::Vector3d::Zero();
  } else {
    // Compute body rates
    if (almostZero(A2)) {
       ratecmd(0) = 0.0;
    } else {
      ratecmd(0) =
          (-B1 * C2 * D3 + B1 * C3 * D2 - B3 * C1 * D2 + B3 * C2 * D1) /
          (A2 * denominator);
    }
  ratecmd(1) = (-C1 * D3 + C3 * D1) / denominator;
  ratecmd(2) = (B1 * D3 - B3 * D1) / denominator;  
  const Eigen::Matrix3d R_trans =
      orientation.toRotationMatrix().transpose() * R_W_B_ref;
  ratecmd = R_trans * ratecmd.head(3);  
  drag_accel = -1.0 * (R_W_B_ref * (D * (R_W_B_ref.transpose() * targetVel_)));
  if(fabs(drag_accel(0))>0.1) drag_accel(0)+= - 0.35;
 
}
return ratecmd;
}
bool geometricCtrl::ctrltriggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
  unsigned char mode = req.data;

  ctrl_mode_ = mode;
  res.success = ctrl_mode_;
  res.message = "controller triggered";
  ROS_INFO_STREAM("controller triggered");
  return true;
}

void geometricCtrl::dynamicReconfigureCallback(geometric_controller::GeometricControllerConfig &config,
                                               uint32_t level) {
  if (max_fb_acc_ != config.max_acc) {
    max_fb_acc_ = config.max_acc;
    ROS_INFO("Reconfigure request : max_acc = %.2f ", config.max_acc);
  } else if (Kpos_x_ != config.Kp_x) {
    Kpos_x_ = config.Kp_x;
    ROS_INFO("Reconfigure request : Kp_x  = %.2f  ", config.Kp_x);
  } else if (Kpos_y_ != config.Kp_y) {
    Kpos_y_ = config.Kp_y;
    ROS_INFO("Reconfigure request : Kp_y  = %.2f  ", config.Kp_y);
  } else if (Kpos_z_ != config.Kp_z) {
    Kpos_z_ = config.Kp_z;
    ROS_INFO("Reconfigure request : Kp_z  = %.2f  ", config.Kp_z);
  } else if (Kvel_x_ != config.Kv_x) {
    Kvel_x_ = config.Kv_x;
    ROS_INFO("Reconfigure request : Kv_x  = %.2f  ", config.Kv_x);
  } else if (Kvel_y_ != config.Kv_y) {
    Kvel_y_ = config.Kv_y;
    ROS_INFO("Reconfigure request : Kv_y =%.2f  ", config.Kv_y);
  } else if (Kvel_z_ != config.Kv_z) {
    Kvel_z_ = config.Kv_z;
    ROS_INFO("Reconfigure request : Kv_z  = %.2f  ", config.Kv_z);
  }

  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
}

bool geometricCtrl::almostZero(const double value)  {
  return fabs(value) < 0.001;
}

bool geometricCtrl::almostZeroThrust(const double thrust_value) {
  return fabs(thrust_value) < 0.01;
}

Eigen::Quaterniond geometricCtrl::vec2quat( const Eigen::Vector4d vec){
Eigen::Quaterniond quat;
quat.w() = vec(0);
quat.x() = vec(1);
quat.y() = vec(2);
quat.z() = vec(3);
return quat;
}
Eigen::Vector4d geometricCtrl::quat2vec( const Eigen::Quaterniond quat){
Eigen::Vector4d vec;
vec(0)= quat.w() ;
vec(1)= quat.x() ;
vec(2)= quat.y() ;
vec(3)= quat.z() ;
return vec;
}
Eigen::Quaterniond geometricCtrl::vec2quatneg( const Eigen::Vector4d vec){
Eigen::Quaterniond quat;
quat.w() = vec(0);
quat.x() = -vec(1);
quat.y() = -vec(2);
quat.z() = -vec(3);
return quat;
}
