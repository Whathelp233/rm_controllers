/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 1/16/21.
// Modified by what_help on 2025/27/9
//
#include "rm_gimbal_controllers/gimbal_base.h"
#include <string>
#include <angles/angles.h>
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include <rm_common/lqr.h>
#include <Eigen/Dense>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>

namespace rm_gimbal_controllers
{
bool Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  XmlRpc::XmlRpcValue xml_rpc_value;
  bool enable_feedforward;
  enable_feedforward = controller_nh.getParam("feedforward", xml_rpc_value);
  if (enable_feedforward)
  {
    ROS_ASSERT(xml_rpc_value.hasMember("mass_origin"));
    ROS_ASSERT(xml_rpc_value.hasMember("gravity"));
    ROS_ASSERT(xml_rpc_value.hasMember("enable_gravity_compensation"));
  }
  mass_origin_.x = enable_feedforward ? (double)xml_rpc_value["mass_origin"][0] : 0.;
  mass_origin_.z = enable_feedforward ? (double)xml_rpc_value["mass_origin"][2] : 0.;
  gravity_ = enable_feedforward ? (double)xml_rpc_value["gravity"] : 0.;
  enable_gravity_compensation_ = enable_feedforward && (bool)xml_rpc_value["enable_gravity_compensation"];

  ros::NodeHandle chassis_vel_nh(controller_nh, "chassis_vel");
  chassis_vel_ = std::make_shared<ChassisVel>(chassis_vel_nh);
  ros::NodeHandle nh_bullet_solver = ros::NodeHandle(controller_nh, "bullet_solver");
  bullet_solver_ = std::make_shared<BulletSolver>(nh_bullet_solver);
  ros::NodeHandle nh_yaw = ros::NodeHandle(controller_nh, "yaw");
  ros::NodeHandle nh_base_yaw = ros::NodeHandle(controller_nh, "base_yaw");
  ros::NodeHandle nh_pitch = ros::NodeHandle(controller_nh, "pitch");
  ros::NodeHandle nh_pid_pitch_pos = ros::NodeHandle(controller_nh, "pitch/pid_pos");



  config_ = { .yaw_k_v_ = getParam(nh_yaw, "k_v", 0.),
              .pitch_k_v_ = getParam(nh_pitch, "k_v", 0.),
              .k_chassis_vel_ = getParam(controller_nh, "yaw/k_chassis_vel", 0.),
              .accel_pitch_ = getParam(controller_nh, "pitch/accel", 99.),
              .accel_yaw_ = getParam(controller_nh, "yaw/accel", 99.) };
  config_rt_buffer_.initRT(config_);
  d_srv_ = new dynamic_reconfigure::Server<rm_gimbal_controllers::GimbalBaseConfig>(controller_nh);
  dynamic_reconfigure::Server<rm_gimbal_controllers::GimbalBaseConfig>::CallbackType cb =
      [this](auto&& PH1, auto&& PH2) { reconfigCB(PH1, PH2); };
  d_srv_->setCallback(cb);

  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (!ctrl_base_yaw_.init(effort_joint_interface,nh_base_yaw)||!ctrl_yaw_.init(effort_joint_interface,nh_yaw)||!ctrl_pitch_.init(effort_joint_interface, nh_pitch) ||
     !pid_pitch_pos_.init(nh_pid_pitch_pos))
    return false;
  ROS_WARN("init_clear");
  robot_state_handle_ = robot_hw->get<rm_control::RobotStateInterface>()->getHandle("robot_state");
  if (!controller_nh.hasParam("imu_name"))
    has_imu_ = false;
  if (has_imu_)
  {
    imu_name_ = getParam(controller_nh, "imu_name", static_cast<std::string>("gimbal_imu"));
    hardware_interface::ImuSensorInterface* imu_sensor_interface =
        robot_hw->get<hardware_interface::ImuSensorInterface>();
    imu_sensor_handle_ = imu_sensor_interface->getHandle(imu_name_);
  }
  else
  {
    ROS_INFO("Param imu_name has not set, use motors' data instead of imu.");
  }

  // Get URDF info about joint
  urdf::Model urdf;
  if (!urdf.initParamWithNodeHandle("robot_description", controller_nh))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  pitch_joint_urdf_ = urdf.getJoint(ctrl_pitch_.getJointName());
  yaw_joint_urdf_ = urdf.getJoint(ctrl_yaw_.getJointName());
  base_yaw_joint_urdf_ = urdf.getJoint(ctrl_base_yaw_.getJointName());
  if (!pitch_joint_urdf_)
  {
    ROS_ERROR("Could not find joint pitch in urdf");
    return false;
  }
  if (!yaw_joint_urdf_)
  {
    ROS_ERROR("Could not find joint yaw in urdf");
    return false;
  }
  if (!base_yaw_joint_urdf_)
  {
    ROS_ERROR("Could not find joint base_yaw in urdf");
    return false;
  }

  gimbal_des_frame_id_ = pitch_joint_urdf_->child_link_name + "_des";
  odom2gimbal_des_.header.frame_id = "odom";
  odom2gimbal_des_.child_frame_id = gimbal_des_frame_id_;
  odom2gimbal_des_.transform.rotation.w = 1.;
  odom2pitch_.header.frame_id = "odom";
  odom2pitch_.child_frame_id = pitch_joint_urdf_->child_link_name;
  odom2pitch_.transform.rotation.w = 1.;
  odom2base_.header.frame_id = "odom";
  odom2base_.child_frame_id = yaw_joint_urdf_->parent_link_name;
  odom2base_.transform.rotation.w = 1.;
  odom2base_yaw_.header.frame_id = "odom";
  odom2base_yaw_.child_frame_id = base_yaw_joint_urdf_->child_link_name;
  odom2base_yaw_.transform.rotation.w = 1.;

  cmd_gimbal_sub_ = controller_nh.subscribe<rm_msgs::GimbalCmd>("command", 1, &Controller::commandCB, this);
  data_track_sub_ = controller_nh.subscribe<rm_msgs::TrackData>("/track", 1, &Controller::trackCB, this);
  publish_rate_ = getParam(controller_nh, "publish_rate", 100.);
  error_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::GimbalDesError>(controller_nh, "error", 100));
  yaw_pos_state_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::GimbalPosState>(nh_yaw, "pos_state", 1));

  pitch_pos_state_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::GimbalPosState>(nh_pitch, "pos_state", 1));

  ramp_rate_pitch_ = new RampFilter<double>(0, 0.001);
  ramp_rate_yaw_ = new RampFilter<double>(0, 0.001);
  
  enable_online_lqr = getParam(controller_nh, "enable_online_lqr", false);

  kf_yaw_ = KalmanFilter(0.01, 0.1, 0.0, 1.0);
  kf_base_yaw_ = KalmanFilter(0.01, 0.1, 0.0, 1.0);
  
  config_.a1_ = getParam(controller_nh, "a1", 0.1);  // 阻尼系数1
  config_.a2_ = getParam(controller_nh, "a2", 0.1);  // 阻尼系数2
  config_.j1_ = getParam(controller_nh, "j1", 1.0);  // 惯性1
  config_.j2_ = getParam(controller_nh, "j2", 1.0);  // 惯性2
  config_.dc_ = getParam(controller_nh, "dc", 0.0);  // 惯性耦合
  config_.q1_ = getParam(controller_nh, "q1", 100.0);  // 状态权重1
  config_.q2_ = getParam(controller_nh, "q2", 10.0);   // 状态权重2
  config_.q3_ = getParam(controller_nh, "q3", 100.0);  // 状态权重3
  config_.q4_ = getParam(controller_nh, "q4", 10.0);   // 状态权重4
  config_.r1_ = getParam(controller_nh, "r1", 1.0);    // 输入权重1 
  config_.r2_ = getParam(controller_nh, "r2", 1.0);    // 输入权重2
try {



  Eigen::MatrixXd A(4,4),B(4,2),Q(4,4),R(2,2);
  n_ = 4;
  m_ = 2;
  
A << 0., 1., 0., 0.,
     0., -config_.a1_/config_.j1_, 0., config_.dc_/config_.j1_,
     0., 0., 0., 1.,
     0., config_.dc_/config_.j2_, 0., -config_.a2_/config_.j2_;

B << 0., 0.,
     1./config_.j1_, 0.,
     0., 0.,
     0., 1./config_.j2_;

  Q << config_.q1_,0.,0.,0.,
  0.,config_.q2_,0.,0.,
  0.,0.,config_.q3_,0.,
  0.,0.,0.,config_.q4_;

  R << config_.r1_,0.,
       0.,config_.r2_;
//  ROS_INFO("b1: %f, j1: %f", b1, j1);
//  ROS_INFO("A matrix:\n%s", A.toString().c_str());  // 如果 Eigen 支持
  state_yaw_.resize(4);
  state_pitch_.resize(4);
  x_ref.resize(4);
  Lqr<double> lqr(A,B,Q,R,K_yaw_); 
      if (!lqr.computeK(A, B, Q, R, K_yaw_)) {
        K_yaw_ = Eigen::MatrixXd::Zero(2, 4);
        ROS_WARN("Using default K matrix");
      }
 // K_yaw_ = Eigen::MatrixXd::Zero(2, 4);
 //Eigen初始化
  Theta_ = Eigen::MatrixXd::Zero(n_, n_ + m_);  // 初始化为零矩阵
  Pcov_ = Eigen::MatrixXd::Identity(n_ + m_, n_ + m_) * 1.0;  // 初始协方差
  u_prev_sample_.resize(m_);
  x_prev_.resize(n_);
  state_yaw_.resize(4);
  state_pitch_.resize(4);
  x_ref.resize(4);
  recent_buffer_len_ = 30;
  Qd_ = Q;  // 使用与初始LQR相同的权重矩阵
  Rd_ = R;
  last_successful_K_ = K_yaw_;  // 初始化为计算的LQR增益
  K_current_ = K_yaw_;
  K_old_ = K_yaw_;
  K_target_ = K_yaw_;
  running_ = true;
  stable_count_ = 0;

  lambda_rls_ = getParam(controller_nh, "lambda_rls", 0.995);  // 遗忘因子
  worker_hz_ = getParam(controller_nh, "worker_hz", 2);  // LQR更新频率
  switch_smooth_T_ = getParam(controller_nh, "switch_smooth_T", 1.0);  // 平滑过渡时间1秒
  stable_needed_ = getParam(controller_nh, "stable_needed", 5);  // 需要连续稳定的次数

  stable_tol_ = getParam(controller_nh, "stable_tol", 0.05);  // 稳定容差
  u_max_normal_ = getParam(controller_nh, "u_max_normal", 20.0);  // 正常最大控制输入
  u_max_ = u_max_normal_;
  N_min_samples_ = 500; // 增加样本数量确保模型收敛
  k_matrix_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("debug/k_matrix", 10);
  //excitation_amplitude_ = 0.05;  // 小扰动幅度
  //use_excitation_ = true;       // 启用扰动

  switching_ = false;
  if (enable_online_lqr) {
  worker_thread_ = std::thread(&Controller::onlineLQRUpdate, this);
}
  ROS_INFO("Initializing gimbal controller...");
} catch (const std::exception& e) {
  ROS_ERROR("Exception in init: %s", e.what());
  return false;
}   
  return true;
}
void Controller::starting(const ros::Time& /*unused*/)
{
  state_ = RATE;
  state_changed_ = true;
  start_ = true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  cmd_gimbal_ = *cmd_rt_buffer_.readFromRT();
  data_track_ = *track_rt_buffer_.readFromNonRT();
  config_ = *config_rt_buffer_.readFromRT();
  ramp_rate_pitch_->setAcc(config_.accel_pitch_);
  ramp_rate_yaw_->setAcc(config_.accel_yaw_);
  ramp_rate_pitch_->input(cmd_gimbal_.rate_pitch);
  ramp_rate_yaw_->input(cmd_gimbal_.rate_yaw);
  //test
  //ROS_INFO("Gimbal yaw: %lf", cmd_gimbal_.rate_yaw);
  //ROS_INFO("Gimbal ptich: %lf", cmd_gimbal_.rate_pitch);
  cmd_gimbal_.rate_pitch = ramp_rate_pitch_->output();
  cmd_gimbal_.rate_yaw = ramp_rate_yaw_->output();

   
  try
  {
    odom2pitch_ = robot_state_handle_.lookupTransform("odom", pitch_joint_urdf_->child_link_name, time);
    odom2base_ = robot_state_handle_.lookupTransform("odom", yaw_joint_urdf_->parent_link_name, time);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1, "%s\n", ex.what());
    return;
  }
  updateChassisVel();
  if (state_ != cmd_gimbal_.mode)
  {
    state_ = cmd_gimbal_.mode;
    state_changed_ = true;
  }
  switch (state_)
  {
    case RATE:
      rate(time, period);
      break;
    case TRACK:
      track(time);
      break;
    case DIRECT:
      direct(time);
      break;
    case TRAJ:
      traj(time);
      break;
  }
  moveJoint(time, period);
}

void Controller::setDes(const ros::Time& time, double yaw_des, double pitch_des)
{
  tf2::Quaternion odom2base, odom2gimbal_des;
  tf2::Quaternion base2gimbal_des;
  tf2::fromMsg(odom2base_.transform.rotation, odom2base);
  odom2gimbal_des.setRPY(0, pitch_des, yaw_des);
  base2gimbal_des = odom2base.inverse() * odom2gimbal_des;
  double roll_temp, base2gimbal_current_des_pitch, base2gimbal_current_des_yaw;
  quatToRPY(toMsg(base2gimbal_des), roll_temp, base2gimbal_current_des_pitch, base2gimbal_current_des_yaw);
  double pitch_real_des, yaw_real_des;

  pitch_des_in_limit_ = setDesIntoLimit(pitch_real_des, pitch_des, base2gimbal_current_des_pitch, pitch_joint_urdf_);
  if (!pitch_des_in_limit_)
  {
    double yaw_temp;
    tf2::Quaternion base2new_des;
    double upper_limit, lower_limit;
    upper_limit = pitch_joint_urdf_->limits ? pitch_joint_urdf_->limits->upper : 1e16;
    lower_limit = pitch_joint_urdf_->limits ? pitch_joint_urdf_->limits->lower : -1e16;
    base2new_des.setRPY(0,
                        std::abs(angles::shortest_angular_distance(base2gimbal_current_des_pitch, upper_limit)) <
                                std::abs(angles::shortest_angular_distance(base2gimbal_current_des_pitch, lower_limit)) ?
                            upper_limit :
                            lower_limit,
                        base2gimbal_current_des_yaw);
    quatToRPY(toMsg(odom2base * base2new_des), roll_temp, pitch_real_des, yaw_temp);
  }

  yaw_des_in_limit_ = setDesIntoLimit(yaw_real_des, yaw_des, base2gimbal_current_des_yaw, yaw_joint_urdf_);
  if (!yaw_des_in_limit_)
  {
    double pitch_temp;
    tf2::Quaternion base2new_des;
    double upper_limit, lower_limit;
    upper_limit = yaw_joint_urdf_->limits ? yaw_joint_urdf_->limits->upper : 1e16;
    lower_limit = yaw_joint_urdf_->limits ? yaw_joint_urdf_->limits->lower : -1e16;
    base2new_des.setRPY(0, base2gimbal_current_des_pitch,
                        std::abs(angles::shortest_angular_distance(base2gimbal_current_des_yaw, upper_limit)) <
                                std::abs(angles::shortest_angular_distance(base2gimbal_current_des_yaw, lower_limit)) ?
                            upper_limit :
                            lower_limit);
    quatToRPY(toMsg(odom2base * base2new_des), roll_temp, pitch_temp, yaw_real_des);
  }

  odom2gimbal_des_.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0., pitch_real_des, yaw_real_des);
  odom2gimbal_des_.header.stamp = time;
  robot_state_handle_.setTransform(odom2gimbal_des_, "rm_gimbal_controllers");
}

void Controller::rate(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter RATE");
    if (start_)
    {
      odom2gimbal_des_.transform.rotation = odom2pitch_.transform.rotation;
      odom2gimbal_des_.header.stamp = time;
      robot_state_handle_.setTransform(odom2gimbal_des_, "rm_gimbal_controllers");
      start_ = false;
    }
  }
  else
  {
    double roll{}, pitch{}, yaw{};
    quatToRPY(odom2gimbal_des_.transform.rotation, roll, pitch, yaw);
    setDes(time, yaw + period.toSec() * cmd_gimbal_.rate_yaw, pitch + period.toSec() * cmd_gimbal_.rate_pitch);
  }
}

void Controller::track(const ros::Time& time)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter TRACK");
  }
  double roll_real, pitch_real, yaw_real;
  quatToRPY(odom2pitch_.transform.rotation, roll_real, pitch_real, yaw_real);
  double yaw_compute = yaw_real;
  double pitch_compute = -pitch_real;
  geometry_msgs::Point target_pos = data_track_.position;
  geometry_msgs::Vector3 target_vel{};
  if (data_track_.id != 12)
    target_vel = data_track_.velocity;
  try
  {
    if (!data_track_.header.frame_id.empty())
    {
      geometry_msgs::TransformStamped transform =
          robot_state_handle_.lookupTransform("odom", data_track_.header.frame_id, data_track_.header.stamp);
      tf2::doTransform(target_pos, target_pos, transform);
      tf2::doTransform(target_vel, target_vel, transform);
    }
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
  double yaw = data_track_.yaw + data_track_.v_yaw * ((time - data_track_.header.stamp).toSec());
  while (yaw > M_PI)
    yaw -= 2 * M_PI;
  while (yaw < -M_PI)
    yaw += 2 * M_PI;
  target_pos.x += target_vel.x * (time - data_track_.header.stamp).toSec() - odom2pitch_.transform.translation.x;
  target_pos.y += target_vel.y * (time - data_track_.header.stamp).toSec() - odom2pitch_.transform.translation.y;
  target_pos.z += target_vel.z * (time - data_track_.header.stamp).toSec() - odom2pitch_.transform.translation.z;
  target_vel.x -= chassis_vel_->linear_->x();
  target_vel.y -= chassis_vel_->linear_->y();
  target_vel.z -= chassis_vel_->linear_->z();
  bool solve_success = bullet_solver_->solve(target_pos, target_vel, cmd_gimbal_.bullet_speed, yaw, data_track_.v_yaw,
                                             data_track_.radius_1, data_track_.radius_2, data_track_.dz,
                                             data_track_.armors_num, chassis_vel_->angular_->z());
  bullet_solver_->judgeShootBeforehand(time, data_track_.v_yaw);

  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
  {
    if (error_pub_->trylock())
    {
      double error =
          bullet_solver_->getGimbalError(target_pos, target_vel, data_track_.yaw, data_track_.v_yaw,
                                         data_track_.radius_1, data_track_.radius_2, data_track_.dz,
                                         data_track_.armors_num, yaw_compute, pitch_compute, cmd_gimbal_.bullet_speed);
      error_pub_->msg_.stamp = time;
      error_pub_->msg_.error = solve_success ? error : 1.0;
      error_pub_->unlockAndPublish();
    } 
    bullet_solver_->bulletModelPub(odom2pitch_, time);
    last_publish_time_ = time;
  }

  if (solve_success)
    setDes(time, bullet_solver_->getYaw(), bullet_solver_->getPitch());
  else
  {
    odom2gimbal_des_.header.stamp = time;
    robot_state_handle_.setTransform(odom2gimbal_des_, "rm_gimbal_controllers");
  }
}

void Controller::direct(const ros::Time& time)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter DIRECT");
  }
  geometry_msgs::Point aim_point_odom = cmd_gimbal_.target_pos.point;
  try
  {
    if (!cmd_gimbal_.target_pos.header.frame_id.empty())
      tf2::doTransform(aim_point_odom, aim_point_odom,
                       robot_state_handle_.lookupTransform("odom", cmd_gimbal_.target_pos.header.frame_id,
                                                           cmd_gimbal_.target_pos.header.stamp));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
  double yaw = std::atan2(aim_point_odom.y - odom2pitch_.transform.translation.y,
                          aim_point_odom.x - odom2pitch_.transform.translation.x);
  double pitch = -std::atan2(aim_point_odom.z - odom2pitch_.transform.translation.z,
                             std::sqrt(std::pow(aim_point_odom.x - odom2pitch_.transform.translation.x, 2) +
                                       std::pow(aim_point_odom.y - odom2pitch_.transform.translation.y, 2)));
  setDes(time, yaw, pitch);
}

void Controller::traj(const ros::Time& time)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter TRAJ");
  }
  setDes(time, cmd_gimbal_.traj_yaw, cmd_gimbal_.traj_pitch);
}

bool Controller::setDesIntoLimit(double& real_des, double current_des, double base2gimbal_current_des,
                                 const urdf::JointConstSharedPtr& joint_urdf)
{
  double upper_limit, lower_limit;
  upper_limit = joint_urdf->limits ? joint_urdf->limits->upper : 1e16;
  lower_limit = joint_urdf->limits ? joint_urdf->limits->lower : -1e16;
  if ((base2gimbal_current_des <= upper_limit && base2gimbal_current_des >= lower_limit) ||
      (angles::two_pi_complement(base2gimbal_current_des) <= upper_limit &&
       angles::two_pi_complement(base2gimbal_current_des) >= lower_limit))
    real_des = current_des;
  else
    return false;
  return true;
}

void Controller::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  geometry_msgs::Vector3 gyro, angular_vel_pitch, angular_vel_yaw,angular_vel_base_yaw;
  //test
  if (has_imu_)
  {
    gyro.x = imu_sensor_handle_.getAngularVelocity()[0];
    gyro.y = imu_sensor_handle_.getAngularVelocity()[1];
    gyro.z = imu_sensor_handle_.getAngularVelocity()[2];
    try
    {
      tf2::doTransform(gyro, angular_vel_pitch,
                       robot_state_handle_.lookupTransform(pitch_joint_urdf_->child_link_name,
                                                           imu_sensor_handle_.getFrameId(), time));
      tf2::doTransform(gyro, angular_vel_yaw,
                       robot_state_handle_.lookupTransform(yaw_joint_urdf_->child_link_name,
                                                           imu_sensor_handle_.getFrameId(), time));
      tf2::doTransform(gyro, angular_vel_base_yaw,
                       robot_state_handle_.lookupTransform(base_yaw_joint_urdf_->child_link_name,
                                                           imu_sensor_handle_.getFrameId(), time));                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
  }
  else
  {
    angular_vel_yaw.z = ctrl_yaw_.joint_.getVelocity();
    angular_vel_pitch.y = ctrl_pitch_.joint_.getVelocity();
    angular_vel_base_yaw.z = ctrl_base_yaw_.joint_.getVelocity();
  }
  double roll_real, pitch_real, yaw_real, roll_des, pitch_des, yaw_des,base_yaw_real, base_yaw_des;
  quatToRPY(odom2gimbal_des_.transform.rotation, roll_des, pitch_des, yaw_des);
  quatToRPY(odom2pitch_.transform.rotation, roll_real, pitch_real, yaw_real);
  quatToRPY(odom2base_yaw_.transform.rotation, roll_real, pitch_real, base_yaw_real);

  //double yaw_angle_error = angles::shortest_angular_distance(yaw_real, yaw_des);

  double pitch_angle_error = angles::shortest_angular_distance(pitch_real, pitch_des);
  pid_pitch_pos_.computeCommand(pitch_angle_error, period);
  
  // LQR control
      // yaw_real = angles::normalize_angle_positive(yaw_real);
      // yaw_des = angles::normalize_angle_positive(yaw_des);
      // base_yaw_real = angles::normalize_angle_positive(base_yaw_real);
      base_yaw_des = yaw_des;
      state_yaw_ << yaw_real, angular_vel_yaw.z, base_yaw_real, angular_vel_base_yaw.z;
      u = -K_yaw_ * state_yaw_;
      //ROS_INFO("State: [%.2f, %.2f, %.2f, %.2f], Command: [%.2f, %.2f]", state_yaw_(0), state_yaw_(1), state_yaw_(2), state_yaw_(3), u(0), u(1));
      double u_yaw = u(0);
     
      //double u_pitch = u(1);
  // pid_yaw_pos_.computeCommand(yaw_angle_error, period);
  double yaw_vel_des = 0., pitch_vel_des = 0.,base_yaw_vel_des=0.;
  if (state_ == RATE)
  {
    yaw_vel_des = cmd_gimbal_.rate_yaw;
    base_yaw_vel_des = cmd_gimbal_.rate_yaw;
    pitch_vel_des = cmd_gimbal_.rate_pitch;
  }
  else if (state_ == TRACK)
  {
    geometry_msgs::Point target_pos;
    geometry_msgs::Vector3 target_vel;
    bullet_solver_->getSelectedArmorPosAndVel(target_pos, target_vel, data_track_.position, data_track_.velocity,
                                              data_track_.yaw, data_track_.v_yaw, data_track_.radius_1,
                                              data_track_.radius_2, data_track_.dz, data_track_.armors_num);
    tf2::Vector3 target_pos_tf, target_vel_tf;
    try
    {
      geometry_msgs::TransformStamped transform = robot_state_handle_.lookupTransform(
          yaw_joint_urdf_->parent_link_name, data_track_.header.frame_id, data_track_.header.stamp);
      tf2::doTransform(target_pos, target_pos, transform);
      tf2::doTransform(target_vel, target_vel, transform);
      tf2::fromMsg(target_pos, target_pos_tf);
      tf2::fromMsg(target_vel, target_vel_tf);

      yaw_vel_des = target_pos_tf.cross(target_vel_tf).z() / std::pow((target_pos_tf.length()), 2);
      transform = robot_state_handle_.lookupTransform(pitch_joint_urdf_->parent_link_name, data_track_.header.frame_id,
                                                      data_track_.header.stamp);
      tf2::doTransform(target_pos, target_pos, transform);
      tf2::doTransform(target_vel, target_vel, transform);
      tf2::fromMsg(target_pos, target_pos_tf);
      tf2::fromMsg(target_vel, target_vel_tf);
      pitch_vel_des = target_pos_tf.cross(target_vel_tf).y() / std::pow((target_pos_tf.length()), 2);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
  }
 if (!pitch_des_in_limit_)
   pitch_vel_des = 0.;
 if (!yaw_des_in_limit_)
   yaw_vel_des = 0.;

  // pid_pitch_pos_.computeCommand(pitch_angle_error, period);
  
  // publish state
  //test

  x_ref << yaw_des, yaw_vel_des, base_yaw_des, base_yaw_vel_des;
  //ROS_INFO("yaw_vel_des: %lf, pitch_vel_des: %lf", yaw_vel_des, pitch_vel_des);
  Eigen::Vector4d x_err = state_yaw_ - x_ref;
  // x_err(0) = yaw_angle_error;
  u = -K_current_ * x_err;
  u_yaw = u(0);
  // ROS_WARN("K matrix:\n%lf",u(1));
  // ROS_WARN("K2 matrix:\n%lf",u(0));
  // pid_yaw_pos_.computeCommand(u_yaw, period);
  //test
  if (loop_count_ % 10 == 0)
  {
    if (yaw_pos_state_pub_ && yaw_pos_state_pub_->trylock())
    {
      yaw_pos_state_pub_->msg_.header.stamp = time;
      yaw_pos_state_pub_->msg_.set_point = yaw_des;
      yaw_pos_state_pub_->msg_.set_point_dot = yaw_vel_des;
      yaw_pos_state_pub_->msg_.process_value = yaw_real;
      yaw_pos_state_pub_->msg_.error = x_err(0);
      // yaw_pos_state_pub_->msg_.command = pid_yaw_pos_.getCurrentCmd();

      yaw_pos_state_pub_->msg_.command = u_yaw;

      yaw_pos_state_pub_->unlockAndPublish();
    }
    if (pitch_pos_state_pub_ && pitch_pos_state_pub_->trylock())
    {
      pitch_pos_state_pub_->msg_.header.stamp = time;
      pitch_pos_state_pub_->msg_.set_point = pitch_des;
      pitch_pos_state_pub_->msg_.set_point_dot = pitch_vel_des;
      pitch_pos_state_pub_->msg_.process_value = pitch_real;
      pitch_pos_state_pub_->msg_.error = angles::shortest_angular_distance(pitch_real, pitch_des);
      pitch_pos_state_pub_->msg_.command = pid_pitch_pos_.getCurrentCmd();
      pitch_pos_state_pub_->unlockAndPublish();
    }
  }
  loop_count_++;
  double yaw_pos = ctrl_yaw_.joint_.getPosition();
  double base_yaw_pos = ctrl_base_yaw_.joint_.getPosition();

// 计算角度差
  double sync_gain = -0.3;  // 同步增益，可调
  double angle_diff = angles::shortest_angular_distance(yaw_pos, base_yaw_pos);
  u_yaw = kf_yaw_.update(u(0));  // 对 yaw 输出滤波
double u_base_yaw = kf_base_yaw_.update(u(1)+ sync_gain * angle_diff);  // 对 base_yaw 输出滤波并加入同步控制

  ctrl_yaw_.setCommand(u_yaw);
  ctrl_base_yaw_.setCommand(u_base_yaw);
  ctrl_pitch_.setCommand(pid_pitch_pos_.getCurrentCmd() + config_.pitch_k_v_ * pitch_vel_des +
                         ctrl_pitch_.joint_.getVelocity() - angular_vel_pitch.y);

  ctrl_yaw_.update(time, period);
  ctrl_base_yaw_.update(time, period);
  ctrl_pitch_.update(time, period);
  ctrl_pitch_.joint_.setCommand(ctrl_pitch_.joint_.getCommand() + feedForward(time));
  updateRLS();
  // onlineLQRUpdate();

}


double Controller::feedForward(const ros::Time& time)
{
  Eigen::Vector3d gravity(0, 0, -gravity_);
  tf2::doTransform(gravity, gravity,
                   robot_state_handle_.lookupTransform(pitch_joint_urdf_->child_link_name, "base_link", time));
  Eigen::Vector3d mass_origin(mass_origin_.x, 0, mass_origin_.z);
  double feedforward = -mass_origin.cross(gravity).y();
  if (enable_gravity_compensation_)
  {
    Eigen::Vector3d gravity_compensation(0, 0, gravity_);
    tf2::doTransform(gravity_compensation, gravity_compensation,
                     robot_state_handle_.lookupTransform(pitch_joint_urdf_->child_link_name,
                                                         pitch_joint_urdf_->parent_link_name, time));
    feedforward -= mass_origin.cross(gravity_compensation).y();
  }
  return feedforward;
}

void Controller::updateChassisVel()
{
  double tf_period = odom2base_.header.stamp.toSec() - last_odom2base_.header.stamp.toSec();
  double linear_x = (odom2base_.transform.translation.x - last_odom2base_.transform.translation.x) / tf_period;
  double linear_y = (odom2base_.transform.translation.y - last_odom2base_.transform.translation.y) / tf_period;
  double linear_z = (odom2base_.transform.translation.z - last_odom2base_.transform.translation.z) / tf_period;
  double last_angular_position_x, last_angular_position_y, last_angular_position_z, angular_position_x,
      angular_position_y, angular_position_z;
  quatToRPY(odom2base_.transform.rotation, angular_position_x, angular_position_y, angular_position_z);
  quatToRPY(last_odom2base_.transform.rotation, last_angular_position_x, last_angular_position_y,
            last_angular_position_z);
  double angular_x = angles::shortest_angular_distance(last_angular_position_x, angular_position_x) / tf_period;
  double angular_y = angles::shortest_angular_distance(last_angular_position_y, angular_position_y) / tf_period;
  double angular_z = angles::shortest_angular_distance(last_angular_position_z, angular_position_z) / tf_period;
  double linear_vel[3]{ linear_x, linear_y, linear_z };
  double angular_vel[3]{ angular_x, angular_y, angular_z };
  chassis_vel_->update(linear_vel, angular_vel, tf_period);
  last_odom2base_ = odom2base_;
}

void Controller::updateRLS()
{
  // 构建当前状态向量（确保顺序一致）
  Eigen::VectorXd x(n_);
  x(0) = ctrl_yaw_.joint_.getPosition();
  x(1) = ctrl_yaw_.joint_.getVelocity();
  x(2) = ctrl_base_yaw_.joint_.getPosition();
  x(3) = ctrl_base_yaw_.joint_.getVelocity();
  
  if (!enable_online_lqr) return;
  
  // 一次性处理K矩阵平滑过渡
  {
    std::lock_guard<std::mutex> lk_rls(rls_mutex_);
    
    if (have_prev_sample_ && !driver_saturated_) {
      // 安全检查维度
      if (x_prev_.size() != n_ || u_prev_sample_.size() != m_) {
        ROS_ERROR_THROTTLE(1.0, "维度错误: x_prev_(%ld) != n_(%d) 或 u_prev_sample_(%ld) != m_(%d)",
                 x_prev_.size(), n_, u_prev_sample_.size(), m_);
        x_prev_.resize(n_);
        u_prev_sample_.resize(m_);
        return;
      }
      
      // 检查输入数据有效性
      if (!x.allFinite() || !x_prev_.allFinite() || !u_prev_sample_.allFinite()) {
        ROS_ERROR_THROTTLE(1.0, "输入数据包含NaN/Inf，跳过更新");
        return;
      }
      
      // 构建回归向量
      Eigen::VectorXd phi(n_ + m_);
      phi << x_prev_, u_prev_sample_;
      
      // 预先计算并检查Pphi是否有效
      Eigen::VectorXd Pphi = Pcov_ * phi;
      if (!Pphi.allFinite()) {
        ROS_ERROR_THROTTLE(1.0, "Pphi包含NaN/Inf，重置协方差矩阵");
        Pcov_ = Eigen::MatrixXd::Identity(n_ + m_, n_ + m_) * 1.0;  // 使用较小的值
        Pphi = Pcov_ * phi;
      }
      
      // 稳健的分母计算
      double denom = lambda_rls_ + phi.dot(Pphi);
      if (denom < 1e-6) {
        ROS_WARN_THROTTLE(1.0, "RLS分母过小(%e)，使用最小阈值", denom);
        denom = 1e-6;
      }
      
      // 计算并检查Kk
      Eigen::VectorXd Kk = Pphi / denom;
      if (!Kk.allFinite()) {
        ROS_ERROR_THROTTLE(1.0, "Kk包含NaN/Inf，跳过更新");
        return;
      }
      
      // 计算预测误差
      Eigen::VectorXd pred = Theta_ * phi;
      if (!pred.allFinite()) {
        ROS_ERROR_THROTTLE(1.0, "pred包含NaN/Inf，重置Theta矩阵");
        // 重置Theta到安全默认值
        resetThetaToDefault();
        return;
      }
      
      Eigen::VectorXd err = x - pred;
      if (err.norm() > 10.0) {  // 限制误差大小
        ROS_WARN_THROTTLE(1.0, "误差范数过大(%f)，限制大小", err.norm());
        err = err * (10.0 / err.norm());
      }
      
      // 限制单次更新幅度
      Eigen::MatrixXd delta = err * Kk.transpose();
      double delta_norm = delta.norm();
      double max_delta = 0.005;  // 更严格的更新限制
      if (delta_norm > max_delta) {
        ROS_WARN_THROTTLE(1.0, "delta过大(%f)，缩放", delta_norm);
        delta = delta * (max_delta / delta_norm);
      }
      
      // 更新Theta并立即验证
      Eigen::MatrixXd Theta_new = Theta_ + delta;
      if (!Theta_new.allFinite()) {
        ROS_ERROR_THROTTLE(1.0, "更新后的Theta包含NaN/Inf，保留原值");
        return;
      }
      
      // 应用更新
      Theta_ = Theta_new;
      
      // 更新协方差矩阵，使用更稳定的Joseph形式
      Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_ + m_, n_ + m_);
      Eigen::MatrixXd temp = I - Kk * phi.transpose();
      Pcov_ = temp * Pcov_ * temp.transpose() + Kk * Kk.transpose() / lambda_rls_;
      
      // 确保协方差矩阵对称
      Pcov_ = 0.5 * (Pcov_ + Pcov_.transpose());
      
      // 检查协方差矩阵
      if (!Pcov_.allFinite()) {
        ROS_ERROR_THROTTLE(1.0, "Pcov包含NaN/Inf，重置");
        Pcov_ = Eigen::MatrixXd::Identity(n_ + m_, n_ + m_) * 1.0;
      }
      
      // 应用物理约束
      applyPhysicalConstraints();
      
      // 维护样本窗口...
    }
    
    // 更新前一个状态和输入
    x_prev_ = x;
    u_prev_sample_.resize(m_);
    u_prev_sample_(0) = ctrl_yaw_.joint_.getCommand();
    u_prev_sample_(1) = ctrl_base_yaw_.joint_.getCommand();
    have_prev_sample_ = true;
  }
}
void Controller::onlineLQRUpdate()
{
  if (!enable_online_lqr) return;
  
  ros::Rate rate(worker_hz_);
  
  // 添加状态变量监控日志
  ROS_INFO("Adaptive LQR started : %d Hz", worker_hz_);
  
  while (ros::ok() && running_) {
    try {
      // 获取Theta矩阵副本
      Eigen::MatrixXd Theta_copy;
      {
        std::lock_guard<std::mutex> lk(rls_mutex_);
        if (sample_count_ < (size_t)N_min_samples_) {
          rate.sleep();
          continue;
        }
        Theta_copy = Theta_;
      }
      
      // 安全检查
      if (!Theta_copy.allFinite() || Theta_copy.rows() != n_ || Theta_copy.cols() != n_ + m_) {
        ROS_WARN_THROTTLE(5.0, "Theta error: %ldx%ld", Theta_copy.rows(), Theta_copy.cols());
        rate.sleep();
        continue;
      }
      
      // 分解Theta矩阵
      Eigen::MatrixXd A_est = Theta_copy.block(0, 0, n_, n_);
      Eigen::MatrixXd B_est = Theta_copy.block(0, n_, n_, m_);
      
      // 计算控制增益
      Eigen::MatrixXd K_new(m_, n_);
      bool ok = computeDLQRdiscrete(A_est, B_est, Qd_, Rd_, K_new);
      
      if (!ok || !K_new.allFinite() || !validateK(A_est, B_est, K_new)) {
        ROS_WARN_THROTTLE(5.0, "LQR compute failed or invalid K");
        rate.sleep();
        continue;
      }
      if (!ok || !K_new.allFinite() || K_new.norm() < 0.1) {
          ROS_WARN("LQR compute failed or K near zero, using initial K");
          K_new = K_yaw_;  // 使用初始K矩阵
        }
      // 平稳过渡逻辑
      double diff = (K_new - last_successful_K_).norm();
      if (last_successful_K_.rows() == 0) {
        last_successful_K_ = K_new;
        stable_count_ = 1;
      } else if (diff < stable_tol_) {
        stable_count_++;
      } else {
        stable_count_ = 1;
        last_successful_K_ = K_new;
      }
      
      if (stable_count_ >= stable_needed_) {
        {
          std::lock_guard<std::mutex> lk(K_mutex_);
          K_target_ = K_new;
          switching_ = true;
          switch_start_time_ = ros::Time::now();
          u_max_ = 0.5 * u_max_normal_;
        }
        ROS_INFO("Adaptive LQR: New gain accepted and scheduled for smooth switching");
        stable_count_ = 0;
        last_successful_K_ = K_new;
      }
      if (stable_count_ >= stable_needed_) {
    {
      std::lock_guard<std::mutex> lk(K_mutex_);
      K_target_ = K_new;
      switching_ = true;
      switch_start_time_ = ros::Time::now();
      u_max_ = 0.5 * u_max_normal_;
      
      // 发布K矩阵
      std_msgs::Float64MultiArray k_msg;
      k_msg.layout.dim.resize(2);
      k_msg.layout.dim[0].label = "rows";
      k_msg.layout.dim[0].size = K_new.rows();
      k_msg.layout.dim[0].stride = K_new.rows() * K_new.cols();
      k_msg.layout.dim[1].label = "cols";
      k_msg.layout.dim[1].size = K_new.cols();
      k_msg.layout.dim[1].stride = K_new.cols();
      
      k_msg.data.clear();
      for (int i = 0; i < K_new.rows(); i++) {
        for (int j = 0; j < K_new.cols(); j++) {
          k_msg.data.push_back(K_new(i, j));
        }
      }
      
      k_matrix_pub_.publish(k_msg);
    }
    // ... 现有代码继续 ...
  }
      
    } catch (const std::exception& e) {
      ROS_ERROR("Adaptive LQR exception: %s", e.what());
    }
    
    rate.sleep();
  }

  ROS_INFO("Adaptive LQR thread terminated");
}
bool Controller::computeDLQRdiscrete(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                           const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
                           Eigen::MatrixXd& K_out){
  Eigen::MatrixXd P = Q;
    Eigen::MatrixXd At = A.transpose(), Bt = B.transpose();

    for (int i=0;i<500;i++){
      Eigen::MatrixXd S = R + Bt * P * B;
      if (S.determinant() == 0) return false;
      Eigen::MatrixXd S_inv = S.inverse();
      Eigen::MatrixXd Pnext = At * P * A - At * P * B * S_inv * Bt * P * A + Q;
      if ((Pnext - P).norm() < 1e-9) { P = Pnext; break; }
      P = Pnext;
    }

    Eigen::MatrixXd denom = R + Bt * P * B;
    if (denom.determinant() == 0) return false;
    K_out = denom.inverse() * Bt * P * A; // m x n
    return true;
                           }
bool Controller::validateK(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& K){
        // 1) 闭环稳定性（离散系统：模 < 1）
  Eigen::MatrixXd M = A - B * K;
  Eigen::EigenSolver<Eigen::MatrixXd> es(M);
  for (int i=0;i<M.rows();++i) {
    if (std::abs(es.eigenvalues()[i]) >= 1.0) {
      ROS_WARN("validateK: closed-loop eigenvalue magnitude >= 1");
      return false;
    }
  }
  // 2) 幅值测试（用小扰动状态检查控制量大小）
  for (int i=0;i<5;i++){
    Eigen::VectorXd xt = Eigen::VectorXd::Random(n_) * 0.1;
    Eigen::VectorXd utest = -K * xt;
    if (utest.cwiseAbs().maxCoeff() > u_max_normal_ * 2.0) {
      ROS_WARN("validateK: control amplitude too large");
      return false;
    }
  }
  return true;
    }                      
double Controller::computeResidual(const Eigen::MatrixXd& Theta_copy){
    int M = std::min((int)recent_phi_.size(), recent_buffer_len_);
  if (M < 5) return 1e6;
  double sumsq = 0.0;
  for (int i=0;i<M;i++){
    Eigen::VectorXd pred = Theta_copy * recent_phi_[i];
    Eigen::VectorXd err = recent_xnext_[i] - pred;
    sumsq += err.squaredNorm();
  }
  return sumsq / (double)M;
}

void Controller::resetThetaToDefault()
{
  // 使用物理模型初始化Theta
  Theta_ = Eigen::MatrixXd::Zero(n_, n_ + m_);
  
  // 构建默认的A和B矩阵
  Eigen::MatrixXd A(n_, n_);
  A << 0., 1., 0., 0.,
       0., -0.1, 0., 0.002,  // 使用安全的默认值
       0., 0., 0., 1.,
       0., 0.002, 0., -0.1;
       
  Eigen::MatrixXd B(n_, m_);
  B << 0., 0.,
       1., 0.,
       0., 0.,
       0., 1.;
       
  // 将A和B放入Theta
  Theta_.block(0, 0, n_, n_) = A;
  Theta_.block(0, n_, n_, m_) = B;
  
  // 重置协方差矩阵
  Pcov_ = Eigen::MatrixXd::Identity(n_ + m_, n_ + m_) * 1.0;
  sample_count_ = 0;
}

// 添加这个新函数到Controller类
void Controller::applyPhysicalConstraints()
{
  // 确保阻尼系数为负
  if (Theta_(1,1) >= 0) {
    Theta_(1,1) = -0.01;  // 固定的安全负值
  }
  if (Theta_(3,3) >= 0) {
    Theta_(3,3) = -0.01;  // 固定的安全负值
  }
  
  // 限制耦合项的大小
  double max_coupling = 0.005;  // 固定的最大耦合值
  if (std::abs(Theta_(1,3)) > max_coupling) {
    Theta_(1,3) = std::copysign(max_coupling, Theta_(1,3));
  }
  if (std::abs(Theta_(3,1)) > max_coupling) {
    Theta_(3,1) = std::copysign(max_coupling, Theta_(3,1));
  }
  
  // 确保B矩阵列对应正确的输入影响
  if (Theta_(1,n_) <= 0) {  // 第一个输入应正向影响yaw速度
    Theta_(1,n_) = 0.5;  // 安全正值
  }
  if (Theta_(3,n_+1) <= 0) {  // 第二个输入应正向影响base_yaw速度
    Theta_(3,n_+1) = 0.5;  // 安全正值
  }
}

void Controller::commandCB(const rm_msgs::GimbalCmdConstPtr& msg)
{
  cmd_rt_buffer_.writeFromNonRT(*msg);
  //ROS_INFO("[Gimbal] Get new command");
}

void Controller::trackCB(const rm_msgs::TrackDataConstPtr& msg)
{
  if (msg->id == 0)
    return;
  track_rt_buffer_.writeFromNonRT(*msg);
}

void Controller::reconfigCB(rm_gimbal_controllers::GimbalBaseConfig& config, uint32_t /*unused*/)
{
  ROS_INFO("[Gimbal Base] Dynamic params change");
  if (!dynamic_reconfig_initialized_&&!enable_online_lqr)
  {
    GimbalConfig init_config = *config_rt_buffer_.readFromNonRT();  // config init use yaml
    config.yaw_k_v_ = init_config.yaw_k_v_;
    config.pitch_k_v_ = init_config.pitch_k_v_;
    config.k_chassis_vel_ = init_config.k_chassis_vel_;
    config.accel_pitch_ = init_config.accel_pitch_;
    config.accel_yaw_ = init_config.accel_yaw_;
    dynamic_reconfig_initialized_ = true;
      config_.q1_ = config.q1;
      config_.q2_ = config.q2;
      config_.q3_ = config.q3;
      config_.q4_ = config.q4;
      config_.r1_ = config.r1;
      config_.r2_ = config.r2;
      config_.a1_ = config.a1;
      config_.a2_ = config.a2;
      config_.j1_ = config.j1;
      config_.j2_ = config.j2;
      config_.dc_ = config.dc_;

    // ... 更新其他参数

    // 重新构建矩阵并计算 K
  Eigen::Matrix4d A, B, Q, R;
  // ... 相同构建代码
  //Lqr<double> lqr(A, B, Q, R, K_yaw_);
  //lqr.computeK(A, B, Q, R, K_yaw_);

  config_rt_buffer_.writeFromNonRT(config_);
  }
  GimbalConfig config_non_rt{ .yaw_k_v_ = config.yaw_k_v_,
                              .pitch_k_v_ = config.pitch_k_v_,
                              .k_chassis_vel_ = config.k_chassis_vel_,
                              .accel_pitch_ = config.accel_pitch_,
                              .accel_yaw_ = config.accel_yaw_ 
                              };
  
  config_rt_buffer_.writeFromNonRT(config_non_rt);
  
}

}  // namespace rm_gimbal_controllers

PLUGINLIB_EXPORT_CLASS(rm_gimbal_controllers::Controller, controller_interface::ControllerBase)
