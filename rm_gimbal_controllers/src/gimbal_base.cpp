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
//
#include "rm_gimbal_controllers/gimbal_base.h"

#include <atomic>
#include <chrono> 
#include <string>
#include <fstream>
#include <future>
#include <iomanip>
#include <mutex>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include <rm_common/lqr.h>






namespace rm_gimbal_controllers
{
bool Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh){
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
  
  if (!ctrl_yaw_.init(effort_joint_interface, nh_yaw) || 
      !ctrl_base_yaw_.init(effort_joint_interface, nh_base_yaw) ||
      !ctrl_pitch_.init(effort_joint_interface, nh_pitch) ||
      !pid_pitch_pos_.init(nh_pid_pitch_pos))
  {
    ROS_ERROR("Failed to initialize joint controllers or PIDs");
    return false;
  }

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

  double r_yaw = getParam(controller_nh, "yaw/tracking_differentiator_r", 200.0);
double r_base_yaw = getParam(controller_nh, "base_yaw/tracking_differentiator_r", 200.0);
td_yaw_ = std::make_unique<NonlinearTrackingDifferentiator<double>>(r_yaw, 0.001);
td_base_yaw_ = std::make_unique<NonlinearTrackingDifferentiator<double>>(r_base_yaw, 0.001);

  pitch_pos_state_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::GimbalPosState>(nh_pitch, "pos_state", 1));

  ramp_rate_pitch_ = new RampFilter<double>(0, 0.001);
  ramp_rate_yaw_ = new RampFilter<double>(0, 0.001);
  
    try {
    // 1. 基础参数
    enable_online_lqr_ = getParam(controller_nh, "enable_online_lqr", false);
    
    // 2. 卡尔曼滤波器
    kf_yaw_ = KalmanFilter(constants::kKalmanProcessNoise, constants::kKalmanMeasurementNoise, constants::kKalmanInitialEstimate, constants::kKalmanInitialError);
    kf_base_yaw_ = KalmanFilter(constants::kKalmanProcessNoise, constants::kKalmanMeasurementNoise, constants::kKalmanInitialEstimate, constants::kKalmanInitialError);

    // 3. 物理参数
    config_.a1_ = getParam(controller_nh, "a1", 0.1);
    config_.a2_ = getParam(controller_nh, "a2", 0.1);
    config_.j1_ = getParam(controller_nh, "j1", 1.0);
    config_.j2_ = getParam(controller_nh, "j2", 1.0);
    config_.dc_ = getParam(controller_nh, "dc", 0.0);
    
    // 4. LQR权重参数
    config_.q1_ = getParam(controller_nh, "q1", 100.0);
    config_.q2_ = getParam(controller_nh, "q2", 10.0);
    config_.q3_ = getParam(controller_nh, "q3", 100.0);
    config_.q4_ = getParam(controller_nh, "q4", 10.0);
    config_.r1_ = getParam(controller_nh, "r1", 1.0);
    config_.r2_ = getParam(controller_nh, "r2", 1.0);
    
    // 5. 矩阵维度
    n_ = 4;
    m_ = 2;
    
    constexpr double dt = 0.001;
    constexpr double velocity_decay = 0.95;
    constexpr double input_gain = 0.8;  // ✅ 实际增益值
    
    // 分配内存
    system_matrix_a_.resize(n_, n_);
    control_matrix_b_.resize(n_, m_);
    state_weight_q_.resize(n_, n_);
    control_weight_r_.resize(m_, m_);

    // A矩阵：离散状态转移矩阵
    system_matrix_a_ << 1.0, dt,  0.0, 0.0,
                        0.0, velocity_decay, 0.0, 0.0,
                        0.0, 0.0, 1.0, dt,
                        0.0, 0.0, 0.0, velocity_decay;

    // B矩阵：控制输入到速度的直接影响
    control_matrix_b_ << 0.0, 0.0,
                         input_gain * dt, 0.0,    // ✅ 使用 input_gain * dt
                         0.0, 0.0,
                         0.0, input_gain * dt;

    // Q和R权重矩阵保持不变
    state_weight_q_ << config_.q1_, 0., 0., 0.,
        0., config_.q2_, 0., 0.,
        0., 0., config_.q3_, 0.,
        0., 0., 0., config_.q4_;

    control_weight_r_ << config_.r1_, 0.,
        0., config_.r2_;

    
    // 7. 初始化状态向量
    state_yaw_.resize(4);
    x_ref.resize(4);
    u.resize(m_);
    u.setZero();
    
    // 8. 计算初始LQR增益
    Lqr<double> lqr(system_matrix_a_, control_matrix_b_, state_weight_q_, control_weight_r_, K_yaw_);
    if (!lqr.computeK(system_matrix_a_, control_matrix_b_, state_weight_q_, control_weight_r_, K_yaw_)) {
      K_yaw_ = Eigen::MatrixXd::Zero(2, 4);
      ROS_WARN("LQR computation failed, using zero K matrix");
    }
    
    // 9. 尝试从文件加载K矩阵
    std::string k_matrix_path = "/home/what/ros_ws/rm_ws/src/rm_controllers/rm_gimbal_controllers/config/k_matrix.yaml";
    Eigen::MatrixXd loaded_K;
    if (loadKFromYaml(k_matrix_path, loaded_K)) {
      K_yaw_ = loaded_K;
      ROS_INFO("Using K matrix loaded from file");
    } else {
      ROS_INFO("Using computed K matrix");
    }
    
    // 10. 初始化所有K矩阵副本
    K_current_ = K_yaw_;
    K_old_ = K_yaw_;
    K_target_ = K_yaw_;
    K_safe_read_ = K_yaw_;  // ✅ 初始化读取缓存
    K_ready_ = true;         // ✅ 标记已就绪
    last_successful_K_ = K_yaw_;
    
    // 11. 初始化RLS
    resetThetaToDefault();
    Pcov_ = Eigen::MatrixXd::Identity(n_ + m_, n_ + m_) * 1.0;
    
    u_prev_sample_.resize(m_);
    x_prev_.resize(n_);
    u_prev_sample_.setZero();
    x_prev_.setZero();
    have_prev_sample_ = false;
    
    // 12. 初始化控制输出
    u_yaw_ = 0.0;
    u_base_yaw_ = 0.0;
    u_yaw_cmd_ = 0.0;
    u_base_yaw_cmd_ = 0.0;
    driver_saturated_ = false;
    
    // 13. 其他参数
    recent_buffer_len_ = constants::kRlsBufferLength;
    Qd_ = state_weight_q_;
    Rd_ = control_weight_r_;
    running_ = false;
    stable_count_ = 0;
    switching_ = false;
    sample_count_ = 0;
    update_strategy_ = STRATEGY_INITIAL;
    successful_updates_ = 0;
    recent_avg_error_ = 0.0;
    error_window_.clear();
    
    lambda_rls_ = getParam(controller_nh, "lambda_rls", 0.995);
    worker_hz_ = getParam(controller_nh, "worker_hz", 2);
    switch_smooth_T_ = getParam(controller_nh, "switch_smooth_T", 1.0);
    stable_needed_ = getParam(controller_nh, "stable_needed", 5);
    stable_tol_ = getParam(controller_nh, "stable_tol", 0.05);
    u_max_normal_ = getParam(controller_nh, "u_max_normal", 20.0);
    u_max_ = u_max_normal_;
    N_min_samples_ = constants::kRlsMinSamples;
    
    k_matrix_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("debug/k_matrix", 10);
    
    ROS_INFO("Gimbal controller initialization completed successfully");
    
  } catch (const std::exception& e) {
    ROS_ERROR("Exception in init: %s", e.what());
    return false;
  }
  
  return true;
}
void Controller::starting(const ros::Time& /*unused*/){
  state_ = RATE;
  state_changed_ = true;
  start_ = true;
  
  // ✅ 初始化所有滤波器状态变量（防止Gazebo崩溃）
  last_yaw_vel_ = 0.0;
  last_base_yaw_vel_ = 0.0;
  last_pitch_vel_ = 0.0;
  yaw_vel_buffer_.clear();
  base_yaw_vel_buffer_.clear();
  pitch_vel_buffer_.clear();
  
  error_history_.clear();
  error_history_.resize(10, 0.0);
  u_yaw_stage1_ = 0.0;
  u_base_yaw_stage1_ = 0.0;
  filter_initialized_ = false;
  u_yaw_history_.clear();
  u_base_yaw_history_.clear();
  u_yaw_final_ = 0.0;
  u_base_yaw_final_ = 0.0;
  
  error_samples_.clear();
  
  // ✅ 初始化K矩阵读取缓存
  {
    std::lock_guard<std::mutex> lk(K_mutex_);
    K_safe_read_ = K_current_;
    K_ready_ = true;
  }
  
  ROS_INFO("Filter states initialized");
  
  ROS_INFO("[Gimbal] enable_online_lqr_=%d, running_=%d", enable_online_lqr_, running_.load());
  
  if (enable_online_lqr_ && !running_) {
    running_ = true;
    worker_thread_ = std::thread(&Controller::onlineLQRUpdate, this);
    ROS_INFO("[Gimbal] Adaptive LQR worker thread started (thread id: %lu)", 
             std::hash<std::thread::id>{}(worker_thread_.get_id()));
  } else {
    if (!enable_online_lqr_) {
      ROS_WARN("[Gimbal] Online LQR is DISABLED in config!");
    }
    if (running_) {
      ROS_WARN("[Gimbal] Worker thread already running!");
    }
  }
}

void Controller::stopping(const ros::Time& /*time*/){
  ROS_INFO("[Gimbal] Stopping controller...");
  
  // 1. 先停止在线LQR(防止主循环继续调用updateRLS)
  enable_online_lqr_ = false;
  
  // 2. 停止工作线程循环
  running_ = false;
  
  // 3. 清零所有控制输出(确保电机安全)
  try {
    ctrl_yaw_.setCommand(0.0);
    ctrl_base_yaw_.setCommand(0.0);
    ctrl_pitch_.setCommand(0.0);
  } catch (...) {
    ROS_ERROR("[Gimbal] Exception while clearing commands");
  }
  
  // 4. 等待工作线程退出
  if (worker_thread_.joinable())
  {
    ROS_INFO("[Gimbal] Waiting for worker thread to stop...");
    
    // 给工作线程300ms时间自然退出
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    
    // 检查是否还需要join
    if (worker_thread_.joinable()) {
      // 使用future实现带超时的join
      std::packaged_task<void()> task([this]() {
        if (worker_thread_.joinable()) {
          worker_thread_.join();
        }
      });
      
      auto future = task.get_future();
      std::thread(std::move(task)).detach();
      
      // 等待最多1秒
      if (future.wait_for(std::chrono::seconds(1)) == std::future_status::timeout) {
        ROS_WARN("[Gimbal] Worker thread timeout, forcing detach");
        if (worker_thread_.joinable()) {
          worker_thread_.detach();
        }
      } else {
        ROS_INFO("[Gimbal] Worker thread stopped successfully");
      }
    } else {
      ROS_INFO("[Gimbal] Worker thread already stopped");
    }
  }
  
  ROS_INFO("[Gimbal] Controller stopped");
}

void Controller::update(const ros::Time& time, const ros::Duration& period){
  cmd_gimbal_ = *cmd_rt_buffer_.readFromRT();
  data_track_ = *track_rt_buffer_.readFromNonRT();
  config_ = *config_rt_buffer_.readFromRT();
  ramp_rate_pitch_->setAcc(config_.accel_pitch_);
  ramp_rate_yaw_->setAcc(config_.accel_yaw_);
  ramp_rate_pitch_->input(cmd_gimbal_.rate_pitch);
  ramp_rate_yaw_->input(cmd_gimbal_.rate_yaw);
    if (cmd_gimbal_.rate_yaw != 0)
    ramp_rate_yaw_->clear(cmd_gimbal_.rate_yaw);
  if (cmd_gimbal_.rate_pitch != 0)
    ramp_rate_pitch_->clear(cmd_gimbal_.rate_pitch);
  cmd_gimbal_.rate_pitch = ramp_rate_pitch_->output();
  cmd_gimbal_.rate_yaw = ramp_rate_yaw_->output();
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

void Controller::setDes(const ros::Time& time, double yaw_des, double pitch_des){
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

void Controller::rate(const ros::Time& time, const ros::Duration& period){
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

void Controller::track(const ros::Time& time){
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

void Controller::direct(const ros::Time& time){
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

void Controller::traj(const ros::Time& time){
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter TRAJ");
  }
  setDes(time, cmd_gimbal_.traj_yaw, cmd_gimbal_.traj_pitch);
}

bool Controller::setDesIntoLimit(double& real_des, double current_des, double base2gimbal_current_des,
                                 const urdf::JointConstSharedPtr& joint_urdf){
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

void Controller::moveJoint(const ros::Time& time, const ros::Duration& period){
  geometry_msgs::Vector3 gyro, angular_vel_pitch, angular_vel_yaw, angular_vel_base_yaw;
  
  if (has_imu_) {
    // IMU 路径（保持不变）
    gyro.x = imu_sensor_handle_.getAngularVelocity()[0];
    gyro.y = imu_sensor_handle_.getAngularVelocity()[1];
    gyro.z = imu_sensor_handle_.getAngularVelocity()[2];
    try {
      tf2::doTransform(gyro, angular_vel_pitch,
                       robot_state_handle_.lookupTransform(pitch_joint_urdf_->child_link_name,
                                                           imu_sensor_handle_.getFrameId(), time));
      tf2::doTransform(gyro, angular_vel_yaw,
                       robot_state_handle_.lookupTransform(yaw_joint_urdf_->child_link_name,
                                                           imu_sensor_handle_.getFrameId(), time));
      tf2::doTransform(gyro, angular_vel_base_yaw,
                       robot_state_handle_.lookupTransform(base_yaw_joint_urdf_->child_link_name,
                                                           imu_sensor_handle_.getFrameId(), time));
    imu_yaw_vel_ = angular_vel_yaw.z;
    imu_base_yaw_vel_ = angular_vel_base_yaw.z;

    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      return;
    }
  } else {
    // ✅ 编码器路径 - 核心修复点
    double yaw_vel_raw = ctrl_yaw_.joint_.getVelocity();
    double base_yaw_vel_raw = ctrl_base_yaw_.joint_.getVelocity();
    double pitch_vel_raw = ctrl_pitch_.joint_.getVelocity();

    // ✅ 1. 异常值检测（防止编码器跳变）
    constexpr double kMaxVelChange = 100.0;  // rad/s²
    
    if (std::abs(yaw_vel_raw - last_yaw_vel_) > kMaxVelChange * 0.001) {
      ROS_WARN_THROTTLE(0.5, "Yaw vel spike detected: %.2f → %.2f, rejecting",
                       last_yaw_vel_, yaw_vel_raw);
      yaw_vel_raw = last_yaw_vel_;
    }
    if (std::abs(base_yaw_vel_raw - last_base_yaw_vel_) > kMaxVelChange * 0.001) {
      ROS_WARN_THROTTLE(0.5, "Base yaw vel spike detected, rejecting");
      base_yaw_vel_raw = last_base_yaw_vel_;
    }
    if (std::abs(pitch_vel_raw - last_pitch_vel_) > kMaxVelChange * 0.001) {
      ROS_WARN_THROTTLE(0.5, "Pitch vel spike detected, rejecting");
      pitch_vel_raw = last_pitch_vel_;
    }
    // ✅ 修复：先更新历史，再设置用于补偿的速度（使用滤波后的值）
    last_yaw_vel_ = yaw_vel_raw;
    last_base_yaw_vel_ = base_yaw_vel_raw;
    last_pitch_vel_ = pitch_vel_raw;
    
<<<<<<< HEAD
    // ✅ 2. 简化滤波（3点中值，减少计算量）
    constexpr size_t kMedianFilterSize = 3;  // ✅ 从5降到3，减少50%计算
    
    yaw_vel_buffer_.push_back(yaw_vel_raw);
    base_yaw_vel_buffer_.push_back(base_yaw_vel_raw);
    pitch_vel_buffer_.push_back(pitch_vel_raw);
    
    if (yaw_vel_buffer_.size() > kMedianFilterSize) yaw_vel_buffer_.pop_front();
    if (base_yaw_vel_buffer_.size() > kMedianFilterSize) base_yaw_vel_buffer_.pop_front();
    if (pitch_vel_buffer_.size() > kMedianFilterSize) pitch_vel_buffer_.pop_front();
    
    // 快速3点中值（无需排序）
    auto median3 = [](const std::deque<double>& buf) -> double {
=======
    last_yaw_vel_ = yaw_vel_raw;
    last_base_yaw_vel_ = base_yaw_vel_raw;
    last_pitch_vel_ = pitch_vel_raw;
    
    // ✅ 2. 中值滤波（移动窗口大小为5）
    constexpr size_t kMedianFilterSize = 5;
    
    yaw_vel_buffer_.push_back(yaw_vel_raw);
    base_yaw_vel_buffer_.push_back(base_yaw_vel_raw);
    pitch_vel_buffer_.push_back(pitch_vel_raw);
    
    if (yaw_vel_buffer_.size() > kMedianFilterSize) yaw_vel_buffer_.pop_front();
    if (base_yaw_vel_buffer_.size() > kMedianFilterSize) base_yaw_vel_buffer_.pop_front();
    if (pitch_vel_buffer_.size() > kMedianFilterSize) pitch_vel_buffer_.pop_front();
    
    // 计算中值 - 添加安全检查防止崩溃
    auto compute_median = [](const std::deque<double>& buf) -> double {
>>>>>>> c659d46648b3b440b38a2844f0b1ed0f93729fe9
      if (buf.empty()) return 0.0;
      if (buf.size() == 1) return buf[0];
      if (buf.size() == 2) return (buf[0] + buf[1]) * 0.5;
      // 3点中值：max(min(a,b), min(max(a,b),c))
      double a = buf[0], b = buf[1], c = buf[2];
      return std::max(std::min(a, b), std::min(std::max(a, b), c));
    };
    
<<<<<<< HEAD
    double yaw_vel_median = median3(yaw_vel_buffer_);
    double base_yaw_vel_median = median3(base_yaw_vel_buffer_);
    double pitch_vel_median = median3(pitch_vel_buffer_);
=======
    double yaw_vel_median = compute_median(yaw_vel_buffer_);
    double base_yaw_vel_median = compute_median(base_yaw_vel_buffer_);
    double pitch_vel_median = compute_median(pitch_vel_buffer_);
>>>>>>> c659d46648b3b440b38a2844f0b1ed0f93729fe9
    
    // ✅ 3. 一阶低通滤波（强滤波）
    constexpr double kVelFilterAlpha = 0.95;  // ✅ 从 0.85 提升到 0.95（更强平滑）
    
    yaw_vel_filtered_ = kVelFilterAlpha * yaw_vel_filtered_ + 
                       (1.0 - kVelFilterAlpha) * yaw_vel_median;
    base_yaw_vel_filtered_ = kVelFilterAlpha * base_yaw_vel_filtered_ + 
                            (1.0 - kVelFilterAlpha) * base_yaw_vel_median;
    pitch_vel_filtered_ = kVelFilterAlpha * pitch_vel_filtered_ + 
                         (1.0 - kVelFilterAlpha) * pitch_vel_median;
    
<<<<<<< HEAD
    // ✅ 批量 NaN 检查（减少分支预测失败）
    bool has_nan = !std::isfinite(yaw_vel_filtered_) || 
                   !std::isfinite(base_yaw_vel_filtered_) || 
                   !std::isfinite(pitch_vel_filtered_);
    if (has_nan) {
      ROS_ERROR_THROTTLE(1.0, "[VEL] NaN detected, resetting filters");
      if (!std::isfinite(yaw_vel_filtered_)) {
        yaw_vel_filtered_ = 0.0;
        yaw_vel_buffer_.clear();
      }
      if (!std::isfinite(base_yaw_vel_filtered_)) {
        base_yaw_vel_filtered_ = 0.0;
        base_yaw_vel_buffer_.clear();
      }
      if (!std::isfinite(pitch_vel_filtered_)) {
        pitch_vel_filtered_ = 0.0;
        pitch_vel_buffer_.clear();
      }
=======
    // ✅ NaN检查速度滤波器输出
    if (!std::isfinite(yaw_vel_filtered_)) {
      ROS_ERROR_THROTTLE(1.0, "yaw_vel_filtered is NaN, resetting");
      yaw_vel_filtered_ = 0.0;
      yaw_vel_buffer_.clear();
    }
    if (!std::isfinite(base_yaw_vel_filtered_)) {
      ROS_ERROR_THROTTLE(1.0, "base_yaw_vel_filtered is NaN, resetting");
      base_yaw_vel_filtered_ = 0.0;
      base_yaw_vel_buffer_.clear();
    }
    if (!std::isfinite(pitch_vel_filtered_)) {
      ROS_ERROR_THROTTLE(1.0, "pitch_vel_filtered is NaN, resetting");
      pitch_vel_filtered_ = 0.0;
      pitch_vel_buffer_.clear();
>>>>>>> c659d46648b3b440b38a2844f0b1ed0f93729fe9
    }
    
    // ✅ 4. 零点漂移消除（小速度归零）
    constexpr double kVelDeadzone = 0.05;  // rad/s
    if (std::abs(yaw_vel_filtered_) < kVelDeadzone) yaw_vel_filtered_ = 0.0;
    if (std::abs(base_yaw_vel_filtered_) < kVelDeadzone) base_yaw_vel_filtered_ = 0.0;
    if (std::abs(pitch_vel_filtered_) < kVelDeadzone) pitch_vel_filtered_ = 0.0;
    
    // 使用滤波后的速度
    angular_vel_yaw.z = yaw_vel_filtered_;
    angular_vel_base_yaw.z = base_yaw_vel_filtered_;
    angular_vel_pitch.y = pitch_vel_filtered_;
    
    // ✅ 修复：使用滤波后的速度作为补偿基准（消除噪声）
    imu_yaw_vel_ = yaw_vel_filtered_;
    imu_base_yaw_vel_ = base_yaw_vel_filtered_;
  }

  // ...existing code for angle calculation...
  double roll_real, pitch_real, yaw_real, roll_des, pitch_des, yaw_des, base_yaw_real, base_yaw_des;
  quatToRPY(odom2gimbal_des_.transform.rotation, roll_des, pitch_des, yaw_des);
  quatToRPY(odom2pitch_.transform.rotation, roll_real, pitch_real, yaw_real);
  
  double dummy_roll, dummy_pitch;
  try {
    // ✅ 实车：使用 ros::Time(0) 获取最新可用的 TF，避免外推错误
    odom2base_yaw_ = robot_state_handle_.lookupTransform("odom", base_yaw_joint_urdf_->child_link_name, 
                                                          ros::Time(0));
    quatToRPY(odom2base_yaw_.transform.rotation, dummy_roll, dummy_pitch, base_yaw_real);
  } catch (tf2::TransformException& ex) {
    // ✅ TF失败时使用关节编码器反馈作为后备（更可靠）
    ROS_WARN_THROTTLE(5.0, "Failed to get base_yaw transform: %s, using joint encoder", ex.what());
    base_yaw_real = ctrl_base_yaw_.joint_.getPosition();
  }
  
  double pitch_angle_error = angles::shortest_angular_distance(pitch_real, pitch_des);
  pid_pitch_pos_.computeCommand(pitch_angle_error, period);
  
  base_yaw_des = yaw_des;
  
  // ...existing code for velocity des calculation...
  double yaw_vel_des = 0., pitch_vel_des = 0., base_yaw_vel_des = 0.;
  if (state_ == RATE) {
    yaw_vel_des = cmd_gimbal_.rate_yaw;
    base_yaw_vel_des = cmd_gimbal_.rate_yaw;
    pitch_vel_des = cmd_gimbal_.rate_pitch;
  } else if (state_ == TRACK) {
    // ...existing TRACK mode code...
  }
  
  // ✅ 实车修复：限位时的平滑处理
  if (!pitch_des_in_limit_) pitch_vel_des = 0.;
  if (!yaw_des_in_limit_) {
    yaw_vel_des = 0.;
    base_yaw_vel_des = 0.;
  }
  
<<<<<<< HEAD
  // ✅ 检查是否接近限位（提前减速）
  bool near_yaw_limit = false;
  if (yaw_joint_urdf_->limits) {
    double yaw_margin = 0.1;  // 限位前 0.1 rad (约 5.7°) 开始减速
    double lower_limit = yaw_joint_urdf_->limits->lower + yaw_margin;
    double upper_limit = yaw_joint_urdf_->limits->upper - yaw_margin;
    near_yaw_limit = (yaw_real < lower_limit) || (yaw_real > upper_limit);
  }
  
=======
>>>>>>> c659d46648b3b440b38a2844f0b1ed0f93729fe9
  // LQR 控制 - 使用try_lock避免阻塞
  Eigen::MatrixXd K_local;
  {
    std::unique_lock<std::mutex> lk(K_mutex_, std::try_to_lock);
    if (lk.owns_lock()) {
      // 成功获取锁
      if (switching_) {
        double t = (ros::Time::now() - switch_start_time_).toSec();
        double alpha = std::min(1.0, t / switch_smooth_T_);
        K_current_ = (1.0 - alpha) * K_old_ + alpha * K_target_;
        if (alpha >= 1.0) {
          switching_ = false;
          K_old_ = K_target_;
          u_max_ = u_max_normal_;
        }
      }
      K_local = K_current_;
      K_safe_read_ = K_current_;  // 更新缓存
    } else {
      // 锁被占用(工作线程正在更新),使用缓存值
      // 注意:这里必须上锁读取缓存,避免数据竞争
      lk.lock();  // 等待锁(< 1ms)
      K_local = K_safe_read_;
      ROS_DEBUG_THROTTLE(5.0, "K_mutex busy, using cached K");
    }
  }
  
  td_yaw_->update(yaw_des, yaw_vel_des);
  td_base_yaw_->update(base_yaw_des, base_yaw_vel_des);
  double yaw_des_smooth = td_yaw_->getX1();
  double yaw_vel_des_smooth = td_yaw_->getX2();
  double base_yaw_des_smooth = td_base_yaw_->getX1();
  double base_yaw_vel_des_smooth = td_base_yaw_->getX2();
  
  // ✅ 实车关键修复：过滤速度反馈以消除编码器噪声
  static double yaw_vel_for_lqr = 0.0;
  static double base_yaw_vel_for_lqr = 0.0;
  
  // 强低通滤波（alpha接近1.0 = 极强平滑）
  constexpr double kLQRVelFilterAlpha = 0.95;  // ✅ 专门用于LQR的速度滤波
  yaw_vel_for_lqr = kLQRVelFilterAlpha * yaw_vel_for_lqr + 
                    (1.0 - kLQRVelFilterAlpha) * angular_vel_yaw.z;
  base_yaw_vel_for_lqr = kLQRVelFilterAlpha * base_yaw_vel_for_lqr + 
                         (1.0 - kLQRVelFilterAlpha) * angular_vel_base_yaw.z;
  
  // 使用过滤后的速度构建状态向量
  state_yaw_ << yaw_real, yaw_vel_for_lqr, base_yaw_real, base_yaw_vel_for_lqr;
  x_ref << yaw_des_smooth, yaw_vel_des_smooth, base_yaw_des_smooth, base_yaw_vel_des_smooth;
  Eigen::Vector4d x_err = state_yaw_ - x_ref;
  
  // ✅ 检查状态向量有效性
  if (!state_yaw_.allFinite() || !x_ref.allFinite()) {
    ROS_ERROR_THROTTLE(1.0, "State or reference contains NaN/Inf, skipping control");
    ctrl_yaw_.setCommand(0.0);
    ctrl_base_yaw_.setCommand(0.0);
    ctrl_pitch_.setCommand(0.0);
    return;
  }
  
<<<<<<< HEAD
  // LQR 控制计算（基于过滤后的速度）
  u = -K_local * x_err;
  
  // ✅ 实车修复：限位时的抗饱和处理
  if (!yaw_des_in_limit_ || near_yaw_limit) {
    // 到达限位或接近限位时，强制减小控制输出
    constexpr double kLimitDampingFactor = 0.1;  // 限位时只保留 10% 的控制量
    
    if (!yaw_des_in_limit_) {
      // 完全到达限位：几乎停止输出（只保留很小的恢复力）
      u(0) *= 0.05;
      u(1) *= 0.05;
      ROS_WARN_THROTTLE(1.0, "[LIMIT] Yaw at limit, control reduced to 5%%");
    } else if (near_yaw_limit) {
      // 接近限位：渐进减速
      u(0) *= kLimitDampingFactor;
      u(1) *= kLimitDampingFactor;
      ROS_DEBUG_THROTTLE(0.5, "[LIMIT] Yaw near limit, control reduced to 10%%");
    }
    
    // 清空滤波器状态，防止积分饱和
    u_yaw_stage1_ = 0.0;
    u_base_yaw_stage1_ = 0.0;
    u_yaw_history_.clear();
    u_base_yaw_history_.clear();
    u_yaw_final_ = 0.0;
    u_base_yaw_final_ = 0.0;
  }
  
  // ✅ 调试日志
  ROS_DEBUG_THROTTLE(1.0, "[LQR] vel_raw=[%.3f,%.3f], vel_filtered=[%.3f,%.3f], u=[%.3f,%.3f], limit=%d, near_limit=%d",
                     angular_vel_yaw.z, angular_vel_base_yaw.z,
                     yaw_vel_for_lqr, base_yaw_vel_for_lqr,
                     u(0), u(1), !yaw_des_in_limit_, near_yaw_limit);
  
=======
  // LQR 控制计算
  u = -K_local * x_err;
  
>>>>>>> c659d46648b3b440b38a2844f0b1ed0f93729fe9
  // ✅ 检查控制输出有效性
  if (!u.allFinite()) {
    ROS_ERROR_THROTTLE(1.0, "LQR output contains NaN/Inf, using zero control");
    u.setZero();
  }
  
<<<<<<< HEAD
  // ✅ 实车修复：LQR 输出后立即过滤（第一道防线）
  static double u_lqr_yaw_filtered = 0.0;
  static double u_lqr_base_yaw_filtered = 0.0;
  
  // 对 LQR 原始输出进行强滤波
  constexpr double kLQROutputFilterAlpha = 0.90;  // ✅ 强平滑 LQR 输出
  u_lqr_yaw_filtered = kLQROutputFilterAlpha * u_lqr_yaw_filtered + 
                       (1.0 - kLQROutputFilterAlpha) * u(0);
  u_lqr_base_yaw_filtered = kLQROutputFilterAlpha * u_lqr_base_yaw_filtered + 
                            (1.0 - kLQROutputFilterAlpha) * u(1);
  
  // ✅ 使用过滤后的 LQR 输出
  constexpr double kLQRtoVelocityScale = 0.06;  // ✅ 保持适中的尺度
  
  // LQR反馈项（使用过滤后的值）
  double u_yaw_feedback = u_lqr_yaw_filtered * kLQRtoVelocityScale;
  double u_base_yaw_feedback = u_lqr_base_yaw_filtered * kLQRtoVelocityScale;
=======
  // ✅ 重要修复: LQR输出u是加速度,需要积分成速度
  // u的单位是rad/s², 需要转换为速度命令rad/s
  // 方法1: 简单积分 v_cmd = v_current + u*dt (但可能累积误差)
  // 方法2: 将u理解为速度控制的补偿量,乘以一个系数
  // 这里采用方法2,将u缩放后作为速度命令
  constexpr double kLQRtoVelocityScale = 0.02 ;  // ✅ 关键参数,需要调试
  
  // LQR反馈项(已经是速度的补偿)
  double u_yaw_feedback = u(0) * kLQRtoVelocityScale;
  double u_base_yaw_feedback = u(1) * kLQRtoVelocityScale;
>>>>>>> c659d46648b3b440b38a2844f0b1ed0f93729fe9
  
  // 前馈项(期望速度)
  double u_yaw_feedforward = config_.yaw_k_v_ * yaw_vel_des_smooth;
  double u_base_yaw_feedforward = config_.yaw_k_v_ * base_yaw_vel_des_smooth;
  
  // 最终速度命令 = 前馈 + 反馈
  double u_yaw_raw = u_yaw_feedforward + u_yaw_feedback;
  double u_base_yaw_raw = u_base_yaw_feedforward + u_base_yaw_feedback;
  
<<<<<<< HEAD
  ROS_DEBUG_THROTTLE(1.0, "[CTRL] u_lqr_raw=[%.3f,%.3f], u_lqr_filt=[%.3f,%.3f], u_fb=[%.3f,%.3f], u_total=[%.3f,%.3f]", 
                     u(0), u(1),
                     u_lqr_yaw_filtered, u_lqr_base_yaw_filtered,
                     u_yaw_feedback, u_base_yaw_feedback,
                     u_yaw_raw, u_base_yaw_raw);
=======
  ROS_DEBUG_THROTTLE(1.0, "u_yaw: ff=%.3f, fb=%.3f, total=%.3f", 
                     u_yaw_feedforward, u_yaw_feedback, u_yaw_raw);
>>>>>>> c659d46648b3b440b38a2844f0b1ed0f93729fe9
  
  // ✅ 检查原始控制量
  if (!std::isfinite(u_yaw_raw)) u_yaw_raw = 0.0;
  if (!std::isfinite(u_base_yaw_raw)) u_base_yaw_raw = 0.0;
  
  // ✅ 5. 三级平滑滤波（彻底消除毛刺）
  // 第一级：状态依赖的自适应滤波
  if (error_history_.size() < 10) {
    error_history_.resize(10, 0.0);  // 首次调用时初始化
  }
  error_history_.pop_front();
  error_history_.push_back(x_err.norm());
  
  double error_var = 0;
  for(const auto& err : error_history_) {
    error_var += std::pow(err - error_history_.back(), 2);
  }
  error_var /= error_history_.size();
  
  // ✅ 实车：使用固定的强滤波（不自适应，优先稳定性）
  constexpr double alpha_adaptive = 0.92;  // ✅ 从 0.75-0.80 提高到 0.92（强平滑）
  
  if (!filter_initialized_) {
    u_yaw_stage1_ = u_yaw_raw;
    u_base_yaw_stage1_ = u_base_yaw_raw;
    filter_initialized_ = true;
  }
  
<<<<<<< HEAD
  // 第一级平滑（强化）
=======
  // 第一级平滑
>>>>>>> c659d46648b3b440b38a2844f0b1ed0f93729fe9
  u_yaw_stage1_ = alpha_adaptive * u_yaw_stage1_ + (1.0 - alpha_adaptive) * u_yaw_raw;
  u_base_yaw_stage1_ = alpha_adaptive * u_base_yaw_stage1_ + (1.0 - alpha_adaptive) * u_base_yaw_raw;
  
  // ✅ 阻断NaN传播
  if (!std::isfinite(u_yaw_stage1_)) u_yaw_stage1_ = 0.0;
  if (!std::isfinite(u_base_yaw_stage1_)) u_base_yaw_stage1_ = 0.0;
  
  // 第二级：移动平均（窗口大小3）
  constexpr size_t kSmoothWindowSize = 3;
  
  u_yaw_history_.push_back(u_yaw_stage1_);
  u_base_yaw_history_.push_back(u_base_yaw_stage1_);
  
  if (u_yaw_history_.size() > kSmoothWindowSize) u_yaw_history_.pop_front();
  if (u_base_yaw_history_.size() > kSmoothWindowSize) u_base_yaw_history_.pop_front();
  
  double u_yaw_stage2 = 0.0, u_base_yaw_stage2 = 0.0;
  // 添加除零保护
  if (!u_yaw_history_.empty()) {
    for (double u : u_yaw_history_) u_yaw_stage2 += u;
    u_yaw_stage2 /= u_yaw_history_.size();
  } else {
    u_yaw_stage2 = u_yaw_stage1_;
  }
<<<<<<< HEAD
  
  if (!u_base_yaw_history_.empty()) {
    for (double u : u_base_yaw_history_) u_base_yaw_stage2 += u;
    u_base_yaw_stage2 /= u_base_yaw_history_.size();
  } else {
    u_base_yaw_stage2 = u_base_yaw_stage1_;
  }
  
  // 第三级：最终输出平滑（实车：极强平滑）
  constexpr double kFinalFilterAlpha = 0.95;  // ✅ 实车专用：从 0.85 提高到 0.95
  
  u_yaw_final_ = kFinalFilterAlpha * u_yaw_final_ + (1.0 - kFinalFilterAlpha) * u_yaw_stage2;
  u_base_yaw_final_ = kFinalFilterAlpha * u_base_yaw_final_ + (1.0 - kFinalFilterAlpha) * u_base_yaw_stage2;
  
  // ✅ 最后一道防线：确保输出有效
  if (!std::isfinite(u_yaw_final_)) {
    ROS_ERROR_THROTTLE(1.0, "u_yaw_final is not finite, resetting filter chain");
    u_yaw_final_ = 0.0;
    u_yaw_stage1_ = 0.0;
    u_yaw_history_.clear();
  }
  if (!std::isfinite(u_base_yaw_final_)) {
    ROS_ERROR_THROTTLE(1.0, "u_base_yaw_final is not finite, resetting filter chain");
    u_base_yaw_final_ = 0.0;
    u_base_yaw_stage1_ = 0.0;
    u_base_yaw_history_.clear();
  }
  
  u_yaw_ = u_yaw_final_;
  u_base_yaw_ = u_base_yaw_final_;
  
  // ✅ 实车修复：限位时的速度方向检查（防止往限位方向继续运动）
  if (!yaw_des_in_limit_ && yaw_joint_urdf_->limits) {
    //double yaw_pos_error = x_err(0);  // yaw 位置误差
    
    // 如果在上限位且还想往正方向走，禁止
    if (yaw_real >= yaw_joint_urdf_->limits->upper - 0.01 && u_yaw_ > 0.0) {
      ROS_WARN_THROTTLE(0.5, "[LIMIT] At upper limit (%.3f >= %.3f), blocking positive command",
                       yaw_real, yaw_joint_urdf_->limits->upper);
      u_yaw_ = 0.0;
      u_base_yaw_ = 0.0;
    }
    // 如果在下限位且还想往负方向走，禁止
    else if (yaw_real <= yaw_joint_urdf_->limits->lower + 0.01 && u_yaw_ < 0.0) {
      ROS_WARN_THROTTLE(0.5, "[LIMIT] At lower limit (%.3f <= %.3f), blocking negative command",
                       yaw_real, yaw_joint_urdf_->limits->lower);
      u_yaw_ = 0.0;
      u_base_yaw_ = 0.0;
    }
  }
  
=======
  
  if (!u_base_yaw_history_.empty()) {
    for (double u : u_base_yaw_history_) u_base_yaw_stage2 += u;
    u_base_yaw_stage2 /= u_base_yaw_history_.size();
  } else {
    u_base_yaw_stage2 = u_base_yaw_stage1_;
  }
  
  // 第三级：最终输出平滑（强滤波）
  constexpr double kFinalFilterAlpha = 0.92;  // ✅ 最终级强平滑
  
  u_yaw_final_ = kFinalFilterAlpha * u_yaw_final_ + (1.0 - kFinalFilterAlpha) * u_yaw_stage2;
  u_base_yaw_final_ = kFinalFilterAlpha * u_base_yaw_final_ + (1.0 - kFinalFilterAlpha) * u_base_yaw_stage2;
  
  // ✅ 最后一道防线：确保输出有效
  if (!std::isfinite(u_yaw_final_)) {
    ROS_ERROR_THROTTLE(1.0, "u_yaw_final is not finite, resetting filter chain");
    u_yaw_final_ = 0.0;
    u_yaw_stage1_ = 0.0;
    u_yaw_history_.clear();
  }
  if (!std::isfinite(u_base_yaw_final_)) {
    ROS_ERROR_THROTTLE(1.0, "u_base_yaw_final is not finite, resetting filter chain");
    u_base_yaw_final_ = 0.0;
    u_base_yaw_stage1_ = 0.0;
    u_base_yaw_history_.clear();
  }
  
  u_yaw_ = u_yaw_final_;
  u_base_yaw_ = u_base_yaw_final_;
  
>>>>>>> c659d46648b3b440b38a2844f0b1ed0f93729fe9
  // ✅ 6. 限制控制输出幅度（速度命令,单位rad/s）
  constexpr double max_command = 10.0;  // ✅ 最大角速度10 rad/s (约573°/s)
  
  // 先处理NaN/Inf，再限幅
  if (!std::isfinite(u_yaw_)) {
    ROS_ERROR_THROTTLE(1.0, "u_yaw is not finite (NaN/Inf), resetting to 0");
    u_yaw_ = 0.0;
  } else if (std::abs(u_yaw_) > max_command) {
    ROS_WARN_THROTTLE(1.0, "u_yaw %.3f exceeds limit, clamping to ±%.1f", u_yaw_, max_command);
    u_yaw_ = std::copysign(max_command, u_yaw_);
  }
  
  if (!std::isfinite(u_base_yaw_)) {
    ROS_ERROR_THROTTLE(1.0, "u_base_yaw is not finite (NaN/Inf), resetting to 0");
    u_base_yaw_ = 0.0;
  } else if (std::abs(u_base_yaw_) > max_command) {
    ROS_WARN_THROTTLE(1.0, "u_base_yaw %.3f exceeds limit, clamping to ±%.1f", u_base_yaw_, max_command);
    u_base_yaw_ = std::copysign(max_command, u_base_yaw_);
  }
  
  // ✅ 7. 小控制量归零（减少抖动）
  // ⚠️ 死区太大会导致停顿明显,降低到0.005 rad/s (约0.3°/s)
  constexpr double kControlDeadzone = 0.001;
  if (std::abs(u_yaw_) < kControlDeadzone) u_yaw_ = 0.0;
  if (std::abs(u_base_yaw_) < kControlDeadzone) u_base_yaw_ = 0.0;
  
  // 记录控制输入
  u_yaw_cmd_ = u_yaw_;
  u_base_yaw_cmd_ = u_base_yaw_;
  
  // 更新 RLS
  if (enable_online_lqr_) {
    updateRLS();
  }
  
  // 发布状态
  if (loop_count_ % 10 == 0) {
    if (yaw_pos_state_pub_ && yaw_pos_state_pub_->trylock()) {
      yaw_pos_state_pub_->msg_.header.stamp = time;
      yaw_pos_state_pub_->msg_.set_point = yaw_des;
      yaw_pos_state_pub_->msg_.set_point_dot = yaw_vel_des;
      yaw_pos_state_pub_->msg_.process_value = yaw_real;
      yaw_pos_state_pub_->msg_.error = x_err(0);
      yaw_pos_state_pub_->msg_.command = u_yaw_;
      yaw_pos_state_pub_->unlockAndPublish();
    }
    if (pitch_pos_state_pub_ && pitch_pos_state_pub_->trylock()) {
      pitch_pos_state_pub_->msg_.header.stamp = time;
      pitch_pos_state_pub_->msg_.set_point = pitch_des;
      pitch_pos_state_pub_->msg_.set_point_dot = pitch_vel_des;
      pitch_pos_state_pub_->msg_.process_value = pitch_real;
      pitch_pos_state_pub_->msg_.error = pitch_angle_error;
      pitch_pos_state_pub_->msg_.command = pid_pitch_pos_.getCurrentCmd();
      pitch_pos_state_pub_->unlockAndPublish();
    }
  }
  loop_count_++;
  
  // ✅ 核心修复：使用滤波后的速度进行编码器噪声补偿
  // 
  // 补偿原理：
  //   内环PID看到的是编码器速度（有噪声），会产生错误的控制输出
  //   通过 IMU/滤波速度（低噪声）估计编码器噪声，提前补偿
  //
  // 补偿公式：
  //   最终命令 = LQR期望速度 + (编码器速度 - 真实速度估计)
  //            = 期望速度 + 编码器噪声估计
  
  // ✅ 对编码器速度也进行滤波，确保补偿项不引入新噪声
  static double encoder_yaw_vel_filtered = 0.0;
  static double encoder_base_yaw_vel_filtered = 0.0;
  
  constexpr double kEncoderFilterAlpha = 0.85;  // 中等强度滤波（保留部分高频响应）
  encoder_yaw_vel_filtered = kEncoderFilterAlpha * encoder_yaw_vel_filtered + 
                             (1.0 - kEncoderFilterAlpha) * ctrl_yaw_.joint_.getVelocity();
  encoder_base_yaw_vel_filtered = kEncoderFilterAlpha * encoder_base_yaw_vel_filtered + 
                                  (1.0 - kEncoderFilterAlpha) * ctrl_base_yaw_.joint_.getVelocity();
  
  // 计算编码器噪声估计（使用滤波后的值）
  double yaw_noise_compensation = encoder_yaw_vel_filtered - imu_yaw_vel_;
  double base_yaw_noise_compensation = encoder_base_yaw_vel_filtered - imu_base_yaw_vel_;
  
  // ✅ 限制补偿量，防止过度补偿
  constexpr double kMaxCompensation = 2.0;  // rad/s
  yaw_noise_compensation = std::max(-kMaxCompensation, 
                                     std::min(kMaxCompensation, yaw_noise_compensation));
  base_yaw_noise_compensation = std::max(-kMaxCompensation, 
                                          std::min(kMaxCompensation, base_yaw_noise_compensation));
  
  // 最终控制命令 = LQR输出 + 噪声补偿
  double yaw_cmd_final = u_yaw_ + yaw_noise_compensation;
  double base_yaw_cmd_final = u_base_yaw_ + base_yaw_noise_compensation;
  
  // ✅ 调试日志
  ROS_DEBUG_THROTTLE(1.0, "[COMPENSATION] yaw: u=%.3f, enc_filt=%.3f, imu=%.3f, comp=%.3f, final=%.3f",
                     u_yaw_, encoder_yaw_vel_filtered, imu_yaw_vel_, 
                     yaw_noise_compensation, yaw_cmd_final);
  
  ctrl_yaw_.setCommand(yaw_cmd_final);
  ctrl_base_yaw_.setCommand(base_yaw_cmd_final);
  
  ctrl_pitch_.setCommand(-(pid_pitch_pos_.getCurrentCmd() + config_.pitch_k_v_ * pitch_vel_des +
                         ctrl_pitch_.joint_.getVelocity() - angular_vel_pitch.y));
  
  // 更新关节状态
  ctrl_yaw_.update(time, period);
  ctrl_base_yaw_.update(time, period);
  ctrl_pitch_.update(time, period);
  ctrl_pitch_.joint_.setCommand(-(ctrl_pitch_.joint_.getCommand() + feedForward(time)));
}

double Controller::feedForward(const ros::Time& time){
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

void Controller::updateChassisVel(){
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

void Controller::updateRLS(){
  if (!enable_online_lqr_ || !running_) {
    ROS_DEBUG_THROTTLE(10.0, "[RLS] Disabled: online_lqr=%d, running=%d", 
                      enable_online_lqr_, running_.load());
    return;
  }
  
  // 构建当前状态向量
  Eigen::VectorXd x(n_);
  try {
    x(0) = ctrl_yaw_.joint_.getPosition();
    x(1) = ctrl_yaw_.joint_.getVelocity();
    x(2) = ctrl_base_yaw_.joint_.getPosition();
    x(3) = ctrl_base_yaw_.joint_.getVelocity();
  } catch (const std::exception& e) {
    ROS_ERROR_THROTTLE(1.0, "[RLS] Error reading joint states: %s", e.what());
    return;
  }
  
  if (!x.allFinite()) {
    ROS_ERROR_THROTTLE(1.0, "[RLS] Current state contains NaN/Inf: [%.2f, %.2f, %.2f, %.2f]",
                      x(0), x(1), x(2), x(3));
    return;
  }
  
  // ✅ 使用try_lock避免阻塞主控制循环
  std::unique_lock<std::mutex> lk_rls(rls_mutex_, std::try_to_lock);
  if (!lk_rls.owns_lock()) {
    // 工作线程正在更新,跳过本次RLS更新(不影响控制稳定性)
    ROS_DEBUG_THROTTLE(5.0, "[RLS] Mutex busy, skipping update");
    return;
  }
  
<<<<<<< HEAD
  // ✅ 详细日志：为什么不进入 RLS 更新
  if (!have_prev_sample_) {
    ROS_WARN_THROTTLE(2.0, "[RLS] No previous sample, initializing (count=%zu)", sample_count_);
    x_prev_ = x;
    u_prev_sample_.resize(m_);
    u_prev_sample_(0) = u_yaw_cmd_;
    u_prev_sample_(1) = u_base_yaw_cmd_;
    have_prev_sample_ = true;
    return;
  }
  
  if (driver_saturated_) {
    ROS_WARN_THROTTLE(2.0, "[RLS] Driver saturated flag set, waiting for normal operation (count=%zu)", sample_count_);
    // ✅ 检查当前输入是否仍然饱和
    constexpr double kSaturationThreshold = 4.8;
    if (std::abs(u_yaw_cmd_) < kSaturationThreshold && std::abs(u_base_yaw_cmd_) < kSaturationThreshold) {
      driver_saturated_ = false;
      ROS_INFO("[RLS] Saturation cleared, resuming updates");
    }
    x_prev_ = x;
    u_prev_sample_(0) = u_yaw_cmd_;
    u_prev_sample_(1) = u_base_yaw_cmd_;
=======
  // ✅ 使用try_lock避免阻塞主控制循环
  std::unique_lock<std::mutex> lk_rls(rls_mutex_, std::try_to_lock);
  if (!lk_rls.owns_lock()) {
    // 工作线程正在更新,跳过本次RLS更新(不影响控制稳定性)
    ROS_DEBUG_THROTTLE(5.0, "RLS mutex busy, skipping update");
>>>>>>> c659d46648b3b440b38a2844f0b1ed0f93729fe9
    return;
  }
  
  if (have_prev_sample_ && !driver_saturated_) {
    // ✅ 实车：放宽速度限制（考虑高速运动场景）
    constexpr double kMaxReasonableVel = 50.0;  // ✅ 提高阈值，允许更高速度
    if (std::abs(x(1)) > kMaxReasonableVel || std::abs(x(3)) > kMaxReasonableVel) {
      ROS_WARN_THROTTLE(1.0, "[RLS] ❌ Skip reason: Abnormal velocity: yaw_vel=%.2f, base_yaw_vel=%.2f (threshold=%.1f)",
                       x(1), x(3), kMaxReasonableVel);
      x_prev_ = x;
      u_prev_sample_(0) = u_yaw_cmd_;
      u_prev_sample_(1) = u_base_yaw_cmd_;
      return;
    }
    
    if (x_prev_.size() != n_ || u_prev_sample_.size() != m_) {
      ROS_ERROR_THROTTLE(1.0, "[RLS] ❌ Skip reason: Dimension error: x_prev=%ld (expect %d), u_prev=%ld (expect %d)",
                        x_prev_.size(), n_, u_prev_sample_.size(), m_);
      x_prev_.resize(n_);
      u_prev_sample_.resize(m_);
      x_prev_.setZero();
      u_prev_sample_.setZero();
      return;
    }
    
    if (!x_prev_.allFinite() || !u_prev_sample_.allFinite()) {
      ROS_ERROR_THROTTLE(1.0, "[RLS] ❌ Skip reason: Previous data invalid: x_prev finite=%d, u_prev finite=%d",
                        x_prev_.allFinite(), u_prev_sample_.allFinite());
      x_prev_ = x;
      u_prev_sample_(0) = u_yaw_cmd_;
      u_prev_sample_(1) = u_base_yaw_cmd_;
      return;
    }
    
    // ✅ 实车：放宽饱和检测（允许更大输入用于辨识）
    constexpr double kSaturationThreshold = 4.8;  // ✅ 提高阈值，接近 max_command=5
    if (std::abs(u_prev_sample_(0)) > kSaturationThreshold || 
        std::abs(u_prev_sample_(1)) > kSaturationThreshold) {
      ROS_WARN_THROTTLE(1.0, "[RLS] ❌ Skip reason: Input saturated u_prev=(%.2f, %.2f), threshold=%.1f, setting flag",
                        u_prev_sample_(0), u_prev_sample_(1), kSaturationThreshold);
      driver_saturated_ = true;
      x_prev_ = x;
      u_prev_sample_(0) = u_yaw_cmd_;
      u_prev_sample_(1) = u_base_yaw_cmd_;
      return;
    } else {
      driver_saturated_ = false;
    }
    
    // ✅ 成功通过所有检查，开始 RLS 更新
    ROS_INFO_THROTTLE(5.0, "[RLS] ✅ Updating: sample=%zu, x=[%.3f,%.2f,%.3f,%.2f], u_prev=[%.2f,%.2f]",
                     sample_count_, x(0), x(1), x(2), x(3), u_prev_sample_(0), u_prev_sample_(1));
    
    // 构建回归向量
    Eigen::VectorXd phi(n_ + m_);
    phi << x_prev_, u_prev_sample_;
    
    // RLS 核心算法
    Eigen::VectorXd Pphi = Pcov_ * phi;
    if (!Pphi.allFinite()) {
      ROS_ERROR_THROTTLE(1.0, "[RLS] Pphi invalid, resetting Pcov");
      Pcov_ = Eigen::MatrixXd::Identity(n_ + m_, n_ + m_) * 0.1;
      Pphi = Pcov_ * phi;
    }
    
    double denom = lambda_rls_ + phi.dot(Pphi);
    if (denom < 1e-8) {
      ROS_WARN_THROTTLE(1.0, "RLS denom too small: %e", denom);
      denom = 1e-8;
    }
    
    Eigen::VectorXd Kk = Pphi / denom;
    if (!Kk.allFinite()) {
      ROS_ERROR_THROTTLE(1.0, "[RLS] Kk invalid, skipping update");
      x_prev_ = x;
      u_prev_sample_(0) = u_yaw_cmd_;
      u_prev_sample_(1) = u_base_yaw_cmd_;
      return;
    }
    
    Eigen::VectorXd pred = Theta_ * phi;
    if (!pred.allFinite()) {
      ROS_ERROR_THROTTLE(1.0, "[RLS] pred invalid, resetting Theta");
      resetThetaToDefault();
      x_prev_ = x;
      u_prev_sample_(0) = u_yaw_cmd_;
      u_prev_sample_(1) = u_base_yaw_cmd_;
      return;
    }
    
    Eigen::VectorXd err = x - pred;
    
    // ✅ 实车：放宽误差限制（允许更大的模型误差）
    constexpr double kMaxErrorNorm = 0.25;  // ✅ 提高阈值，适应实车噪声
    if (err.norm() > kMaxErrorNorm) {
      ROS_WARN_THROTTLE(2.0, "[RLS] Error norm %.4f > %.2f, clamping",
                       err.norm(), kMaxErrorNorm);
      err = err * (kMaxErrorNorm / err.norm());
    }
    
    Eigen::MatrixXd delta = err * Kk.transpose();
    
    // ✅ 实车：适中的更新步长（平衡收敛速度和稳定性）
    constexpr double kMaxDeltaNorm = 0.008;  // ✅ 提高步长，加快收敛
    double delta_norm = delta.norm();
    if (delta_norm > kMaxDeltaNorm) {
      ROS_DEBUG_THROTTLE(2.0, "[RLS] Delta norm %.4f > %.4f, scaling",
                        delta_norm, kMaxDeltaNorm);
      delta = delta * (kMaxDeltaNorm / delta_norm);
    }
    
    Eigen::MatrixXd Theta_new = Theta_ + delta;
    if (!Theta_new.allFinite()) {
      ROS_ERROR_THROTTLE(1.0, "[RLS] Theta_new invalid, skipping update");
      x_prev_ = x;
      u_prev_sample_(0) = u_yaw_cmd_;
      u_prev_sample_(1) = u_base_yaw_cmd_;
      return;
    }
    
    Theta_ = Theta_new;
    
    // 更新协方差矩阵
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_ + m_, n_ + m_);
    Eigen::MatrixXd temp = I - Kk * phi.transpose();
    Pcov_ = temp * Pcov_ * temp.transpose() + Kk * Kk.transpose() / lambda_rls_;
    Pcov_ = 0.5 * (Pcov_ + Pcov_.transpose());
    
    if (!Pcov_.allFinite()) {
      ROS_ERROR_THROTTLE(1.0, "Pcov invalid, resetting");
      Pcov_ = Eigen::MatrixXd::Identity(n_ + m_, n_ + m_) * 0.1;
    }
    
    // ✅ 实车：放宽协方差矩阵限制
    double pcov_trace = Pcov_.trace();
    constexpr double kMaxPcovTrace = 8.0;  // ✅ 提高上限，允许更大的不确定性
    if (pcov_trace > kMaxPcovTrace) {
      ROS_WARN_THROTTLE(5.0, "[RLS] Pcov trace %.2f > %.1f, scaling",
                       pcov_trace, kMaxPcovTrace);
      Pcov_ *= (kMaxPcovTrace / pcov_trace);
    }
    
    // 计算平均误差
    constexpr size_t kErrorSampleSize = 50;
    
    double tracking_error = err.norm();
    error_samples_.push_back(tracking_error);
    if (error_samples_.size() > kErrorSampleSize) {
      error_samples_.pop_front();
    }
    
    double sum_error = 0.0;
    for (double e : error_samples_) {
      sum_error += e;
    }
    recent_avg_error_ = error_samples_.empty() ? 0.0 : sum_error / error_samples_.size();
    
    // 应用物理约束
    applyPhysicalConstraints();
    
    sample_count_++;
    
    // 定期打印 RLS 更新状态
    if (sample_count_ % 50 == 0) {
      ROS_INFO("[RLS] Sample %zu: err=%.4f, sat=%d, vel=[%.2f,%.2f], u=[%.2f,%.2f]",
               sample_count_, recent_avg_error_, driver_saturated_,
               x(1), x(3), u_prev_sample_(0), u_prev_sample_(1));
    }
    if (sample_count_ % 100 == 0) {
      ROS_INFO("[RLS] Theta [%zu]: decay=[%.3f,%.3f], gain=[%.2f,%.2f]",
               sample_count_,
               Theta_(1,1), Theta_(3,3),
               Theta_(1,n_)/0.001, Theta_(3,n_+1)/0.001);
    }
  }
  
  // 更新前一个状态和输入
  x_prev_ = x;
  u_prev_sample_.resize(m_);
  u_prev_sample_(0) = u_yaw_cmd_;
  u_prev_sample_(1) = u_base_yaw_cmd_;
  have_prev_sample_ = true;
}

void Controller::onlineLQRUpdate(){
  ROS_INFO("Adaptive LQR worker thread started");
  
  int current_worker_hz = constants::kWorkerHzInitial;
  ros::Rate rate(current_worker_hz);
  
  ros::Duration(0.5).sleep();
  
  int consecutive_failures = 0;
  constexpr int kMaxConsecutiveFailures = 10;
  
  while (ros::ok() && running_) {
    if (!running_ || !enable_online_lqr_) {
      rate.sleep();
      continue;
    }
    
    try {
      // 1. 自适应确定所需样本数
      size_t required_samples;
      switch (update_strategy_) {
        case STRATEGY_INITIAL:
          required_samples = constants::kRlsMinSamplesInitial;  // 100
          current_worker_hz = constants::kWorkerHzInitial;      // 2Hz
          break;
        case STRATEGY_QUICK:
          required_samples = constants::kRlsMinSamplesQuick;    // 20
          current_worker_hz = constants::kWorkerHzQuick;        // 10Hz
          break;
        case STRATEGY_NORMAL:
          required_samples = constants::kRlsMinSamplesNormal;   // 50
          current_worker_hz = constants::kWorkerHzNormal;       // 5Hz
          break;
        case STRATEGY_STABLE:
          required_samples = constants::kRlsMinSamplesNormal;   // 50
          current_worker_hz = constants::kWorkerHzNormal;       // 5Hz
          break;
        default:
          required_samples = constants::kRlsMinSamplesInitial;
          current_worker_hz = constants::kWorkerHzInitial;
      }
      
      rate = ros::Rate(current_worker_hz);
      
      // 2. 获取Theta和误差信息
      Eigen::MatrixXd Theta_copy;
      size_t current_samples;
      double current_error;
      {
        std::unique_lock<std::mutex> lk(rls_mutex_, std::try_to_lock);
        
        if (!lk.owns_lock()) {
          // 主线程正在updateRLS,跳过本次更新
          ROS_DEBUG_THROTTLE(5.0, "rls_mutex busy, skipping LQR update");
          rate.sleep();
          continue;
        }
        
        if (!running_) break;
        
        current_samples = sample_count_;
        if (current_samples < required_samples) {
          if (current_samples % 10 == 0) {
            ROS_INFO_THROTTLE(2.0, "Strategy: %d, Samples: %zu/%zu (%.1f%%)", 
                             update_strategy_, current_samples, required_samples,
                             100.0 * current_samples / required_samples);
          }
          rate.sleep();
          continue;
        }
        
        Theta_copy = Theta_;
        current_error = recent_avg_error_;
      }
      
      // ✅ 退出前再次检查
      if (!running_) break;
      
      // 3. 自适应更新策略
      updateStrategy(current_error);
      
      // 安全检查
      if (!Theta_copy.allFinite() || Theta_copy.rows() != n_ || Theta_copy.cols() != n_ + m_) {
        ROS_WARN_THROTTLE(5.0, "Theta error: %ldx%ld", Theta_copy.rows(), Theta_copy.cols());
        consecutive_failures++;
        if (consecutive_failures >= kMaxConsecutiveFailures) {
          std::lock_guard<std::mutex> lk(rls_mutex_);
          resetThetaToDefault();
          consecutive_failures = 0;
          update_strategy_ = STRATEGY_INITIAL;
        }
        rate.sleep();
        continue;
      }
      
      // 分解Theta矩阵
      Eigen::MatrixXd A_est = Theta_copy.block(0, 0, n_, n_);
      Eigen::MatrixXd B_est = Theta_copy.block(0, n_, n_, m_);
      
      // ✅ 4. 根据策略选择求解器
      Eigen::MatrixXd K_new(m_, n_);
      bool ok = false;
      
      if (update_strategy_ == STRATEGY_INITIAL) {
        // ✅ 初始阶段：使用精确求解器（严格收敛）
        ok = computeDLQRdiscretePrecise(A_est, B_est, Qd_, Rd_, K_new);
        ROS_DEBUG("Using PRECISE solver for INITIAL strategy");
      } 
      else if (update_strategy_ == STRATEGY_STABLE && successful_updates_ >= 20) {
        // ✅ 稳定阶段（已有足够更新）：可以使用快速求解器
        ok = computeDLQRdiscreteFast(A_est, B_est, Qd_, Rd_, K_new);
        ROS_DEBUG("Using FAST solver for STABLE strategy");
      }
      else {
        // ✅ 正常/快速适应阶段：使用标准求解器（平衡速度和精度）
        ok = computeDLQRdiscrete(A_est, B_est, Qd_, Rd_, K_new);
        ROS_DEBUG("Using STANDARD solver for strategy %d", update_strategy_);
      }
      
      if (!ok) {
        ROS_WARN_THROTTLE(5.0, "LQR computation failed");
        consecutive_failures++;
        rate.sleep();
        continue;
      }
      
      // ✅ 5. 根据策略选择验证严格程度
      bool validation_ok = false;
      if (update_strategy_ == STRATEGY_INITIAL) {
        // 初始阶段：严格验证
        validation_ok = validateK(A_est, B_est, K_new);
      } else {
        // 其他阶段：快速验证
        validation_ok = validateKFast(A_est, B_est, K_new);
      }
      
      if (!K_new.allFinite() || !validation_ok) {
        ROS_WARN_THROTTLE(5.0, "K validation failed");
        consecutive_failures++;
        rate.sleep();
        continue;
      }
      
      // 成功，重置失败计数
      consecutive_failures = 0;
      
      // 6. 自适应稳定性判断
      int required_stable_count = (update_strategy_ == STRATEGY_QUICK) ? 2 : 
                                  (update_strategy_ == STRATEGY_STABLE) ? 3 : 5;
      double stable_tolerance = (update_strategy_ == STRATEGY_QUICK) ? 0.1 : 
                               (update_strategy_ == STRATEGY_STABLE) ? 0.02 : 0.05;
      
      double diff = (K_new - last_successful_K_).norm();
      if (last_successful_K_.rows() == 0) {
        last_successful_K_ = K_new;
        stable_count_ = 1;
      } else if (diff < stable_tolerance) {
        stable_count_++;
      } else {
        stable_count_ = 1;
        last_successful_K_ = K_new;
      }
      
      if (stable_count_ >= required_stable_count) {
        // ✅ 更新前检查是否应该退出
        if (!running_) break;
        
        // 7. 自适应切换时间
        double switch_time = (update_strategy_ == STRATEGY_QUICK) ? 0.3 : 
                            (update_strategy_ == STRATEGY_STABLE) ? 0.5 : 1.0;
        
        // ✅ 使用try_lock避免阻塞在这里
        {
          std::unique_lock<std::mutex> lk(K_mutex_, std::try_to_lock);
          if (!lk.owns_lock()) {
            ROS_DEBUG("K_mutex busy during LQR update, will retry next cycle");
            rate.sleep();
            continue;  // 下次再试
          }
          if (!running_) break;  // 再次检查
          K_target_ = K_new;
          K_safe_read_ = K_current_;  // ✅ 在切换前先缓存当前值
          switching_ = true;
          switch_start_time_ = ros::Time::now();
          switch_smooth_T_ = switch_time;
          u_max_ = 0.5 * u_max_normal_;
        }
        
        ROS_INFO("LQR updated [Strategy: %d, Hz: %d, Samples: %zu, Switch: %.2fs, Solver: %s]", 
                 update_strategy_, current_worker_hz, current_samples, switch_time,
                 (update_strategy_ == STRATEGY_INITIAL) ? "PRECISE" : 
                 (update_strategy_ == STRATEGY_STABLE && successful_updates_ >= 20) ? "FAST" : "STANDARD");
        
        stable_count_ = 0;
        last_successful_K_ = K_new;
        successful_updates_++;
        
        // 初始阶段完成后切换策略
        if (update_strategy_ == STRATEGY_INITIAL && successful_updates_ >= 3) {
          update_strategy_ = STRATEGY_NORMAL;
          ROS_INFO("Switched to NORMAL strategy");
        }
        
        saveKToYaml(K_new, "/home/what/ros_ws/rm_ws/src/rm_controllers/rm_gimbal_controllers/config/k_matrix.yaml");
      }
      
    } catch (const std::exception& e) {
      ROS_ERROR("Adaptive LQR exception: %s", e.what());
      consecutive_failures++;
    }
    
    if (!running_) break;
    
    rate.sleep();
  }
  
  ROS_INFO("Adaptive LQR thread exiting gracefully");
}

bool Controller::computeDLQRdiscrete(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                                     const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
                                     Eigen::MatrixXd& K_out){
  // 基本检查
  if (A.rows() != n_ || A.cols() != n_ || B.rows() != n_ || B.cols() != m_) {
    return false;
  }
  if (!A.allFinite() || !B.allFinite() || !Q.allFinite() || !R.allFinite()) {
    return false;
  }
  
  Eigen::MatrixXd P = Q;
  Eigen::MatrixXd At = A.transpose();
  Eigen::MatrixXd Bt = B.transpose();
  
  constexpr double kStandardConvergenceTol = 1e-7;  // 标准收敛条件
  constexpr int kStandardMaxIterations = 500;
  constexpr double kMinDeterminant = 1e-10;
  constexpr double kMaxPNorm = 1e10;
  bool converged = false;

  for (int i = 0; i < kStandardMaxIterations; i++) {
    Eigen::MatrixXd S = R + Bt * P * B;
    
    // ✅ 修复：正确的行列式检查
    double det_S = S.determinant();
    if (std::abs(det_S) < kMinDeterminant) {
      ROS_ERROR("S matrix near singular at iteration %d, det = %e", i, det_S);
      return false;
    }
    
    Eigen::MatrixXd S_inv = S.inverse();
    
    if (!S_inv.allFinite()) {
      return false;
    }
    
    Eigen::MatrixXd Pnext = At * P * A - At * P * B * S_inv * Bt * P * A + Q;
    Pnext = 0.5 * (Pnext + Pnext.transpose());
    
    double diff = (Pnext - P).norm();
    if (diff < kStandardConvergenceTol) {
      P = Pnext;
      converged = true;
      ROS_DEBUG("Standard DLQR converged at iteration %d", i);
      break;
    }
    
    if (!Pnext.allFinite() || Pnext.norm() > kMaxPNorm) {
      return false;
    }
    
    P = Pnext;
  }
  
  // ✅ 标准模式：不收敛也警告但继续
  if (!converged) {
    ROS_WARN_THROTTLE(10.0, "Standard DLQR did not fully converge");
  }

  Eigen::MatrixXd denom = R + Bt * P * B;
  double det_denom = denom.determinant();
  if (std::abs(det_denom) < kMinDeterminant) {
    ROS_ERROR("Final denom matrix near singular, det = %e", det_denom);
    return false;
  }
  
  K_out = denom.inverse() * Bt * P * A;
  
  if (!K_out.allFinite() || K_out.rows() != m_ || K_out.cols() != n_) {
    return false;
  }
  
  return true;
}

bool Controller::computeDLQRdiscreteFast(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                                         const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
                                         Eigen::MatrixXd& K_out) {
  // ✅ 1. 快速维度检查
  if (A.rows() != n_ || A.cols() != n_ || B.rows() != n_ || B.cols() != m_) {
    return false;
  }
  
  // ✅ 2. 使用更宽松的收敛判据（提速）
  constexpr double kFastConvergenceTol = 1e-6;  // 原来是1e-9
  constexpr int kFastMaxIterations = 100;        // 原来是500
  
  Eigen::MatrixXd P = Q;
  Eigen::MatrixXd At = A.transpose();
  Eigen::MatrixXd Bt = B.transpose();
  
  constexpr double kMinDeterminant = 1e-10;
  bool converged = false;

  for (int i = 0; i < kFastMaxIterations; i++) {
    Eigen::MatrixXd S = R + Bt * P * B;
    
    double det_S = S.determinant();
    if (std::abs(det_S) < kMinDeterminant) {
      return false;
    }
    
    // ✅ 3. 对于小矩阵，直接求逆更快
    Eigen::MatrixXd S_inv = S.inverse();
    
    if (!S_inv.allFinite()) {
      return false;
    }
    
    Eigen::MatrixXd Pnext = At * P * A - At * P * B * S_inv * Bt * P * A + Q;
    Pnext = 0.5 * (Pnext + Pnext.transpose());
    
    // ✅ 4. 快速收敛检查
    if ((Pnext - P).norm() < kFastConvergenceTol) {
      P = Pnext;
      converged = true;
      break;
    }
    
    if (!Pnext.allFinite() || Pnext.norm() > 1e10) {
      return false;
    }
    
    P = Pnext;
  }
  
  if (!converged) {
    // ✅ 5. 未收敛但接近时也接受（加速）
    ROS_WARN_THROTTLE(10.0, "Fast DLQR did not fully converge, using current P");
  }

  Eigen::MatrixXd denom = R + Bt * P * B;
  double det_denom = denom.determinant();
  if (std::abs(det_denom) < kMinDeterminant) {
    return false;
  }
  
  K_out = denom.inverse() * Bt * P * A;
  
  if (!K_out.allFinite() || K_out.rows() != m_ || K_out.cols() != n_) {
    return false;
  }
  
  return true;
}

bool Controller::computeDLQRdiscretePrecise(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                                            const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
                                            Eigen::MatrixXd& K_out) {
  // ✅ 1. 严格的维度和有效性检查
  if (A.rows() != n_ || A.cols() != n_) {
    ROS_ERROR("A matrix size error: %ldx%ld, expected %dx%d", A.rows(), A.cols(), n_, n_);
    return false;
  }
  if (B.rows() != n_ || B.cols() != m_) {
    ROS_ERROR("B matrix size error: %ldx%ld, expected %dx%d", B.rows(), B.cols(), n_, m_);
    return false;
  }
  if (!A.allFinite() || !B.allFinite() || !Q.allFinite() || !R.allFinite()) {
    ROS_ERROR("Input matrices contain NaN/Inf");
    return false;
  }
  
  // ✅ 2. 严格检查可控性
  Eigen::MatrixXd controllability(n_, n_ * m_);
  Eigen::MatrixXd A_power = Eigen::MatrixXd::Identity(n_, n_);
  for (int i = 0; i < n_; ++i) {
    controllability.block(0, i * m_, n_, m_) = A_power * B;
    A_power = A_power * A;
  }
  
  double controllability_rank = controllability.fullPivLu().rank();
  if (controllability_rank < n_) {
    ROS_ERROR("System not fully controllable (rank=%f/%d)", controllability_rank, n_);
    return false;  // ✅ 初期严格：不可控直接拒绝
  }
  
  // ✅ 3. 使用严格的收敛条件
  Eigen::MatrixXd P = Q;
  Eigen::MatrixXd At = A.transpose();
  Eigen::MatrixXd Bt = B.transpose();
  
  constexpr double kPreciseConvergenceTol = 1e-9;  // 严格收敛
  constexpr int kPreciseMaxIterations = 1000;      // 更多迭代次数
  constexpr double kMinDeterminant = 1e-10;
  constexpr double kMaxPNorm = 1e10;
  bool converged = false;

  for (int i = 0; i < kPreciseMaxIterations; i++) {
    Eigen::MatrixXd S = R + Bt * P * B;
    
    double det_S = S.determinant();
    if (std::abs(det_S) < kMinDeterminant) {
      ROS_ERROR("S matrix near singular at iteration %d, det = %e", i, det_S);
      return false;
    }
    
    // ✅ 使用SVD求逆（更稳定）
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(S, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double cond_S = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
    if (cond_S > 1e8) {
      ROS_WARN("S matrix badly conditioned (cond = %e) at iteration %d", cond_S, i);
    }
    
    Eigen::MatrixXd S_inv = svd.solve(Eigen::MatrixXd::Identity(m_, m_));
    
    if (!S_inv.allFinite()) {
      ROS_ERROR("S_inv contains NaN/Inf at iteration %d", i);
      return false;
    }
    
    Eigen::MatrixXd Pnext = At * P * A - At * P * B * S_inv * Bt * P * A + Q;
    Pnext = 0.5 * (Pnext + Pnext.transpose());
    
    // ✅ 严格的收敛检查
    double diff = (Pnext - P).norm();
    if (diff < kPreciseConvergenceTol) {
      P = Pnext;
      converged = true;
      ROS_DEBUG("Precise DLQR converged at iteration %d, diff = %e", i, diff);
      break;
    }
    
    if (!Pnext.allFinite() || Pnext.norm() > kMaxPNorm) {
      ROS_ERROR("P matrix unstable at iteration %d (norm=%f)", i, Pnext.norm());
      return false;
    }
    
    P = Pnext;
  }
  
  // ✅ 4. 必须收敛才接受
  if (!converged) {
    ROS_ERROR("Precise DLQR did not converge within %d iterations", kPreciseMaxIterations);
    return false;  // 初期严格：不收敛直接拒绝
  }

  // 5. 计算最终增益
  Eigen::MatrixXd denom = R + Bt * P * B;
  double det_denom = denom.determinant();
  if (std::abs(det_denom) < kMinDeterminant) {
    ROS_ERROR("Final denom matrix near singular, det = %e", det_denom);
    return false;
  }
  
  K_out = denom.inverse() * Bt * P * A;
  
  // 6. 验证输出
  if (!K_out.allFinite()) {
    ROS_ERROR("K_out contains NaN/Inf");
    return false;
  }
  
  if (K_out.rows() != m_ || K_out.cols() != n_) {
    ROS_ERROR("K_out size error: %ldx%ld, expected %dx%d", 
              K_out.rows(), K_out.cols(), m_, n_);
    return false;
  }
  
  double k_norm = K_out.norm();
  if (k_norm < constants::kMinKNorm || k_norm > 1000.0) {
    ROS_WARN("K norm unusual: %f (expected 0.1~1000)", k_norm);
  }
  
  ROS_DEBUG("Precise DLQR computation successful, K norm = %f", k_norm);
  return true;
}

void Controller::updateStrategy(double current_error) {
  // ✅ 维护误差窗口
  constexpr size_t kErrorWindowSize = 20;
  error_window_.push_back(current_error);
  if (error_window_.size() > kErrorWindowSize) {
    error_window_.pop_front();
  }
  
  // 计算平均误差
  double avg_error = 0.0;
  for (double err : error_window_) {
    avg_error += err;
  }
  avg_error /= error_window_.size();
  recent_avg_error_ = avg_error;
  
  // ✅ 自适应策略切换
  if (update_strategy_ == STRATEGY_INITIAL) {
    // 初始阶段：等待足够的成功更新
    if (successful_updates_ >= 3) {
      update_strategy_ = STRATEGY_NORMAL;
      ROS_INFO("Strategy: INITIAL → NORMAL");
    }
  } 
  else if (update_strategy_ == STRATEGY_NORMAL) {
    // 正常阶段：根据误差切换
    if (avg_error > constants::kLargeErrorThreshold) {
      update_strategy_ = STRATEGY_QUICK;
      ROS_INFO("Strategy: NORMAL → QUICK (large error: %.4f)", avg_error);
    } else if (avg_error < constants::kSmallErrorThreshold && successful_updates_ >= 10) {
      update_strategy_ = STRATEGY_STABLE;
      ROS_INFO("Strategy: NORMAL → STABLE (small error: %.4f)", avg_error);
    }
  }
  else if (update_strategy_ == STRATEGY_QUICK) {
    // 快速适应：误差降低后返回正常
    if (avg_error < constants::kLargeErrorThreshold * 0.5) {
      update_strategy_ = STRATEGY_NORMAL;
      ROS_INFO("Strategy: QUICK → NORMAL (error reduced: %.4f)", avg_error);
    }
  }
  else if (update_strategy_ == STRATEGY_STABLE) {
    // 稳定阶段：误差增大时返回正常
    if (avg_error > constants::kSmallErrorThreshold * 2.0) {
      update_strategy_ = STRATEGY_NORMAL;
      ROS_INFO("Strategy: STABLE → NORMAL (error increased: %.4f)", avg_error);
    }
  }
}

void Controller::saveKToYaml(const Eigen::MatrixXd& K, const std::string& path) {
  std::ofstream ofs(path, std::ios::out);  // 用覆盖模式打开文件
  if (!ofs.is_open()) return;
  ofs << "K: [";
  for (int i = 0; i < K.rows(); ++i) {
    for (int j = 0; j < K.cols(); ++j) {
      ofs << std::setprecision(12) << K(i, j);
      if (!(i == K.rows() - 1 && j == K.cols() - 1)) ofs << ", ";
    }
  }
  ofs << "]\n";
  ofs.close();
}

bool Controller::validateK(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, 
                          const Eigen::MatrixXd& K) {
  // 1) 闭环稳定性（离散系统：模 < 1）
  Eigen::MatrixXd M = A - B * K;
  Eigen::EigenSolver<Eigen::MatrixXd> es(M);
  
  for (int i = 0; i < M.rows(); ++i) {
    double eigenvalue_magnitude = std::abs(es.eigenvalues()[i]);
    if (eigenvalue_magnitude >= constants::kEigenvalueStabilityThreshold) {
      ROS_WARN("validateK: closed-loop eigenvalue[%d] magnitude = %f >= 1.0", 
               i, eigenvalue_magnitude);
      return false;
    }
  }
  
  // 2) 幅值测试
  for (int i = 0; i < constants::kValidationTestCount; i++) {
    Eigen::VectorXd xt = Eigen::VectorXd::Random(n_) * constants::kValidationStateScale;
    Eigen::VectorXd utest = -K * xt;
    double max_control = utest.cwiseAbs().maxCoeff();
    
    if (max_control > u_max_normal_ * constants::kMaxControlMultiplier) {
      ROS_WARN("validateK: control amplitude %f > limit %f", 
               max_control, u_max_normal_ * constants::kMaxControlMultiplier);
      return false;
    }
  }
  
  return true;
}

bool Controller::validateKFast(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, 
                               const Eigen::MatrixXd& K) {
  // ✅ 1. 简化的稳定性检查（只检查最大特征值）
  Eigen::MatrixXd M = A - B * K;
  
  if (!M.allFinite()) {
    return false;
  }
  
  // ✅ 2. 使用功率迭代法快速估计最大特征值（比完全特征值分解快）
  Eigen::VectorXd v = Eigen::VectorXd::Random(n_);
  v.normalize();
  
  for (int i = 0; i < 10; i++) {  // 只迭代10次
    v = M * v;
    double norm = v.norm();
    if (norm > constants::kEigenvalueStabilityThreshold) {
      ROS_WARN_THROTTLE(5.0, "Fast validation: max eigenvalue ~%.3f >= 1.0", norm);
      return false;
    }
    v.normalize();
  }
  
  // ✅ 3. 简化的控制幅度测试（只测试1个而不是5个）
  Eigen::VectorXd xt = Eigen::VectorXd::Random(n_) * constants::kValidationStateScale;
  Eigen::VectorXd utest = -K * xt;
  double max_control = utest.cwiseAbs().maxCoeff();
  
  if (max_control > u_max_normal_ * constants::kMaxControlMultiplier) {
    return false;
  }
  
  return true;
}

double Controller::computeResidual(const Eigen::MatrixXd& Theta_copy){
    int M = std::min((int)recent_phi_.size(), recent_buffer_len_);
  if (M < 5) return 1e6; // 样本太少，返回大残差
  double sumsq = 0.0;
  for (int i=0;i<M;i++){
    Eigen::VectorXd pred = Theta_copy * recent_phi_[i];
    Eigen::VectorXd err = recent_xnext_[i] - pred;
    sumsq += err.squaredNorm();
  }
  return sumsq / (double)M;
}

// 在resetThetaToDefault()函数中，修改默认模型
void Controller::resetThetaToDefault() {
  Theta_ = Eigen::MatrixXd::Zero(n_, n_ + m_);
  
  constexpr double dt = 0.001;
  constexpr double velocity_decay = 0.65;  // ✅ 降低到 0.85（增强阻尼）
  constexpr double input_gain = 1.0;       // ✅ 降低到 0.5（减少响应）
  
  // 位置行：纯积分关系
  Theta_(0, 1) = dt;
  Theta_(2, 3) = dt;
  
  // 速度行：衰减 + 输入响应
  Theta_(1, 1) = velocity_decay;
  Theta_(3, 3) = velocity_decay;
  Theta_(1, n_) = input_gain * dt;
  Theta_(3, n_+1) = input_gain * dt;
  
  // ✅ 完全移除耦合项
  Theta_(1, 0) = 0.0;
  Theta_(1, 2) = 0.0;
  Theta_(1, 3) = 0.0;
  Theta_(3, 0) = 0.0;
  Theta_(3, 1) = 0.0;
  Theta_(3, 2) = 0.0;
  
  Pcov_ = Eigen::MatrixXd::Identity(n_ + m_, n_ + m_) * 0.1;
  sample_count_ = 0;
  
  ROS_INFO("Theta reset: decay=%.3f, input_gain=%.3f", velocity_decay, input_gain);
}

void Controller::applyPhysicalConstraints() {
  constexpr double dt = 0.001;
  
  // 1. 固定位置行
  Theta_(0, 1) = dt;
  Theta_(2, 3) = dt;
  Theta_(0, 0) = 0.0;
  Theta_(0, 2) = 0.0;
  Theta_(0, 3) = 0.0;
  Theta_(2, 0) = 0.0;
  Theta_(2, 1) = 0.0;
  Theta_(2, 2) = 0.0;
  
  // 2. ✅ 严格限制速度衰减范围
  constexpr double kSafeMinDecay = 0.70;  // 提高下限（更强阻尼）
  constexpr double kSafeMaxDecay = 0.90;  // 降低上限（远离1.0）
  
  // Yaw 速度衰减
  if (Theta_(1, 1) < kSafeMinDecay) {
    ROS_WARN_THROTTLE(2.0, "Theta(1,1) = %.4f < %.3f, clamping", 
                     Theta_(1, 1), kSafeMinDecay);
    Theta_(1, 1) = kSafeMinDecay;
  } else if (Theta_(1, 1) > kSafeMaxDecay) {
    ROS_WARN_THROTTLE(2.0, "Theta(1,1) = %.4f > %.3f, clamping", 
                     Theta_(1, 1), kSafeMaxDecay);
    Theta_(1, 1) = kSafeMaxDecay;
  }
  
  // Base Yaw 速度衰减
  if (Theta_(3, 3) < kSafeMinDecay) {
    ROS_WARN_THROTTLE(2.0, "Theta(3,3) = %.4f < %.3f, clamping",
                     Theta_(3, 3), kSafeMinDecay);
    Theta_(3, 3) = kSafeMinDecay;
  } else if (Theta_(3, 3) > kSafeMaxDecay) {
    ROS_WARN_THROTTLE(2.0, "Theta(3,3) = %.4f > %.3f, clamping",
                     Theta_(3, 3), kSafeMaxDecay);
    Theta_(3, 3) = kSafeMaxDecay;
  }
  
  // 3. ✅ 严格限制输入增益
  double gain_yaw = Theta_(1, n_) / dt;
  double gain_base_yaw = Theta_(3, n_+1) / dt;
  
  constexpr double kSafeMinGain = 0.2;
  constexpr double kSafeMaxGain = 1.2;  // ✅ 进一步降低上限
  
  if (gain_yaw < kSafeMinGain) {
    gain_yaw = 0.5;
    ROS_WARN_THROTTLE(2.0, "Yaw gain too low, reset to 0.5");
  } else if (gain_yaw > kSafeMaxGain) {
    ROS_WARN_THROTTLE(2.0, "Yaw gain %.3f > %.3f, clamping", gain_yaw, kSafeMaxGain);
    gain_yaw = kSafeMaxGain;
  }
  
  if (gain_base_yaw < kSafeMinGain) {
    gain_base_yaw = 0.5;
    ROS_WARN_THROTTLE(2.0, "Base yaw gain too low, reset to 0.5");
  } else if (gain_base_yaw > kSafeMaxGain) {
    ROS_WARN_THROTTLE(2.0, "Base yaw gain %.3f > %.3f, clamping", 
                     gain_base_yaw, kSafeMaxGain);
    gain_base_yaw = kSafeMaxGain;
  }
  
  Theta_(1, n_) = gain_yaw * dt;
  Theta_(3, n_+1) = gain_base_yaw * dt;
  
  // 4. ✅ 完全禁用耦合项
  Theta_(1, 0) = 0.0;
  Theta_(1, 2) = 0.0;
  Theta_(1, 3) = 0.0;
  Theta_(3, 0) = 0.0;
  Theta_(3, 1) = 0.0;
  Theta_(3, 2) = 0.0;
  
  // 5. 定期打印
  static int log_count = 0;
  if (++log_count % 500 == 0) {
    ROS_INFO("Theta - decay: [%.4f, %.4f], gain: [%.3f, %.3f]",
             Theta_(1, 1), Theta_(3, 3),
             Theta_(1, n_)/dt, Theta_(3, n_+1)/dt);
  }
}

void Controller::commandCB(const rm_msgs::GimbalCmdConstPtr& msg){
  cmd_rt_buffer_.writeFromNonRT(*msg);
  //ROS_INFO("[Gimbal] Get new command");
}

bool Controller::loadKFromYaml(const std::string& path, Eigen::MatrixXd& K_out) {
  // 检查文件是否存在
  std::ifstream file(path);
  if (!file.is_open()) {
    ROS_WARN("Could not open K matrix file: %s", path.c_str());
    return false;
  }
  
  std::string line;
  while (std::getline(file, line)) {
    // 找到以"K: ["开头的行
    if (line.find("K: [") != std::string::npos) {
      // 提取括号内的内容
      size_t start = line.find('[');
      size_t end = line.find(']');
      if (start != std::string::npos && end != std::string::npos && end > start) {
        std::string values = line.substr(start + 1, end - start - 1);
        
        // 解析逗号分隔的值
        std::stringstream ss(values);
        std::string token;
        std::vector<double> k_values;
        
        while (std::getline(ss, token, ',')) {
          // 去除空格
          token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
          if (!token.empty()) {
            try {
              k_values.push_back(std::stod(token));
            } catch (const std::exception& e) {
              ROS_ERROR("Error parsing K value: %s", token.c_str());
              return false;
            }
          }
        }
        
        // 检查值的数量是否匹配
        if (k_values.size() == static_cast<size_t>(m_ * n_)) {
          K_out.resize(m_, n_);
          int idx = 0;
          for (int i = 0; i < m_; ++i) {
            for (int j = 0; j < n_; ++j) {
              K_out(i, j) = k_values[idx++];
            }
          }
          ROS_INFO("Successfully loaded K matrix from %s", path.c_str());
          return true;
        } else {
          ROS_ERROR("K matrix size mismatch: expected %dx%d (%d values), got %zu values",
                    m_, n_, m_*n_, k_values.size());
          return false;
        }
      }
    }
  }
  
  ROS_WARN("K matrix not found in file: %s", path.c_str());
  return false;
}

void Controller::trackCB(const rm_msgs::TrackDataConstPtr& msg){
  if (msg->id == 0)
    return;
  track_rt_buffer_.writeFromNonRT(*msg);
}

void Controller::reconfigCB(rm_gimbal_controllers::GimbalBaseConfig& config, uint32_t /*unused*/)
{
ROS_INFO("[Gimbal Base] Dynamic params change");
  
  if (!dynamic_reconfig_initialized_)
  {
    // 仅在第一次调用时从RT缓冲区读取初始值
    GimbalConfig init_config = *config_rt_buffer_.readFromNonRT();
    config.yaw_k_v_ = init_config.yaw_k_v_;
    config.pitch_k_v_ = init_config.pitch_k_v_;
    config.k_chassis_vel_ = init_config.k_chassis_vel_;
    config.accel_pitch_ = init_config.accel_pitch_;
    config.accel_yaw_ = init_config.accel_yaw_;
    dynamic_reconfig_initialized_ = true;
  }
  
  // 运行时更新配置（每次都执行）
  GimbalConfig config_non_rt;
  config_non_rt.yaw_k_v_ = config.yaw_k_v_;
  config_non_rt.pitch_k_v_ = config.pitch_k_v_;
  config_non_rt.k_chassis_vel_ = config.k_chassis_vel_;
  config_non_rt.accel_pitch_ = config.accel_pitch_;
  config_non_rt.accel_yaw_ = config.accel_yaw_;
  
  // LQR参数（如果启用在线更新）
  if (enable_online_lqr_) {
    config_non_rt.q1_ = config.q1;
    config_non_rt.q2_ = config.q2;
    config_non_rt.q3_ = config.q3;
    config_non_rt.q4_ = config.q4;
    config_non_rt.r1_ = config.r1;
    config_non_rt.r2_ = config.r2;
    config_non_rt.a1_ = config.a1;
    config_non_rt.a2_ = config.a2;
    config_non_rt.j1_ = config.j1;
    config_non_rt.j2_ = config.j2;
    config_non_rt.dc_ = config.dc_;
  }
  
  config_rt_buffer_.writeFromNonRT(config_non_rt);
  
}
Controller::~Controller(){
  ROS_INFO("[Gimbal] Destructor called");
  
  // 1. 设置停止标志
  running_ = false;
  
  // 2. 等待工作线程退出
  if (worker_thread_.joinable())
  {
    ROS_INFO("[Gimbal] Waiting for worker thread...");
    
    // 设置超时等待（防止死锁）
    auto future = std::async(std::launch::async, [this]() {
      if (worker_thread_.joinable()) {
        worker_thread_.join();
      }
    });
    
    // 等待最多5秒
    if (future.wait_for(std::chrono::seconds(5)) == std::future_status::timeout) {
      ROS_ERROR("[Gimbal] Worker thread join timeout, forcing termination");
      // 注意：这种情况下线程可能不会正常清理
    } else {
      ROS_INFO("[Gimbal] Worker thread joined successfully");
    }
  }
  
  // 3. 清理动态分配的资源
  delete ramp_rate_pitch_;
  delete ramp_rate_yaw_;
  delete d_srv_;
  
  ROS_INFO("[Gimbal] Destructor completed");
}
}  // namespace rm_gimbal_controllers

PLUGINLIB_EXPORT_CLASS(rm_gimbal_controllers::Controller, controller_interface::ControllerBase)
