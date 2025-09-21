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
#include "rm_gimbal_controllers/dual_stage_gimbal.h"

#include <string>
#include <angles/angles.h>
#include <rm_common/ori_tool.h>
#include <pluginlib/class_list_macros.hpp>
#include <rm_gimbal_controllers/stack_yaw_gimbal.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>

namespace rm_gimbal_controllers
{

bool DualStageController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  auto nh_yaw = ros::NodeHandle(controller_nh, "yaw");
  auto nh_pitch = ros::NodeHandle(controller_nh, "pitch");
  auto nh_pid_yaw_pos = ros::NodeHandle(controller_nh, "yaw/pid_pos");
  auto nh_pid_pitch_pos = ros::NodeHandle(controller_nh, "pitch/pid_pos");

  config_ =
  {
    .yaw_k_v_ = getParam(nh_yaw, "k_v", 0.),
    .pitch_k_v_ = getParam(nh_pitch, "k_v", 0.),
    .k_chassis_vel_ = getParam(controller_nh, "yaw/k_chassis_vel", 0.),
    .accel_yaw_ = getParam(controller_nh, "yaw/accel", 99.),
    .accel_pitch_ = getParam(controller_nh, "pitch/accel", 99.)
  };

  if (!ControllerBase::init(robot_hw, root_nh, controller_nh)) return false;

  auto* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (!ctrl_yaw_.init(effort_joint_interface, nh_yaw) || !ctrl_pitch_.init(effort_joint_interface, nh_pitch) ||
      !pid_yaw_pos_.init(nh_pid_yaw_pos) || !pid_pitch_pos_.init(nh_pid_pitch_pos))
    return false;
  // Get URDF info about joint
  urdf::Model urdf;
  if (!urdf.initParamWithNodeHandle("robot_description", controller_nh))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }

  pitch_joint_urdf_ = urdf.getJoint(this->ctrl_pitch_.getJointName());
  yaw_joint_urdf_ = urdf.getJoint(this->ctrl_yaw_.getJointName());
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

  yaw_pos_state_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::GimbalPosState>(nh_yaw, "pos_state", 1));
  pitch_pos_state_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::GimbalPosState>(nh_pitch, "pos_state", 1));

  ROS_INFO("DualStageGimbal inited.");
  return true;
}

void DualStageController::update(const ros::Time& time, const ros::Duration& period)
{
  cmd_gimbal_ = *cmd_rt_buffer_.readFromRT();
  data_track_ = *track_rt_buffer_.readFromNonRT();
  config_ = *config_rt_buffer_.readFromRT();
  ramp_rate_pitch_->setAcc(config_.accel_pitch_);
  ramp_rate_yaw_->setAcc(config_.accel_yaw_);
  ramp_rate_pitch_->input(cmd_gimbal_.rate_pitch);
  ramp_rate_yaw_->input(cmd_gimbal_.rate_yaw);
  cmd_gimbal_.rate_pitch = ramp_rate_pitch_->output();
  cmd_gimbal_.rate_yaw = ramp_rate_yaw_->output();
  try
  {
    odom2pitch_ = robot_state_handle_.lookupTransform("odom", pitch_joint_urdf_->child_link_name, time);
    odom2base_ = robot_state_handle_.lookupTransform("odom", yaw_joint_urdf_->parent_link_name, time);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
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
  }
  moveJoint(time, period);
}

void DualStageController::rate(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter RATE");
    odom2gimbal_des_.transform.rotation = odom2pitch_.transform.rotation;
    odom2gimbal_des_.header.stamp = time;
    robot_state_handle_.setTransform(odom2gimbal_des_, "rm_gimbal_controllers");
  }
  else
  {
    double roll{}, pitch{}, yaw{};
    quatToRPY(odom2gimbal_des_.transform.rotation, roll, pitch, yaw);
    setDes(time, yaw + period.toSec() * cmd_gimbal_.rate_yaw, pitch + period.toSec() * cmd_gimbal_.rate_pitch);
  }
}

void DualStageController::track(const ros::Time& time)
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
  target_pos.x += target_vel.x * (time - data_track_.header.stamp).toSec() - odom2pitch_.transform.translation.x;
  target_pos.y += target_vel.y * (time - data_track_.header.stamp).toSec() - odom2pitch_.transform.translation.y;
  target_pos.z += target_vel.z * (time - data_track_.header.stamp).toSec() - odom2pitch_.transform.translation.z;
  target_vel.x -= chassis_vel_->linear_->x();
  target_vel.y -= chassis_vel_->linear_->y();
  target_vel.z -= chassis_vel_->linear_->z();
  bool solve_success = bullet_solver_->solve(target_pos, target_vel, cmd_gimbal_.bullet_speed, yaw, data_track_.v_yaw,
                                             data_track_.radius_1, data_track_.radius_2, data_track_.dz,
                                             data_track_.armors_num, chassis_vel_->angular_->z());
  //bullet_solver_->judgeShootBeforehand(time);

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

void DualStageController::direct(const ros::Time& time)
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

void DualStageController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  geometry_msgs::Vector3 gyro, angular_vel_pitch, angular_vel_yaw;
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
  }
  double roll_real, pitch_real, yaw_real, roll_des, pitch_des, yaw_des;
  quatToRPY(odom2gimbal_des_.transform.rotation, roll_des, pitch_des, yaw_des);
  quatToRPY(odom2pitch_.transform.rotation, roll_real, pitch_real, yaw_real);
  double yaw_angle_error = angles::shortest_angular_distance(yaw_real, yaw_des);
  double pitch_angle_error = angles::shortest_angular_distance(pitch_real, pitch_des);
  pid_pitch_pos_.computeCommand(pitch_angle_error, period);
  pid_yaw_pos_.computeCommand(yaw_angle_error, period);

  double yaw_vel_des = 0., pitch_vel_des = 0.;
  if (state_ == RATE)
  {
    yaw_vel_des = cmd_gimbal_.rate_yaw;
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

  pid_pitch_pos_.computeCommand(pitch_angle_error, period);
  pid_yaw_pos_.computeCommand(yaw_angle_error, period);

  // publish state
  if (loop_count_ % 10 == 0)
  {
    if (yaw_pos_state_pub_ && yaw_pos_state_pub_->trylock())
    {
      yaw_pos_state_pub_->msg_.header.stamp = time;
      yaw_pos_state_pub_->msg_.set_point = yaw_des;
      yaw_pos_state_pub_->msg_.set_point_dot = yaw_vel_des;
      yaw_pos_state_pub_->msg_.process_value = yaw_real;
      yaw_pos_state_pub_->msg_.error = angles::shortest_angular_distance(yaw_real, yaw_des);
      yaw_pos_state_pub_->msg_.command = pid_yaw_pos_.getCurrentCmd();
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

  ctrl_yaw_.setCommand(pid_yaw_pos_.getCurrentCmd() - config_.k_chassis_vel_ * chassis_vel_->angular_->z() +
                       config_.yaw_k_v_ * yaw_vel_des + ctrl_yaw_.joint_.getVelocity() - angular_vel_yaw.z);
  ctrl_pitch_.setCommand(pid_pitch_pos_.getCurrentCmd() + config_.pitch_k_v_ * pitch_vel_des +
                         ctrl_pitch_.joint_.getVelocity() - angular_vel_pitch.y);

  ctrl_yaw_.update(time, period);
  ctrl_pitch_.update(time, period);
  ctrl_pitch_.joint_.setCommand(ctrl_pitch_.joint_.getCommand() + feedForward(time));
}

void DualStageController::commandCB(const rm_msgs::GimbalCmdConstPtr& msg)
{
  ControllerBase::commandCB(msg);
}

void DualStageController::trackCB(const rm_msgs::TrackDataConstPtr& msg)
{
  ControllerBase::trackCB(msg);
}

}  // namespace rm_gimbal_controllers

PLUGINLIB_EXPORT_CLASS(rm_gimbal_controllers::DualStageController, controller_interface::ControllerBase)
