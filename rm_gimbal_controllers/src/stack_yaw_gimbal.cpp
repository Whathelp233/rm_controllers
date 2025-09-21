//
// Created by cilotta on 24-10-18.
//

#include "rm_gimbal_controllers/stack_yaw_gimbal.h"

#include <string>
#include <angles/angles.h>
#include <rm_common/ori_tool.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>

namespace rm_gimbal_controllers
{
bool StackYawController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  ControllerBase::init(robot_hw, root_nh, controller_nh);
  auto nh_yaw = ros::NodeHandle(controller_nh, "yaw");
  auto nh_pitch = ros::NodeHandle(controller_nh, "pitch");
  auto nh_pid_yaw_pos = ros::NodeHandle(controller_nh, "yaw/pid_pos");
  auto nh_pid_pitch_pos = ros::NodeHandle(controller_nh, "pitch/pid_pos");
  auto nh_base_yaw = ros::NodeHandle(controller_nh, "base_yaw");
  auto nh_pid_base_yaw_pos = ros::NodeHandle(controller_nh, "base_yaw/pid_pos");

  auto* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (!ctrl_base_yaw_.init(effort_joint_interface, nh_base_yaw) || !ctrl_yaw_.init(effort_joint_interface, nh_yaw)
      || !ctrl_pitch_.init(effort_joint_interface, nh_pitch) || !pid_base_yaw_pos_.init(nh_pid_base_yaw_pos) ||
      !pid_yaw_pos_.init(nh_pid_yaw_pos) || !pid_pitch_pos_.init(nh_pid_pitch_pos))
      return false;

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
  odom2base_.child_frame_id = base_yaw_joint_urdf_->parent_link_name;
  odom2base_.transform.rotation.w = 1.;

  return true;
}

void StackYawController::update(const ros::Time& time, const ros::Duration& period)
{
  cmd_gimbal_ = *cmd_rt_buffer_.readFromRT();
  data_track_ = *track_rt_buffer_.readFromNonRT();
  config_ = *config_rt_buffer_.readFromRT();
  ramp_rate_pitch_->setAcc(config_.accel_pitch_);
  ramp_rate_yaw_->setAcc(config_.accel_yaw_);
  ramp_rate_base_yaw_->setAcc(config_.accel_base_yaw_);
  ramp_rate_pitch_->input(cmd_gimbal_.rate_pitch);
  ramp_rate_base_yaw_->input(cmd_gimbal_.rate_base_yaw);
  ramp_rate_yaw_->input(cmd_gimbal_.rate_yaw);
  cmd_gimbal_.rate_pitch = ramp_rate_pitch_->output();
  cmd_gimbal_.rate_base_yaw = ramp_rate_base_yaw_->output();
  cmd_gimbal_.rate_yaw = ramp_rate_yaw_->output();
  try
  {
    odom2pitch_ = robot_state_handle_.lookupTransform("odom", pitch_joint_urdf_->child_link_name, time);
    odom2base_ = robot_state_handle_.lookupTransform("odom", base_yaw_joint_urdf_->parent_link_name, time);
    odom2base_yaw_ = robot_state_handle_.lookupTransform("odom", base_yaw_joint_urdf_->child_link_name, time);
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

void StackYawController::setDes(const ros::Time& time, double pitch_des, double base_yaw_des, double yaw_des)
{
  tf2::Quaternion odom2base, odom2gimbal_des, base2gimbal_des;
  fromMsg(odom2base_.transform.rotation, odom2base);
  odom2gimbal_des.setRPY(0, pitch_des, yaw_des);
  base2gimbal_des = odom2base.inverse() * odom2gimbal_des;
  double roll_temp, base2gimbal_current_des_pitch, base2gimbal_current_des_yaw;
  quatToRPY(toMsg(base2gimbal_des), roll_temp, base2gimbal_current_des_pitch, base2gimbal_current_des_yaw);
  double pitch_real_des, yaw_real_des, base_yaw_real_des;

  pitch_des_in_limit_ = setDesIntoLimit(pitch_real_des, pitch_des, base2gimbal_current_des_pitch, pitch_joint_urdf_);
  if (!pitch_des_in_limit_)
  {
    double yaw_temp;
    tf2::Quaternion base2new_des;
    double upper_limit = pitch_joint_urdf_->limits ? pitch_joint_urdf_->limits->upper : 1e16;
    double lower_limit = pitch_joint_urdf_->limits ? pitch_joint_urdf_->limits->lower : -1e16;
    base2new_des.setRPY(0,
                        std::abs(angles::shortest_angular_distance(base2gimbal_current_des_pitch, upper_limit)) <
                                std::abs(angles::shortest_angular_distance(base2gimbal_current_des_pitch, lower_limit)) ?
                            upper_limit :
                            lower_limit,
                        base2gimbal_current_des_yaw);
    quatToRPY(toMsg(odom2base * base2new_des), roll_temp, pitch_real_des, yaw_temp);
  }
  base_yaw_des_in_limit_ = setDesIntoLimit(base_yaw_real_des, base_yaw_des, base2gimbal_current_des_yaw, base_yaw_joint_urdf_);
  if (!base_yaw_des_in_limit_)
  {
    double pitch_temp;
    tf2::Quaternion base2new_des;
    double upper_limit = base_yaw_joint_urdf_->limits ? base_yaw_joint_urdf_->limits->upper : 1e16;
    double lower_limit = base_yaw_joint_urdf_->limits ? base_yaw_joint_urdf_->limits->lower : -1e16;
    base2new_des.setRPY(0, base2gimbal_current_des_pitch,
                        std::abs(angles::shortest_angular_distance(base2gimbal_current_des_yaw, upper_limit)) <
                                std::abs(angles::shortest_angular_distance(base2gimbal_current_des_yaw, lower_limit)) ?
                            upper_limit :
                            lower_limit);
    quatToRPY(toMsg(odom2base * base2new_des), roll_temp, pitch_temp, yaw_real_des);
  }
  yaw_des_in_limit_ = ControllerBase::setDesIntoLimit(yaw_real_des, yaw_des, base2gimbal_current_des_yaw, yaw_joint_urdf_);
  if (!yaw_des_in_limit_)
  {
    double pitch_temp;
    tf2::Quaternion base2new_des;
    double upper_limit = yaw_joint_urdf_->limits ? yaw_joint_urdf_->limits->upper : 1e16;
    double lower_limit = yaw_joint_urdf_->limits ? yaw_joint_urdf_->limits->lower : -1e16;
    base2new_des.setRPY(0, base2gimbal_current_des_pitch,
                        std::abs(angles::shortest_angular_distance(base2gimbal_current_des_yaw, upper_limit)) <
                                std::abs(angles::shortest_angular_distance(base2gimbal_current_des_yaw, lower_limit)) ?
                            upper_limit :
                            lower_limit);
    quatToRPY(toMsg(odom2base * base2new_des), roll_temp, pitch_temp, yaw_real_des);
  }

  odom2gimbal_des_.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0., pitch_real_des, yaw_real_des + base_yaw_real_des);
  odom2gimbal_des_.header.stamp = time;
  robot_state_handle_.setTransform(odom2gimbal_des_, "rm_gimbal_controllers");
}

void StackYawController::rate(const ros::Time& time, const ros::Duration& period)
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
    double temp_{}, base_yaw{};
    quatToRPY(odom2gimbal_des_.transform.rotation, roll, pitch, yaw);
    quatToRPY(odom2base_yaw_.transform.rotation, temp_, temp_, base_yaw);
    setDes(time, pitch + period.toSec() * cmd_gimbal_.rate_pitch, base_yaw + period.toSec() * cmd_gimbal_.rate_base_yaw,
      yaw - base_yaw + period.toSec() * cmd_gimbal_.rate_yaw);
  }
}

void StackYawController::track(const ros::Time& time)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter TRACK");
  }
  double roll_real, pitch_real, base_yaw_real;
  double yaw_eq; //Equals to (base_yaw_pos + yaw_pos), means the equivalent of yaw of two axes gimbal
  double temp_;
  quatToRPY(odom2pitch_.transform.rotation, roll_real, pitch_real, yaw_eq);
  quatToRPY(odom2base_yaw_.transform.rotation, temp_, temp_, base_yaw_real);
  double yaw_compute = yaw_eq;
  //yaw_real = yaw_eq;
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
    setDes(time, bullet_solver_->getPitch(), bullet_solver_->getYaw(), bullet_solver_->getYaw());
  else
  {
    odom2gimbal_des_.header.stamp = time;
    robot_state_handle_.setTransform(odom2gimbal_des_, "rm_gimbal_controllers");
  }
}

void StackYawController::direct(const ros::Time& time)
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
  double base_yaw = std::atan2(aim_point_odom.y - odom2pitch_.transform.translation.y,
                          aim_point_odom.x - odom2pitch_.transform.translation.x);
  // double yaw = std::atan2(aim_point_odom.y - odom2pitch_.transform.translation.y,
  //                         aim_point_odom.x - odom2pitch_.transform.translation.x);
  double pitch = -std::atan2(aim_point_odom.z - odom2pitch_.transform.translation.z,
                             std::sqrt(std::pow(aim_point_odom.x - odom2pitch_.transform.translation.x, 2) +
                                       std::pow(aim_point_odom.y - odom2pitch_.transform.translation.y, 2)));
  setDes(time, pitch, base_yaw, 0);
}

void StackYawController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  geometry_msgs::Vector3 gyro, angular_vel_pitch, angular_vel_yaw_eq;
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
      tf2::doTransform(gyro, angular_vel_yaw_eq,
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
    angular_vel_yaw_eq.z = ctrl_base_yaw_.joint_.getVelocity() + ctrl_yaw_.joint_.getVelocity();
    angular_vel_pitch.y = ctrl_pitch_.joint_.getVelocity();
  }
  double roll_real, pitch_real, yaw_real, base_yaw_real, roll_des, pitch_des, yaw_des, base_yaw_des, yaw_eq;
  double roll_temp, pitch_temp;
  quatToRPY(odom2gimbal_des_.transform.rotation, roll_des, pitch_des, yaw_eq);
  quatToRPY(odom2pitch_.transform.rotation, roll_real, pitch_real, yaw_real);
  quatToRPY(odom2base_yaw_.transform.rotation, roll_temp, pitch_temp, base_yaw_real);
  yaw_des = yaw_eq;
  base_yaw_des = yaw_des;
  double base_yaw_angle_error = angles::shortest_angular_distance(base_yaw_real, yaw_des);
  double yaw_angle_error = angles::shortest_angular_distance(yaw_real - base_yaw_real, 0);
  //double duration_yaw_angle_error = angles::shortest_angular_distance(yaw_real, yaw_des);
  double pitch_angle_error = angles::shortest_angular_distance(pitch_real, pitch_des);
  pid_pitch_pos_.computeCommand(pitch_angle_error, period);
  pid_yaw_pos_.computeCommand(yaw_angle_error, period);
  pid_base_yaw_pos_.computeCommand(base_yaw_angle_error, period);

  double base_yaw_vel_des = 0., yaw_vel_des = 0., pitch_vel_des = 0.;
  if (state_ == RATE)
  {
    base_yaw_vel_des = cmd_gimbal_.rate_base_yaw;
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
      base_yaw_vel_des = target_pos_tf.cross(target_vel_tf).z() / std::pow((target_pos_tf.length()), 2);
      geometry_msgs::TransformStamped transform = robot_state_handle_.lookupTransform(
          base_yaw_joint_urdf_->parent_link_name, data_track_.header.frame_id, data_track_.header.stamp);
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
  if (!base_yaw_des_in_limit_)
    base_yaw_vel_des = 0.;
  if (!yaw_des_in_limit_)
    yaw_vel_des = 0.;

  pid_base_yaw_pos_.computeCommand(base_yaw_angle_error, period);
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
    if (base_yaw_pos_state_pub_ && base_yaw_pos_state_pub_->trylock())
    {
      base_yaw_pos_state_pub_->msg_.header.stamp = time;
      base_yaw_pos_state_pub_->msg_.set_point = base_yaw_des;
      base_yaw_pos_state_pub_->msg_.set_point_dot = base_yaw_vel_des;
      base_yaw_pos_state_pub_->msg_.process_value = base_yaw_real;
      base_yaw_pos_state_pub_->msg_.error = angles::shortest_angular_distance(base_yaw_real, base_yaw_des);
      base_yaw_pos_state_pub_->msg_.command = pid_base_yaw_pos_.getCurrentCmd();
      base_yaw_pos_state_pub_->unlockAndPublish();
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
                       config_.yaw_k_v_ * yaw_vel_des + ctrl_yaw_.joint_.getVelocity() - angular_vel_yaw_eq.z);
  ctrl_pitch_.setCommand(pid_pitch_pos_.getCurrentCmd() + config_.pitch_k_v_ * pitch_vel_des +
                         ctrl_pitch_.joint_.getVelocity() - angular_vel_pitch.y);
  ctrl_base_yaw_.setCommand(pid_base_yaw_pos_.getCurrentCmd() - config_.k_chassis_vel_ * chassis_vel_->angular_->z() +
                       config_.base_yaw_k_v_ * base_yaw_vel_des + ctrl_base_yaw_.joint_.getVelocity() - angular_vel_yaw_eq.z);

  ctrl_yaw_.update(time, period);
  ctrl_pitch_.update(time, period);
  ctrl_base_yaw_.update(time, period);
  ctrl_pitch_.joint_.setCommand(ctrl_pitch_.joint_.getCommand() + feedForward(time));
}

}  // namespace rm_StackYawControllers

PLUGINLIB_EXPORT_CLASS(rm_gimbal_controllers::StackYawController, controller_interface::ControllerBase)
