//
// Created by cilotta on 24-10-18.
//

#pragma once

#include "rm_gimbal_controllers/gimbal_base.h"

#include <effort_controllers/joint_velocity_controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_msgs/TrackData.h>
#include <rm_gimbal_controllers/bullet_solver.h>
#include <control_toolbox/pid.h>
#include <urdf/model.h>
#include <realtime_tools/realtime_publisher.h>

namespace rm_gimbal_controllers
{

class StackYawController : public ControllerBase<rm_control::RobotStateInterface,
                                                 hardware_interface::ImuSensorInterface,
                                                 hardware_interface::EffortJointInterface>
{
public:
    StackYawController() = default;
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
    void setDes(const ros::Time& time, double pitch_des, double base_yaw_des, double yaw_des);
protected:
    void rate(const ros::Time& time, const ros::Duration& period) override;
    void track(const ros::Time& time) override;
    void direct(const ros::Time& time) override;

    void moveJoint(const ros::Time& time, const ros::Duration& period) override;

    effort_controllers::JointVelocityController ctrl_base_yaw_;
    control_toolbox::Pid pid_base_yaw_pos_;

    urdf::JointConstSharedPtr base_yaw_joint_urdf_;

    bool base_yaw_des_in_limit_{};
    geometry_msgs::TransformStamped odom2base_yaw_;

};

}
