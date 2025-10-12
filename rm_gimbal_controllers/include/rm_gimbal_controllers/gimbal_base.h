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
        
#pragma once

#include <effort_controllers/joint_velocity_controller.h>
#include <effort_controllers/joint_effort_controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_common/filters/filters.h>
#include <rm_common/lqr.h>
#include <rm_msgs/GimbalCmd.h>
#include <rm_msgs/TrackData.h>
#include <rm_msgs/GimbalDesError.h>
#include <rm_msgs/GimbalPosState.h>
#include <rm_gimbal_controllers/GimbalBaseConfig.h>
#include <rm_gimbal_controllers/bullet_solver.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Eigen>
#include <control_toolbox/pid.h>
#include <urdf/model.h>
#include <dynamic_reconfigure/server.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64MultiArray.h>

// ...existing code...

template<typename T>
class NonlinearTrackingDifferentiator {
public:
  NonlinearTrackingDifferentiator(T r, T h, T x1_limit = 1e6, T x2_limit = 1e6) 
    : r_(r), h_(h), x1_(0), x2_(0), x1_limit_(x1_limit), x2_limit_(x2_limit) {}
  
  void update(T v, T v_dot_expected = 0) {
    // ✅ 使用兼容 C++11/14 的 clamp 实现
    T v_clamped = clamp(v, -x1_limit_, x1_limit_);
    T v_dot_clamped = clamp(v_dot_expected, -x2_limit_, x2_limit_);
    
    T fh = fhan(x1_ - v_clamped, x2_ - v_dot_clamped, r_, h_);
    
    x1_ += h_ * x2_;
    x2_ += h_ * fh;
    
    // 输出限幅
    x1_ = clamp(x1_, -x1_limit_, x1_limit_);
    x2_ = clamp(x2_, -x2_limit_, x2_limit_);
  }
  
  T getX1() const { return x1_; }
  T getX2() const { return x2_; }
  
  void reset(T x1 = 0, T x2 = 0) {
    x1_ = clamp(x1, -x1_limit_, x1_limit_);
    x2_ = clamp(x2, -x2_limit_, x2_limit_);
  }
  
  void setLimits(T x1_limit, T x2_limit) {
    x1_limit_ = x1_limit;
    x2_limit_ = x2_limit;
  }
  
private:
  // ✅ 添加：兼容 C++11/14 的 clamp 函数
  T clamp(T value, T min_val, T max_val) const {
    return std::max(min_val, std::min(value, max_val));
  }
  
  T fhan(T x1, T x2, T r, T h) {
    constexpr T epsilon = 1e-10;
    T d = r * h * h;
    
    if (d < epsilon) {
      return -r * sign(x1);
    }
    
    T a0 = h * x2;
    T y = x1 + a0;
    T abs_y = std::abs(y);
    
    if (abs_y < epsilon) {
      return -r * sign(a0);
    }
    
    T a1 = std::sqrt(d * (d + 8 * abs_y));
    T a2 = a0 + (y >= 0 ? 1 : -1) * (a1 - d) / 2;
    T sy = (sign(y + d) - sign(y - d)) / 2;
    T a = (a0 + y - a2) * sy + a2;
    T sa = (sign(a + d) - sign(a - d)) / 2;
    
    return -r * (a / d - sign(a)) * sa - r * sign(a);
  }
  
  T sign(T x) const {
    if (x > epsilon_) return 1;
    if (x < -epsilon_) return -1;
    return 0;
  }
  
  static constexpr T epsilon_ = 1e-10;
  
  T r_;          // 速度因子
  T h_;          // 采样时间
  T x1_;         // 跟踪信号
  T x2_;         // 微分信号
  T x1_limit_;   // 位置限幅
  T x2_limit_;   // 速度限幅
};

// ...existing code...


namespace rm_gimbal_controllers
{
struct GimbalConfig
{
  double yaw_k_v_, pitch_k_v_, k_chassis_vel_;
  double accel_pitch_{}, accel_yaw_{};
  double q1_, q2_, q3_, q4_, r1_, r2_, a1_, a2_, j1_, j2_, dc_;
};
class KalmanFilter {
public:
  KalmanFilter() : x_(0.0), P_(1.0), Q_(0.01), R_(0.1) {}  // 默认构造函数
  KalmanFilter(double process_noise, double measurement_noise, double initial_estimate, double initial_error)
    : x_(initial_estimate), P_(initial_error), Q_(process_noise), R_(measurement_noise) {}  // 带参数构造函数

  double update(double measurement) {
    // 预测步骤
    double x_pred = x_;
    double P_pred = P_ + Q_;

    // 更新步骤
    double K = P_pred / (P_pred + R_);  // 卡尔曼增益
    x_ = x_pred + K * (measurement - x_pred);
    P_ = (1 - K) * P_pred;

    return x_;
  }

private:
  double x_, P_, Q_, R_;
};

class ChassisVel
{
public:
  ChassisVel(const ros::NodeHandle& nh)
  {
    double num_data;
    nh.param("num_data", num_data, 20.0);
    nh.param("debug", is_debug_, true);
    linear_ = std::make_shared<Vector3WithFilter<double>>(num_data);
    angular_ = std::make_shared<Vector3WithFilter<double>>(num_data);
    if (is_debug_)
    {
      real_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(nh, "real", 1));
      filtered_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(nh, "filtered", 1));
    }
  }
  std::shared_ptr<Vector3WithFilter<double>> linear_;
  std::shared_ptr<Vector3WithFilter<double>> angular_;
  void update(double linear_vel[3], double angular_vel[3], double period)
  {
    if (period < 0)
      return;
    if (period > 0.1)
    {
      linear_->clear();
      angular_->clear();
    }
    linear_->input(linear_vel);
    angular_->input(angular_vel);
    if (is_debug_ && loop_count_ % 10 == 0)
    {
      if (real_pub_->trylock())
      {
        real_pub_->msg_.linear.x = linear_vel[0];
        real_pub_->msg_.linear.y = linear_vel[1];
        real_pub_->msg_.linear.z = linear_vel[2];
        real_pub_->msg_.angular.x = angular_vel[0];
        real_pub_->msg_.angular.y = angular_vel[1];
        real_pub_->msg_.angular.z = angular_vel[2];

        real_pub_->unlockAndPublish();
      }
      if (filtered_pub_->trylock())
      {
        filtered_pub_->msg_.linear.x = linear_->x();
        filtered_pub_->msg_.linear.y = linear_->y();
        filtered_pub_->msg_.linear.z = linear_->z();
        filtered_pub_->msg_.angular.x = angular_->x();
        filtered_pub_->msg_.angular.y = angular_->y();
        filtered_pub_->msg_.angular.z = angular_->z();

        filtered_pub_->unlockAndPublish();
      }
    }
    loop_count_++;
  }

private:
  bool is_debug_;
  int loop_count_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist>> real_pub_{}, filtered_pub_{};
};

class Controller : public controller_interface::MultiInterfaceController<rm_control::RobotStateInterface,
                                                                         hardware_interface::ImuSensorInterface,
                                                                         hardware_interface::EffortJointInterface>
{
public:
  Controller() = default;
  ~Controller();
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void setDes(const ros::Time& time, double yaw_des, double pitch_des);
  bool loadKFromYaml(const std::string& path, Eigen::MatrixXd& K_out);
  void stopping(const ros::Time& time) override;
  

private:
  void rate(const ros::Time& time, const ros::Duration& period);
  void track(const ros::Time& time);
  void direct(const ros::Time& time);
  void traj(const ros::Time& time);
  bool setDesIntoLimit(double& real_des, double current_des, double base2gimbal_current_des,
                       const urdf::JointConstSharedPtr& joint_urdf);

  void onlineLQRUpdate();
  void updateRLS();
  bool computeDLQRdiscrete(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                           const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
                           Eigen::MatrixXd& K_out);
  bool validateK(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& K);
  bool validateKFast(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, 
                               const Eigen::MatrixXd& K);
  bool computeDLQRdiscreteFast(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                                         const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
                                         Eigen::MatrixXd& K_out);
  double computeResidual(const Eigen::MatrixXd& Theta_copy);
  void resetThetaToDefault();
  void applyPhysicalConstraints();

  void saveKToYaml(const Eigen::MatrixXd& K, const std::string& path);

  void moveJoint(const ros::Time& time, const ros::Duration& period);
  double feedForward(const ros::Time& time);
  void updateChassisVel();
  void commandCB(const rm_msgs::GimbalCmdConstPtr& msg);
  void trackCB(const rm_msgs::TrackDataConstPtr& msg);
  void reconfigCB(rm_gimbal_controllers::GimbalBaseConfig& config, uint32_t);
  void updateStrategy(double current_error);

  bool computeDLQRdiscretePrecise(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                                   const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
                                   Eigen::MatrixXd& K_out);

  //LQR
  //Lqr lqr_yaw_, lqr_pitch_;  // 或自定义LQR类
  Eigen::MatrixXd K_yaw_;     // LQR增益矩阵
  Eigen::VectorXd state_yaw_;  // 状态向量
  Eigen::Vector4d x_ref;
  Eigen::VectorXd u;
  //RLS
  int n_; // 状态维度
  int m_; // 输入维度（外环输出维度）
  Eigen::MatrixXd Theta_;   // n x (n+m)
  Eigen::MatrixXd Pcov_;    // (n+m) x (n+m)
  double lambda_rls_;       // 遗忘因子

  std::mutex rls_mutex_;
  Eigen::VectorXd x_prev_;        // x_k
  Eigen::VectorXd u_prev_sample_; // u_k (vel_ref)
  bool have_prev_sample_;
  size_t sample_count_;
  std::vector<Eigen::VectorXd> recent_phi_;
  std::vector<Eigen::VectorXd> recent_xnext_;
  int recent_buffer_len_;

  double u_yaw_cmd_;      // 保存yaw速度命令
  double u_base_yaw_cmd_; // 保存base_yaw速度命令
  double u_yaw_;          // 最终yaw速度命令
  double u_base_yaw_;     // 最终base_yaw速度命令
  //RLS_END

  //LQR_UPDATE
  Eigen::MatrixXd K_use_;
  Eigen::MatrixXd K_current_; // m x n (只在持有K_mutex_时访问)
  Eigen::MatrixXd K_target_;
  Eigen::MatrixXd K_old_;
  Eigen::MatrixXd K_safe_read_;  // ✅ 主循环读取用(受K_mutex_保护)
  std::mutex K_mutex_;
  std::atomic<bool> switching_;
  std::atomic<bool> K_ready_;  // ✅ 标记K_safe_read_是否已初始化
  ros::Time switch_start_time_;
  double switch_smooth_T_;
  //LQR_UPDATE_END

   // Q, R for discrete LQR (user-tunable)
  Eigen::MatrixXd Qd_, Rd_;

  // ---------- worker thread ----------
  std::thread worker_thread_;
  std::atomic<bool> running_;
  int worker_hz_;
  int N_min_samples_;
  double residual_threshold_;
  double stable_tol_;
  int stable_needed_;
  Eigen::MatrixXd last_successful_K_;
  int stable_count_;

  // ---------- safety / storage ----------
  double u_max_normal_;
  double u_max_;
  bool driver_saturated_;
  std::string save_k_path_;

  // ---------- bookkeeping ------------
  Eigen::VectorXd vel_ref_prev_;
  Eigen::VectorXd u_prev_out_;


  Eigen::MatrixXd system_matrix_a_;   
  Eigen::MatrixXd control_matrix_b_;    
  Eigen::MatrixXd state_weight_q_;      
  Eigen::MatrixXd control_weight_r_;    

  KalmanFilter kf_yaw_;
  KalmanFilter kf_base_yaw_;

  bool enable_online_lqr_;

  //LQR END
  rm_control::RobotStateHandle robot_state_handle_;
  hardware_interface::ImuSensorHandle imu_sensor_handle_;
  bool has_imu_ = true;
  effort_controllers::JointVelocityController ctrl_pitch_;
  effort_controllers::JointVelocityController  ctrl_yaw_;
  effort_controllers::JointVelocityController  ctrl_base_yaw_;
  // hardware_interface::JointHandle ctrl_yaw_;

  control_toolbox::Pid pid_yaw_pos_, pid_pitch_pos_;

  std::shared_ptr<BulletSolver> bullet_solver_;

  // ROS Interface
  ros::Time last_publish_time_{};
  std::unique_ptr<realtime_tools::RealtimePublisher<rm_msgs::GimbalPosState>> yaw_pos_state_pub_, pitch_pos_state_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::GimbalDesError>> error_pub_;
  ros::Subscriber cmd_gimbal_sub_;
  ros::Subscriber data_track_sub_;
  realtime_tools::RealtimeBuffer<rm_msgs::GimbalCmd> cmd_rt_buffer_;
  realtime_tools::RealtimeBuffer<rm_msgs::TrackData> track_rt_buffer_;
  urdf::JointConstSharedPtr pitch_joint_urdf_, yaw_joint_urdf_,base_yaw_joint_urdf_;
  
  rm_msgs::GimbalCmd cmd_gimbal_;
  rm_msgs::TrackData data_track_;
  std::string gimbal_des_frame_id_{}, imu_name_{};
  double publish_rate_{};
  bool state_changed_{};
  bool pitch_des_in_limit_{}, yaw_des_in_limit_{};
  int loop_count_{};

  ros::Publisher k_matrix_pub_;


  // Transform
  geometry_msgs::TransformStamped odom2gimbal_des_, odom2pitch_, odom2base_, last_odom2base_,odom2base_yaw_;

  // Gravity Compensation
  geometry_msgs::Vector3 mass_origin_;
  double gravity_;
  bool enable_gravity_compensation_;

  // Chassis
  std::shared_ptr<ChassisVel> chassis_vel_;

  bool dynamic_reconfig_initialized_{};
  GimbalConfig config_{};
  realtime_tools::RealtimeBuffer<GimbalConfig> config_rt_buffer_;
  dynamic_reconfigure::Server<rm_gimbal_controllers::GimbalBaseConfig>* d_srv_{};

  RampFilter<double>*ramp_rate_pitch_{}, *ramp_rate_yaw_{};

  enum
  {
    RATE,
    TRACK,
    DIRECT,
    TRAJ
  };
  int state_ = RATE;
  bool start_ = false;

    enum UpdateStrategy {
    STRATEGY_INITIAL,      // 初始学习：保守
    STRATEGY_NORMAL,       // 正常运行：平衡
    STRATEGY_QUICK,        // 快速适应：激进
    STRATEGY_STABLE        // 已稳定：精细调整
  };

  std::deque<double> yaw_vel_history_;
  std::deque<double> base_yaw_vel_history_;
  std::deque<double> pitch_vel_history_;
  static constexpr int kVelFilterSize = 5;  // 5点移动平均
  
  // ✅ 添加：速度低通滤波器
  double yaw_vel_filtered_{0.0};
  double base_yaw_vel_filtered_{0.0};
  double pitch_vel_filtered_{0.0};
  static constexpr double kVelFilterAlpha = 0.3;  // 截止频率约50Hz
  
  UpdateStrategy update_strategy_{STRATEGY_INITIAL};
  double recent_avg_error_{0.0};
  std::deque<double> error_window_;

  // ✅ 速度滤波器状态变量（防止Gazebo崩溃）
  double last_yaw_vel_{0.0};
  double last_base_yaw_vel_{0.0};
  double last_pitch_vel_{0.0};
  std::deque<double> yaw_vel_buffer_;
  std::deque<double> base_yaw_vel_buffer_;
  std::deque<double> pitch_vel_buffer_;
  
  // ✅ 输出滤波器状态变量
  std::deque<double> error_history_;
  double u_yaw_stage1_{0.0};
  double u_base_yaw_stage1_{0.0};
  bool filter_initialized_{false};
  std::deque<double> u_yaw_history_;
  std::deque<double> u_base_yaw_history_;
  double u_yaw_final_{0.0};
  double u_base_yaw_final_{0.0};
  
  // ✅ updateStrategy 误差采样
  std::deque<double> error_samples_;
  int successful_updates_{0};
  
  std::unique_ptr<NonlinearTrackingDifferentiator<double>> td_yaw_;
  std::unique_ptr<NonlinearTrackingDifferentiator<double>> td_base_yaw_;
};

// 替换第 365 行开始的 namespace constants 部分

namespace constants {

// ==================== Kalman Filter ====================
constexpr double kKalmanProcessNoise = 0.005;
constexpr double kKalmanMeasurementNoise = 0.3;
constexpr double kKalmanInitialEstimate = 0.0;
constexpr double kKalmanInitialError = 1.0;

// ==================== RLS Parameters ====================
constexpr double kRlsMinDenominator = 1e-6;
constexpr double kRlsMaxErrorNorm = 10.0;
constexpr double kRlsMaxDeltaNorm = 0.05;
constexpr int kRlsMinSamples = 200;
constexpr int kRlsBufferLength = 200;

// ==================== Control Limits ====================
constexpr double kMaxControlCommand = 15.0;
constexpr double kMaxControlMultiplier = 2.0;

// ==================== LQR Parameters ====================
constexpr int kLqrMaxIterations = 500;
constexpr double kLqrConvergenceTolerance = 1e-9;
constexpr double kMinDeterminant = 1e-10;
constexpr double kMinKNorm = 0.1;
constexpr double kEigenvalueStabilityThreshold = 1.0;

// ==================== Adaptive Smoothing ====================
constexpr int kErrorHistorySize = 10;
constexpr double kErrorVarianceThreshold = 0.01;
constexpr double kAlphaHigh = 0.95;
constexpr double kAlphaLow = 0.85;

// ==================== Theta Initialization ====================
constexpr double kThetaPositionIntegral = 1.0;
constexpr double kThetaVelocityDecay = 0.98;
constexpr double kThetaInputGain = 0.8;
constexpr double kThetaCouplingStrength = 0.01;
constexpr double kThetaCovarianceInit = 1.0;

// ==================== Physical Constraints ====================
constexpr double kMinVelocityDecay = 0.85;
constexpr double kMaxVelocityDecay = 0.995;
constexpr double kDefaultVelocityDecay = 0.95;
constexpr double kDefaultInputGain = 0.8;
constexpr double kMinInputGain = 0.1;
constexpr double kMaxInputGain = 10.0;
constexpr double kMaxCouplingStrength = 0.15;

// ==================== Timing ====================
constexpr double kWorkerThreadStartupDelay = 2.0;
constexpr double kDestructorJoinTimeout = 5.0;
constexpr int kStatusPublishInterval = 10;
constexpr int kSampleProgressInterval = 50;

// ==================== Validation ====================
constexpr int kValidationTestCount = 5;
constexpr double kValidationStateScale = 0.1;
constexpr int kMinResidualSamples = 5;
constexpr double kMaxResidualValue = 1e6;

// ==================== Adaptive Update Strategy ====================
constexpr int kRlsMinSamplesInitial = 100;
constexpr int kRlsMinSamplesNormal = 50;
constexpr int kRlsMinSamplesQuick = 20;

constexpr int kWorkerHzInitial = 2;
constexpr int kWorkerHzNormal = 5;
constexpr int kWorkerHzQuick = 10;

constexpr double kLargeErrorThreshold = 0.1;
constexpr double kSmallErrorThreshold = 0.01;

}  // namespace constants
}  // namespace rm_gimbal_controllers
