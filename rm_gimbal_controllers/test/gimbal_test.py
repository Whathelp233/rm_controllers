#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LQR云台控制器性能测试脚本
用于验证正弦跟踪性能和调参效果
"""

import rospy
from rm_msgs.msg import GimbalCmd
from control_msgs.msg import JointControllerState
import math
import sys

class GimbalPerformanceTest:
    def __init__(self):
        rospy.init_node('gimbal_sine_test', anonymous=True)
        
        # 发布命令
        self.cmd_pub = rospy.Publisher(
            '/controllers/gimbal_controller/command',
            GimbalCmd,
            queue_size=10
        )
        
        # 订阅状态反馈
        self.state_sub = rospy.Subscriber(
            '/controllers/gimbal_controller/yaw/state',
            JointControllerState,
            self.state_callback
        )
        
        # 测试参数
        self.frequency = 1.0  # 正弦频率(Hz)
        self.amplitude = 1.0  # 速度振幅(rad/s)
        self.rate = rospy.Rate(50)  # 50Hz发布
        
        # 性能统计
        self.error_list = []
        self.max_error = 0.0
        self.current_velocity = 0.0
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("LQR云台正弦跟踪测试")
        rospy.loginfo("=" * 60)
        rospy.loginfo("频率: %.1f Hz", self.frequency)
        rospy.loginfo("振幅: %.2f rad/s", self.amplitude)
        rospy.loginfo("=" * 60)
        
    def state_callback(self, msg):
        """接收状态反馈"""
        self.current_velocity = msg.process_value_dot
        
        # 计算误差
        error = abs(msg.set_point_dot - msg.process_value_dot)
        self.error_list.append(error)
        if error > self.max_error:
            self.max_error = error
            
    def run_sine_test(self, duration=30):
        """
        运行正弦跟踪测试
        Args:
            duration: 测试时长(秒)
        """
        start_time = rospy.Time.now()
        t = 0.0
        last_print_time = rospy.Time.now()
        
        rospy.loginfo("开始测试,持续 %d 秒...", duration)
        
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > duration:
                break
                
            # 生成正弦命令
            cmd = GimbalCmd()
            omega = 2 * math.pi * self.frequency
            cmd.rate_yaw = self.amplitude * math.sin(omega * t)
            self.cmd_pub.publish(cmd)
            
            # 每秒打印一次状态
            if (rospy.Time.now() - last_print_time).to_sec() > 1.0:
                if len(self.error_list) > 0:
                    avg_error = sum(self.error_list[-50:]) / len(self.error_list[-50:])
                    rospy.loginfo(
                        "[%.1fs] 命令: %.3f rad/s | 实际: %.3f rad/s | 平均误差: %.4f | 最大误差: %.4f",
                        elapsed, cmd.rate_yaw, self.current_velocity, avg_error, self.max_error
                    )
                last_print_time = rospy.Time.now()
            
            t += 0.02  # 50Hz对应的时间步长
            self.rate.sleep()
            
        self.print_summary()
        
    def run_step_test(self, target=1.0, duration=5):
        """
        运行阶跃响应测试
        Args:
            target: 目标位置(rad)
            duration: 测试时长(秒)
        """
        rospy.loginfo("阶跃响应测试: 目标 %.2f rad", target)
        
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > duration:
                break
                
            cmd = GimbalCmd()
            cmd.position_yaw = target
            self.cmd_pub.publish(cmd)
            self.rate.sleep()
            
        self.print_summary()
        
    def print_summary(self):
        """打印测试总结"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("测试完成!")
        
        if len(self.error_list) > 100:
            # 去掉前2秒的启动数据
            stable_errors = self.error_list[100:]
            avg_error = sum(stable_errors) / len(stable_errors)
            
            rospy.loginfo("稳态平均误差: %.4f rad/s", avg_error)
            rospy.loginfo("最大误差: %.4f rad/s", self.max_error)
            rospy.loginfo("误差百分比: %.2f%%", (avg_error / self.amplitude) * 100)
            
            # 性能评估
            if avg_error < 0.05 * self.amplitude:
                rospy.loginfo("✅ 性能优秀 (误差 < 5%%)")
            elif avg_error < 0.10 * self.amplitude:
                rospy.loginfo("✅ 性能良好 (误差 < 10%%)")
            elif avg_error < 0.20 * self.amplitude:
                rospy.logwarn("⚠️ 性能一般 (误差 < 20%%), 建议调参")
            else:
                rospy.logerr("❌ 性能不佳 (误差 > 20%%), 需要调参!")
        else:
            rospy.logwarn("数据不足,无法统计")
            
        rospy.loginfo("=" * 60)


def print_usage():
    print("""
使用方法:
    rosrun rm_gimbal_controllers gimbal_test.py <mode> [options]

模式:
    sine [freq] [amp] [duration]  - 正弦跟踪测试(默认: 1Hz, 1rad/s, 30s)
    step [target] [duration]      - 阶跃响应测试(默认: 1rad, 5s)

示例:
    # 默认正弦测试(1Hz, 1rad/s, 30秒)
    rosrun rm_gimbal_controllers gimbal_test.py sine
    
    # 自定义正弦测试(2Hz, 0.5rad/s, 60秒)
    rosrun rm_gimbal_controllers gimbal_test.py sine 2 0.5 60
    
    # 阶跃测试
    rosrun rm_gimbal_controllers gimbal_test.py step 1.0 5
""")


if __name__ == '__main__':
    try:
        if len(sys.argv) < 2:
            print_usage()
            sys.exit(0)
            
        mode = sys.argv[1]
        tester = GimbalPerformanceTest()
        
        if mode == 'sine':
            freq = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
            amp = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
            duration = int(sys.argv[4]) if len(sys.argv) > 4 else 30
            
            tester.frequency = freq
            tester.amplitude = amp
            tester.run_sine_test(duration)
            
        elif mode == 'step':
            target = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
            duration = int(sys.argv[3]) if len(sys.argv) > 3 else 5
            tester.run_step_test(target, duration)
            
        else:
            rospy.logerr("未知模式: %s", mode)
            print_usage()
            
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("错误: %s", str(e))
        print_usage()
