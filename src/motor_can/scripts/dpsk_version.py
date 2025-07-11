#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import can
import time
import yaml
import rospy
import json
from std_msgs.msg import String
from serial_comms.msg import Distances
from serial_comms.msg import INSPVAE
import threading
import sys
import select

rate = 68  # Hz   166.66>> 68.26

class ServoDriveController:
    def __init__(self, channel='can0', interface='socketcan'):
        self.bus = can.interface.Bus(channel=channel, interface=interface)
        # 设置状态列表 - 添加DEBUG状态
        self.status_list = [
            "STOP",      # 停止状态
            "FORWARD",   # 前进状态
            "BACKWARD",  # 后退状态
            "START",     # 位置模式自动运行状态
            "DEBUG"      # 边缘校准和超声波矫正状态
        ]
        # 定义状态及其对应的速度配置
        self.status_config = {
            "START": {  # 位置模式自动运行状态
                "position_left": 651883,  
                "position_right": -651883,
                "move_velocity": 250 * rate,
                "velocity_brush": -100 * rate
            },
            "STOP": {  # 停止状态
                "velocity_up": 0,
                "velocity_low": 0,
                "velocity_brush": 0
            },
            "FORWARD": {  # 前进状态
                "velocity_up": 637 * rate,   # 实际速度0.2m/s
                "velocity_low": -637 * rate,
                "velocity_brush": 0 * rate
            },
            "BACKWARD": {  # 后退状态
                "velocity_up": -637 * rate,
                "velocity_low": 637 * rate,
                "velocity_brush": 1500 * rate
            },
            "DEBUG": {  # 出界检测和矫正状态
                "edge_detection_threshold": 200,  # 出界检测阈值(cm)
                "alignment_threshold": 3,         # 对齐误差阈值(cm)
                "approach_speed": 100 * rate,      # 接近速度
                "rotation_speed": 50 * rate,       # 旋转速度
                "fine_tune_speed": 30 * rate,      # 微调速度
                "timeout": 20.0                   # 最大校准时间(秒)
            }
        }
        
        # 状态变量
        self.last_state = None
        self.current_status = self.status_list[0]  # 当前状态默认停止
        self.current_velocity_up = 0
        self.current_velocity_low = 0
        self.current_velocity_brush = 0
        self.last_left_speed = None
        self.last_right_speed = None
        self.last_brush_speed = None
        self.stop_flag = False   
        
        # DEBUG状态专用变量
        self.debug_phase = "IDLE"  # IDLE, APPROACH_EDGE, ROTATE_TO_EDGE, FINE_TUNE, COMPLETE, FAILED
        self.debug_phase_start_time = rospy.Time.now()
        self.active_side = None    # 标记当前出界的侧边 (LEFT/RIGHT/NONE)
        self.edge_detected = False
        self.alignment_error = 0.0
        self.fine_tune_iterations = 0
        self.calibration_timeout = False
        
        # 超声波传感器距离
        self.distance_a = 0  # 左前方超声波
        self.distance_b = 0  # 右前方超声波

        # ROS 通信
        self.state_pub = rospy.Publisher('/robot_state', String, queue_size=10)
        rospy.Subscriber('/robot_cmd', String, self.status_callback)
        rospy.Subscriber('distance_data', Distances, self.distance_callback)
        
        # 可选：IMU订阅（如果需要）
        if rospy.get_param("~use_imu", False):
            rospy.Subscriber('/inspvae_data', INSPVAE, self.imu_callback)

    def set_state(self, new_state):
        if new_state not in self.status_config:
            rospy.logwarn(f"尝试设置无效状态: {new_state}")
            return False
        if new_state == self.current_status:
            return False  # 状态未改变
            
        # DEBUG状态特殊处理
        if new_state == "DEBUG":
            rospy.loginfo(">>>>>>>>>> 进入DEBUG边缘校准模式 <<<<<<<<<<")
            # 重置DEBUG状态变量
            self.debug_phase = "APPROACH_EDGE"
            self.debug_phase_start_time = rospy.Time.now()
            self.active_side = None
            self.edge_detected = False
            self.alignment_error = 0.0
            self.fine_tune_iterations = 0
            self.calibration_timeout = False
        
        # 其他状态切换处理
        # ... (保留原有的状态切换逻辑)

        self.current_status = new_state
        self.last_state = self.current_status
        rospy.loginfo(f"状态已更新为: {self.current_status}")
        return True
    
    # ... (其他方法保持不变) ...
    
    def publish_state(self):
        """发布机器人状态信息，包含速度和状态"""
        state_msg = {
            "status": self.current_status,
            "velocity_up": self.current_velocity_up / rate,  
            "velocity_low": self.current_velocity_low / rate,
            "velocity_brush": self.current_velocity_brush / rate,
            "imu_yaw": self.imu_yaw,  
            "sensors_status": self.sensors_status,  
            "timestamp": time.time(),
            "debug_phase": self.debug_phase,                 # DEBUG状态
            "active_side": self.active_side,                 # 当前出界的侧边
            "alignment_error": self.alignment_error,         # 对齐误差
            "fine_tune_iterations": self.fine_tune_iterations # 微调迭代次数
        }
        self.state_pub.publish(json.dumps(state_msg))

    def distance_callback(self, msg):
        """超声波距离检测回调 - 更新A、B传感器距离值"""
        self.distance_a = msg.distance_a  # 左前方超声波
        self.distance_b = msg.distance_b  # 右前方超声波
        
        # 更新传感器状态标志
        self.sensors_status = 0
        if self.distance_a >= self.status_config["DEBUG"]["edge_detection_threshold"]:
            self.sensors_status |= 0x01  # 左前方出界
        if self.distance_b >= self.status_config["DEBUG"]["edge_detection_threshold"]:
            self.sensors_status |= 0x02  # 右前方出界
        
        # DEBUG状态下的实时信号处理
        if self.current_status == "DEBUG":
            config = self.status_config["DEBUG"]
            edge_threshold = config["edge_detection_threshold"]
            
            # 检测哪个传感器先出界
            if self.debug_phase == "APPROACH_EDGE" and not self.edge_detected:
                if self.distance_a >= edge_threshold:
                    self.active_side = "LEFT"
                    self.edge_detected = True
                    rospy.loginfo(f"左侧出界检测! 距离A: {self.distance_a:.1f}cm")
                elif self.distance_b >= edge_threshold:
                    self.active_side = "RIGHT"
                    self.edge_detected = True
                    rospy.loginfo(f"右侧出界检测! 距离B: {self.distance_b:.1f}cm")

    def execute_state(self, event=None):
        # ...（位置模式和速度模式配置保持不变）...
        
        # 处理DEBUG状态
        if self.current_status == "DEBUG":
            self.handle_debug_state()
            
        # ...（其他状态处理保持不变）...
    
    def handle_debug_state(self):
        """处理DEBUG状态下的出界检测和对齐操作"""
        config = self.status_config["DEBUG"]
        max_timeout = config["timeout"]
        elapsed = (rospy.Time.now() - self.debug_phase_start_time).to_sec()
        
        # 超时处理
        if elapsed > max_timeout and not self.debug_phase == "COMPLETE":
            rospy.logwarn("DEBUG校准超时，即将切换到STOP状态")
            self.debug_phase = "FAILED"
            self.calibration_timeout = True
        
        # 状态机处理
        if self.debug_phase == "APPROACH_EDGE":
            self.handle_approach_edge(config)
            
        elif self.debug_phase == "ROTATE_TO_EDGE":
            self.handle_rotate_to_edge(config)
            
        elif self.debug_phase == "FINE_TUNE":
            self.handle_fine_tune(config)
            
        elif self.debug_phase == "COMPLETE":
            self.handle_complete()
            
        elif self.debug_phase == "FAILED":
            rospy.logerr("DEBUG校准失败，切换到STOP状态")
            self.set_state("STOP")
    
    def handle_approach_edge(self, config):
        """慢速接近边缘直到一侧出界"""
        # 设置慢速前进
        if self.last_left_speed != config["approach_speed"] or self.last_right_speed != -config["approach_speed"]:
            self.set_target_velocity(2, config["approach_speed"])
            self.set_target_velocity(3, -config["approach_speed"])
            self.last_left_speed = config["approach_speed"]
            self.last_right_speed = -config["approach_speed"]
            rospy.loginfo("正在慢速接近边缘...")
        
        # 检测到一侧出界
        if self.edge_detected:
            rospy.loginfo(f"检测到{self.active_side}侧出界，即将开始旋转")
            self.debug_phase = "ROTATE_TO_EDGE"
            # 停止移动
            self.set_target_velocity(2, 0)
            self.set_target_velocity(3, 0)
            self.last_left_speed = 0
            self.last_right_speed = 0
            rospy.sleep(0.5)  # 短暂停顿
    
    def handle_rotate_to_edge(self, config):
        """旋转机器人使其与边缘平行"""
        rotation_speed = config["rotation_speed"]
        rotation_direction = -1 if self.active_side == "LEFT" else 1
        
        # 根据出界侧确定旋转方向
        left_speed = rotation_speed * rotation_direction
        right_speed = rotation_speed * (-rotation_direction)
        
        # 设置旋转速度
        if self.last_left_speed != left_speed or self.last_right_speed != right_speed:
            self.set_target_velocity(2, left_speed)
            self.set_target_velocity(3, right_speed)
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            rospy.loginfo(f"旋转校正: {self.active_side}侧出界, 旋转方向: {rotation_direction}")
        
        # 旋转检测
        rospy.sleep(0.7)  # 固定时间旋转 (根据实际情况调整)
        
        # 检查旋转后状态
        self.update_alignment_error()
        
        # 停止旋转并进入微调阶段
        self.set_target_velocity(2, 0)
        self.set_target_velocity(3, 0)
        self.last_left_speed = 0
        self.last_right_speed = 0
        rospy.loginfo(f"初步旋转完成, 误差: {self.alignment_error:.1f}cm")
        self.debug_phase = "FINE_TUNE"
        rospy.sleep(0.5)  # 短暂停顿让读数稳定
    
    def handle_fine_tune(self, config):
        """精细调整位置"""
        self.fine_tune_iterations += 1
        self.update_alignment_error()
        
        # 检查是否达到误差阈值
        threshold = config["alignment_threshold"]
        if abs(self.alignment_error) < threshold or self.fine_tune_iterations > 5:
            rospy.loginfo(f"微调完成! 误差: {self.alignment_error:.1f}cm (迭代次数: {self.fine_tune_iterations})")
            self.debug_phase = "COMPLETE"
            return
        
        rospy.loginfo(f"微调迭代 {self.fine_tune_iterations}: 误差: {self.alignment_error:.1f}cm")
        
        # 确定调整方向
        fine_tune_speed = config["fine_tune_speed"]
        rotation_direction = 1 if self.alignment_error > 0 else -1
        duration = min(0.1 + abs(self.alignment_error) / 100.0, 1.0)  # 根据误差调整旋转时间
        
        left_speed = fine_tune_speed * rotation_direction
        right_speed = -fine_tune_speed * rotation_direction
        
        # 执行微调移动
        self.set_target_velocity(2, left_speed)
        self.set_target_velocity(3, right_speed)
        rospy.sleep(duration)
        
        # 停止移动
        self.set_target_velocity(2, 0)
        self.set_target_velocity(3, 0)
        rospy.sleep(0.5)  # 等待读数稳定
    
    def handle_complete(self):
        """校准完成处理"""
        if self.last_left_speed != 0 or self.last_right_speed != 0:
            self.set_target_velocity(2, 0)
            self.set_target_velocity(3, 0)
            self.last_left_speed = 0
            self.last_right_speed = 0
            
        # 发布校准完成消息
        self.publish_alignment_complete()
        
        # 短暂延时后恢复STOP状态
        rospy.sleep(2.0)
        self.set_state("STOP")
    
    def update_alignment_error(self):
        """更新对齐误差值"""
        # 同时检查新出现的另一侧出界情况
        threshold = self.status_config["DEBUG"]["edge_detection_threshold"]
        
        # 如果另一侧也出界了，计算两侧距离差
        if (self.distance_a >= threshold and self.distance_b >= threshold):
            self.alignment_error = abs(self.distance_a - self.distance_b)
            rospy.loginfo(f"两侧均出界, 距离差: A={self.distance_a:.1f}, B={self.distance_b:.1f}, 误差={self.alignment_error:.1f}cm")
        else:
            # 只有一侧出界时，使用高度差作为误差估计
            # 初始出界时这一侧的距离值会骤增
            if self.active_side == "LEFT":
                self.alignment_error = abs(self.distance_a - self.distance_b)
            else:
                self.alignment_error = abs(self.distance_b - self.distance_a)
            rospy.loginfo(f"单侧出界, A={self.distance_a:.1f}, B={self.distance_b:.1f}, 估计误差={self.alignment_error:.1f}cm")
    
    def publish_alignment_complete(self):
        """发布校准完成消息"""
        alignment_msg = {
            "status": "ALIGNMENT_COMPLETE",
            "active_side": self.active_side,
            "alignment_error": self.alignment_error,
            "iterations": self.fine_tune_iterations,
            "timestamp": time.time(),
            "message": f"边缘校准完成 - {self.active_side}侧出界检测, 最终误差: {self.alignment_error:.1f}cm"
        }
        self.state_pub.publish(json.dumps(alignment_msg))
        rospy.loginfo(">>>>>> 边缘校准和超声波矫正完成 <<<<<<")

# ... (其余类方法和main函数保持不变) ...
