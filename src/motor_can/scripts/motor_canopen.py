#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import can
import time
import yaml
import rospy
import json
from std_msgs.msg import String
from serial_comms.msg import Distances
from serial_comms.msg import INSPVAE  # 确保导入正确的消息类型
import threading
import sys
import select
# import os

rate = 68  # Hz   166.66>> 68.26

class ServoDriveController:
    # def __init__(self, channel='vcan0', interface='socketcan'):
    def __init__(self, channel='can0', interface='socketcan'):
        self.bus = can.interface.Bus(channel=channel, interface=interface)
        self.last_left_speed = 0
        self.last_right_speed = 0
        self.last_brush_speed = 0
        #设置状态列表
        self.status_list = [
            "STOP",  # 停止状态
            "FORWARD",  # 前进状态
            "BACKWARD",  # 后退状态
            "START",  # 速度模式初始化并使能
            "LOADING", # 进仓
            "UNLOADING", # 出仓
            "UPSTOP", # 上电机停
            "LOWSTOP", # 下电机停
        ]
        # 定义状态及其对应的速度配置
        self.status_config = {
            "START": {  # 速度模式初始化并使能
                # 原位置模式未启用
                # "position_left": 65188,  # 左侧电机目标位置 由300cm转换而来  651883
                # "position_right": -65188,  # 右侧电机目标位置              651883
                # "velocity_up": 250 * rate,
                # "velocity_low": 250 * rate, #自动速度无法设置负值，二者速度相同
                # "velocity_brush": -100 * rate #后续添加距离到位后反转的判断
            },

            "STOP": {  # 停止状态
                "velocity_up": 0,
                "velocity_low": 0,
                "velocity_brush": 0
            },
            "FORWARD": {  # 前进状态
                "velocity_up": -250 * rate,
                "velocity_low": 250 * rate,
                "velocity_brush": -1500 * rate
            },
            "BACKWARD": {  # 后退状态
                "velocity_up": 250 * rate,
                "velocity_low": -250 * rate,
                "velocity_brush": 1500 * rate
            },
            "LOADING": {
                "velocity_up": -250 *rate,
                "velocity_low": 250 *rate,
                "velocity_brush": 0
            },
            "UNLOADING":{
                "velocity_up": 250 *rate,
                "velocity_low": -250 *rate,
                "velocity_brush": 0
            },
            "UPSTOP":{
                "velocity_up": 0,
                "velocity_low": 0,
                "velocity_brush": 0
            },
            "LOWSTOP":{
                "velocity_up": 0,
                "velocity_low": 0,
                "velocity_brush": 0
            },
            "ROLLER_ACCEL":{
                #目前用与自动模式与手动模式的切换
            },
            "ROLLER_DECEL":{
                #目前用于清空自动模式的状态
            }
            # "FORWARD": {  # 测试电机功耗前进状态
            #     #下发100到电机减速20：1，实际为5RPM ，发250最终12.5RPM，速度0.078m/s
            #     "velocity_up": 637 * rate,   #实际速度0.2m/s
            #     "velocity_low": -637 * rate,
            #     "velocity_brush": 0 * rate
            # },
            # "BACKWARD": {  # 测试滚刷启动总功耗后退状态
            #     "velocity_up": -637 * rate,
            #     "velocity_low": 637 * rate,
            #     "velocity_brush": 1500 * rate   #滚刷速比8
            # }
        }
        self.last_state = None  # 记录上一次的状态
        self.current_status = self.status_list[0]  # 当前默认停止状态
        self.current_velocity_up = 0   # ID = 3
        self.current_velocity_low = 0  # ID = 2
        self.current_velocity_brush = 0  # ID = 4
        #统计超声波传感器触发次数
        self.counter_a =0
        self.counter_b =0
        self.counter_c =0
        self.counter_d =0
        self.threshold = 30 #规定检测超过阈值的次数

        self.stop_flag = False   
        self.position_engaged = False           # 标记位置模式是否已激活
        self.position_mode_configured = False  # 标记位置模式是否已配置
        self.left_position = 0
        self.right_position = 0
        # self.forward_target = self.status_config["START"]["position_left"]
        # self.backward_target = -self.status_config["START"]["position_left"]
        # self.backward_target = self.status_config["START"]["position_right"]
        self.position_direction = 1  # 1: forward, -1: backward
        self.target_sent_flag = False  # 标记目标指令是否已下发

        self.need_speed_mode_init = False
        self.enable_drive_flag = False
        self.stop_velocity = 0  # 停止速度
        self.imu_yaw = 0.0  # IMU偏航角 单位度
        self.initial_yaw = None

        self.sensors_status = 0 #表示4个超声波传感器触发状态
        self.complete_state = False
        self.side_detected = False  #进出仓时超声波检测边缘标志
        self.prev_motion_state = None  # 记录进入单侧停止前的运动状态。
        self.is_upstop = False
        self.is_lowstop = False
        self.auto_mode = False #默认自动模式
        self.auto_step = None # 当前自动程序所在状态
        self.count = 1 # 切换自动与手动 
        #控制不同状态下的发布频率,初始化默认为一秒2次
        self.publish_timer = rospy.Timer(rospy.Duration(0.5), lambda event: self.publish_state())
        
        # PID参数
        self.pid_kp = 100.0
        self.pid_ki = 0.1  # 如果需要加速响应，也可以适当调整积分增益
        self.pid_kd = 0.5  # 如果系统有震荡，可以调整微分增益来抑制震荡
        self.pid_integral = 0.0
        self.pid_last_error = 0.0
        self.target_yaw = 0.0  # 期望偏航角（可根据需要设定）

        self.state_pub = rospy.Publisher('/robot_state', String, queue_size=10)
        rospy.Subscriber('/robot_cmd', String, self.status_callback)
        rospy.Subscriber('/inspvae_data', INSPVAE, self.imu_callback)

    def set_state(self, new_state):
        if new_state not in self.status_config:
            rospy.logwarn(f"尝试设置无效状态: {new_state}")
            return False
        if new_state == self.current_status and new_state != "ROLLER_ACCEL":
            return False  # 状态未改变
        # 检查是否从START切换到其他模式
        # if self.current_status == "START" and new_state in ["FORWARD", "BACKWARD", "STOP"]:
        #     self.need_speed_mode_init = True
        # 自动模式记录当前状态，UPSTOP与LOWSTOP待确认
        if self.auto_mode and new_state in ["FORWARD", "BACKWARD", "LOADING", "UNLOADING"]:
            self.auto_step = new_state
            print("auto_step:",self.auto_step)
        # 初始化为速度模式，添加恢复状态
        if new_state == "START":
            self.complete_state = False
            self.enable_drive_flag = True
            # self.auto_mode = True # 手动设置其他状态应关闭自动
            # 自动模式下恢复未完成动作
            # if self.auto_mode and self.auto_step:
            #     rospy.loginfo(f"恢复自动流程，继续执行: {self.auto_step}")
            #     # 延迟一点时间，确保初始化完成
            #     threading.Timer(3.0, lambda: self.set_state(self.auto_step)).start()
            #     return True

        # STOP时3秒后，降低发布频率为半小时一次（30min*60=1800秒）    
        if new_state == "STOP":
            # self.auto_mode = False
            if self.publish_timer is not None:
                self.publish_timer.shutdown()
                self.publish_timer = rospy.Timer(rospy.Duration(0.5), lambda event: self.publish_state())
            threading.Thread(target=self.delayed_publish_freq_switch,args=(3,),daemon=True).start()
        else: #其他状态保持原频率
            if self.publish_timer is not None:
                self.publish_timer.shutdown()
                self.publish_timer = rospy.Timer(rospy.Duration(0.5), lambda event: self.publish_state())
        # 进入单侧停止时，记录当前运动状态
        if new_state == "UPSTOP":
            self.prev_motion_state = self.last_state
        elif new_state == "LOWSTOP":
            self.prev_motion_state = self.last_state
        elif new_state == "ROLLER_DECEL":#清空自动模式下记忆的状态
            self.auto_step = None
        elif new_state == "ROLLER_ACCEL": #切换手动与自动模式
            if( self.count % 2 ):
                self.auto_mode = False
                print("手动模式开")
            else:    
                self.auto_mode = True
                print("自动模式开")
            self.count += 1
        self.current_status = new_state
        self.last_state = self.current_status
        rospy.loginfo(f"状态已更新为: {self.current_status}")
        return True
    
    
    def enter_absolute_position_mode(self, motor_id, position):
        """设置电机进入绝对位置模式并设置目标位置"""
        # 1. 设置位置模式
        self.set_position_mode(motor_id)
        
        # 2. 设置目标位置
        self.set_position_pulse(motor_id, position)
        
        # 3. 设置为绝对位置立即生效模式并启用
        self.position_mode_enable(motor_id)
        
        rospy.loginfo(f"电机 {motor_id} 已进入绝对位置立即生效模式，目标位置: {position}")

    def set_position_mode(self, motor_id):# 0x03>>0x01 ， 位置模式
        self.send_command(motor_id, [0x2F, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00])
        
    def set_position_pulse(self, motor_id, pulse):
        data = [
            0x23, 0x7A, 0x60, 0x00,
            pulse & 0xFF,
            (pulse >> 8) & 0xFF,
            (pulse >> 16) & 0xFF,
            (pulse >> 24) & 0xFF
        ]
        # print(f"设置电机 {motor_id} 目标速度: {pulse} RPM")
        self.send_command(motor_id, data)
    def set_velocoty_pulse(self, motor_id, pulse):
        data = [
            0x23, 0x81, 0x60, 0x00,
            pulse & 0xFF,
            (pulse >> 8) & 0xFF,
            (pulse >> 16) & 0xFF,
            (pulse >> 24) & 0xFF
        ]
        self.send_command(motor_id, data)
    
    def position_mode_enable(self, motor_id):
        '''设置电机工作在绝对位置模式，立即模式'''
        self.send_command(motor_id, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00])
        # time.sleep(0.1)
        self.send_command(motor_id, [0x2B, 0x40, 0x60, 0x00, 0x3F, 0x00, 0x00, 0x00])
        self.position_mode_configured = True  # 标记位置模式已配置
        rospy.loginfo(f"电机 {motor_id} 绝对位置模式已启用并触发执行")
        # 11 0x00000601 2B 40 60 00 2F 00 00 00
        # 控制字 6040 h -00 h 设置为 002F h 设置驱动器工作在绝对位置立即模式，并
        # 使能。
        # 13 0x00000601 2B 40 60 00 3F 00 00 00
        # 控制字 6040 h -00 h 设置为 003F h 设置驱动器工作在绝对位置立即模式，
        # 6040 h -00 h 的 bit4 上升沿执行位置指令。

    def read_motor_position(self, motor_id):
        '''发送读取位置指令'''
        self.send_command(motor_id, [0x40, 0x63, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        # 这里需要监听CAN总线返回的数据，实际项目中应用can库的recv()或回调
        msg = self.bus.recv(timeout=0.5)
        if msg and msg.arbitration_id == (0x580 + motor_id):
            # 解析返回的4字节位置
            pos = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
            # 处理有符号数
            if pos & 0x80000000:
                pos -= 0x100000000
            #rospy.loginfo(f"电机 {motor_id} 当前位置: {pos}")
            return pos
        return None

    def publish_state(self):
        """发布机器人状态信息，包含速度和状态"""
        state_msg = {
            "status": self.current_status,
            "velocity_up": self.current_velocity_up / rate,  # 单位转换为RPM
            "velocity_low": self.current_velocity_low / rate,
            "velocity_brush": self.current_velocity_brush / rate,
            "imu_yaw": self.imu_yaw,  # IMU偏航角
            "sensors_status": self.sensors_status,  # 超声波传感器状态
            "complete_state":self.complete_state, # 任务完成状态
            "auto_mode": self.auto_mode, # 自动模式开关,默认开
            # "auto_step": self.auto_step, # 当前自动程序所在状态
            "timestamp": time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))  # 2025-07-15 14:58:43

        }
        self.state_pub.publish(json.dumps(state_msg))

    def status_callback(self, msg):
        """处理状态消息"""
        try:
            cmd_obj = json.loads(msg.data)
            command = cmd_obj.get("command", None)
            if command:
                # print("cmd:", command)
                self.set_state(command)  
            else:
                rospy.logwarn(f"未找到command字段: {msg.data}")
        except Exception as e:
            rospy.logwarn(f"消息解析失败，尝试按字符串处理: {msg.data}, 错误: {e}")
            self.set_state(msg.data)

    def imu_callback(self, msg):
        """处理IMU数据"""
        try:
            self.imu_yaw = msg.yaw if hasattr(msg, "yaw") else msg.get("yaw", 0.0)
            # if self.imu_yaw > 180:
            #     self.imu_yaw -= 360
            if self.initial_yaw is None:
                self.initial_yaw = self.imu_yaw
                print(f"Initial IMU yaw set to: {self.initial_yaw} degrees")
            
            # 计算相对角度：将当前yaw值减去初始yaw值
            if self.initial_yaw is not None:
                relative_yaw = self.imu_yaw - self.initial_yaw

                # 处理yaw角度范围，确保在-180到180度之间
                if relative_yaw > 180:
                    relative_yaw -= 360
                elif relative_yaw < -180:
                    relative_yaw += 360

            self.imu_yaw = relative_yaw
            # self.imu_yaw = 0
            
        except json.JSONDecodeError as e:
            rospy.logerr(f"解析IMU数据失败: {e}")

    def send_command(self, motor_id, command_data):
        frame_id = 0x600 + motor_id
        msg = can.Message(arbitration_id=frame_id, data=command_data, is_extended_id=False)
        self.bus.send(msg)
        time.sleep(0.05)
    
    def set_velocity_mode(self, motor_id):
        self.send_command(motor_id, [0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00])
        
    def set_target_velocity(self, motor_id, velocity):
        data = [
            0x23, 0xFF, 0x60, 0x00,
            velocity & 0xFF,
            (velocity >> 8) & 0xFF,
            (velocity >> 16) & 0xFF,
            (velocity >> 24) & 0xFF
        ]
        # print(f"设置电机 {motor_id} 目标速度: {velocity} RPM")
        self.send_command(motor_id, data)
    
    def set_acceleration(self, motor_id, acceleration):
        data = [
            0x23, 0x83, 0x60, 0x00,
            acceleration & 0xFF,
            (acceleration >> 8) & 0xFF,
            (acceleration >> 16) & 0xFF,
            (acceleration >> 24) & 0xFF
        ]
        self.send_command(motor_id, data)
    
    def set_deceleration(self, motor_id, deceleration):
        data = [
            0x23, 0x84, 0x60, 0x00,
            deceleration & 0xFF,
            (deceleration >> 8) & 0xFF,
            (deceleration >> 16) & 0xFF,
            (deceleration >> 24) & 0xFF
        ]
        self.send_command(motor_id, data)
    
    def enable_drive(self, motor_id):
        self.send_command(motor_id, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00])
    
    def disable_drive(self, motor_id):
        self.send_command(motor_id, [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00])
        
    def start_motor(self, motor_id):
        if not 1 <= motor_id <= 127:
            raise ValueError(f"电机ID {motor_id} 超出有效范围 (1-127)")
        motor_id_byte = motor_id & 0xFF
        self.send_command(motor_id, [0x01, motor_id_byte])
    
    def configure_motor(self, motor_id, velocity, acceleration, deceleration):
        rospy.loginfo(f"配置电机 {motor_id}: 速度={int(velocity/rate)}, 加速度={acceleration}, 减速度={deceleration}")
        self.start_motor(motor_id)
        self.set_velocity_mode(motor_id)
        self.set_target_velocity(motor_id, velocity)  #输出转换为脉冲/秒
        self.set_acceleration(motor_id, acceleration)
        self.set_deceleration(motor_id, deceleration)
        self.enable_drive(motor_id)

    def shutdown(self):
        rospy.loginfo("正在关闭电机控制器...")
        self.set_target_velocity(2, 0)
        self.set_target_velocity(3, 0)
        self.set_target_velocity(4, 0)
        self.disable_drive(2)
        self.disable_drive(3)
        self.disable_drive(4)

        self.bus.shutdown()

    @staticmethod
    def load_config(config_file="/home/orangepi/demo01/src/motor_can/config/servo_config.yaml"):
    # def load_config(config_file="/home/ubuntu/demo01/src/motor_can/config/servo_config.yaml"):
        try:
            with open(config_file, 'r') as file:
                config = yaml.safe_load(file)
                return config
        except FileNotFoundError:
            rospy.logerr(f"错误: 配置文件 {config_file} 未找到")
            return {}
        except Exception as e:
            rospy.logerr(f"加载配置文件时出错: {e}")
            return {}
    
    def update_status_by_key(self, key):
        key_mapping = {
            's': "STOP",
            'f': "FORWARD",
            'b': "BACKWARD",
            'a': "START",  # 速度模式初始化并使能
            'l': "LOADING",
            'u': "UNLOADING",
            '1': "UPSTOP",
            '2': "LOWSTOP"
        }
        if key in key_mapping:
            if key != 'a':
                self.auto_mode = False
            else:
                self.auto_mode = True
            self.set_state(key_mapping[key])
            # self.auto_mode = (key == 'a')
        else:
            rospy.loginfo(f"无效按键: {key}")

    def distance_callback(self, msg):
        """超声波距离检测回调"""
        if msg.distance_a < 250:
            self.sensors_status |= 0x01  # 设置传感器A状态
        else:
            self.sensors_status &= ~0x01
        if msg.distance_b < 250:
            self.sensors_status |= 0x02
        else:
            self.sensors_status &= ~0x02
        if msg.distance_c < 250:
            self.sensors_status |= 0x04
        else:
            self.sensors_status &= ~0x04
        if msg.distance_d < 250:
            self.sensors_status |= 0x08
        else:
            self.sensors_status &= ~0x08
        # 前进边缘检测
        if self.auto_mode: # 自动模式未开启，待完善
            if self.current_status == self.status_list[1]:  # FORWARD
                if (msg.distance_a > 250):
                    self.counter_a += 1
                    if self.counter_a >= self.threshold:
                        # self.set_state("STOP")
                        # time.sleep(1)
                        self.set_state("LOADING")
                else:
                    self.counter_a = 0

            if self.current_status == self.status_list[2]:  # BACKWARD
                if (msg.distance_b > 250):
                    self.counter_b += 1
                    if self.counter_b >= self.threshold:
                        # self.set_state("STOP")
                        # time.sleep(1)
                        #自动程序：出仓>后退>到边缘自动切换前进>到边缘切换进仓>发布完成消息>STOP停止使能。
                        self.set_state("FORWARD")
                else:
                    self.counter_b = 0
        else: # 手动模式，仅在前进与后退中切换
            if self.current_status == self.status_list[1]:  # FORWARD
                if (msg.distance_a > 250):
                    self.counter_a += 1
                    print("counter_a:",self.counter_a)
                    # print("time1:",rospy.get_time())
                    if self.counter_a >= self.threshold:
                        self.set_state("BACKWARD")
                    # print(">>>>>>>>time2:",rospy.get_time())

                    # self.set_state("STOP")
                    # time.sleep(1)
                else:
                    self.counter_a = 0
            if self.current_status == self.status_list[2]:  # BACKWARD
                if (msg.distance_b > 250):
                    self.counter_b += 1
                    print("counter_b:",self.counter_b)
                    # print("time3:",rospy.get_time())
                    if self.counter_b >= self.threshold:
                        self.set_state("FORWARD")
                        # print(">>>>>>>>time4:",rospy.get_time())

                else:
                    self.counter_b = 0
        # 进出仓状态并设置执行动作，后续按需修改以设置进出仓检测,进仓判断不使用超声波、出仓判断两侧均 < 250 再切换下个状态。
        if self.current_status in [self.status_list[4],self.status_list[5]] and self.side_detected:  # 边缘LOADING、UNLOADING
        # if self.current_status == "UNLOADING" and self.side_detected:  # 边缘LOADING、UNLOADING
            if msg.distance_a > 250 and msg.distance_c > 250:
                self.set_state("STOP")
                self.side_detected = False
                time.sleep(1)
                #清空自动流程状态
                self.complete_state = True
                self.auto_step = None
            elif (msg.distance_a > 250 and msg.distance_c < 250):
                self.set_state("LOWSTOP")
                #确保停到位
                time.sleep(2)
                self.set_state("STOP")
                self.complete_state = False

            elif (msg.distance_c > 250 and msg.distance_a < 250):
                self.set_state("UPSTOP")
                time.sleep(2)
                self.set_state("STOP")
                self.complete_state = False

            # 发布两侧边缘到位的完成消息
        # if self.current_status == self.status_list[5] and self.side_detected:  # UNLOADING
        #     if (msg.distance_a > 250):
        #         self.set_state("STOP")
        #         time.sleep(1)
        if self.current_status == "UNLOADING":#自动模式
            # 到达板子上
            if (msg.distance_a < 150):
                self.set_state("BACKWARD")

    def pid_correction(self, current_yaw):
        """根据IMU当前偏航角进行PID矫正，返回速度修正量"""
        error = self.target_yaw - current_yaw
        self.pid_integral += error
        derivative = error - self.pid_last_error
        if abs(error) > 0.2:  # 如果误差小于0.3度，则不进行修正
            correction = (self.pid_kp * error +
                        self.pid_ki * self.pid_integral +
                        self.pid_kd * derivative) 
            # * 10  # 放大修正量
        else:
            correction = 0  # 在小范围内不进行调整

        self.pid_last_error = error
        return correction
        # return correction if self.current_status in ["FORWARD","LOADING"] else -correction
    
    def execute_state(self, event=None):
        # 实时根据当前状态和IMU矫正左右轮速度
        # 1. START状态：速度模式初始化电机
        if self.enable_drive_flag and self.current_status == "START":
            config = self.status_config["START"]

            rospy.loginfo("设置速度模式，初始化电机...")
            config = self.load_config()       

            for motor in config["motors"]:
                motor_id = motor.get("id")
                velocity = motor.get("velocity")
                acceleration = motor.get("acceleration")
                deceleration = motor.get("deceleration")
                if None in (motor_id, velocity, acceleration, deceleration):
                    rospy.logwarn(f"跳过无效配置: {motor}")
                    continue
                try:
                    self.configure_motor(
                        motor_id=motor_id,
                        velocity=int(velocity*rate),
                        acceleration=int(acceleration*rate),
                        deceleration=int(deceleration*rate)
                    )
                except Exception as e:
                    rospy.logerr(f"配置电机 {motor_id} 时出错: {e}")
            self.enable_drive_flag = False
            rospy.loginfo("速度模式初始化完成")
            # 初始化完成后自动切换到auto_step
            if self.auto_mode and self.auto_step:
                rospy.loginfo(f"初始化完成，恢复自动流程: {self.auto_step}")
                self.set_state(self.auto_step)
            return

        # 2. FORWARD/BACKWARD状态：IMU矫正+ 单侧停止
        if self.current_status in ["FORWARD", "BACKWARD"]:
            correction = self.pid_correction(self.imu_yaw) * rate
            left_speed = int(self.status_config[self.current_status]["velocity_up"] + correction)
            right_speed = int(self.status_config[self.current_status]["velocity_low"] + correction)
            brush_speed = self.status_config[self.current_status]["velocity_brush"]
            # rospy.loginfo(f"IMU矫正: yaw={self.imu_yaw:.2f}, correction={correction:.2f}")
            if (self.last_left_speed != left_speed or
                self.last_right_speed != right_speed or
                self.last_brush_speed != brush_speed):
                rospy.loginfo(f"IMU矫正: yaw={self.imu_yaw:.2f}, correction={correction:.2f}")
                rospy.loginfo(f"左轮速度: {left_speed}, 右轮速度: {right_speed}")
                
                self.set_target_velocity(3, left_speed)
                self.set_target_velocity(2, right_speed)
                self.set_target_velocity(4, brush_speed)
                self.last_left_speed = left_speed
                self.last_right_speed = right_speed
                self.last_brush_speed = brush_speed

            if -5 < self.imu_yaw < -2:
                # time.sleep(1.0)
                print("last_L_speed:",self.last_left_speed)
                print("last_R_speed:",self.last_right_speed)
                if self.current_status == "FORWARD":
                    self.set_state("UPSTOP")
                    self.is_upstop = True
                else:
                    self.set_state("LOWSTOP")
                    self.is_lowstop = True
            if 2 < self.imu_yaw < 5:
                # time.sleep(1.0)
                if self.current_status == "FORWARD":
                    self.set_state("LOWSTOP")
                    self.is_lowstop = True
                else:
                    self.set_state("UPSTOP")
                    self.is_upstop = True                
            
            # 实时发布状态
            self.current_velocity_up = left_speed
            self.current_velocity_low = right_speed
            self.current_velocity_brush = brush_speed

        # 3. 单侧停止状态（UPSTOP/LOWSTOP）
        elif self.current_status == "UPSTOP":
            left_speed = 0
            right_speed = self.last_right_speed  # 右轮保持切换前速度
            brush_speed = self.last_brush_speed
            if (self.last_left_speed != left_speed or
                self.last_right_speed != right_speed or
                self.last_brush_speed != brush_speed):
                self.set_target_velocity(3, left_speed)
                self.set_target_velocity(2, right_speed)
                self.set_target_velocity(4, brush_speed)
                self.last_left_speed = left_speed
                self.last_right_speed = right_speed
                self.last_brush_speed = brush_speed
            # 恢复上个状态
            if self.is_upstop and -1 < self.imu_yaw < 0:
                if self.prev_motion_state:
                    self.set_state(self.prev_motion_state)
                    self.prev_motion_state = None
                self.is_upstop = False

            self.current_velocity_up = left_speed
            self.current_velocity_low = right_speed
            self.current_velocity_brush = brush_speed
        
        elif self.current_status == "LOWSTOP":
            left_speed = self.last_left_speed  # 左轮保持切换前速度
            right_speed = 0
            brush_speed = self.last_brush_speed
            if (self.last_left_speed != left_speed or
                self.last_right_speed != right_speed or
                self.last_brush_speed != brush_speed):
                self.set_target_velocity(3, left_speed)
                self.set_target_velocity(2, right_speed)
                self.set_target_velocity(4, brush_speed)
                self.last_left_speed = left_speed
                self.last_right_speed = right_speed
                self.last_brush_speed = brush_speed


            if self.is_lowstop and 0 < self.imu_yaw < 1:
                if self.prev_motion_state:
                    self.set_state(self.prev_motion_state)
                    self.prev_motion_state = None
                self.is_lowstop = False
            self.current_velocity_up = left_speed
            self.current_velocity_low = right_speed
            self.current_velocity_brush = brush_speed

        # 4. STOP状态或IMU角度异常
        elif self.current_status == "STOP" or not -5 < self.imu_yaw < 5:
            if (self.last_left_speed != 0 or
                self.last_right_speed != 0 or
                self.last_brush_speed != 0):

                self.set_target_velocity(2, 0)
                self.set_target_velocity(3, 0)
                self.set_target_velocity(4, 0)
                # 停止使能电机，下次需使能
                self.disable_drive(2)
                self.disable_drive(3)
                self.disable_drive(4)
                self.last_left_speed = 0
                self.last_right_speed = 0
                self.last_brush_speed = 0
                self.need_speed_mode_init = True


            self.current_velocity_low = 0
            self.current_velocity_up = 0
            self.current_velocity_brush = 0
            # self.publish_state()
        elif self.current_status == "START":
            config = self.status_config["START"]
            # 读取当前位置
        #     left_pos = self.read_motor_position(2)
        #     right_pos = self.read_motor_position(3)
        #     if left_pos is not None:
        #         self.left_position = left_pos
        #     if right_pos is not None:
        #         self.right_position = right_pos

        #     # 状态机：目标在4096/-4096 <-> 0之间切换
        #     # 用self.position_target_flag标记当前目标（True: 4096/-4096, False: 0）
        #     if not hasattr(self, "position_target_flag"):
        #         self.position_target_flag = True  # 初始目标为4096/-4096

        #     if self.position_target_flag:
        #         target_left = config["position_left"]
        #         target_right = config["position_right"]
        #     else:
        #         target_left = 0
        #         target_right = 0
        # # 只在切换目标时下发一次目标指令
        #     if not self.target_sent_flag:
        #         self.set_velocoty_pulse(2, config["velocity_low"])
        #         self.set_velocoty_pulse(3, config["velocity_up"])
        #         self.enter_absolute_position_mode(2, target_left)
        #         self.enter_absolute_position_mode(3, target_right)
        #         self.target_sent_flag = True
        #         rospy.loginfo(f"下发目标: 左{target_left}, 右{target_right}")

        #     # 判断是否到达目标（允许一定误差）
        #     if (abs(self.left_position - target_left) < 1000 and
        #         abs(self.right_position - target_right) < 1000):
        #         # 切换目标
        #         self.position_target_flag = not self.position_target_flag
        #         self.target_sent_flag = False  # 允许下发新目标

        #     # 刷子电机速度控制（同前）
        #     brush_speed = config["velocity_brush"]
        #     if self.last_brush_speed != brush_speed:
        #         self.set_target_velocity(4, brush_speed)
        #         self.last_brush_speed = brush_speed

        # 5. LOADING/UNLOADING状态：IMU矫正+边缘检测
        elif self.current_status in ["LOADING", "UNLOADING"]:
            correction = self.pid_correction(self.imu_yaw) * rate
            left_speed = int(self.status_config[self.current_status]["velocity_up"] + correction)
            right_speed = int(self.status_config[self.current_status]["velocity_low"] + correction)
            brush_speed = self.status_config[self.current_status]["velocity_brush"]
            # rospy.loginfo(f"IMU矫正: yaw={self.imu_yaw:.2f}, correction={correction:.2f}")
            # 通过上下双传感器检测是否到位,哪边到位哪边停，直到两边均到位
            # 设置UNLOADING到BACKWARD以实现自动运行程序第一步。
            # if self.current_status == "UNLOADING":#自动模式
                #到位检测判断
                # self.set_state("BACKWARD")

            self.side_detected = True
            # 左右轮速度矫正（左轮-修正，右轮+修正）
            if (self.last_left_speed != left_speed or
                self.last_right_speed != right_speed or
                self.last_brush_speed != brush_speed):
                rospy.loginfo(f"IMU矫正: yaw={self.imu_yaw:.2f}, correction={correction:.2f}")
                rospy.loginfo(f"左轮速度: {left_speed}, 右轮速度: {right_speed}")
                
                self.set_target_velocity(3, left_speed)
                self.set_target_velocity(2, right_speed)
                self.set_target_velocity(4, brush_speed)
                self.last_left_speed = left_speed
                self.last_right_speed = right_speed
                self.last_brush_speed = brush_speed

            # 实时发布状态
            self.current_velocity_up = left_speed
            self.current_velocity_low = right_speed
            self.current_velocity_brush = brush_speed
            # # 设置移动速度（只需每次切换目标时设置一次即可）
            # self.set_velocoty_pulse(2, config["velocity_low"])
            # self.set_velocoty_pulse(3, config["velocity_up"])

            # # 判断是否到达目标（允许一定误差）
            # if (abs(self.left_position - target_left) < 1000 and
            #     abs(self.right_position - target_right) < 1000):
            #     # 切换目标
            #     self.position_target_flag = not self.position_target_flag
            #     # 下发新的目标位置
            #     if self.position_target_flag:
            #         new_left = config["position_left"]
            #         new_right = config["position_right"]
            #     else:
            #         new_left = 0
            #         new_right = 0
            #     self.enter_absolute_position_mode(2, new_left)
            #     self.enter_absolute_position_mode(3, new_right)
            #     rospy.loginfo(f"到达目标，切换方向，新的目标: 左{new_left}, 右{new_right}")

            # # 如果刚进入START状态或刚切换目标，需要立即下发目标
            # if not hasattr(self, "last_target_left") or self.last_target_left != target_left or self.last_target_right != target_right:
            #     self.enter_absolute_position_mode(2, target_left)
            #     self.enter_absolute_position_mode(3, target_right)
            #     self.last_target_left = target_left
            #     self.last_target_right = target_right

            # # 设置刷子电机速度（保持速度模式）
            # brush_speed = config["velocity_brush"]
            # if self.last_brush_speed != brush_speed:
            #     self.set_target_velocity(4, brush_speed)
            #     self.last_brush_speed = brush_speed
            #     rospy.loginfo(f"刷子速度设置: {brush_speed/rate}")

            # self.position_engaged = True
            # self.current_velocity_brush = brush_speed
            # self.current_velocity_low = 0
            # self.current_velocity_up = 0

    
    def delayed_publish_freq_switch(self, delay_sec=3):
        # 延时后切换到低频率
        time.sleep(delay_sec)
        if self.current_status == "STOP":
            if self.publish_timer is not None:
                self.publish_timer.shutdown()
            self.publish_timer = rospy.Timer(rospy.Duration(1800), lambda event: self.publish_state())

        
    @staticmethod
    def keyboard_listener(controller):
        rospy.loginfo("按键控制：s=停止, f=前进, b=后退")
        while not rospy.is_shutdown():
            # 非阻塞读取键盘
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.readline().strip()
                controller.update_status_by_key(key)    

def main():
    rospy.init_node("motor_canopen_node")
    controller = ServoDriveController()
    config = controller.load_config()
    if not config or "motors" not in config or not config["motors"]:
        rospy.logerr("未找到有效配置，请检查配置文件")
        return
    rospy.loginfo("开始自动配置驱动器...")
    for motor in config["motors"]:
        motor_id = motor.get("id")
        velocity = motor.get("velocity")
        acceleration = motor.get("acceleration")
        deceleration = motor.get("deceleration")
        if None in (motor_id, velocity, acceleration, deceleration):
            rospy.logwarn(f"跳过无效配置: {motor}")
            continue
        try:
            rospy.loginfo(f"配置电机 {motor_id}...")
            controller.configure_motor(
                motor_id=motor_id,
                velocity=int(velocity*rate),
                acceleration=int(acceleration*rate),
                deceleration=int(deceleration*rate)
            )
            controller.current_status = controller.status_list[0]  # 初始化为停止状态
        except Exception as e:
            rospy.logerr(f"配置电机 {motor_id} 时出错: {e}")
    rospy.loginfo("电机初始化完成（Ctrl+C 退出）")
    # 订阅速度命令话题
    rospy.Subscriber("distance_data", Distances, lambda msg: controller.distance_callback(msg))
    #通过检测按键修改运行状态
    # 启动键盘监听线程
    t = threading.Thread(target=ServoDriveController.keyboard_listener, args=(controller,), daemon=True)
    t.start()
    try:
    # 每0.2秒执行一次状态执行器
        rospy.Timer(rospy.Duration(0.2), controller.execute_state)
        # 每0.5秒发布一次状态
        # rospy.Timer(rospy.Duration(0.5), lambda event: controller.publish_state())
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("程序终止")
    finally:
        controller.shutdown()

if __name__ == "__main__":
    main()
