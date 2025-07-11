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
        #设置状态列表
        self.status_list = [
            "STOP",  # 停止状态
            "FORWARD",  # 前进状态
            "BACKWARD",  # 后退状态
            "START",  # 位置模式自动运行状态
            "ROLLER_ACCEL",# 滚刷加速状态
            "ROLLER_DECEL"  # 滚刷减速状态
        ]
        # 定义状态及其对应的速度配置
        self.status_config = {
            "START": {  # 位置模式自动运行状态
                "position_left": 65188,  # 左侧电机目标位置 由300cm转换而来  651883
                "position_right": -65188,  # 右侧电机目标位置              651883
                "velocity_up": 250 * rate,
                "velocity_low": 250 * rate, #自动速度无法设置负值，二者速度相同
                "velocity_brush": -100 * rate #后续添加距离到位后反转的判断
            },

            "STOP": {  # 停止状态
                "velocity_up": 0,
                "velocity_low": 0,
                "velocity_brush": 0
            },
            "FORWARD": {  # 前进状态
                "velocity_up": 250 * rate,
                "velocity_low": -250 * rate,
                "velocity_brush": -1500 * rate
            },
            "BACKWARD": {  # 后退状态
                "velocity_up": -250 * rate,
                "velocity_low": 250 * rate,
                "velocity_brush": 1500 * rate
            },
            "ROLLER_ACCEL": {  # 滚刷加速状态
                "velocity_up": self.last_left_speed,
                "velocity_low": self.last_right_speed,
                "velocity_brush": (self.last_brush_speed + 1000 * rate ) if self.last_brush_speed is not None else 0  # 滚刷加速到1000 RPM
            },
            "ROLLER_DECEL": {  # 滚刷减速状态
                "velocity_up": self.last_left_speed,
                "velocity_low": self.last_right_speed,
                "velocity_brush": (self.last_brush_speed - 1000 * rate ) if self.last_brush_speed is not None else 0  # 滚刷减速到-1000 RPM
            }
        }
        self.last_state = None  # 记录上一次的状态
        self.current_status = self.status_list[0]  # 当前默认停止状态
        self.current_velocity_up = 0   # ID = 3
        self.current_velocity_low = 0  # ID = 2
        self.current_velocity_brush = 0  # ID = 4
        self.last_left_speed = None
        self.last_right_speed = None
        self.last_brush_speed = None
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
        self.need_position_mode_init = False
        self.stop_velocity = 0  # 停止速度
        self.imu_yaw = 0.0  # IMU偏航角 单位度
        self.initial_yaw = None

        self.sensors_status = 0 #表示4个超声波传感器触发状态

        # PID参数
        self.pid_kp = 20.0
        self.pid_ki = 0.0
        self.pid_kd = 0.2
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
        if new_state == self.current_status:
            return False  # 状态未改变
        # 检查是否从START切换到其他模式
        if self.current_status == "START" and new_state in ["FORWARD", "BACKWARD", "STOP", "ROLLER_ACCEL", "ROLLER_DECEL"]:
            self.need_speed_mode_init = True

         # 状态改变时重置位置模式标志
        if new_state == "START":
            self.position_engaged = False
            self.position_mode_configured = False
            self.need_position_mode_init = True

        self.current_status = new_state
        self.last_state = self.current_status
        rospy.loginfo(f"状态已更新为: {self.current_status}")
        return True
    
    def enter_absolute_position_mode(self, motor_id, position):
        """设置电机进入绝对位置模式并设置目标位置"""
        # 1. 设置位置模式
        self.set_position_mode(motor_id)
        
        # 2. 设置目标位置
        self.set_position_pluse(motor_id, position)
        
        # 3. 设置为绝对位置立即生效模式并启用
        self.position_mode_enable(motor_id)
        
        rospy.loginfo(f"电机 {motor_id} 已进入绝对位置立即生效模式，目标位置: {position}")

    def set_position_mode(self, motor_id):# 0x03>>0x01 ， 位置模式
        self.send_command(motor_id, [0x2F, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00])
        
    def set_position_pluse(self, motor_id, pluse):
        data = [
            0x23, 0x7A, 0x60, 0x00,
            pluse & 0xFF,
            (pluse >> 8) & 0xFF,
            (pluse >> 16) & 0xFF,
            (pluse >> 24) & 0xFF
        ]
        # print(f"设置电机 {motor_id} 目标速度: {pluse} RPM")
        self.send_command(motor_id, data)
    def set_velocoty_pluse(self, motor_id, pluse):
        data = [
            0x23, 0x81, 0x60, 0x00,
            pluse & 0xFF,
            (pluse >> 8) & 0xFF,
            (pluse >> 16) & 0xFF,
            (pluse >> 24) & 0xFF
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
            rospy.loginfo(f"电机 {motor_id} 当前位置: {pos}")
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
            "timestamp": time.time()
        }
        self.state_pub.publish(json.dumps(state_msg))

    def status_callback(self, msg):
        """处理状态消息"""
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
            'a': "START"  # 位置模式自动运行状态
        }
        if key in key_mapping:
            self.set_state(key_mapping[key])
        else:
            rospy.loginfo(f"无效按键: {key}")

    def distance_callback(self, msg):
        """超声波距离检测回调"""
        if msg.distance_a < 200:
            self.sensors_status |= 0x01  # 设置传感器A状态
        else:
            self.sensors_status &= ~0x01
        if msg.distance_b < 200:
            self.sensors_status |= 0x02
        else:
            self.sensors_status &= ~0x02
        if msg.distance_c < 200:
            self.sensors_status |= 0x04
        else:
            self.sensors_status &= ~0x04
        if msg.distance_d < 200:
            self.sensors_status |= 0x08
        else:
            self.sensors_status &= ~0x08
        # 前进边缘检测
        # if not self.stop_flag and self.current_status == self.status_list[1]:  # FORWARD
        if self.current_status == self.status_list[1]:  # FORWARD
            if (msg.distance_a > 200):
                self.set_state("STOP")

        if self.current_status == self.status_list[2]:  # BACKWARD
            if (msg.distance_b > 200):
                self.set_state("STOP")


    def pid_correction(self, current_yaw):
        """根据IMU当前偏航角进行PID矫正，返回速度修正量"""
        error = self.target_yaw - current_yaw
        self.pid_integral += error
        derivative = error - self.pid_last_error
        correction = (self.pid_kp * error +
                      self.pid_ki * self.pid_integral +
                      self.pid_kd * derivative)
        self.pid_last_error = error
        return correction
    
    def execute_state(self, event=None):
        # 实时根据当前状态和IMU矫正左右轮速度
        if self.need_position_mode_init and self.current_status == "START":
            config = self.status_config["START"]
            # 设置两轮为位置模式
            self.enter_absolute_position_mode(2, config["position_left"])
            self.enter_absolute_position_mode(3, config["position_right"])
            self.set_position_mode(2)
            self.set_position_mode(3)
            self.need_position_mode_init = False
            rospy.loginfo("已重新初始化两轮为位置模式")
        # if self.need_speed_mode_init:
        if self.need_speed_mode_init and self.current_status in ["FORWARD", "BACKWARD", "STOP", "ROLLER_ACCEL", "ROLLER_DECEL"]:

            rospy.loginfo("从START切换到速度模式，重新初始化电机...")
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
            self.need_speed_mode_init = False
            rospy.loginfo("速度模式初始化完成")

        if self.current_status in ["FORWARD", "BACKWARD", "ROLLER_ACCEL", "ROLLER_ACCEL"] and -15 < self.imu_yaw < 15:
            correction = self.pid_correction(self.imu_yaw)
            left_speed = int(self.status_config[self.current_status]["velocity_up"] - correction)
            right_speed = int(self.status_config[self.current_status]["velocity_low"] + correction)
            brush_speed = self.status_config[self.current_status]["velocity_brush"]
            rospy.loginfo(f"IMU矫正: yaw={self.imu_yaw:.2f}, correction={correction:.2f}")
            # 左右轮速度矫正（左轮-修正，右轮+修正）
            if (self.last_left_speed != left_speed or
                self.last_right_speed != right_speed or
                self.last_brush_speed != brush_speed):
                rospy.loginfo(f"IMU矫正: yaw={self.imu_yaw:.2f}, correction={correction:.2f}")
                rospy.loginfo(f"左轮速度: {left_speed}, 右轮速度: {right_speed}")
                
                self.set_target_velocity(2, left_speed)
                self.set_target_velocity(3, right_speed)
                self.set_target_velocity(4, brush_speed)
                self.last_left_speed = left_speed
                self.last_right_speed = right_speed
                self.last_brush_speed = brush_speed
            # 实时发布状态
            self.current_velocity_low = left_speed
            self.current_velocity_up = right_speed
            self.current_velocity_brush = brush_speed
            # self.publish_state()
        elif self.current_status == "STOP" or not -15 < self.imu_yaw <15:
            if (self.last_left_speed != 0 or
                self.last_right_speed != 0 or
                self.last_brush_speed != 0):

                self.set_target_velocity(2, 0)
                self.set_target_velocity(3, 0)
                self.set_target_velocity(4, 0)
                self.last_left_speed = 0
                self.last_right_speed = 0
                self.last_brush_speed = 0

            self.current_velocity_low = 0
            self.current_velocity_up = 0
            self.current_velocity_brush = 0
            # self.publish_state()
        elif self.current_status == "START":
            config = self.status_config["START"]
            # 读取当前位置
            left_pos = self.read_motor_position(2)
            right_pos = self.read_motor_position(3)
            if left_pos is not None:
                self.left_position = left_pos
            if right_pos is not None:
                self.right_position = right_pos

            # 状态机：目标在4096/-4096 <-> 0之间切换
            # 用self.position_target_flag标记当前目标（True: 4096/-4096, False: 0）
            if not hasattr(self, "position_target_flag"):
                self.position_target_flag = True  # 初始目标为4096/-4096

            if self.position_target_flag:
                target_left = config["position_left"]
                target_right = config["position_right"]
            else:
                target_left = 0
                target_right = 0
        # 只在切换目标时下发一次目标指令
            if not self.target_sent_flag:
                self.set_velocoty_pluse(2, config["velocity_low"])
                self.set_velocoty_pluse(3, config["velocity_up"])
                self.enter_absolute_position_mode(2, target_left)
                self.enter_absolute_position_mode(3, target_right)
                self.target_sent_flag = True
                rospy.loginfo(f"下发目标: 左{target_left}, 右{target_right}")

            # 判断是否到达目标（允许一定误差）
            if (abs(self.left_position - target_left) < 1000 and
                abs(self.right_position - target_right) < 1000):
                # 切换目标
                self.position_target_flag = not self.position_target_flag
                self.target_sent_flag = False  # 允许下发新目标

            # 刷子电机速度控制（同前）
            brush_speed = config["velocity_brush"]
            if self.last_brush_speed != brush_speed:
                self.set_target_velocity(4, brush_speed)
                self.last_brush_speed = brush_speed

            # # 设置移动速度（只需每次切换目标时设置一次即可）
            # self.set_velocoty_pluse(2, config["velocity_low"])
            # self.set_velocoty_pluse(3, config["velocity_up"])

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
    #t = threading.Thread(target=ServoDriveController.keyboard_listener, args=(controller,), daemon=True)
    #t.start()
    try:
    # 每0.5秒执行一次状态执行器
        rospy.Timer(rospy.Duration(0.2), controller.execute_state)
        # 每2秒发布一次状态
        rospy.Timer(rospy.Duration(0.5), lambda event: controller.publish_state())
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("程序终止")
    finally:
        controller.shutdown()

if __name__ == "__main__":
    main()
