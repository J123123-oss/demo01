#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial
import time
import rospy
import json
import yaml
from std_msgs.msg import String, Int8, Float32, Float32MultiArray, Bool
from serial_comms.msg import Distances
from serial_comms.msg import Sensors
from serial_comms.msg import INSPVAE
from serial_comms.msg import BatteryStatus, Environment  # 确保导入正确的消息类型

from std_srvs.srv import Trigger
import threading
import sys
import select

rate = 68  # Hz

class ServoDriveController:
    def __init__(self):  # 串口参数与send.py一致
        # 初始化串口（替换原CAN总线）
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB1')  # ~表示私有参数
        self.baudrate = rospy.get_param('~baudrate', 2000000)

        self.ser = self.create_serial(self.serial_port, self.baudrate)
        self.last_left_speed = 0
        self.last_right_speed = 0
        self.last_brush_speed = 0
        self.has_reverse_flag = False
        self.has_reverse_counter = 0
        self.reverse_start_time = None
        self.main_board = True
        self.imu_sensor = True
        self.motor_driver = True
        self.motor_base = 350
        self.base_speed = 17000
        self.flag = 0

        self.speed_pluse_max = 25840
        self.reversed_start_time = None
        self.REVERSE_TIME_THRESHOLD = 3.0
        self.unloading_timer = 20.0
        self.unloading_start_time = None
        self.start_time = 0
        self.elevator_stage = 0
        self.elevator_start_time = 0
        self.LOW_BATTERY_THRESHOLD = 40

        self.velocity_publish_count = 0
        self.velocity_publish_interval = 5
        self.last_velocity_up = 0
        self.last_velocity_low = 0
        self.last_velocity_brush = 0

        # 状态列表和配置（保持不变）
        self.status_list = [
            "STOP", "FORWARD", "BACKWARD", "START", "LOADING", "UNLOADING",
            "UPSTOP", "LOWSTOP", "PISTON_OUT", "PISTON_IN", "CHARGE_OUT", "RETURN_DOCK"
        ]
        self.status_config = {
            "START": {},
            "STOP": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": 0},
            "FORWARD": {
                "velocity_up": self.motor_base * rate,
                "velocity_low": -self.motor_base * rate,
                "velocity_brush": -1600 * rate
            },
            "BACKWARD": {
                "velocity_up": -self.motor_base * rate,
                "velocity_low": self.motor_base * rate,
                "velocity_brush": -1600 * rate
            },
            "LOADING": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": -1600 * rate},
            "UNLOADING": {"velocity_up": -self.motor_base * rate, "velocity_low": self.motor_base * rate, "velocity_brush": 0},
            "UPSTOP": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": 0},
            "LOWSTOP": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": 0},
            "REVERSE": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": 0},
            "PISTON_OUT": {},
            "PISTON_IN": {},
            "CHARGE_OUT": {},
            "RETURN_DOCK": {}
        }
        self.last_state = None
        self.current_status = self.status_list[0]
        self.current_velocity_up = 0
        self.current_velocity_low = 0
        self.current_velocity_brush = 0
        self.counter_a = 0
        self.counter_b = 0
        self.counter_c = 0
        self.counter_d = 0
        self.threshold = 30

        self.stop_flag = False
        self.position_engaged = False
        self.position_mode_configured = False
        self.left_position = 0
        self.right_position = 0
        self.position_direction = 1
        self.target_sent_flag = False

        self.need_speed_mode_init = False
        self.enable_drive_flag = False
        self.stop_velocity = 0
        self.imu_yaw = 0.0
        self.initial_yaw = None

        self.sensors_status = 0
        self.complete_state = False
        self.prev_motion_state = None
        self.is_upstop = False
        self.is_lowstop = False
        self.auto_mode = True
        self.auto_step = None
        self.count = 1

        self.publish_timer = rospy.Timer(rospy.Duration(1.0), lambda event: self.publish_state())
        self.fault_check_timer = rospy.Timer(rospy.Duration(60.0), lambda event: self.check_and_clear_faults())

        self.pid_integral = 0.0
        self.pid_last_error = 0.0
        self.target_yaw = 0.0
        self.pid_kp = 100
        self.pid_ki = 0.1
        self.pid_kd = 10
        self.pid_correction_max = 150

        self.progress = 0
        self.battery_total_voltage = None
        self.battery_current = None
        self.battery_remaining = None
        self.battery_temperatures = []
        self.relay_status = None

        self.wind_speed = None
        self.wind_direction = None
        self.illuminance = None
        self.rainfall = None

        self.state_pub = rospy.Publisher('/robot_state', String, queue_size=10)
        self.motor_cmd_pub = rospy.Publisher('/motor_cmd', Int8, queue_size=10)
        rospy.Subscriber('/robot_cmd', String, self.status_callback)
        rospy.Subscriber('/inspvae_data', INSPVAE, self.imu_callback)
        rospy.Subscriber('/battery_status', BatteryStatus, self.battery_status_callback)
        rospy.Subscriber('/relay_status', Bool, self.relay_callback)
        rospy.Subscriber("proximity_sensor_data", Sensors, lambda msg: self.proximity_callback(msg))
        rospy.Subscriber('/environment_data', Environment, self.environment_data_callback)


    def create_serial(self, port, baudrate):
        """创建串口连接（替代原CAN总线连接）"""
        while True:
            try:
                ser = serial.Serial(port, baudrate, timeout=1)
                rospy.loginfo(f"串口连接成功: {ser.portstr}")
                return ser
            except (serial.SerialException, OSError) as e:
                rospy.logerr(f"串口连接失败: {e}，3秒后重试...")
                time.sleep(3)

    def calculate_checksum(self, data):
        """计算校验和（适配转换器协议）"""
        return sum(data[2:]) & 0xff
    def environment_data_callback(self, msg):
        self.wind_speed = msg.wind_speed
        self.wind_direction = msg.wind_direction
        self.illuminance = msg.illuminance
        self.rainfall = msg.rainfall
    def send_can_command(self, can_id, data):
        """
        发送CAN指令（按照转换器私有协议封装）
        格式参考send.py：帧头(0xaa) + 帧类型 + CAN ID拆分 + 数据 + 帧尾(0x55) + 校验和
        """
        # 1. 构造帧头和帧类型
        frame = [0xaa]
        data_len = len(data)
        # 帧类型：0xc0（数据帧标识） + 数据长度（低4位） + 标准帧标志（bit5=0）
        frame_type = 0xc0 | (data_len & 0x0f)
        frame.append(frame_type)

        # 2. 拆分CAN ID（标准帧11位，拆分为2字节）
        frame.append(can_id & 0xff)       # ID低8位
        frame.append((can_id >> 8) & 0xff)  # ID高8位

        # 3. 添加数据
        frame.extend(data)

        # 4. 添加帧尾和校验和
        frame.append(0x55)
        checksum = self.calculate_checksum(frame)
        frame.append(checksum)

        # 5. 发送数据
        try:
            self.ser.write(bytes(frame))
            rospy.logdebug(f"发送CAN指令: ID=0x{can_id:X}, 数据={[hex(b) for b in data]}")
            time.sleep(0.05)  # 确保指令发送完成
        except serial.SerialException as e:
            rospy.logerr(f"串口发送失败: {e}，尝试重连...")
            self.ser = self.create_serial(self.ser.port, self.ser.baudrate)

    def set_state(self, new_state):
        """状态切换时发送对应的使能/停止指令"""
        if new_state not in self.status_config:
            rospy.logwarn(f"无效状态: {new_state}")
            return False
        if new_state == self.current_status and new_state not in ["PISTON_OUT", "PISTON_IN", "STOP"]:
            return False

        # 使能指令（START状态时发送）
        if new_state == "START":
            rospy.loginfo("发送电机使能指令序列...")
            # 按顺序发送5条使能指令（ID=0x601）
            enable_commands = [
                [0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00],
                [0x23, 0xFF, 0x60, 0x00, 0x90, 0x1A, 0x3F, 0x00],
                [0x23, 0x83, 0x60, 0x00, 0xD0, 0x84, 0x00, 0x00],
                [0x23, 0x84, 0x60, 0x00, 0x70, 0x8E, 0x01, 0x00],
                [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00]
            ]
            for cmd in enable_commands:
                self.send_can_command(0x601, cmd)

        # 停止指令（STOP状态时发送）
        elif new_state == "STOP":
            rospy.loginfo("发送电机停止指令...")
            stop_commands = [
                [0x2B, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00],
                [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00]
            ]
            for cmd in stop_commands:
                self.send_can_command(0x601, cmd)

        # 其他状态处理（保持原逻辑）
        if self.current_status in ["FORWARD", "BACKWARD"] and (new_state == "CHARGE_OUT" or new_state == "RETURN_DOCK"):
            return False
        if self.auto_mode and new_state in ["FORWARD", "BACKWARD"]:
            self.auto_step = new_state
        if new_state in ["START", "CHARGE_OUT", "RETURN_DOCK"]:
            self.complete_state = False
            self.enable_drive_flag = True
            self.progress = 0
            self.motor_driver = True
            self.imu_sensor = True
            self.main_board = True
        if new_state == "STOP":
            self.elevator_stage = 0
            if self.publish_timer:
                self.publish_timer.shutdown()
                self.publish_timer = rospy.Timer(rospy.Duration(1.0), lambda event: self.publish_state())
            if self.fault_check_timer:
                self.fault_check_timer.shutdown()
                self.fault_check_timer = rospy.Timer(rospy.Duration(60.0), lambda event: self.check_and_clear_faults())
            threading.Thread(target=self.delayed_publish_freq_switch, args=(1,), daemon=True).start()
        else:
            if self.publish_timer:
                self.publish_timer.shutdown()
                self.publish_timer = rospy.Timer(rospy.Duration(1.0), lambda event: self.publish_state())
            if self.fault_check_timer:
                self.fault_check_timer.shutdown()
                self.fault_check_timer = rospy.Timer(rospy.Duration(60.0), lambda event: self.check_and_clear_faults())
        if new_state in ["FORWARD", "BACKWARD"]:
            if self.current_status != "REVERSE":
                self.prev_motion_state = new_state
        if new_state in ["REVERSE", "UPSTOP", "LOWSTOP"]:
            if self.prev_motion_state is None:
                self.prev_motion_state = self.last_state
        elif new_state == "PISTON_IN":
            self.initial_yaw = None
            self.auto_step = None
        elif new_state == "PISTON_OUT":
            self.auto_mode = not self.auto_mode
            rospy.loginfo(f"{'手动模式' if not self.auto_mode else '自动模式'}已开启")
            self.count += 1

        self.current_status = new_state
        self.last_state = self.current_status
        rospy.loginfo(f"状态已更新为: {self.current_status}")
        return True

    # 以下为原逻辑保持不变的函数（仅删除原CANopen相关控制函数）
    def lock_motor(self):
        rospy.loginfo("电机向下转动，锁止")
        self.motor_cmd_pub.publish(Int8(data=-1))
        self.initial_yaw = None

    def publish_state(self):
        try:
            self.velocity_publish_count += 1
            if self.velocity_publish_count >= self.velocity_publish_interval:
                self.last_velocity_low = 0  # 无法通过串口直接读取速度，暂设为0
                self.velocity_publish_count = 0

            velocity_up = self.last_velocity_up
            velocity_low = self.last_velocity_low
            velocity_brush = self.last_velocity_brush

            state_msg = {
                "status": self.current_status,
                "battery": self.battery_remaining,
                "battery_temperatures": self.battery_temperatures,
                "battery_total_voltage": self.battery_total_voltage,
                "battery_current": self.battery_current,
                "progress": self.progress,
                "imu_yaw": round(self.imu_yaw, 2) if self.imu_yaw is not None else 0.00,
                "velocity_up": round(velocity_up / rate, 2),
                "velocity_low": round(velocity_low / rate, 2),
                "velocity_brush": round(velocity_brush / rate, 2),
                "sensors_status": self.sensors_status,
                "device_status": {
                    "main_board": self.main_board,
                    "imu_sensor": self.imu_sensor,
                    "motor_driver": self.motor_driver,
                    "comm_module": True
                },
                "complete_state": self.complete_state,
                "auto_mode": self.auto_mode,
                "relay_status": self.relay_status,
                "wind_speed": self.wind_speed,
                "wind_direction": self.wind_direction,
                "illuminance": self.illuminance,
                "rainfall":self.rainfall,
                "timestamp": time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
            }
            self.state_pub.publish(json.dumps(state_msg))
        except Exception as e:
            rospy.logerr(f"状态发布失败: {e}")
            error_msg = {"status": "ERROR"}
            self.state_pub.publish(json.dumps(error_msg))

    def status_callback(self, msg):
        try:
            cmd_obj = json.loads(msg.data)
            command = cmd_obj.get("command", None)
            if command == "GET_STATUS":
                self.publish_state()
            elif command in self.status_list:
                self.set_state(command)
            else:
                rospy.logwarn(f"未找到有效command: {msg.data}")
        except Exception as e:
            rospy.logwarn(f"消息解析失败: {e}，尝试直接处理字符串: {msg.data}")
            self.set_state(msg.data)
            self.publish_state()

    def imu_callback(self, msg):
        try:
            self.imu_yaw = msg.yaw if hasattr(msg, "yaw") else 0.0
            if self.initial_yaw is None:
                self.initial_yaw = self.imu_yaw
                rospy.loginfo(f"初始IMU偏航角: {self.initial_yaw}度")
            if self.initial_yaw is not None:
                relative_yaw = self.imu_yaw - self.initial_yaw
                if relative_yaw > 180:
                    relative_yaw -= 360
                elif relative_yaw < -180:
                    relative_yaw += 360
                self.imu_yaw = relative_yaw
        except Exception as e:
            rospy.logerr(f"IMU数据处理失败: {e}")

    def battery_status_callback(self, msg):
        self.battery_remaining = msg.batttery_remaining
        self.battery_total_voltage = round(msg.total_voltage, 2)
        self.battery_current = round(msg.current, 2)
        self.battery_temperatures = [round(t, 1) for t in msg.temperatures] if hasattr(msg, "temperatures") else []

    def relay_callback(self, msg):
        self.relay_status = msg.data

    def delayed_publish_freq_switch(self, delay_sec=3):
        time.sleep(delay_sec)
        if self.current_status == "STOP":
            if self.publish_timer:
                self.publish_timer.shutdown()
            self.publish_timer = rospy.Timer(rospy.Duration(1800), lambda event: self.publish_state())
            if self.fault_check_timer:
                self.fault_check_timer.shutdown()
            self.fault_check_timer = rospy.Timer(rospy.Duration(7200), lambda event: self.check_and_clear_faults())

    def proximity_callback(self, msg):
        # 保持原传感器处理逻辑
        if msg.sensor_a:
            self.sensors_status |= 0x01
        else:
            self.sensors_status &= ~0x01
        if msg.sensor_b:
            self.sensors_status |= 0x02
        else:
            self.sensors_status &= ~0x02
        if msg.sensor_c:
            self.sensors_status |= 0x04
        else:
            self.sensors_status &= ~0x04
        if msg.sensor_d:
            self.sensors_status |= 0x08
        else:
            self.sensors_status &= ~0x08

        # 自动模式逻辑（保持不变）
        if self.auto_mode and self.current_status == "RETURN_DOCK" and self.elevator_stage == 2:
            self.set_state("FORWARD")
        if self.auto_mode and self.current_status == "CHARGE_OUT" and self.elevator_stage == 2:
            if msg.sensor_a or msg.sensor_c:
                if self.current_status != "UNLOADING":
                    self.set_state("UNLOADING")
                    self.progress = 10
                    self.unloading_start_time = time.time()
            if self.current_status == "UNLOADING" and self.unloading_start_time:
                if time.time() - self.unloading_start_time >= self.unloading_timer:
                    self.set_state("STOP")
                    self.progress = 0
                    self.unloading_start_time = None
                    self.stop_imu()
        if self.auto_mode and self.current_status == "START" and self.elevator_stage == 2:
            if msg.sensor_a or msg.sensor_c:
                if self.current_status != "UNLOADING":
                    self.set_state("UNLOADING")
                    self.progress = 10
                    self.unloading_start_time = time.time()
            elif not msg.sensor_a and not msg.sensor_c:
                self.set_state("BACKWARD")
                self.progress = 20
            if self.current_status == "UNLOADING" and self.unloading_start_time:
                if time.time() - self.unloading_start_time >= self.unloading_timer:
                    self.set_state("BACKWARD")
                    self.progress = 20
                    self.unloading_start_time = None

        if self.current_status == "REVERSE":
            if msg.sensor_a and msg.sensor_c:
                self.set_state("STOP")
            elif msg.sensor_b and msg.sensor_d:
                self.set_state("STOP")
            elif msg.sensor_a or msg.sensor_b:
                self.set_state("LOWSTOP")
            elif msg.sensor_c or msg.sensor_d:
                self.set_state("UPSTOP")

        if self.auto_mode:
            if self.current_status == "FORWARD":
                if msg.sensor_a and msg.sensor_c:
                    threading.Timer(5.0, self.lock_motor).start()
                    self.set_state("STOP")
                    self.complete_state = True
                    self.progress = 100
                    self.auto_step = None
                    self.elevator_stage = 0
                    self.stop_imu()
                    rospy.loginfo("进仓完成")
                elif msg.sensor_a and not msg.sensor_c:
                    self.set_state("LOWSTOP")
                elif msg.sensor_c and not msg.sensor_a:
                    self.set_state("UPSTOP")
            if self.current_status == "BACKWARD":
                if msg.sensor_b and msg.sensor_d:
                    self.initial_yaw = None
                    self.set_state("LOADING")
                    time.sleep(3)
                    self.set_state("FORWARD")
                    self.progress = 60
                elif msg.sensor_b and not msg.sensor_d:
                    self.set_state("LOWSTOP")
                elif msg.sensor_d and not msg.sensor_b:
                    self.set_state("UPSTOP")
        else:
            if self.current_status == "FORWARD":
                if msg.sensor_a and msg.sensor_c:
                    threading.Timer(5.0, self.lock_motor).start()
                    self.set_state("STOP")
                    self.complete_state = True
                    self.progress = 100
                    self.auto_step = None
                    self.elevator_stage = 0
                    self.stop_imu()
                elif msg.sensor_a and not msg.sensor_c:
                    self.set_state("LOWSTOP")
                elif msg.sensor_c and not msg.sensor_a:
                    self.set_state("UPSTOP")
            elif self.current_status == "BACKWARD":
                if msg.sensor_b and msg.sensor_d:
                    self.set_state("STOP")
                    self.complete_state = True
                    self.progress = 100
                    self.auto_step = None
                    self.elevator_stage = 0
                elif msg.sensor_b and not msg.sensor_d:
                    self.set_state("LOWSTOP")
                elif msg.sensor_d and not msg.sensor_b:
                    self.set_state("UPSTOP")

        if self.current_status == "LOWSTOP":
            if msg.sensor_c:
                threading.Timer(5.0, self.lock_motor).start()
                self.set_state("STOP")
                self.complete_state = True
                self.initial_yaw = None
                self.progress = 100
                self.auto_step = None
                self.is_lowstop = False
                self.elevator_stage = 0
                self.stop_imu()
                rospy.loginfo("由LOWSTOP至进仓完成")
            else:
                self.complete_state = False
            if msg.sensor_d:
                self.initial_yaw = None
                self.set_state("LOADING")
                time.sleep(3)
                self.set_state("FORWARD")
                self.progress = 60
        if self.current_status == "UPSTOP":
            if msg.sensor_a:
                threading.Timer(5.0, self.lock_motor).start()
                self.set_state("STOP")
                self.complete_state = True
                self.initial_yaw = None
                self.progress = 100
                self.auto_step = None
                self.is_upstop = False
                self.elevator_stage = 0
                self.stop_imu()
                rospy.loginfo("由UPSTOP至进仓完成")
            else:
                self.complete_state = False
            if msg.sensor_b:
                self.initial_yaw = None
                self.set_state("LOADING")
                time.sleep(3)
                self.set_state("FORWARD")
                self.progress = 60

    def pid_correction(self, current_yaw):
        error = self.target_yaw - current_yaw
        if abs(error) < 0.05:
            return 0
        if abs(error) > 0.3:
            self.pid_integral = 0
        self.pid_integral += error
        integral_max = 30
        self.pid_integral = max(min(self.pid_integral, integral_max), -integral_max)
        derivative = error - self.pid_last_error
        correction = (self.pid_kp * error + self.pid_ki * self.pid_integral + self.pid_kd * derivative)
        self.pid_last_error = error
        return max(min(-correction, self.pid_correction_max), -self.pid_correction_max)

    def execute_state(self, event=None):
        # 仅保留状态逻辑，删除原CANopen速度控制（如需速度控制需额外发送对应指令）
        if self.enable_drive_flag and self.current_status in ["START", "CHARGE_OUT", "RETURN_DOCK"]:
            if self.battery_remaining is not None and self.battery_remaining < self.LOW_BATTERY_THRESHOLD:
                rospy.logerr("电池电量过低，无法启动")
                self.enable_drive_flag = False
                self.main_board = False
                self.set_state("STOP")
                self.stop_imu()
                return
            if self.elevator_stage == 0:
                rospy.loginfo("电缸抬起...")
                self.motor_cmd_pub.publish(Int8(data=1))
                self.elevator_start_time = rospy.get_time()
                self.elevator_stage = 1
                self.start_imu()
            elif self.elevator_stage == 1:
                elapsed = rospy.get_time() - self.elevator_start_time
                if elapsed >= 20:
                    rospy.loginfo("初始化完成")
                    self.enable_drive_flag = False
                    self.elevator_stage = 2
                    self.start_time = rospy.get_time()
                    if self.auto_mode and self.auto_step and self.current_status == "START":
                        self.set_state(self.auto_step)

        if self.current_status in ["FORWARD", "BACKWARD"]:
            correction = self.pid_correction(self.imu_yaw) * rate
            left_speed = int(self.status_config[self.current_status]["velocity_up"] + correction)
            right_speed = int(self.status_config[self.current_status]["velocity_low"] + correction)
            brush_speed = self.status_config[self.current_status]["velocity_brush"]
            right_speed = max(min(right_speed, self.speed_pluse_max), -self.speed_pluse_max)
            left_speed = max(min(left_speed, self.speed_pluse_max), -self.speed_pluse_max)
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            self.last_brush_speed = brush_speed

            angle_condition_met = (-5 < self.imu_yaw < -2.5 or 2.5 < self.imu_yaw < 5)
            if angle_condition_met:
                if self.reversed_start_time is None:
                    self.reversed_start_time = rospy.get_time()
                    rospy.logwarn(f"角度偏差: {self.imu_yaw:.2f}度，开始计时...")
                elapsed = rospy.get_time() - self.reversed_start_time
                if elapsed >= self.REVERSE_TIME_THRESHOLD:
                    rospy.logwarn(f"角度偏差持续{elapsed:.1f}秒，进入REVERSE")
                    self.set_state("REVERSE")
                    self.reversed_start_time = None
            else:
                if self.reversed_start_time is not None:
                    rospy.loginfo(f"角度偏差消失，重置计时器")
                    self.reversed_start_time = None

        elif self.current_status == "REVERSE":
            self.reversed_start_time = None
            if not self.has_reverse_flag:
                self.has_reverse_counter += 1
                if self.has_reverse_counter > 10:
                    rospy.logwarn("连续后退10次，停止")
                    self.has_reverse_counter = 0
                    self.set_state("STOP")
                    self.motor_driver = False
                    self.imu_sensor = False
                    return
                right_speed = -int(self.last_right_speed)
                left_speed = -int(self.last_left_speed)
                brush_speed = self.last_brush_speed
                right_speed = max(min(right_speed, self.speed_pluse_max), -self.speed_pluse_max)
                left_speed = max(min(left_speed, self.speed_pluse_max), -self.speed_pluse_max)
                self.last_left_speed = left_speed
                self.last_right_speed = right_speed
                self.last_brush_speed = brush_speed
                self.has_reverse_flag = True
                self.reverse_start_time = time.time()
                time.sleep(2.0)
            else:
                if self.imu_yaw >= 0:
                    self.flag = -1
                elif self.imu_yaw < 0:
                    self.flag = 1
                if abs(self.imu_yaw) > 1:
                    right_speed = left_speed = int(-self.base_speed * 0.8 * self.flag)
                else:
                    right_speed = left_speed = int(-self.base_speed * 0.6 * self.flag)
                right_speed = max(min(right_speed, self.speed_pluse_max), -self.speed_pluse_max)
                left_speed = max(min(left_speed, self.speed_pluse_max), -self.speed_pluse_max)
                brush_speed = self.last_brush_speed
                self.last_left_speed = left_speed
                self.last_right_speed = right_speed
                self.last_brush_speed = brush_speed
                if abs(self.imu_yaw) < 0.05:
                    if self.prev_motion_state:
                        restore_state = self.prev_motion_state
                        self.prev_motion_state = None
                        self.set_state(restore_state)
                    self.is_upstop = False
                    self.is_lowstop = False
                    self.has_reverse_flag = False

        elif self.current_status == "UPSTOP":
            left_speed = 0
            right_speed = self.last_right_speed
            brush_speed = self.last_brush_speed
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            self.last_brush_speed = brush_speed

        elif self.current_status == "LOWSTOP":
            left_speed = self.last_left_speed
            right_speed = 0
            brush_speed = self.last_brush_speed
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            self.last_brush_speed = brush_speed

        elif self.current_status == "STOP" or not -5 < self.imu_yaw < 5:
            if self.last_left_speed != 0 or self.last_right_speed != 0 or self.last_brush_speed != 0:
                self.has_reverse_counter = 0
                self.last_left_speed = 0
                self.last_right_speed = 0
                self.last_brush_speed = 0
                self.need_speed_mode_init = True

        elif self.current_status == "UNLOADING":
            correction = self.pid_correction(self.imu_yaw) * rate
            left_speed = int(self.status_config[self.current_status]["velocity_up"] + correction)
            right_speed = int(self.status_config[self.current_status]["velocity_low"] + correction)
            brush_speed = self.status_config[self.current_status]["velocity_brush"]
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            self.last_brush_speed = brush_speed

        elif self.current_status == "LOADING":
            left_speed = int(self.status_config[self.current_status]["velocity_up"])
            right_speed = int(self.status_config[self.current_status]["velocity_low"])
            brush_speed = self.status_config[self.current_status]["velocity_brush"]
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            self.last_brush_speed = brush_speed

    def start_imu(self):
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service('/imu_parser_node/start_imu', timeout=5)
                start_srv = rospy.ServiceProxy('/imu_parser_node/start_imu', Trigger)
                resp = start_srv()
                rospy.loginfo(resp.message)
                break
            except Exception as e:
                rospy.logwarn(f"等待IMU服务: {e}")
                time.sleep(1)

    def stop_imu(self):
        try:
            rospy.wait_for_service('/imu_parser_node/stop_imu')
            stop_srv = rospy.ServiceProxy('/imu_parser_node/stop_imu', Trigger)
            resp = stop_srv()
            rospy.loginfo(resp.message)
        except Exception as e:
            rospy.logwarn(f"调用IMU停止服务失败: {e}")

    def check_and_clear_faults(self):
        rospy.loginfo("执行故障检查（简化版）")

    @staticmethod
    def keyboard_listener(controller):
        rospy.loginfo("按键控制：s=停止, f=前进, b=后退, a=启动")
        while not rospy.is_shutdown():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.readline().strip()
                if key:
                    controller.update_status_by_key(key)

    def update_status_by_key(self, key):
        key_mapping = {
            's': "STOP",
            'f': "FORWARD",
            'b': "BACKWARD",
            'a': "START",
            'r': "REVERSE",
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

    def shutdown(self):
        rospy.loginfo("关闭电机控制器...")
        # 发送停止指令
        stop_commands = [
            [0x2B, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00],
            [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00]
        ]
        for cmd in stop_commands:
            self.send_can_command(0x601, cmd)
        self.ser.close()

    @staticmethod
    def load_config(config_file="/home/ubuntu/demo01/src/motor_can/config/servo_config.yaml"):
        try:
            with open(config_file, 'r') as file:
                return yaml.safe_load(file)
        except Exception as e:
            rospy.logerr(f"加载配置失败: {e}")
            return {}

def main():
    rospy.init_node("motor_can_node")
    # 初始化串口（根据实际设备修改端口）
    controller = ServoDriveController()
    # config = controller.load_config()
    # if not config or "motors" not in config:
    #     rospy.logerr("未找到有效配置")
    #     return
    # rospy.loginfo("电机控制器初始化完成")

    t = threading.Thread(target=ServoDriveController.keyboard_listener, args=(controller,), daemon=True)
    t.start()

    try:
        rospy.Timer(rospy.Duration(0.05), controller.execute_state)
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("程序终止")
    finally:
        controller.shutdown()

if __name__ == "__main__":
    main()