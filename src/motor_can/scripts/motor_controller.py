#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rospy
import json
import yaml
import threading
from std_msgs.msg import String, Int8, Bool
from serial_comms.msg import Sensors, INSPVAE, BatteryStatus

# 全局常量
RATE = 24  # rpm*24速比

class RobotController:
    def __init__(self, motor_driver):
        # 注入电机驱动实例（解耦核心）
        self.motor_driver = motor_driver
        # ROS初始化
        rospy.init_node('robot_controller_node', anonymous=True)
        self.rate = rospy.Rate(20)
        # 状态变量
        self.last_left_speed = 0
        self.last_right_speed = 0
        self.last_brush_speed = 0
        self.has_reverse_flag = False
        self.has_reverse_counter = 0
        self.reverse_start_time = None
        self.main_board = True
        self.imu_sensor = True
        self.motor_base = 30
        self.base_speed = 30
        self.brush_base_speed = 100 * 20
        self.flag = 0
        self.speed_pluse_max = 60 * RATE
        self.reversed_start_time = None
        self.REVERSE_TIME_THRESHOLD = 3.0
        self.unloading_timer = 2.0
        self.unloading_start_time = None
        self.start_time = 0
        self.elevator_stage = 0
        self.elevator_start_time = 0
        self.LOW_BATTERY_THRESHOLD = 40
        self.velocity_publish_count = 0
        self.velocity_publish_interval = 2
        self.last_velocity_up = 0
        self.last_velocity_low = 0
        self.last_velocity_brush = 0
        self.last_sensor_a = False
        self.sensor_a_count = 0
        # 状态配置
        self.status_list = [
            "STOP", "FORWARD", "BACKWARD", "START", "LOADING", "UNLOADING",
            "UPSTOP", "LOWSTOP", "PISTON_OUT", "PISTON_IN", "CHARGE_OUT",
            "RETURN_DOCK", "PAUSE"
        ]
        self.status_config = {
            "START": {},
            "STOP": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": 0},
            "UNLOADING": {
                "velocity_up": -self.motor_base * RATE,
                "velocity_low": self.motor_base * RATE,
                "velocity_brush": -self.brush_base_speed
            },
            "FORWARD": {
                "velocity_up": -self.motor_base * RATE,
                "velocity_low": self.motor_base * RATE,
                "velocity_brush": -self.brush_base_speed
            },
            "BACKWARD": {
                "velocity_up": self.motor_base * RATE,
                "velocity_low": -self.motor_base * RATE,
                "velocity_brush": -self.brush_base_speed
            },
            "LOADING": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": -self.brush_base_speed},
            "PAUSE": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": -self.brush_base_speed},
            "UPSTOP": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": -self.brush_base_speed},
            "LOWSTOP": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": -self.brush_base_speed},
            "REVERSE": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": -self.brush_base_speed},
            "PISTON_OUT": {}, "PISTON_IN": {}, "CHARGE_OUT": {}, "RETURN_DOCK": {}
        }
        self.current_status = self.status_list[0]
        self.last_state = None
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
        # PID参数
        self.pid_integral = 0.0
        self.pid_last_error = 0.0
        self.target_yaw = 0.0
        self.pid_kp = 80
        self.pid_ki = 0
        self.pid_kd = 20
        self.pid_correction_max = 80
        self.progress = 0
        # 环境与电池状态
        self.battery_total_voltage = None
        self.battery_current = None
        self.battery_remaining = None
        self.battery_temperatures = []
        self.relay_status = None
        self.wind_speed = None
        self.wind_direction = None
        self.illuminance = None
        self.rainfall = None
        # ROS发布订阅
        self.state_pub = rospy.Publisher('/robot_state', String, queue_size=10)
        self.motor_cmd_pub = rospy.Publisher('/motor_cmd', Int8, queue_size=10)
        rospy.Subscriber('/robot_cmd', String, self.status_callback)
        rospy.Subscriber('/inspvae_data', INSPVAE, self.imu_callback)
        rospy.Subscriber('/battery_status', BatteryStatus, self.battery_status_callback)
        rospy.Subscriber('/relay_status', Bool, self.relay_callback)
        # rospy.Subscriber('/environment_data', Environment, self.environment_data_callback)
        rospy.Subscriber("proximity_sensor_data", Sensors, self.proximity_callback)
        # 定时器
        self.publish_timer = rospy.Timer(rospy.Duration(1.0), lambda event: self.publish_state())
        # 同步锁
        self.state_switch_lock = threading.Lock()

    # -------------------------- 状态管理 --------------------------
    def set_state(self, new_state):
        """设置机器人状态"""
        if new_state not in self.status_config:
            rospy.logwarn(f"⚠️ 无效状态: {new_state}")
            return False
        if new_state == self.current_status and new_state not in ["PISTON_OUT", "PISTON_IN", "STOP"]:
            return False
        if self.current_status in ["FORWARD", "BACKWARD"] and new_state in ["CHARGE_OUT", "RETURN_DOCK"]:
            return False
        # 自动模式处理
        if self.auto_mode and new_state in ["FORWARD", "BACKWARD"]:
            self.auto_step = new_state
        # 启动类状态初始化
        if new_state in ["START", "CHARGE_OUT", "RETURN_DOCK"]:
            self.complete_state = False
            self.enable_drive_flag = True
            self.progress = 0
            self.motor_driver.motor_driver_status = True
            self.imu_sensor = True
            self.main_board = True
        # STOP状态处理
        if new_state == "STOP":
            self.elevator_stage = 0
            self.publish_timer.shutdown()
            self.publish_timer = rospy.Timer(rospy.Duration(1.0), lambda event: self.publish_state())
            self.motor_driver.stop_heartbeat()
            threading.Thread(target=self.delayed_publish_freq_switch, args=(1,), daemon=True).start()
        else:
            self.publish_timer.shutdown()
            self.publish_timer = rospy.Timer(rospy.Duration(1.0), lambda event: self.publish_state())
        # 运动状态记录
        if new_state in ["FORWARD", "BACKWARD"] and self.current_status != "REVERSE":
            self.prev_motion_state = new_state
        if new_state in ["REVERSE", "UPSTOP", "LOWSTOP"] and self.prev_motion_state is None:
            self.prev_motion_state = self.last_state
        elif new_state == "PISTON_IN":
            self.initial_yaw = None
            self.auto_step = None
        elif new_state == "PISTON_OUT":
            self.auto_mode = not self.auto_mode
            rospy.loginfo(f"🔘 {'手动模式' if self.auto_mode else '自动模式'}开启")
            self.count += 1
        # 更新状态
        self.last_state = self.current_status
        self.current_status = new_state
        rospy.loginfo(f"📌 状态更新为: {self.current_status}")
        return True

    # -------------------------- 回调函数 --------------------------
    def status_callback(self, msg):
        """指令回调（/robot_cmd）"""
        try:
            cmd_obj = json.loads(msg.data)
            command = cmd_obj.get("command", None)
            if command == "GET_STATUS":
                self.publish_state()
            elif command in self.status_list:
                self.set_state(command)
            else:
                rospy.logwarn(f"⚠️ 无效指令: {msg.data}")
        except Exception as e:
            rospy.logwarn(f"⚠️ 解析失败: {msg.data}，错误: {e}")
            self.set_state(msg.data)
            self.publish_state()

    def imu_callback(self, msg):
        """IMU数据回调"""
        try:
            self.imu_yaw = msg.yaw if hasattr(msg, "yaw") else 0.0
            if self.initial_yaw is None:
                self.initial_yaw = self.imu_yaw
                rospy.loginfo(f"🧭 初始偏航角: {self.initial_yaw}°")
            # 计算相对偏航角
            relative_yaw = self.imu_yaw - self.initial_yaw
            if relative_yaw > 180:
                relative_yaw -= 360
            elif relative_yaw < -180:
                relative_yaw += 360
            self.imu_yaw = relative_yaw
        except Exception as e:
            rospy.logerr(f"❌ IMU解析失败: {e}")

    def battery_status_callback(self, msg):
        """电池状态回调"""
        self.battery_remaining = msg.batttery_remaining
        self.battery_total_voltage = round(msg.total_voltage, 2)
        self.battery_current = round(msg.current, 2)
        self.battery_temperatures = [round(t, 1) for t in msg.temperatures] if hasattr(msg, "temperatures") else []

    def relay_callback(self, msg):
        """继电器回调"""
        self.relay_status = msg.data

    def environment_data_callback(self, msg):
        """环境数据回调"""
        self.wind_speed = msg.wind_speed
        self.wind_direction = msg.wind_direction
        self.illuminance = msg.illuminance
        self.rainfall = msg.rainfall

    def proximity_callback(self, msg):
        """接近传感器回调"""
        # 更新传感器状态
        self.sensors_status = 0
        if msg.sensor_a: self.sensors_status |= 0x01
        if msg.sensor_b: self.sensors_status |= 0x02
        if msg.sensor_c: self.sensors_status |= 0x04
        if msg.sensor_d: self.sensors_status |= 0x08
        # 传感器A计数
        if not self.last_sensor_a and msg.sensor_a:
            self.sensor_a_count += 1
            rospy.loginfo(f"📊 传感器A触发次数: {self.sensor_a_count}")
        self.last_sensor_a = msg.sensor_a

        # 特殊状态逻辑
        if self.auto_mode:
            if self.current_status == "RETURN_DOCK" and self.elevator_stage == 2:
                self.set_state("FORWARD")
            elif self.current_status == "CHARGE_OUT" and self.elevator_stage == 2:
                self._handle_charge_out_sensor(msg)
            elif self.current_status == "START" and self.elevator_stage == 2:
                self._handle_start_sensor(msg)
        # REVERSE状态边界
        if self.current_status == "REVERSE":
            if (msg.sensor_a and msg.sensor_c) or (msg.sensor_b and msg.sensor_d):
                self.set_state("STOP")
            elif msg.sensor_a or msg.sensor_b:
                self.set_state("LOWSTOP")
            elif msg.sensor_c or msg.sensor_d:
                self.set_state("UPSTOP")
        # 运动状态处理
        self._handle_motion_sensor(msg)
        # 停止状态处理
        self._handle_stop_sensor(msg)

    # -------------------------- 传感器处理辅助 --------------------------
    def _handle_charge_out_sensor(self, msg):
        """CHARGE_OUT状态传感器逻辑"""
        if msg.sensor_a or msg.sensor_c and self.current_status != "UNLOADING":
            self.set_state("UNLOADING")
            self.progress = 10
            self.unloading_start_time = time.time()
        if self.current_status == "UNLOADING" and self.unloading_start_time:
            if time.time() - self.unloading_start_time >= self.unloading_timer:
                self.set_state("STOP")
                self.progress = 0
                self.unloading_start_time = None

    def _handle_start_sensor(self, msg):
        """START状态传感器逻辑"""
        if msg.sensor_a or msg.sensor_c and self.current_status != "UNLOADING":
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

    def _handle_motion_sensor(self, msg):
        """运动状态（FORWARD/BACKWARD）传感器逻辑"""
        if self.current_status == "FORWARD":
            if msg.sensor_a and msg.sensor_c:
                self._complete_loading()
            elif msg.sensor_a and not msg.sensor_c:
                self.set_state("LOWSTOP")
            elif msg.sensor_c and not msg.sensor_a:
                self.set_state("UPSTOP")
        elif self.current_status == "BACKWARD":
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

    def _handle_stop_sensor(self, msg):
        """停止状态（UPSTOP/LOWSTOP）传感器逻辑"""
        if self.current_status == "LOWSTOP":
            if msg.sensor_c:
                self._complete_loading()
            elif msg.sensor_d:
                self.initial_yaw = None
                self.set_state("LOADING")
                time.sleep(3)
                self.set_state("FORWARD")
                self.progress = 60
        elif self.current_status == "UPSTOP":
            if msg.sensor_a:
                self._complete_loading()
            elif msg.sensor_b:
                self.initial_yaw = None
                self.set_state("LOADING")
                time.sleep(3)
                self.set_state("FORWARD")
                self.progress = 60

    def _complete_loading(self):
        """完成进仓流程"""
        threading.Timer(0.1, self.lock_motor).start()
        self.set_state("PAUSE")
        self.complete_state = True
        self.initial_yaw = None
        self.progress = 100
        self.auto_step = None
        self.elevator_stage = 0
        rospy.loginfo("🎉 进仓完成")
        time.sleep(3)
        self.set_state("BACKWARD")
        self.progress = 10

    # -------------------------- PID矫正 --------------------------
    def pid_correction(self, current_yaw):
        """PID偏航角矫正"""
        error = self.target_yaw - current_yaw
        if abs(error) < 0.05:
            return 0
        if abs(error) > 0.3:
            self.pid_integral = 0
        self.pid_integral += error
        derivative = error - self.pid_last_error
        self.pid_integral = max(min(self.pid_integral, 30), -30)
        correction = self.pid_kp * error + self.pid_ki * self.pid_integral + self.pid_kd * derivative
        self.pid_last_error = error
        return -max(min(-correction, self.pid_correction_max), -self.pid_correction_max)

    # -------------------------- 状态执行 --------------------------
    def execute_state(self, event=None):
        """执行当前状态逻辑"""
        # 启动类状态
        if self.enable_drive_flag and self.current_status in ["START", "CHARGE_OUT", "RETURN_DOCK"]:
            self._execute_start_state()
        # 运动状态
        elif self.current_status in ["FORWARD", "BACKWARD"]:
            self._execute_motion_state()
        # 反向矫正状态
        elif self.current_status == "REVERSE":
            self._execute_reverse_state()
        # 停止状态
        elif self.current_status in ["UPSTOP", "LOWSTOP"]:
            self._execute_stop_state()
        # STOP状态
        elif self.current_status == "STOP" or not -5 < self.imu_yaw < 5:
            self._execute_stop_all()
        # 其他状态
        elif self.current_status == "UNLOADING":
            self._execute_unloading_state()
        elif self.current_status in ["LOADING", "PAUSE"]:
            self._execute_loading_pause_state()

    def _execute_start_state(self):
        """执行启动状态"""
        # 低电量检查
        if self.battery_remaining is not None and self.battery_remaining < self.LOW_BATTERY_THRESHOLD:
            rospy.logerr("🔋 电量过低，无法启动")
            self.enable_drive_flag = False
            self.main_board = False
            self.set_state("STOP")
            return
        # 电缸抬起
        if self.elevator_stage == 0:
            rospy.loginfo("🔼 电缸抬起...")
            self.motor_cmd_pub.publish(Int8(data=1))
            self.elevator_start_time = rospy.get_time()
            self.elevator_stage = 1
        # 配置电机
        elif self.elevator_stage == 1:
            if rospy.get_time() - self.elevator_start_time >= 0.1:
                rospy.loginfo("⚙️ 配置电机...")
                self._configure_motors_from_config()
                rospy.loginfo("✅ 电机配置完成")
                self.enable_drive_flag = False
                self.elevator_stage = 2
                if self.auto_mode and self.auto_step and self.current_status == "START":
                    self.set_state(self.auto_step)

    def _execute_motion_state(self):
        """执行运动状态"""
        correction = self.pid_correction(self.imu_yaw)
        config = self.status_config[self.current_status]
        left_speed = int(config["velocity_up"] + correction)
        right_speed = int(config["velocity_low"] + correction)
        brush_speed = config["velocity_brush"]
        # 速度限幅
        left_speed = max(min(left_speed, self.speed_pluse_max), -self.speed_pluse_max)
        right_speed = max(min(right_speed, self.speed_pluse_max), -self.speed_pluse_max)
        # 更新速度
        if (self.last_left_speed != left_speed or self.last_right_speed != right_speed or self.last_brush_speed != brush_speed):
            self.motor_driver.set_target_velocity(3, left_speed)
            self.motor_driver.set_target_velocity(2, right_speed)
            self.motor_driver.set_target_velocity(4, brush_speed)
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            self.last_brush_speed = brush_speed
        # 角度偏差检测
        angle_condition_met = (-5 < self.imu_yaw < -2.5 or 2.5 < self.imu_yaw < 5)
        if angle_condition_met:
            if self.reversed_start_time is None:
                self.reversed_start_time = rospy.get_time()
                rospy.logwarn(f"⚠️ 角度偏差: {self.imu_yaw:.2f}°")
            elif rospy.get_time() - self.reversed_start_time >= self.REVERSE_TIME_THRESHOLD:
                self.set_state("REVERSE")
                self.reversed_start_time = None
        else:
            self.reversed_start_time = None

    def _execute_reverse_state(self):
        """执行反向矫正状态"""
        self.reversed_start_time = None
        if not self.has_reverse_flag:
            self.has_reverse_counter += 1
            if self.has_reverse_counter > 10:
                rospy.logwarn("⚠️ 连续矫正10次，停止")
                self.set_state("STOP")
                return
            # 反向速度
            left_speed = -self.last_left_speed
            right_speed = -self.last_right_speed
            brush_speed = self.last_brush_speed
            # 速度限幅
            left_speed = max(min(left_speed, self.speed_pluse_max), -self.speed_pluse_max)
            right_speed = max(min(right_speed, self.speed_pluse_max), -self.speed_pluse_max)
            # 更新速度
            self.motor_driver.set_target_velocity(3, left_speed)
            self.motor_driver.set_target_velocity(2, right_speed)
            self.motor_driver.set_target_velocity(4, brush_speed)
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            self.last_brush_speed = brush_speed
            self.has_reverse_flag = True
            time.sleep(2.0)
        else:
            # 精细矫正
            self.flag = -1 if self.imu_yaw >= 0 else 1
            speed = int(-self.base_speed * RATE * (0.8 if abs(self.imu_yaw) > 1 else 0.6) * self.flag)
            left_speed = right_speed = speed
            # 速度限幅
            left_speed = max(min(left_speed, self.speed_pluse_max), -self.speed_pluse_max)
            right_speed = max(min(right_speed, self.speed_pluse_max), -self.speed_pluse_max)
            # 更新速度
            self.motor_driver.set_target_velocity(3, left_speed)
            self.motor_driver.set_target_velocity(2, right_speed)
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            # 矫正完成
            if abs(self.imu_yaw) < 0.2:
                if self.prev_motion_state:
                    self.set_state(self.prev_motion_state)
                self.has_reverse_flag = False

    def _execute_stop_state(self):
        """执行UPSTOP/LOWSTOP状态"""
        if self.current_status == "UPSTOP":
            left_speed = 0
            right_speed = self.last_right_speed
        else:
            left_speed = self.last_left_speed
            right_speed = 0
        brush_speed = self.last_brush_speed
        # 更新速度
        self.motor_driver.set_target_velocity(3, left_speed)
        self.motor_driver.set_target_velocity(2, right_speed)
        self.motor_driver.set_target_velocity(4, brush_speed)
        self.last_left_speed = left_speed
        self.last_right_speed = right_speed
        self.last_brush_speed = brush_speed

    def _execute_stop_all(self):
        """执行STOP状态（停止所有电机）"""
        if (self.last_left_speed != 0 or self.last_right_speed != 0 or self.last_brush_speed != 0):
            self.has_reverse_counter = 0
            for motor_id in [2,3,4]:
                self.motor_driver.set_target_velocity(motor_id, 0)
                self.motor_driver.disable_drive(motor_id)
            self.last_left_speed = 0
            self.last_right_speed = 0
            self.last_brush_speed = 0

    def _execute_unloading_state(self):
        """执行UNLOADING状态"""
        correction = self.pid_correction(self.imu_yaw)
        config = self.status_config["UNLOADING"]
        left_speed = int(config["velocity_up"] + correction)
        right_speed = int(config["velocity_low"] + correction)
        brush_speed = config["velocity_brush"]
        # 更新速度
        self.motor_driver.set_target_velocity(3, left_speed)
        self.motor_driver.set_target_velocity(2, right_speed)
        self.motor_driver.set_target_velocity(4, brush_speed)
        self.last_left_speed = left_speed
        self.last_right_speed = right_speed
        self.last_brush_speed = brush_speed

    def _execute_loading_pause_state(self):
        """执行LOADING/PAUSE状态"""
        config = self.status_config[self.current_status]
        left_speed = config["velocity_up"]
        right_speed = config["velocity_low"]
        brush_speed = config["velocity_brush"]
        # 更新速度
        self.motor_driver.set_target_velocity(3, left_speed)
        self.motor_driver.set_target_velocity(2, right_speed)
        self.motor_driver.set_target_velocity(4, brush_speed)
        self.last_left_speed = left_speed
        self.last_right_speed = right_speed
        self.last_brush_speed = brush_speed

    # -------------------------- 电机配置 --------------------------
    def _configure_motors_from_config(self, config_file="/home/orangepi/demo01/src/motor_can/config/servo_config.yaml"):
        """从配置文件加载电机配置"""
        config = self.load_config(config_file)
        for motor in config.get("motors", []):
            motor_id = motor.get("id")
            velocity = motor.get("velocity")
            acceleration = motor.get("acceleration")
            deceleration = motor.get("deceleration")
            if None in (motor_id, velocity, acceleration, deceleration):
                rospy.logwarn(f"⚠️ 跳过无效配置: {motor}")
                continue
            try:
                self.configure_motor(
                    motor_id=motor_id,
                    velocity=int(velocity * RATE),
                    acceleration=int(acceleration * RATE),
                    deceleration=int(deceleration * RATE)
                )
            except Exception as e:
                rospy.logerr(f"❌ 配置电机{motor_id}失败: {e}")

    def configure_motor(self, motor_id, velocity, acceleration, deceleration):
        """配置电机（速度+初始化）"""
        fault_code = self.motor_driver.read_fault_code(motor_id)
        if fault_code and fault_code != 0:
            self.motor_driver.clear_fault(motor_id)
            time.sleep(0.3)
        rospy.loginfo(f"⚙️ 配置电机{motor_id}: 速度={velocity/RATE} RPM")
        self.motor_driver.start_motor(motor_id)
        self.motor_driver.set_velocity_mode(motor_id)
        self.motor_driver.set_target_velocity(motor_id, velocity)
        self.motor_driver.enable_drive(motor_id)
        self.motor_driver.start_heartbeat(motor_id)

    # -------------------------- 状态发布 --------------------------
    def publish_state(self):
        """发布机器人状态"""
        try:
            self.velocity_publish_count += 1
            if self.velocity_publish_count >= self.velocity_publish_interval:
                self.last_velocity_up = self.motor_driver.get_actual_velocity(3)
                self.last_velocity_low = self.motor_driver.get_actual_velocity(2)
                self.last_velocity_brush = self.motor_driver.get_actual_velocity(4)
                self.velocity_publish_count = 0
            # 构建状态消息
            state_msg = {
                "status": self.current_status,
                # "battery": self.battery_remaining,
                # "battery_temperatures": self.battery_temperatures,
                # "battery_total_voltage": self.battery_total_voltage,
                # "battery_current": self.battery_current,
                "progress": self.progress,
                "imu_yaw": round(self.imu_yaw, 2) if self.imu_yaw else 0.00,
                "velocity_up": round(self.last_velocity_up * 20/24, 2),
                "velocity_low": round(self.last_velocity_low * 20/24, 2),
                "velocity_brush": round(self.last_velocity_brush, 2),
                "sensors_status": self.sensors_status,
                "device_status": {
                    "main_board": self.main_board,
                    "imu_sensor": self.imu_sensor,
                    "motor_driver": self.motor_driver.motor_driver_status,
                    "comm_module": True
                },
                "complete_state": self.complete_state,
                # "auto_mode": self.auto_mode,
                # "relay_status": self.relay_status,
                # "wind_speed": self.wind_speed,
                # "wind_direction": self.wind_direction,
                # "illuminance": self.illuminance,
                # "rainfall": self.rainfall,
                "timestamp": time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
            }
            self.state_pub.publish(json.dumps(state_msg))
        except Exception as e:
            rospy.logerr(f"❌ 状态发布失败: {e}")
            self.state_pub.publish(json.dumps({"status": "ERROR", "error": str(e)}))

    # -------------------------- 工具方法 --------------------------
    def lock_motor(self):
        """锁止电机"""
        rospy.loginfo("🔒 电机锁止")
        self.motor_cmd_pub.publish(Int8(data=-1))
        self.initial_yaw = None

    def delayed_publish_freq_switch(self, delay_sec=3):
        """延迟切换发布频率"""
        time.sleep(delay_sec)
        if self.current_status == "STOP":
            self.publish_timer.shutdown()
            self.publish_timer = rospy.Timer(rospy.Duration(1800), lambda event: self.publish_state())

    @staticmethod
    def load_config(config_file="/home/orangepi/demo01/src/motor_can/config/servo_config.yaml"):
        """加载配置文件"""
        try:
            with open(config_file, 'r') as file:
                return yaml.safe_load(file)
        except FileNotFoundError:
            rospy.logerr(f"❌ 配置文件未找到: {config_file}")
            return {}
        except Exception as e:
            rospy.logerr(f"❌ 加载配置失败: {e}")
            return {}

    def update_status_by_key(self, key):
        """按键更新状态"""
        key_mapping = {
            's': "STOP", 'f': "FORWARD", 'b': "BACKWARD", 'a': "START",
            'r': "REVERSE", 'l': "LOADING", 'p': "PAUSE", 'u': "UNLOADING",
            '1': "UPSTOP", '2': "LOWSTOP"
        }
        if key in key_mapping:
            self.auto_mode = (key == 'a')
            self.set_state(key_mapping[key])
        else:
            rospy.loginfo(f"⚠️ 无效按键: {key}")

    @staticmethod
    def keyboard_listener(controller):
        """键盘监听"""
        import select
        import sys
        rospy.loginfo("⌨️  按键控制：s=停止, f=前进, b=后退, a=启动, r=反转, l=进仓, p=暂停, u=出仓, 1=上停, 2=下停")
        while not rospy.is_shutdown():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.readline().strip()
                if key:
                    controller.update_status_by_key(key)

    # -------------------------- 安全关闭 --------------------------
    def shutdown(self):
        """关闭控制器"""
        rospy.loginfo("🔌 关闭机器人控制器...")
        # 停止定时器
        self.publish_timer.shutdown()
        # 停止电机驱动
        self.motor_driver.shutdown()
        rospy.loginfo("✅ 控制器关闭完成")

# 单独测试入口（需先启动ROS核心）
if __name__ == "__main__":
    # 初始化测试驱动
    from motor_driver import CanMotorDriver
    test_driver = CanMotorDriver()
    # 初始化控制器
    controller = RobotController(motor_driver=test_driver)
    try:
        # 启动键盘监听
        t = threading.Thread(target=RobotController.keyboard_listener, args=(controller,), daemon=True)
        t.start()
        # 启动状态执行
        rospy.Timer(rospy.Duration(0.05), controller.execute_state)
        controller.set_state("STOP")
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("🛑 用户终止")
    finally:
        controller.shutdown()