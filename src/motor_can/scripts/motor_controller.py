#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rospy
import json
import yaml
import threading
from std_msgs.msg import String, Int8, Bool
from serial_comms.msg import INSPVAE, BatteryStatus, Sensors
from std_srvs.srv import Trigger
from motor_driver import MotorDriver  # 导入电机驱动类

class MotorController:
    def __init__(self):
        # ROS初始化
        rospy.init_node('motor_controller_node', anonymous=True)
        self.rate = rospy.Rate(20)
        # 初始化电机驱动
        self.motor_driver = MotorDriver()
        # 状态变量
        self.last_left_speed = 0
        self.last_right_speed = 0
        self.last_brush_speed = 0
        self.has_reverse_flag = False
        self.has_reverse_counter = 0
        self.reverse_start_time = None
        self.main_board = True
        self.imu_sensor = True
        self.motor_base = 1000
        self.brush_base_speed = 1800
        self.brush_forward = rospy.get_param('~brush_forward', False)
        self.flag = 0
        self.speed_pluse_max = 1500 * 1
        self.reversed_start_time = None
        self.REVERSE_TIME_THRESHOLD = 2.0
        self.unloading_timer = 0.1
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
        self.heartbeat_running = False
        self.heartbeat_thread = None
        self.last_sensor_b = False
        self.sensor_b_count = 0
        self.last_sensor_time = 0
        self.last_switch_time = 0
        self.SWITCH_DELAY = 5
        self.PROXIMITY_ENABLE_DELAY = 5.0
        self.startup_time = None
        self.state_change_protect_delay = 5.0
        self.last_state_change_time = 0.0
        self.GLOBAL_REPEAT_DELAY = 3.5
        self.last_critical_switch_time = 0.0
        # 状态配置
        self.status_list = [
            "STOP", "FORWARD", "BACKWARD", "START", "LOADING", "UNLOADING",
            "UPSTOP", "LOWSTOP", "PISTON_OUT", "PISTON_IN", "CHARGE_OUT",
            "RETURN_DOCK", "PAUSE"
        ]
        self.status_config = {
            "START": {},
            "STOP": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": 0},
            "UNLOADING": {"velocity_up": -self.motor_base, "velocity_low": self.motor_base, "velocity_brush": -self.brush_base_speed},
            "FORWARD": {"velocity_up": self.motor_base, "velocity_low": -self.motor_base, "velocity_brush": -self.brush_base_speed},
            "BACKWARD": {"velocity_up": -self.motor_base, "velocity_low": self.motor_base, "velocity_brush": self.brush_base_speed},
            "LOADING": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": 0},
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
        self.pid_kd = 0.5
        self.pid_correction_max = 500
        self.progress = 0
        # 电池相关
        self.battery_total_voltage = None
        self.battery_current = None
        self.battery_remaining = None
        self.battery_temperatures = []
        self.relay_status = None
        # ROS发布订阅
        self.state_pub = rospy.Publisher('/robot_state', String, queue_size=10)
        self.motor_cmd_pub = rospy.Publisher('/motor_cmd', Int8, queue_size=10)
        rospy.Subscriber('/robot_cmd', String, self.status_callback)
        rospy.Subscriber('/inspvae_data', INSPVAE, self.imu_callback)
        rospy.Subscriber('/battery_status', BatteryStatus, self.battery_status_callback)
        rospy.Subscriber('/relay_status', Bool, self.relay_callback)
        rospy.Subscriber("proximity_sensor_data", Sensors, self.proximity_callback)
        # 同步锁和传感器标记
        self.state_switch_lock = threading.Lock()
        self.sensor_triggered = {"a": False, "b": False}
        # 发布定时器
        self.publish_timer = rospy.Timer(rospy.Duration(1.0), self.publish_state)

    # -------------------------- 状态管理 --------------------------
    def set_state(self, new_state):
        """设置机器人状态（状态合法性校验+状态切换逻辑）"""
        if new_state not in self.status_config:
            rospy.logwarn(f"⚠️ 无效状态：{new_state}")
            return False
        if new_state == self.current_status and new_state not in ["PISTON_OUT", "PISTON_IN", "STOP"]:
            return False
        # 状态切换限制
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
            self.motor_driver.motor_driver = True
            self.imu_sensor = True
            self.main_board = True
        # STOP状态处理
        if new_state == "STOP":
            self.elevator_stage = 0
            self.stop_heartbeat()
            threading.Thread(target=self.delayed_publish_freq_switch, args=(1,), daemon=True).start()
        # 运动状态记录
        if new_state in ["FORWARD", "BACKWARD"] and self.current_status != "REVERSE":
            self.prev_motion_state = new_state
        if new_state in ["REVERSE", "UPSTOP", "LOWSTOP"] and self.prev_motion_state is None:
            self.prev_motion_state = self.last_state
        # 模式切换（PISTON_OUT触发）
        elif new_state == "PISTON_OUT":
            self.auto_mode = not self.auto_mode
            rospy.loginfo(f"🔘 {'手动模式开' if self.auto_mode else '自动模式开'}")
            self.count += 1
        # 关键状态计时
        critical_states = ["FORWARD", "BACKWARD"]
        if new_state in critical_states:
            self.last_critical_switch_time = time.time()
        # 更新状态
        self.last_state = self.current_status
        self.current_status = new_state
        self.last_state_change_time = time.time()
        rospy.loginfo(f"📌 状态已更新为: {self.current_status}")
        return True

    # -------------------------- 回调函数 --------------------------
    def status_callback(self, msg):
        """机器人指令回调（处理/robot_cmd话题）"""
        try:
            cmd_obj = json.loads(msg.data)
            command = cmd_obj.get("command", None)
            if command == "GET_STATUS":
                self.publish_state()
            elif command == "BRUSH_FORWARD":
                self.brush_forward = not self.brush_forward
            elif command in self.status_list:
                self.set_state(command)
            else:
                rospy.logwarn(f"⚠️ 无效指令：{msg.data}")
        except Exception as e:
            rospy.logwarn(f"⚠️ 指令解析失败：{msg.data}，错误：{e}")
            if msg.data == "BRUSH_FORWARD":
                self.brush_forward = not self.brush_forward
            self.set_state(msg.data)
            self.publish_state()

    def imu_callback(self, msg):
        """IMU数据回调（处理姿态角）"""
        try:
            self.imu_yaw = msg.yaw if hasattr(msg, "yaw") else 0.0
            if self.initial_yaw is None:
                self.initial_yaw = self.imu_yaw
                rospy.loginfo(f"🧭 初始IMU偏航角：{self.initial_yaw}°")
            # 计算相对偏航角
            relative_yaw = self.imu_yaw - self.initial_yaw
            if relative_yaw > 180:
                relative_yaw -= 360
            elif relative_yaw < -180:
                relative_yaw += 360
            self.imu_yaw = relative_yaw
        except Exception as e:
            rospy.logerr(f"❌ IMU数据解析失败：{e}")

    def battery_status_callback(self, msg):
        """电池状态回调"""
        self.battery_remaining = msg.batttery_remaining
        self.battery_total_voltage = round(msg.total_voltage, 2)
        self.battery_current = round(msg.current, 2)
        self.battery_temperatures = [round(t, 1) for t in msg.temperatures] if hasattr(msg, "temperatures") else []

    def relay_callback(self, msg):
        """继电器状态回调"""
        self.relay_status = msg.data

    def proximity_callback(self, msg):
        """接近传感器回调（统一处理自动/手动模式）"""
        # 更新传感器状态
        if msg.sensor_b:
            self.sensors_status |= 0x01
        else:
            self.sensors_status &= ~0x01
        if msg.sensor_a:
            self.sensors_status |= 0x02
        else:
            self.sensors_status &= ~0x02
        # 传感器消抖
        sensor_b_trigger = msg.sensor_b and not self.sensor_triggered["a"]
        sensor_a_trigger = msg.sensor_a and not self.sensor_triggered["b"]
        sensor_aoth_trigger = msg.sensor_b and msg.sensor_a
        # 特殊状态逻辑
        self._handle_special_state_sensor(msg)
        # REVERSE状态边界检测
        if self.current_status == "REVERSE":
            if msg.sensor_b or msg.sensor_a:
                rospy.logwarn("⚠️ 边界触发，切换为STOP")
                self.set_state("STOP")
            return
        # 运动状态传感器处理
        self._handle_motion_state(msg, sensor_b_trigger, sensor_a_trigger, sensor_aoth_trigger)
        # 停止状态传感器处理
        self._handle_stop_states(msg)

    # -------------------------- 传感器状态处理 --------------------------
    def _handle_special_state_sensor(self, msg):
        """特殊状态（RETURN_DOCK/CHARGE_OUT/START）的传感器逻辑"""
        # RETURN_DOCK状态
        if self.auto_mode and self.current_status == "RETURN_DOCK" and self.elevator_stage == 2:
            self.set_state("FORWARD")
        # CHARGE_OUT状态
        if self.auto_mode and self.current_status == "CHARGE_OUT" and self.elevator_stage == 2:
            if msg.sensor_b or msg.sensor_a and self.current_status != "UNLOADING":
                self.set_state("UNLOADING")
                self.progress = 10
                self.unloading_start_time = time.time()
            if self.current_status == "UNLOADING" and self.unloading_start_time:
                if time.time() - self.unloading_start_time >= self.unloading_timer:
                    self.set_state("STOP")
                    self.progress = 0
                    self.unloading_start_time = None
        # START状态
        if self.auto_mode and self.current_status == "START" and self.elevator_stage == 2:
            self.startup_time = time.time()
            self.set_state("BACKWARD")
            self.progress = 20

    def _handle_motion_state(self, msg, sensor_b_trigger, sensor_a_trigger, sensor_aoth_trigger):
        """FORWARD/BACKWARD状态的传感器逻辑"""
        with self.state_switch_lock:
            if self._is_global_repeat_protected():
                rospy.logdebug("⚠️ 状态切换保护期，忽略传感器触发")
                return
            # FORWARD状态
            if self.current_status == "FORWARD":
                if sensor_aoth_trigger:
                    self._complete_motion_and_reverse("FORWARD")
                    self.last_critical_switch_time = time.time()
                elif sensor_b_trigger and not msg.sensor_a:
                    self._handle_single_sensor_trigger("UPSTOP", "a")
                elif sensor_a_trigger and not msg.sensor_b:
                    self._handle_single_sensor_trigger("LOWSTOP", "b")
            # BACKWARD状态
            elif self.current_status == "BACKWARD":
                if sensor_aoth_trigger:
                    self._complete_motion_and_reverse("BACKWARD")
                    self.last_critical_switch_time = time.time()
                elif sensor_b_trigger and not msg.sensor_a:
                    self.set_state("UPSTOP")
                    self.sensor_triggered["a"] = True
                elif sensor_a_trigger and not msg.sensor_b:
                    self.set_state("LOWSTOP")
                    self.sensor_triggered["b"] = True

    def _handle_single_sensor_trigger(self, target_state, sensor_key):
        """处理单侧传感器触发（防频繁切换）"""
        current_time = time.time()
        if current_time - self.last_switch_time < self.SWITCH_DELAY:
            rospy.logwarn(f"⚠️ {target_state}切换间隔过短，忽略")
            return
        self.set_state(target_state)
        self.sensor_triggered[sensor_key] = True
        self.last_switch_time = current_time

    def _handle_stop_states(self, msg):
        """UPSTOP/LOWSTOP状态的传感器逻辑（等待对侧触发）"""
        with self.state_switch_lock:
            if self.current_status == "LOWSTOP" and msg.sensor_b and not self.sensor_triggered["a"]:
                self._switch_from_stop_state("LOWSTOP")
            if self.current_status == "UPSTOP" and msg.sensor_a and not self.sensor_triggered["b"]:
                self._switch_from_stop_state("UPSTOP")

    # -------------------------- 状态切换辅助 --------------------------
    def _is_global_repeat_protected(self):
        """判断是否在全局防重复保护期内"""
        return time.time() - self.last_critical_switch_time < self.GLOBAL_REPEAT_DELAY

    def _complete_motion_and_reverse(self, current_motion):
        """完成运动并反向切换状态"""
        current_time = time.time()
        if self._is_global_repeat_protected() or current_time - self.last_switch_time < self.SWITCH_DELAY:
            rospy.logwarn("⚠️ 反向切换条件不满足，忽略")
            return
        rospy.loginfo(f"🔄 {current_motion}状态双侧传感器触发，反向切换")
        self.brush_forward = not self.brush_forward
        # 状态重置
        self.complete_state = True
        self.initial_yaw = None
        self.progress = 100
        self.auto_step = None
        self.elevator_stage = 0
        # 反向切换
        target_state = "BACKWARD" if current_motion == "FORWARD" else "FORWARD"
        self.last_switch_time = current_time
        self.set_state(target_state)
        self.progress = 10
        self.last_critical_switch_time = current_time
        self.sensor_triggered = {"a": False, "b": False}

    def _switch_from_stop_state(self, stop_state):
        """从停止状态切换为反向运动"""
        if self._is_global_repeat_protected():
            return
        rospy.loginfo(f"🔄 {stop_state}状态对侧传感器触发，反向切换")
        self.initial_yaw = None
        self.brush_forward = not self.brush_forward
        # 切换到之前的运动状态
        if self.prev_motion_state == "FORWARD":
            self.set_state("BACKWARD")
        elif self.prev_motion_state == "BACKWARD":
            self.set_state("FORWARD")
        self.progress = 60
        self.sensor_triggered = {"a": False, "b": False}

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
        # 积分限幅
        self.pid_integral = max(min(self.pid_integral, 30), -30)
        # 计算矫正量
        correction = self.pid_kp * error + self.pid_ki * self.pid_integral + self.pid_kd * derivative
        self.pid_last_error = error
        return max(min(-correction, self.pid_correction_max), -self.pid_correction_max)

    # -------------------------- 状态执行 --------------------------
    def execute_state(self):
        """执行当前状态对应的电机控制逻辑"""
        if self.motor_driver.rtu_client is None:
            rospy.logwarn("⚠️ 电机驱动未连接，跳过控制执行")
            return
        # START/CHARGE_OUT/RETURN_DOCK状态
        if self.enable_drive_flag and self.current_status in ["START", "CHARGE_OUT", "RETURN_DOCK"]:
            self._execute_start_state()
        # 运动状态（FORWARD/BACKWARD）
        elif self.current_status in ["FORWARD", "BACKWARD"]:
            self._execute_motion_state()
        # REVERSE状态（角度矫正）
        elif self.current_status == "REVERSE":
            self._execute_reverse_state()
        # 停止状态（UPSTOP/LOWSTOP）
        elif self.current_status in ["UPSTOP", "LOWSTOP"]:
            self._execute_stop_state()
        # STOP状态
        elif self.current_status == "STOP" or not (-5 < self.imu_yaw < 5):
            self._execute_stop_all()
        # 其他状态（UNLOADING/LOADING/PAUSE）
        elif self.current_status == "UNLOADING":
            self._execute_unloading_state()
        elif self.current_status in ["LOADING", "PAUSE"]:
            self._execute_loading_pause_state()

    def _execute_start_state(self):
        """执行启动类状态逻辑"""
        # 低电量检查
        if self.battery_remaining is not None and self.battery_remaining < self.LOW_BATTERY_THRESHOLD:
            rospy.logerr("🔋 电量过低，停止启动")
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
                rospy.loginfo("⚙️  配置电机...")
                self._configure_motors_from_config()
                rospy.loginfo("✅ 电机配置完成")
                self.enable_drive_flag = False
                self.elevator_stage = 2
                # 自动模式切换到目标状态
                if self.auto_mode and self.auto_step and self.current_status == "START":
                    self.set_state(self.auto_step)

    def _execute_motion_state(self):
        """执行FORWARD/BACKWARD运动状态"""
        # PID矫正
        correction = self.pid_correction(self.imu_yaw)
        # 获取速度配置
        config = self.status_config[self.current_status]
        left_speed = int(config["velocity_up"] + correction)
        right_speed = int(config["velocity_low"] + correction)
        brush_speed = int(config["velocity_brush"])
        # 速度限幅
        right_speed = max(min(right_speed, self.speed_pluse_max), -self.speed_pluse_max)
        left_speed = max(min(left_speed, self.speed_pluse_max), -self.speed_pluse_max)
        # 更新电机速度
        if (self.last_left_speed != left_speed or self.last_right_speed != right_speed or self.last_brush_speed != brush_speed):
            self.motor_driver.set_target_velocity(2, left_speed)
            self.motor_driver.set_target_velocity(1, right_speed)
            self.motor_driver.set_target_velocity(3, brush_speed)
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            self.last_brush_speed = brush_speed
        # 角度偏差检测（触发REVERSE状态）
        if (-7 < self.imu_yaw < -2 or 2 < self.imu_yaw < 7):
            if self.reversed_start_time is None:
                self.reversed_start_time = rospy.get_time()
                rospy.logwarn(f"⚠️  角度偏差：{self.imu_yaw:.2f}°")
            elif rospy.get_time() - self.reversed_start_time >= self.REVERSE_TIME_THRESHOLD:
                rospy.logwarn("⚠️  角度偏差超时，进入反转矫正")
                self.set_state("REVERSE")
                self.reversed_start_time = None
        else:
            self.reversed_start_time = None

    def _execute_reverse_state(self):
        """执行REVERSE角度矫正状态"""
        self.reversed_start_time = None
        if not self.has_reverse_flag:
            # 首次进入：反向速度
            self.has_reverse_counter += 1
            if self.has_reverse_counter > 100:
                rospy.logwarn("⚠️  连续反转100次，停止运行")
                self.has_reverse_counter = 0
                self.set_state("STOP")
                self.motor_driver.motor_driver = False
                self.imu_sensor = False
                return
            right_speed = -int(self.last_right_speed)
            left_speed = -int(self.last_left_speed)
            brush_speed = self.last_brush_speed
            # 速度限幅
            right_speed = max(min(right_speed, self.speed_pluse_max), -self.speed_pluse_max)
            left_speed = max(min(left_speed, self.speed_pluse_max), -self.speed_pluse_max)
            # 更新速度
            if (self.last_left_speed != left_speed or self.last_right_speed != right_speed or self.last_brush_speed != brush_speed):
                self.motor_driver.set_target_velocity(2, left_speed)
                self.motor_driver.set_target_velocity(1, right_speed)
                self.motor_driver.set_target_velocity(3, brush_speed)
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            self.last_brush_speed = brush_speed
            self.has_reverse_flag = True
            self.reverse_start_time = time.time()
        else:
            # 持续矫正：根据角度调整速度
            self.flag = -1 if self.imu_yaw >= 0 else 1
            if abs(self.imu_yaw) > 1:
                speed = int(-self.motor_base * 0.8 * self.flag)
            else:
                speed = int(-self.motor_base * 0.6 * self.flag)
            right_speed = left_speed = speed
            # 速度限幅
            right_speed = max(min(right_speed, self.speed_pluse_max), -self.speed_pluse_max)
            left_speed = max(min(left_speed, self.speed_pluse_max), -self.speed_pluse_max)
            # 更新速度
            if (self.last_left_speed != left_speed or self.last_right_speed != right_speed or self.last_brush_speed != self.last_brush_speed):
                self.motor_driver.set_target_velocity(2, left_speed)
                self.motor_driver.set_target_velocity(1, right_speed)
                self.motor_driver.set_target_velocity(3, self.last_brush_speed)
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            # 矫正完成：返回之前状态
            if abs(self.imu_yaw) < 0.2:
                if self.prev_motion_state:
                    self.set_state(self.prev_motion_state)
                self.is_upstop = False
                self.is_lowstop = False
                self.has_reverse_flag = False

    def _execute_stop_state(self):
        """执行UPSTOP/LOWSTOP停止状态"""
        if self.current_status == "UPSTOP":
            left_speed = 0
            right_speed = self.last_right_speed
            self.motor_driver.set_target_velocity(2, left_speed)
            self.motor_driver.set_target_velocity(1, right_speed)
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
        elif self.current_status == "LOWSTOP":
            left_speed = self.last_left_speed
            right_speed = 0
            self.motor_driver.set_target_velocity(2, left_speed)
            self.motor_driver.set_target_velocity(1, right_speed)
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed

    def _execute_stop_all(self):
        """执行STOP状态（停止所有电机）"""
        if (self.last_left_speed != 0 or self.last_right_speed != 0 or self.last_brush_speed != 0):
            self.has_reverse_counter = 0
            # 禁用所有电机
            for motor_id in self.motor_driver.motor_address_map.keys():
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
        brush_speed = int(config["velocity_brush"])
        # 更新速度
        if (self.last_left_speed != left_speed or self.last_right_speed != right_speed or self.last_brush_speed != brush_speed):
            self.motor_driver.set_target_velocity(2, left_speed)
            self.motor_driver.set_target_velocity(1, right_speed)
            self.motor_driver.set_target_velocity(3, brush_speed)
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            self.last_brush_speed = brush_speed

    def _execute_loading_pause_state(self):
        """执行LOADING/PAUSE状态"""
        config = self.status_config[self.current_status]
        left_speed = int(config["velocity_up"])
        right_speed = int(config["velocity_low"])
        brush_speed = int(config["velocity_brush"])
        # 更新速度
        if (self.last_left_speed != left_speed or self.last_right_speed != right_speed or self.last_brush_speed != brush_speed):
            self.motor_driver.set_target_velocity(2, left_speed)
            self.motor_driver.set_target_velocity(1, right_speed)
            self.motor_driver.set_target_velocity(3, brush_speed)
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            self.last_brush_speed = brush_speed

    # -------------------------- 电机配置 --------------------------
    def _configure_motors_from_config(self, config_file="/home/orangepi/demo01/src/motor_can/config/servo_config.yaml"):
        """从配置文件加载电机配置"""
        try:
            with open(config_file, 'r') as file:
                config = yaml.safe_load(file)
                for motor in config.get("motors", []):
                    motor_id = motor.get("id")
                    velocity = motor.get("velocity")
                    if None in (motor_id, velocity):
                        rospy.logwarn(f"⚠️  跳过无效电机配置：{motor}")
                        continue
                    self.motor_driver.configure_motor(motor_id, int(velocity))
        except FileNotFoundError:
            rospy.logerr(f"❌ 配置文件未找到：{config_file}")
        except Exception as e:
            rospy.logerr(f"❌ 加载电机配置失败：{e}")

    # -------------------------- 心跳相关 --------------------------
    def start_heartbeat(self, motor_id):
        """启动电机心跳监测"""
        self.heartbeat_running = True
        self.heartbeat_thread = threading.Thread(
            target=self._cycle_heartbeat, args=(motor_id,), daemon=True
        )
        self.heartbeat_thread.start()
        rospy.loginfo(f"❤️ 电机{motor_id}心跳启动")

    def _cycle_heartbeat(self, motor_id, interval=1.5):
        """心跳循环（监测电机故障）"""
        while self.heartbeat_running and not rospy.is_shutdown():
            if self.motor_driver.rtu_client is None:
                time.sleep(interval)
                continue
            # 读取故障码监测状态
            self.motor_driver.read_fault_code(motor_id)
            time.sleep(interval)

    def stop_heartbeat(self):
        """停止心跳监测"""
        self.heartbeat_running = False
        if self.heartbeat_thread and self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join(timeout=2)
        rospy.loginfo("❤️ 心跳监测停止")

    # -------------------------- 状态发布 --------------------------
    def publish_state(self, event=None):
        """发布机器人状态到/robot_state话题"""
        try:
            # 构建状态消息
            state_msg = {
                "status": self.current_status,
                "battery": self.battery_remaining,
                "battery_temperatures": self.battery_temperatures,
                "battery_total_voltage": self.battery_total_voltage,
                "battery_current": self.battery_current,
                "progress": self.progress,
                "imu_yaw": round(self.imu_yaw, 2) if self.imu_yaw is not None else 0.00,
                "velocity_up": round(self.last_left_speed * 20 / 15, 2),
                "velocity_low": round(self.last_right_speed * 20 / 15, 2),
                "velocity_brush": round(self.last_brush_speed * 20 / 15, 2),
                "sensors_status": self.sensors_status,
                "device_status": {
                    "main_board": self.main_board,
                    "imu_sensor": self.imu_sensor,
                    "motor_driver": self.motor_driver.motor_driver,
                    "comm_module": self.motor_driver.rtu_client.is_socket_open() if self.motor_driver.rtu_client is not None else False
                },
                "complete_state": self.complete_state,
                "auto_mode": self.auto_mode,
                "timestamp": time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
            }
            self.state_pub.publish(json.dumps(state_msg, ensure_ascii=False))
        except Exception as e:
            rospy.logerr(f"❌ 状态发布失败：{e}")
            error_msg = {"status": "ERROR", "error": str(e)}
            self.state_pub.publish(json.dumps(error_msg))

    def delayed_publish_freq_switch(self, delay_sec=3):
        """延迟切换状态发布频率（STOP状态专用）"""
        time.sleep(delay_sec)
        if self.current_status == "STOP" and not rospy.is_shutdown():
            self.publish_timer.shutdown()
            self.publish_timer = rospy.Timer(rospy.Duration(1800), self.publish_state)

    # -------------------------- 安全关闭 --------------------------
    def shutdown(self):
        """安全关闭控制器"""
        rospy.loginfo("🔌 关闭机器人控制器...")
        # 停止心跳
        self.stop_heartbeat()
        # 停止电机驱动
        self.motor_driver.close()
        # 停止定时器
        if hasattr(self, 'publish_timer'):
            self.publish_timer.shutdown()
        rospy.loginfo("✅ 机器人控制器已关闭")