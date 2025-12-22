#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rospy
import json
import yaml
import threading
from std_msgs.msg import String, Int8, Bool
from serial_comms.msg import Sensors, INSPVAE, BatteryStatus
from motor_can import CanMotorDriver  # 导入CAN电机驱动

# 全局常量
RATE = 68  

class RobotController:
    def __init__(self):
        # ROS初始化
        rospy.init_node('robot_controller_node', anonymous=True)
        self.rate = rospy.Rate(20)
        # 初始化CAN电机驱动
        self.motor_driver = CanMotorDriver(channel='can0', interface='socketcan')
        # 状态变量
        self.last_left_speed = 0
        self.last_right_speed = 0
        self.last_brush_speed = 0
        self.has_reverse_flag = False
        self.has_reverse_counter = 0
        self.reverse_start_time = None
        self.main_board = True
        self.imu_sensor = True
        self.motor_base = 600
        self.base_speed = 30000
        self.brush_speed = 1600
        self.flag = 0
        self.speed_pluse_max = 47600
        self.reversed_start_time = None
        self.REVERSE_TIME_THRESHOLD = 3.0
        self.unloading_timer = 0.1
        self.unloading_start_time = None
        self.start_time = 0
        self.elevator_stage = 0
        self.elevator_start_time = 0
        self.LOW_BATTERY_THRESHOLD = 40
        self.velocity_publish_count = 0
        self.velocity_publish_interval = 3
        self.last_velocity_up = 0
        self.last_velocity_low = 0
        self.last_velocity_brush = 0
        self.last_switch_time = 0
        self.SWITCH_DELAY = 5
        self.PROXIMITY_ENABLE_DELAY = 5.0
        self.startup_time = None
        self.state_change_protect_delay = 5.0
        self.last_state_change_time = 0.0
        self.GLOBAL_REPEAT_DELAY = 3.0
        self.last_critical_switch_time = 0.0
        # 状态配置
        self.status_list = [
            "STOP", "FORWARD", "BACKWARD", "START", "LOADING", "UNLOADING",
            "UPSTOP", "LOWSTOP", "PISTON_OUT", "PISTON_IN", "CHARGE_OUT", "RETURN_DOCK"
        ]
        self.status_config = {
            "START": {},
            "STOP": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": 0},
            "FORWARD": {
                "velocity_up": self.motor_base * RATE,
                "velocity_low": -self.motor_base * RATE,
                "velocity_brush": -self.brush_speed * RATE
            },
            "BACKWARD": {
                "velocity_up": -self.motor_base * RATE,
                "velocity_low": self.motor_base * RATE,
                "velocity_brush": -self.brush_speed * RATE
            },
            "LOADING": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": -self.brush_speed * RATE},
            "UNLOADING": {
                "velocity_up": -self.motor_base * RATE,
                "velocity_low": self.motor_base * RATE,
                "velocity_brush": -self.brush_speed * RATE
            },
            "UPSTOP": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": 0},
            "LOWSTOP": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": 0},
            "REVERSE": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": 0},
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
        # 同步锁与传感器标记
        self.state_switch_lock = threading.Lock()
        self.sensor_triggered = {"a": False, "b": False}
        # PID参数
        self.pid_integral = 0.0
        self.pid_last_error = 0.0
        self.target_yaw = 0.0
        self.pid_kp = 100
        self.pid_ki = 0.0
        self.pid_kd = 10
        self.pid_correction_max = 200
        self.progress = 0
        # 电池与继电器状态
        self.battery_total_voltage = None
        self.battery_current = None
        self.battery_remaining = None
        self.battery_temperatures = []
        self.relay_status = None
        self.relay_auto_off = None
        # ROS发布订阅
        self.state_pub = rospy.Publisher('/robot_state', String, queue_size=10)
        self.motor_cmd_pub = rospy.Publisher('/motor_cmd', Int8, queue_size=10)
        rospy.Subscriber('/robot_cmd', String, self.status_callback)
        rospy.Subscriber('/inspvae_data', INSPVAE, self.imu_callback)
        rospy.Subscriber('/battery_status', BatteryStatus, self.battery_status_callback)
        rospy.Subscriber('/relay_status', Bool, self.relay_callback)
        rospy.Subscriber('/relay_auto_off', Bool, self.relay_auto_off_callback)
        rospy.Subscriber("proximity_sensor_data", Sensors, self.proximity_callback)
        # 定时器
        self.publish_timer = rospy.Timer(rospy.Duration(1.0), lambda event: self.publish_state())
        self.fault_check_timer = rospy.Timer(rospy.Duration(60.0), lambda event: self.check_and_clear_faults())

    # -------------------------- 状态管理 --------------------------
    def set_state(self, new_state):
        """设置机器人状态（含合法性校验）"""
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
            self.fault_check_timer.shutdown()
            self.fault_check_timer = rospy.Timer(rospy.Duration(60.0), lambda event: self.check_and_clear_faults())
            threading.Thread(target=self.delayed_publish_freq_switch, args=(1,), daemon=True).start()
        else:
            self.publish_timer.shutdown()
            self.publish_timer = rospy.Timer(rospy.Duration(1.0), lambda event: self.publish_state())
            self.fault_check_timer.shutdown()
            self.fault_check_timer = rospy.Timer(rospy.Duration(60.0), lambda event: self.check_and_clear_faults())
        # 运动状态记录
        if new_state in ["FORWARD", "BACKWARD"] and self.current_status != "REVERSE":
            self.prev_motion_state = new_state
        elif new_state == "REVERSE" and self.prev_motion_state is None:
            self.prev_motion_state = self.last_state
        elif new_state == "PISTON_IN":
            self.initial_yaw = None
            self.auto_step = None
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
        """机器人指令回调（/robot_cmd）"""
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
            rospy.logwarn(f"⚠️ 指令解析失败: {msg.data}，错误: {e}")
            self.set_state(msg.data)
            self.publish_state()

    def imu_callback(self, msg):
        """IMU数据回调（/inspvae_data）"""
        try:
            self.imu_yaw = msg.yaw if hasattr(msg, "yaw") else 0.0
            if self.initial_yaw is None:
                self.initial_yaw = self.imu_yaw
                rospy.loginfo(f"🧭 初始IMU偏航角: {self.initial_yaw}°")
            # 计算相对偏航角
            relative_yaw = self.imu_yaw - self.initial_yaw
            if relative_yaw > 180:
                relative_yaw -= 360
            elif relative_yaw < -180:
                relative_yaw += 360
            self.imu_yaw = relative_yaw
        except Exception as e:
            rospy.logerr(f"❌ IMU数据解析失败: {e}")

    def battery_status_callback(self, msg):
        """电池状态回调（/battery_status）"""
        self.battery_remaining = msg.batttery_remaining
        self.battery_total_voltage = round(msg.total_voltage, 2)
        self.battery_current = round(msg.current, 2)
        self.battery_temperatures = [round(t, 1) for t in msg.temperatures] if hasattr(msg, "temperatures") else []

    def relay_callback(self, msg):
        """继电器状态回调（/relay_status）"""
        self.relay_status = msg.data

    def relay_auto_off_callback(self, msg):
        """自动关继电器回调（/relay_auto_off）"""
        self.relay_auto_off = msg.data

    def proximity_callback(self, msg):
        """接近传感器回调（proximity_sensor_data）"""
        # 更新传感器状态
        if msg.sensor_a:
            self.sensors_status |= 0x01
        else:
            self.sensors_status &= ~0x01
        if msg.sensor_b:
            self.sensors_status |= 0x02
        else:
            self.sensors_status &= ~0x02
        # 传感器消抖
        sensor_a_trigger = msg.sensor_a and not self.sensor_triggered["a"]
        sensor_b_trigger = msg.sensor_b and not self.sensor_triggered["b"]
        sensor_both_trigger = msg.sensor_a and msg.sensor_b
        # 特殊状态逻辑
        self._handle_special_state_sensor(msg)
        # REVERSE状态边界检测
        if self.current_status == "REVERSE":
            if msg.sensor_a or msg.sensor_b:
                rospy.logwarn("⚠️ 边界触发，切换为STOP")
                self.set_state("STOP")
            return
        # 运动状态与停止状态处理
        self._handle_motion_state(msg, sensor_a_trigger, sensor_b_trigger, sensor_both_trigger)
        self._handle_stop_states(msg)

    # -------------------------- 传感器状态处理 --------------------------
    def _handle_special_state_sensor(self, msg):
        """特殊状态（RETURN_DOCK/CHARGE_OUT/START）的传感器逻辑"""
        if self.auto_mode and self.current_status == "RETURN_DOCK" and self.elevator_stage == 2:
            self.set_state("FORWARD")
        if self.auto_mode and self.current_status == "CHARGE_OUT" and self.elevator_stage == 2:
            if (msg.sensor_a or msg.sensor_b) and self.current_status != "UNLOADING":
                self.set_state("UNLOADING")
                self.progress = 10
                self.unloading_start_time = time.time()
            if self.current_status == "UNLOADING" and self.unloading_start_time:
                if time.time() - self.unloading_start_time >= self.unloading_timer:
                    self.set_state("STOP")
                    self.progress = 0
                    self.unloading_start_time = None
        if self.auto_mode and self.current_status == "START" and self.elevator_stage == 2:
            self.startup_time = time.time()
            self.set_state("BACKWARD")
            self.progress = 20

    def _handle_motion_state(self, msg, sensor_a_trigger, sensor_b_trigger, sensor_both_trigger):
        """FORWARD/BACKWARD状态的传感器逻辑"""
        with self.state_switch_lock:
            if self.is_global_repeat_protected():
                return
            # FORWARD状态
            if self.current_status == "FORWARD":
                if sensor_both_trigger:
                    self._complete_motion_and_reverse("FORWARD")
                    self.last_critical_switch_time = time.time()
                elif sensor_a_trigger and not msg.sensor_b:
                    self._handle_single_sensor_trigger("UPSTOP", "a")
                elif sensor_b_trigger and not msg.sensor_a:
                    self._handle_single_sensor_trigger("LOWSTOP", "b")
            # BACKWARD状态
            elif self.current_status == "BACKWARD":
                if sensor_both_trigger:
                    self._complete_motion_and_reverse("BACKWARD")
                    self.last_critical_switch_time = time.time()
                elif sensor_a_trigger and not msg.sensor_b:
                    self.set_state("UPSTOP")
                    self.sensor_triggered["a"] = True
                elif sensor_b_trigger and not msg.sensor_a:
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
        """UPSTOP/LOWSTOP状态的传感器逻辑"""
        with self.state_switch_lock:
            if self.current_status == "LOWSTOP" and msg.sensor_a and not self.sensor_triggered["a"]:
                self._switch_from_stop_state("LOWSTOP")
            if self.current_status == "UPSTOP" and msg.sensor_b and not self.sensor_triggered["b"]:
                self._switch_from_stop_state("UPSTOP")

    # -------------------------- 状态切换辅助 --------------------------
    def is_global_repeat_protected(self):
        """判断是否在全局防重复保护期内"""
        if time.time() - self.last_critical_switch_time < self.GLOBAL_REPEAT_DELAY:
            rospy.logwarn(f"⚠️ 3秒保护期内，剩余{self.GLOBAL_REPEAT_DELAY - (time.time() - self.last_critical_switch_time):.1f}秒")
            return True
        return False

    def _complete_motion_and_reverse(self, current_motion):
        """完成运动并反向切换"""
        current_time = time.time()
        if self.is_global_repeat_protected() or current_time - self.last_switch_time < self.SWITCH_DELAY:
            return
        rospy.loginfo(f"🔄 {current_motion}状态双侧传感器触发，反向切换")
        self.set_state("LOADING")
        time.sleep(0.5)
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
        if self.is_global_repeat_protected():
            return
        rospy.loginfo(f"🔄 {stop_state}状态对侧传感器触发，反向切换")
        self.initial_yaw = None
        self.set_state("LOADING")
        time.sleep(2)
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
    def execute_state(self, event=None):
        """执行当前状态对应的控制逻辑"""
        # 启动类状态
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
        # 其他状态（UNLOADING/LOADING）
        elif self.current_status == "UNLOADING":
            self._execute_unloading_state()
        elif self.current_status == "LOADING":
            self._execute_loading_state()

    def _execute_start_state(self):
        """执行启动类状态逻辑"""
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
                rospy.loginfo("⚙️ 配置电机速度模式...")
                self._configure_motors_from_config()
                rospy.loginfo("✅ 速度模式初始化完成")
                self.enable_drive_flag = False
                self.elevator_stage = 2
                # 自动模式切换到目标状态
                if self.auto_mode and self.auto_step and self.current_status == "START":
                    self.set_state(self.auto_step)

    def _execute_motion_state(self):
        """执行FORWARD/BACKWARD运动状态"""
        correction = self.pid_correction(self.imu_yaw)
        config = self.status_config[self.current_status]
        left_speed = int(config["velocity_up"] + correction * RATE)
        right_speed = int(config["velocity_low"] + correction * RATE)
        brush_speed = config["velocity_brush"]
        # 速度限幅
        right_speed = max(min(right_speed, self.speed_pluse_max), -self.speed_pluse_max)
        left_speed = max(min(left_speed, self.speed_pluse_max), -self.speed_pluse_max)
        # 更新电机速度
        if (self.last_left_speed != left_speed or self.last_right_speed != right_speed or self.last_brush_speed != brush_speed):
            self.motor_driver.set_target_velocity(3, left_speed)
            self.motor_driver.set_target_velocity(2, right_speed)
            self.motor_driver.set_target_velocity(4, brush_speed)
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            self.last_brush_speed = brush_speed
        # 角度偏差检测
        angle_condition_met = (-7 < self.imu_yaw < -3.5 or 3.5 < self.imu_yaw < 7)
        if angle_condition_met:
            if self.reversed_start_time is None:
                self.reversed_start_time = rospy.get_time()
                rospy.logwarn(f"⚠️ 角度偏差: {self.imu_yaw:.2f}°，开始计时")
            elif rospy.get_time() - self.reversed_start_time >= self.REVERSE_TIME_THRESHOLD:
                rospy.logwarn(f"⚠️ 偏差持续{self.REVERSE_TIME_THRESHOLD}秒，进入REVERSE状态")
                self.set_state("REVERSE")
                self.reversed_start_time = None
        else:
            if self.reversed_start_time is not None:
                rospy.loginfo(f"✅ 角度偏差消失({self.imu_yaw:.2f}°)，重置计时器")
                self.reversed_start_time = None

    def _execute_reverse_state(self):
        """执行REVERSE角度矫正状态"""
        self.reversed_start_time = None
        if not self.has_reverse_flag:
            self.has_reverse_counter += 1
            if self.has_reverse_counter > 10:
                rospy.logwarn("⚠️ 连续矫正10次，切换为STOP")
                self.has_reverse_counter = 0
                self.set_state("STOP")
                return
            # 反向速度
            right_speed = -int(self.last_right_speed)
            left_speed = -int(self.last_left_speed)
            brush_speed = self.last_brush_speed
            # 速度限幅
            right_speed = max(min(right_speed, self.speed_pluse_max), -self.speed_pluse_max)
            left_speed = max(min(left_speed, self.speed_pluse_max), -self.speed_pluse_max)
            # 更新速度
            if (self.last_left_speed != left_speed or self.last_right_speed != right_speed or self.last_brush_speed != brush_speed):
                self.motor_driver.set_target_velocity(3, left_speed)
                self.motor_driver.set_target_velocity(2, right_speed)
                self.motor_driver.set_target_velocity(4, brush_speed)
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            self.last_brush_speed = brush_speed
            self.has_reverse_flag = True
            self.reverse_start_time = time.time()
            time.sleep(2.0)
        else:
            # 持续矫正
            self.flag = -1 if self.imu_yaw >= 0 else 1
            if abs(self.imu_yaw) > 1:
                speed = int(-self.base_speed * 0.8 * self.flag)
            else:
                speed = int(-self.base_speed * 0.6 * self.flag)
            right_speed = left_speed = speed
            # 速度限幅
            right_speed = max(min(right_speed, self.speed_pluse_max), -self.speed_pluse_max)
            left_speed = max(min(left_speed, self.speed_pluse_max), -self.speed_pluse_max)
            # 更新速度
            if (self.last_left_speed != left_speed or self.last_right_speed != right_speed):
                self.motor_driver.set_target_velocity(3, left_speed)
                self.motor_driver.set_target_velocity(2, right_speed)
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            # 矫正完成
            if abs(self.imu_yaw) < 0.05:
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
            brush_speed = self.last_brush_speed
        else:  # LOWSTOP
            left_speed = self.last_left_speed
            right_speed = 0
            brush_speed = self.last_brush_speed
        # 更新速度
        if (self.last_left_speed != left_speed or self.last_right_speed != right_speed or self.last_brush_speed != brush_speed):
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
            self.motor_driver.set_target_velocity(2, 0)
            self.motor_driver.set_target_velocity(3, 0)
            self.motor_driver.set_target_velocity(4, 0)
            self.motor_driver.disable_drive(2)
            self.motor_driver.disable_drive(3)
            self.motor_driver.disable_drive(4)
            self.last_left_speed = 0
            self.last_right_speed = 0
            self.last_brush_speed = 0

    def _execute_unloading_state(self):
        """执行UNLOADING状态"""
        correction = self.pid_correction(self.imu_yaw)
        config = self.status_config["UNLOADING"]
        left_speed = int(config["velocity_up"] + correction * RATE)
        right_speed = int(config["velocity_low"] + correction * RATE)
        brush_speed = config["velocity_brush"]
        # 更新速度
        if (self.last_left_speed != left_speed or self.last_right_speed != right_speed or self.last_brush_speed != brush_speed):
            self.motor_driver.set_target_velocity(3, left_speed)
            self.motor_driver.set_target_velocity(2, right_speed)
            self.motor_driver.set_target_velocity(4, brush_speed)
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            self.last_brush_speed = brush_speed

    def _execute_loading_state(self):
        """执行LOADING状态"""
        config = self.status_config["LOADING"]
        left_speed = int(config["velocity_up"])
        right_speed = int(config["velocity_low"])
        brush_speed = config["velocity_brush"]
        # 更新速度
        if (self.last_left_speed != left_speed or self.last_right_speed != right_speed or self.last_brush_speed != brush_speed):
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
        """配置电机（速度/加速度/减速度）"""
        fault_code = self.motor_driver.read_fault_code(motor_id)
        if fault_code and fault_code != 0:
            rospy.logwarn(f"⚠️ 电机{motor_id}存在故障，尝试清除...")
            self.motor_driver.clear_fault(motor_id)
            time.sleep(0.3)
        rospy.loginfo(f"⚙️ 配置电机{motor_id}: 速度={int(velocity/RATE)}, 加速度={acceleration}, 减速度={deceleration}")
        self.motor_driver.start_motor(motor_id)
        self.motor_driver.set_velocity_mode(motor_id)
        self.motor_driver.set_target_velocity(motor_id, velocity)
        self.motor_driver.set_acceleration(motor_id, acceleration)
        self.motor_driver.set_deceleration(motor_id, deceleration)
        self.motor_driver.enable_drive(motor_id)

    # -------------------------- 故障检查 --------------------------
    def check_and_clear_faults(self):
        """定期检查并清除电机故障"""
        for motor_id in [2, 3, 4]:
            fault_code = self.motor_driver.read_fault_code(motor_id)
            actual_velocity = self.motor_driver.get_actual_velocity(motor_id)
            actual_current = self.motor_driver.get_actual_current(motor_id)
            # 故障处理
            if fault_code and fault_code != 0:
                rospy.logerr(f"❌ 电机{motor_id}故障码: 0x{fault_code:04X}")
                actual_torque = self.motor_driver.get_actual_torque(motor_id)
                if actual_torque:
                    rospy.loginfo(f"故障时转矩: {actual_torque/1000}额定转矩")
                if actual_velocity:
                    rospy.loginfo(f"故障时速度: {actual_velocity/RATE/20} RPM")
                self.set_state("STOP")
                continue
            # 转矩利用率检查
            actual_torque = self.motor_driver.get_actual_torque(motor_id)
            max_torque = self.motor_driver.get_max_torque(motor_id)
            if max_torque and actual_torque:
                utilization = abs(actual_torque) / max_torque * 100
                rospy.loginfo(f"电机{motor_id}：转矩利用率={utilization:.1f}%")
                if utilization > 80:
                    rospy.logwarn(f"⚠️ 电机{motor_id}转矩利用率过高")

    # -------------------------- 状态发布 --------------------------
    def publish_state(self):
        """发布机器人状态到/robot_state"""
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
                "battery": self.battery_remaining,
                "battery_temperatures": self.battery_temperatures,
                "battery_total_voltage": self.battery_total_voltage,
                "battery_current": self.battery_current,
                "progress": self.progress,
                "imu_yaw": round(self.imu_yaw, 2) if self.imu_yaw else 0.00,
                "velocity_up": round(self.last_velocity_up / RATE, 2),
                "velocity_low": round(self.last_velocity_low / RATE, 2),
                "velocity_brush": round(self.last_velocity_brush / RATE, 2),
                "sensors_status": self.sensors_status,
                "device_status": {
                    "main_board": self.main_board,
                    "imu_sensor": self.imu_sensor,
                    "motor_driver": self.motor_driver.motor_driver_status,
                    "comm_module": True
                },
                "complete_state": self.complete_state,
                "auto_mode": self.auto_mode,
                "relay_status": self.relay_status,
                "relay_auto_off": self.relay_auto_off,
                "timestamp": time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
            }
            self.state_pub.publish(json.dumps(state_msg))
        except Exception as e:
            rospy.logerr(f"❌ 状态发布失败: {e}")
            self.state_pub.publish(json.dumps({"status": "ERROR", "error": str(e)}))

    def delayed_publish_freq_switch(self, delay_sec=3):
        """延迟切换发布频率（STOP状态专用）"""
        time.sleep(delay_sec)
        if self.current_status == "STOP":
            self.publish_timer.shutdown()
            self.publish_timer = rospy.Timer(rospy.Duration(1800), lambda event: self.publish_state())
            self.fault_check_timer.shutdown()
            self.fault_check_timer = rospy.Timer(rospy.Duration(7200), lambda event: self.check_and_clear_faults())

    # -------------------------- 工具方法 --------------------------
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
        rospy.loginfo(f"⌨️  接收到按键: {key}")
        key_mapping = {
            's': "STOP", 'f': "FORWARD", 'b': "BACKWARD", 'a': "START",
            'r': "REVERSE", 'l': "LOADING", 'u': "UNLOADING", '1': "UPSTOP", '2': "LOWSTOP"
        }
        if key in key_mapping:
            if key != 'a':
                self.auto_mode = False
            else:
                self.auto_mode = True
            self.set_state(key_mapping[key])
        else:
            rospy.loginfo(f"⚠️ 无效按键: {key}")

    @staticmethod
    def keyboard_listener(controller):
        """键盘监听（静态方法）"""
        import select
        import sys
        rospy.loginfo("⌨️  按键控制：s=停止, f=前进, b=后退, a=启动, r=反转, l=进仓, u=出仓, 1=上停, 2=下停")
        while not rospy.is_shutdown():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.readline().strip()
                if key:
                    controller.update_status_by_key(key)

    # -------------------------- 安全关闭 --------------------------
    def shutdown(self):
        """安全关闭控制器"""
        rospy.loginfo("🔌 关闭机器人控制器...")
        # 停止定时器
        self.publish_timer.shutdown()
        self.fault_check_timer.shutdown()
        # 停止电机驱动
        self.motor_driver.shutdown()
        rospy.loginfo("✅ 机器人控制器已关闭")