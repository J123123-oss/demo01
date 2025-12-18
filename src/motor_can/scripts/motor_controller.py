#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rospy
import json
import threading
from std_msgs.msg import String, Int8, Bool
from serial_comms.msg import INSPVAE, BatteryStatus, Sensors

# 速比配置
RATE = 1

class MotorController:
    """电机控制器：负责业务逻辑，通过驱动层操作电机"""
    def __init__(self, driver_manager):
        self.driver_manager = driver_manager  # 驱动管理器实例
        self.rate = rospy.Rate(20)

        # 基础配置参数
        self.motor_base = 2
        self.brush_base_speed = 4
        self.speed_pluse_max = 2 * RATE
        self.REVERSE_TIME_THRESHOLD = 3.0
        self.UNLOADING_TIMER = 0.1
        self.LOW_BATTERY_THRESHOLD = 40
        self.SWITCH_DELAY = 5  # 触发延时阈值（秒）
        self.GLOBAL_REPEAT_DELAY = 3.0  # 全局防重复触发时间

        # 状态变量
        self.status_list = [
            "STOP", "FORWARD", "BACKWARD", "START", "LOADING", "UNLOADING",
            "UPSTOP", "LOWSTOP", "PISTON_OUT", "PISTON_IN", "CHARGE_OUT",
            "RETURN_DOCK", "PAUSE"
        ]
        self.status_config = {
            "STOP": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": 0},
            "UNLOADING": {"velocity_up": -self.motor_base * RATE, "velocity_low": self.motor_base * RATE, "velocity_brush": -self.brush_base_speed},
            "FORWARD": {"velocity_up": self.motor_base * RATE, "velocity_low": -self.motor_base * RATE, "velocity_brush": -self.brush_base_speed},
            "BACKWARD": {"velocity_up": -self.motor_base * RATE, "velocity_low": self.motor_base * RATE, "velocity_brush": -self.brush_base_speed},
            "LOADING": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": -self.brush_base_speed},
            "PAUSE": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": -self.brush_base_speed},
            "UPSTOP": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": -self.brush_base_speed},
            "LOWSTOP": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": -self.brush_base_speed},
            "REVERSE": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": -self.brush_base_speed},
            "START": {}, "PISTON_OUT": {}, "PISTON_IN": {}, "CHARGE_OUT": {}, "RETURN_DOCK": {}
        }

        # 运行时状态
        self.current_status = self.status_list[0]
        self.last_state = None
        self.auto_mode = True
        self.auto_step = None
        self.imu_yaw = 0.0
        self.initial_yaw = None
        self.battery_remaining = None
        self.sensors_status = 0
        self.sensor_triggered = {"a": False, "b": False}
        self.last_switch_time = 0.0
        self.last_critical_switch_time = 0.0
        self.prev_motion_state = None

        # 电机速度缓存
        self.last_left_speed = 0
        self.last_right_speed = 0
        self.last_brush_speed = 0

        # PID参数
        self.pid_integral = 0.0
        self.pid_last_error = 0.0
        self.target_yaw = 0.0
        self.pid_kp = 1
        self.pid_ki = 0
        self.pid_kd = 0.1
        self.pid_correction_max = 1

        # 状态切换锁
        self.state_switch_lock = threading.Lock()

        # ROS发布订阅
        self.state_pub = rospy.Publisher('/robot_state', String, queue_size=10)
        self.motor_cmd_pub = rospy.Publisher('/motor_cmd', Int8, queue_size=10)
        rospy.Subscriber('/robot_cmd', String, self.status_callback)
        rospy.Subscriber('/inspvae_data', INSPVAE, self.imu_callback)
        rospy.Subscriber('/battery_status', BatteryStatus, self.battery_status_callback)
        rospy.Subscriber('/relay_status', Bool, self.relay_callback)
        rospy.Subscriber("proximity_sensor_data", Sensors, self.proximity_callback)

        # 发布定时器
        self.publish_timer = rospy.Timer(rospy.Duration(1.0), self.publish_state)

    def is_global_repeat_protected(self):
        """判断是否在全局防重复触发保护期内"""
        current_time = time.time()
        return current_time - self.last_critical_switch_time < self.GLOBAL_REPEAT_DELAY

    def set_state(self, new_state):
        """设置机器人状态（带合法性校验）"""
        if new_state not in self.status_config:
            rospy.logwarn(f"⚠️ 无效状态: {new_state}")
            return False
        if new_state == self.current_status and new_state not in ["PISTON_OUT", "PISTON_IN", "STOP"]:
            return False

        # 状态切换逻辑
        with self.state_switch_lock:
            # 特殊状态处理
            if new_state in ["START", "CHARGE_OUT", "RETURN_DOCK"]:
                self.complete_state = False
                self.enable_drive_flag = True
                self.progress = 0
            if new_state == "STOP":
                self.stop_all_motors()
                self.publish_timer.shutdown()
                self.publish_timer = rospy.Timer(rospy.Duration(1800), self.publish_state)
            else:
                self.publish_timer.shutdown()
                self.publish_timer = rospy.Timer(rospy.Duration(1.0), self.publish_state)

            # 关键状态记录
            critical_states = ["FORWARD", "BACKWARD"]
            if new_state in critical_states:
                self.last_critical_switch_time = time.time()
                self.prev_motion_state = new_state

            self.last_state = self.current_status
            self.current_status = new_state
            rospy.loginfo(f"📌 状态更新为: {self.current_status}")
        return True

    def stop_all_motors(self):
        """停止所有电机"""
        for motor_id in [1, 2, 3]:
            driver = self.driver_manager.get_driver(motor_id)
            if driver:
                driver.disable()
        self.last_left_speed = 0
        self.last_right_speed = 0
        self.last_brush_speed = 0

    def pid_correction(self, current_yaw):
        """PID角度矫正"""
        error = self.target_yaw - current_yaw
        if abs(error) < 0.05:
            return 0
        if abs(error) > 0.3:
            self.pid_integral = 0
        self.pid_integral += error
        self.pid_integral = max(min(self.pid_integral, 30), -30)  # 积分限幅
        derivative = error - self.pid_last_error
        correction = self.pid_kp * error + self.pid_ki * self.pid_integral + self.pid_kd * derivative
        self.pid_last_error = error
        return -max(min(-correction, self.pid_correction_max), -self.pid_correction_max)

    def execute_state(self):
        """执行当前状态对应的逻辑"""
        # 驱动未就绪时跳过
        if not any(driver.connection_status for driver in self.driver_manager.drivers.values()):
            rospy.logwarn("⚠️ 无可用电机驱动，跳过状态执行")
            return

        # 按状态执行不同逻辑
        if self.current_status in ["FORWARD", "BACKWARD"]:
            self._execute_motion_state()
        elif self.current_status == "REVERSE":
            self._execute_reverse_state()
        elif self.current_status in ["UPSTOP", "LOWSTOP"]:
            self._execute_stop_state()
        elif self.current_status == "UNLOADING":
            self._execute_unloading_state()
        elif self.current_status in ["LOADING", "PAUSE"]:
            self._execute_load_pause_state()
        elif self.current_status in ["START", "CHARGE_OUT", "RETURN_DOCK"]:
            self._execute_start_state()

    def _execute_motion_state(self):
        """执行前进/后退状态"""
        # PID角度矫正
        correction = self.pid_correction(self.imu_yaw)
        # 获取目标速度
        config = self.status_config[self.current_status]
        left_speed = int(config["velocity_up"] + correction)
        right_speed = int(config["velocity_low"] + correction)
        brush_speed = int(config["velocity_brush"])
        # 速度限幅
        left_speed = max(min(left_speed, self.speed_pluse_max), -self.speed_pluse_max)
        right_speed = max(min(right_speed, self.speed_pluse_max), -self.speed_pluse_max)
        # 速度变化时更新
        if (self.last_left_speed != left_speed or 
            self.last_right_speed != right_speed or 
            self.last_brush_speed != brush_speed):
            self.driver_manager.get_driver(2).set_speed(left_speed)  # 左电机（ID=2）
            self.driver_manager.get_driver(1).set_speed(right_speed)  # 右电机（ID=1）
            self.driver_manager.get_driver(3).set_speed(brush_speed)  # 刷电机（ID=3）
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            self.last_brush_speed = brush_speed

    def _execute_reverse_state(self):
        """执行反转矫正状态"""
        # 反转逻辑（保留原有逻辑）
        if not hasattr(self, "has_reverse_flag") or not self.has_reverse_flag:
            self.has_reverse_flag = True
            self.reverse_start_time = time.time()
            # 反向速度
            left_speed = -self.last_left_speed
            right_speed = -self.last_right_speed
            brush_speed = self.last_brush_speed
        else:
            # 按角度偏差调整速度
            self.flag = -1 if self.imu_yaw >= 0 else 1
            if abs(self.imu_yaw) > 1:
                left_speed = right_speed = int(-self.motor_base * RATE * 0.8 * self.flag)
            else:
                left_speed = right_speed = int(-self.motor_base * RATE * 0.6 * self.flag)
            brush_speed = self.last_brush_speed

        # 速度限幅并设置
        left_speed = max(min(left_speed, self.speed_pluse_max), -self.speed_pluse_max)
        right_speed = max(min(right_speed, self.speed_pluse_max), -self.speed_pluse_max)
        self.driver_manager.get_driver(2).set_speed(left_speed)
        self.driver_manager.get_driver(1).set_speed(right_speed)
        self.driver_manager.get_driver(3).set_speed(brush_speed)

        # 角度矫正完成后恢复原状态
        if abs(self.imu_yaw) < 0.2 and self.prev_motion_state:
            self.set_state(self.prev_motion_state)
            self.has_reverse_flag = False

    def _execute_stop_state(self):
        """执行上停/下停状态"""
        config = self.status_config[self.current_status]
        left_speed = int(config["velocity_up"])
        right_speed = int(config["velocity_low"])
        brush_speed = int(config["velocity_brush"])

        if self.current_status == "UPSTOP":
            right_speed = self.last_right_speed  # 右电机保持原速度
        elif self.current_status == "LOWSTOP":
            left_speed = self.last_left_speed  # 左电机保持原速度

        if (self.last_left_speed != left_speed or 
            self.last_right_speed != right_speed or 
            self.last_brush_speed != brush_speed):
            self.driver_manager.get_driver(2).set_speed(left_speed)
            self.driver_manager.get_driver(1).set_speed(right_speed)
            self.driver_manager.get_driver(3).set_speed(brush_speed)
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            self.last_brush_speed = brush_speed

    def _execute_unloading_state(self):
        """执行卸载状态"""
        correction = self.pid_correction(self.imu_yaw)
        config = self.status_config[self.current_status]
        left_speed = int(config["velocity_up"] + correction)
        right_speed = int(config["velocity_low"] + correction)
        brush_speed = int(config["velocity_brush"])

        if (self.last_left_speed != left_speed or 
            self.last_right_speed != right_speed or 
            self.last_brush_speed != brush_speed):
            self.driver_manager.get_driver(2).set_speed(left_speed)
            self.driver_manager.get_driver(1).set_speed(right_speed)
            self.driver_manager.get_driver(3).set_speed(brush_speed)
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            self.last_brush_speed = brush_speed

    def _execute_load_pause_state(self):
        """执行加载/暂停状态"""
        config = self.status_config[self.current_status]
        left_speed = int(config["velocity_up"])
        right_speed = int(config["velocity_low"])
        brush_speed = int(config["velocity_brush"])

        if (self.last_left_speed != left_speed or 
            self.last_right_speed != right_speed or 
            self.last_brush_speed != brush_speed):
            self.driver_manager.get_driver(2).set_speed(left_speed)
            self.driver_manager.get_driver(1).set_speed(right_speed)
            self.driver_manager.get_driver(3).set_speed(brush_speed)
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            self.last_brush_speed = brush_speed

    def _execute_start_state(self):
        """执行启动/充电/返回码头状态"""
        # 电量检查
        if self.battery_remaining is not None and self.battery_remaining < self.LOW_BATTERY_THRESHOLD:
            rospy.logerr("🔋 电量过低，停止启动")
            self.set_state("STOP")
            return

        # 电缸抬起（保留原有逻辑）
        if not hasattr(self, "elevator_stage") or self.elevator_stage == 0:
            self.motor_cmd_pub.publish(Int8(data=1))
            self.elevator_start_time = rospy.get_time()
            self.elevator_stage = 1
        elif self.elevator_stage == 1:
            elapsed = rospy.get_time() - self.elevator_start_time
            if elapsed >= 0.1:
                self.elevator_stage = 2
                if self.auto_mode and self.auto_step and self.current_status == "START":
                    self.set_state(self.auto_step)

    def publish_state(self, event=None):
        """发布机器人状态"""
        try:
            # 获取各电机实际速度
            left_speed = self.driver_manager.get_driver(2).get_actual_speed() if self.driver_manager.get_driver(2) else 0.0
            right_speed = self.driver_manager.get_driver(1).get_actual_speed() if self.driver_manager.get_driver(1) else 0.0
            brush_speed = self.driver_manager.get_driver(3).get_actual_speed() if self.driver_manager.get_driver(3) else 0.0

            # 构建状态消息
            state_msg = {
                "status": self.current_status,
                "battery": self.battery_remaining,
                "imu_yaw": round(self.imu_yaw, 2) if self.imu_yaw is not None else 0.00,
                "velocity_up": round(left_speed, 2),
                "velocity_low": round(right_speed, 2),
                "velocity_brush": round(brush_speed, 2),
                "sensors_status": self.sensors_status,
                "device_status": {
                    "motor_drivers": {
                        1: self.driver_manager.get_driver(1).connection_status if self.driver_manager.get_driver(1) else False,
                        2: self.driver_manager.get_driver(2).connection_status if self.driver_manager.get_driver(2) else False,
                        3: self.driver_manager.get_driver(3).connection_status if self.driver_manager.get_driver(3) else False
                    }
                },
                "auto_mode": self.auto_mode,
                "timestamp": time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
            }
            self.state_pub.publish(json.dumps(state_msg, ensure_ascii=False))
        except Exception as e:
            rospy.logerr(f"❌ 状态发布异常: {e}")
            error_msg = {"status": "ERROR", "error": str(e)}
            self.state_pub.publish(json.dumps(error_msg))

    # -------------------------- ROS回调函数 --------------------------
    def status_callback(self, msg):
        """机器人指令回调"""
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
            rospy.logwarn(f"⚠️ 指令解析失败: {msg.data}, 错误: {e}")

    def imu_callback(self, msg):
        """IMU数据回调"""
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
        """电池状态回调"""
        self.battery_remaining = msg.batttery_remaining

    def relay_callback(self, msg):
        """继电器状态回调"""
        self.relay_status = msg.data

    def proximity_callback(self, msg):
        """接近传感器回调"""
        # 更新传感器状态
        if msg.sensor_b:
            self.sensors_status |= 0x01
        else:
            self.sensors_status &= ~0x01
        if msg.sensor_a:
            self.sensors_status |= 0x02
        else:
            self.sensors_status &= ~0x02

        # 传感器触发逻辑（保留原有逻辑）
        sensor_b_trigger = msg.sensor_b and not self.sensor_triggered["a"]
        sensor_a_trigger = msg.sensor_a and not self.sensor_triggered["b"]
        sensor_aoth_trigger = msg.sensor_b and msg.sensor_a

        # 运动状态下的传感器处理
        if self.current_status in ["FORWARD", "BACKWARD"]:
            self._handle_motion_sensor(msg, sensor_b_trigger, sensor_a_trigger, sensor_aoth_trigger)
        # 停止状态下的传感器处理
        elif self.current_status in ["UPSTOP", "LOWSTOP"]:
            self._handle_stop_sensor(msg)

    def _handle_motion_sensor(self, msg, sensor_b_trigger, sensor_a_trigger, sensor_aoth_trigger):
        """运动状态下的传感器处理"""
        if self.is_global_repeat_protected():
            return

        with self.state_switch_lock:
            # 双侧触发：反向切换
            if sensor_aoth_trigger:
                self._complete_motion_and_reverse()
            # 单侧触发：进入停止状态
            elif sensor_b_trigger and not msg.sensor_a:
                current_time = time.time()
                if current_time - self.last_switch_time >= self.SWITCH_DELAY:
                    self.set_state("UPSTOP")
                    self.sensor_triggered["a"] = True
                    self.last_switch_time = current_time
            elif sensor_a_trigger and not msg.sensor_b:
                current_time = time.time()
                if current_time - self.last_switch_time >= self.SWITCH_DELAY:
                    self.set_state("LOWSTOP")
                    self.sensor_triggered["b"] = True
                    self.last_switch_time = current_time

    def _handle_stop_sensor(self, msg):
        """停止状态下的传感器处理"""
        if self.is_global_repeat_protected():
            return

        with self.state_switch_lock:
            # 上停状态：等待sensor_a触发
            if self.current_status == "UPSTOP" and msg.sensor_a and not self.sensor_triggered["b"]:
                self._switch_from_stop_state()
            # 下停状态：等待sensor_b触发
            elif self.current_status == "LOWSTOP" and msg.sensor_b and not self.sensor_triggered["a"]:
                self._switch_from_stop_state()

    def _complete_motion_and_reverse(self):
        """完成运动并反向"""
        current_time = time.time()
        if current_time - self.last_switch_time < self.SWITCH_DELAY:
            return

        rospy.loginfo(f"{self.current_status}状态双侧传感器触发，反向切换")
        self.set_state("LOADING")
        time.sleep(1.0)

        # 反向切换状态
        target_state = "BACKWARD" if self.current_status == "FORWARD" else "FORWARD"
        self.set_state(target_state)
        self.sensor_triggered = {"a": False, "b": False}
        self.last_switch_time = current_time
        self.last_critical_switch_time = current_time

    def _switch_from_stop_state(self):
        """从停止状态切换到反向运动"""
        rospy.loginfo(f"{self.current_status}状态对侧传感器触发，反向切换")
        self.set_state("LOADING")
        time.sleep(1.0)

        # 恢复之前的运动状态
        if self.prev_motion_state:
            self.set_state(self.prev_motion_state)
        self.sensor_triggered = {"a": False, "b": False}
        self.last_critical_switch_time = time.time()

    def shutdown(self):
        """关闭控制器"""
        rospy.loginfo("🔌 关闭电机控制器...")
        # 停止所有电机
        self.stop_all_motors()
        # 停止定时器
        self.publish_timer.shutdown()
        rospy.loginfo("✅ 电机控制器已关闭")