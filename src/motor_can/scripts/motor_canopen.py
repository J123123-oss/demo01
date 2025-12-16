#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import can
import time
import rospy
import json
import yaml
from std_msgs.msg import String, Int8, Float32, Float32MultiArray,Bool
from serial_comms.msg import Distances
from serial_comms.msg import Sensors
from serial_comms.msg import INSPVAE  # 确保导入正确的消息类型
from serial_comms.msg import BatteryStatus  # 确保导入正确的消息类型
from std_srvs.srv import Trigger
import threading
import sys
import select

rate = 68  # Hz   166.66>> 68.26

class ServoDriveController:
    def __init__(self, channel='can0', interface='socketcan'):
    # def __init__(self, channel='vcan0', interface='socketcan'):
        self.channel = channel
        self.interface = interface
        self.bus = self.create_can_bus()
        self.last_left_speed = 0
        self.last_right_speed = 0
        self.last_brush_speed = 0
        self.has_reverse_flag = False
        self.has_reverse_counter = 0
        self.reverse_start_time = None
        self.main_board = True # 主控板状态MQTT
        self.imu_sensor = True # IMU传感器状态MQTT
        self.motor_driver = True # 电机驱动器状态MQTT
        self.motor_base = 350
        self.base_speed = 30000   #设置后退基础速度值  * 0.8 > * 1
        self.brush_speed = 1600
        self.flag = 0  # 用于后退时的速度方向标志，1: IMU>0

        self.speed_pluse_max = 47600  #700*rate#(380*rate)  #23800 #32467      #23800   # 17000
        # 计时阶段参数
        self.reversed_start_time = None  # 记录首次检测到偏差的时间
        self.REVERSE_TIME_THRESHOLD = 3.0  # 需要持续的时间阈值(秒)
        self.unloading_timer = 0.1
        self.unloading_start_time = None
        self.start_time = 0
        self.elevator_stage = 0  # 电缸升降阶段: 0=待抬升,1=抬升中,2=抬升完成
        self.elevator_start_time = 0
        self.LOW_BATTERY_THRESHOLD = 40  # 电池低电量阈值，单位百分比

        # 添加实时速度发布计数器
        self.velocity_publish_count = 0
        self.velocity_publish_interval = 3  # 每5次发布一次速度数据
        # 添加上次速度值的存储变量
        self.last_velocity_up = 0
        self.last_velocity_low = 0
        self.last_velocity_brush = 0
        # 状态切换延时
        self.last_switch_time = 0
        self.SWITCH_DELAY = 1 # 触发延时阈值，单位：秒

        #设置状态列表
        self.status_list = [
            "STOP",  # 停止状态[默认状态]
            "FORWARD",  # 前进状态
            "BACKWARD",  # 后退状态
            "START",  # 速度模式初始化并使能[3]
            "LOADING", # 进仓
            "UNLOADING", # 出仓
            "UPSTOP", # 上电机停
            "LOWSTOP", # 下电机停
            "PISTON_OUT", #电缸伸出
            "PISTON_IN", #电缸缩进
            "CHARGE_OUT", #充电状态     [10]
            "RETURN_DOCK" #取消充电    [11]
        ]
        # 定义状态及其对应的速度配置
        self.status_config = {
            "START": {},
            "STOP": {
                "velocity_up": 0,
                "velocity_low": 0,
                "velocity_brush": 0
            },
            "FORWARD": {
                "velocity_up": self.motor_base * rate,
                "velocity_low": -self.motor_base * rate,
                "velocity_brush": -self.brush_speed * rate
            },
            "BACKWARD": {
                "velocity_up": -self.motor_base * rate,
                "velocity_low": self.motor_base * rate,
                "velocity_brush": -self.brush_speed * rate
            },
            "LOADING": {
                "velocity_up": 0,
                "velocity_low": 0,
                "velocity_brush": -self.brush_speed * rate
            },
            "UNLOADING":{
                "velocity_up": -self.motor_base *rate,
                "velocity_low": self.motor_base *rate,
                "velocity_brush": -self.brush_speed * rate 
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
            "REVERSE": {
                "velocity_up": 0,
                "velocity_low": 0,
                "velocity_brush": 0
            },
            "PISTON_OUT":{},
            "PISTON_IN":{},
            "CHARGE_OUT":{},
            "RETURN_DOCK":{}
        }
        self.last_state = None
        self.current_status = self.status_list[0]
        self.current_velocity_up = 0
        self.current_velocity_low = 0
        self.current_velocity_brush = 0
        #统计超声波传感器触发次数
        self.counter_a =0
        self.counter_b =0
        self.counter_c =0
        self.counter_d =0
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
        self.prev_motion_state = None  # 记录进入单侧停止前的运动状态
        self.is_upstop = False
        self.is_lowstop = False
        self.auto_mode = True
        self.auto_step = None
        self.count = 1
        # 新增：状态切换锁，避免重复触发
        self.state_switch_lock = threading.Lock()
        self.sensor_triggered = {"a": False, "b": False}  # 传感器触发标记（消抖）
        
        #控制不同状态下的发布频率
        self.publish_timer = rospy.Timer(rospy.Duration(1.0), lambda event: self.publish_state())
        self.fault_check_timer = rospy.Timer(rospy.Duration(60.0), lambda event: self.check_and_clear_faults())
        
        self.pid_integral = 0.0
        self.pid_last_error = 0.0
        self.target_yaw = 0.0
        self.pid_kp = 100
        self.pid_ki = 0.0
        self.pid_kd = 10
        self.pid_correction_max = 200

        self.progress = 0
        self.battery_total_voltage = None
        self.battery_current = None
        self.battery_remaining = None
        self.battery_temperatures = []
        #继电器状态
        self.relay_status = None
        self.relay_auto_off = None

        self.state_pub = rospy.Publisher('/robot_state', String, queue_size=10)
        self.motor_cmd_pub = rospy.Publisher('/motor_cmd', Int8, queue_size=10)
        rospy.Subscriber('/robot_cmd', String, self.status_callback)
        rospy.Subscriber('/inspvae_data', INSPVAE, self.imu_callback)
        rospy.Subscriber('/battery_status', BatteryStatus, self.battery_status_callback)
        rospy.Subscriber('/relay_status', Bool, self.relay_callback)
        rospy.Subscriber('/relay_auto_off', Bool, self.relay_auto_off_callback)

    def set_state(self, new_state):
        if new_state not in self.status_config:
            rospy.logwarn(f"尝试设置无效状态: {new_state}")
            return False
        if new_state == self.current_status and new_state not in ["PISTON_OUT", "PISTON_IN", "STOP"]:
            return False
        if self.current_status in ["FORWARD", "BACKWARD"] and ( new_state == "CHARGE_OUT" or new_state == "RETURN_DOCK" ):
            return False
            
        if self.auto_mode and new_state in ["FORWARD", "BACKWARD"]:
            self.auto_step = new_state

        if new_state == "START" or new_state == "CHARGE_OUT" or new_state == "RETURN_DOCK":
            self.complete_state = False
            self.enable_drive_flag = True
            self.progress = 0
            self.motor_driver = True
            self.imu_sensor = True
            self.main_board = True

        if new_state == "STOP":
            self.elevator_stage = 0
            if self.publish_timer is not None:
                self.publish_timer.shutdown()
                self.publish_timer = rospy.Timer(rospy.Duration(1.0), lambda event: self.publish_state())
            if self.fault_check_timer is not None:    
                self.fault_check_timer.shutdown()
                self.fault_check_timer = rospy.Timer(rospy.Duration(60.0), lambda event: self.check_and_clear_faults())
            threading.Thread(target=self.delayed_publish_freq_switch,args=(1,),daemon=True).start()
        else:
            if self.publish_timer is not None:
                self.publish_timer.shutdown()
                self.publish_timer = rospy.Timer(rospy.Duration(1.0), lambda event: self.publish_state())
            if self.fault_check_timer is not None:    
                self.fault_check_timer.shutdown()
                self.fault_check_timer = rospy.Timer(rospy.Duration(60.0), lambda event: self.check_and_clear_faults())

        # 记录运动状态（FORWARD/BACKWARD），用于后续反向切换
        if new_state in ["FORWARD", "BACKWARD"]:
            if self.current_status != "REVERSE":
                self.prev_motion_state = new_state
        # if new_state in ["REVERSE", "UPSTOP", "LOWSTOP"]:
        elif new_state == "REVERSE":
            if self.prev_motion_state is None:
                self.prev_motion_state = self.last_state
        
        elif new_state == "PISTON_IN":
            self.initial_yaw = None
            self.auto_step = None
        elif new_state == "PISTON_OUT":
            if self.count % 2:
                self.auto_mode = False
                rospy.loginfo("手动模式开")
            else:    
                self.auto_mode = True
                rospy.loginfo("自动模式开")
            self.count += 1
            
        self.current_status = new_state
        self.last_state = self.current_status
        rospy.loginfo(f"状态已更新为: {self.current_status}")
        return True
    
    def lock_motor(self):
        rospy.loginfo("电机向下转动，锁止")
        self.motor_cmd_pub.publish(Int8(data=-1))
        self.initial_yaw = None

    def publish_state(self):
        try:
            self.velocity_publish_count += 1
            if self.velocity_publish_count >= self.velocity_publish_interval:
                self.last_velocity_up = self.get_actual_velocity(3)
                self.last_velocity_low = self.get_actual_velocity(2)
                self.last_velocity_brush = self.get_actual_velocity(4)
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
                "relay_auto_off": self.relay_auto_off,
                "timestamp": time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
            }
            self.state_pub.publish(json.dumps(state_msg))
        except Exception:
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
                rospy.logwarn(f"未找到command字段: {msg.data}")
        except Exception as e:
            rospy.logwarn(f"消息解析失败，尝试按字符串处理: {msg.data}, 错误: {e}")
            self.set_state(msg.data)
            self.publish_state()

    def imu_callback(self, msg):
        try:
            self.imu_yaw = msg.yaw if hasattr(msg, "yaw") else msg.get("yaw", 0.0)
            if self.initial_yaw is None:
                self.initial_yaw = self.imu_yaw
                rospy.loginfo(f"Initial IMU yaw set to: {self.initial_yaw} degrees")
            
            relative_yaw = self.imu_yaw - self.initial_yaw
            if relative_yaw > 180:
                relative_yaw -= 360
            elif relative_yaw < -180:
                relative_yaw += 360
            self.imu_yaw = relative_yaw
        except json.JSONDecodeError as e:
            rospy.logerr(f"解析IMU数据失败: {e}")
            
    def battery_status_callback(self, msg):
        self.battery_remaining = msg.batttery_remaining
        self.battery_total_voltage = round(msg.total_voltage, 2) 
        self.battery_current = round(msg.current, 2)
        self.battery_temperatures = [round(t, 1) for t in msg.temperatures] if hasattr(msg, "temperatures") else []

    def relay_callback(self, msg):
        self.relay_status = msg.data
        
    def relay_auto_off_callback(self, msg):
        self.relay_auto_off = msg.data

    def create_can_bus(self):
        while True:
            try:
                return can.interface.Bus(channel=self.channel, interface=self.interface)
            except (can.CanError, OSError) as e:
                rospy.logerr(f"motor_canopen CAN连接失败: {e}，3秒后重试...")
                time.sleep(3)

    def reconnect_can_bus(self):
        rospy.logwarn("尝试重连CAN总线...")
        try:
            if self.bus is not None:
                self.bus.shutdown()
        except Exception:
            pass
        self.bus = self.create_can_bus()

    def send_command(self, motor_id, command_data):
        frame_id = 0x600 + motor_id
        msg = can.Message(arbitration_id=frame_id, data=command_data, is_extended_id=False)
        for attempt in range(3):
            try:
                self.bus.send(msg)
                time.sleep(0.05)
                return
            except (can.CanError, OSError) as e:
                rospy.logerr(f"CAN通信错误: {e}，尝试重连...")
                self.reconnect_can_bus()
        rospy.logerr("CAN发送失败，已重试3次")

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
        self.send_command(motor_id, data)

    def get_actual_velocity(self, motor_id):
        self.send_command(motor_id, [0x40, 0x6C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        start_time = time.time()
        while time.time() - start_time < 0.5:
            try:
                msg = self.bus.recv(timeout=0.1)
            except (can.CanError, OSError) as e:
                rospy.logerr(f"CAN接收错误: {e}，尝试重连...")
                self.reconnect_can_bus()
                continue
            if msg and msg.arbitration_id == (0x580 + motor_id):
                if len(msg.data) >= 8 and msg.data[0] == 0x43 and msg.data[1] == 0x6C and msg.data[2] == 0x60:
                    velocity = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
                    if velocity > 0x7FFFFFFF:
                        velocity -= 0x100000000
                    return velocity
        rospy.logwarn(f"读取电机 {motor_id} 当前速度失败")
        return 0

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
        fault_code = self.read_fault_code(motor_id)
        if fault_code and fault_code != 0:
            rospy.logwarn(f"电机 {motor_id} 存在故障 (0x{fault_code:04X}), 尝试清除...")
            self.clear_fault(motor_id)
            time.sleep(0.3)

        rospy.loginfo(f"配置电机 {motor_id}: 速度={int(velocity/rate)}, 加速度={acceleration}, 减速度={deceleration}")
        self.start_motor(motor_id)
        self.set_velocity_mode(motor_id)
        self.set_target_velocity(motor_id, velocity)
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
        rospy.loginfo(f"接收到按键: {key}")
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
        else:
            rospy.loginfo(f"无效按键: {key}")

    def proximity_callback(self, msg):
        """核心修改：统一自动/手动模式下的sensor_a/sensor_b处理逻辑"""
        # 1. 更新传感器状态
        if msg.sensor_a:
            self.sensors_status |= 0x01
        else:
            self.sensors_status &= ~0x01
        if msg.sensor_b:
            self.sensors_status |= 0x02
        else:
            self.sensors_status &= ~0x02
        
        # 2. 传感器消抖（仅首次触发时处理）
        sensor_a_trigger = msg.sensor_a and not self.sensor_triggered["a"]
        sensor_b_trigger = msg.sensor_b and not self.sensor_triggered["b"]
        sensor_both_trigger = msg.sensor_a and msg.sensor_b

        # 3. RETURN_DOCK/CHARGE_OUT 特殊逻辑（保留原有）
        if self.auto_mode and self.current_status == "RETURN_DOCK":
            if self.elevator_stage == 2:
                self.set_state("FORWARD")

        if self.auto_mode and self.current_status == "CHARGE_OUT":
            if self.elevator_stage == 2:
                if msg.sensor_a or msg.sensor_b:
                    if self.current_status != "UNLOADING":
                        self.set_state("UNLOADING")
                        self.progress = 10
                        self.unloading_start_time = time.time()
                if self.current_status == "UNLOADING" and self.unloading_start_time:
                    elapsed = time.time() - self.unloading_start_time
                    if elapsed >= self.unloading_timer:
                        self.set_state("STOP")
                        self.progress = 0
                        self.unloading_start_time = None

        # 4. START状态逻辑（保留原有）
        if self.auto_mode and self.current_status == "START":
            if self.elevator_stage == 2:
                if msg.sensor_a or msg.sensor_b:
                    if self.current_status != "UNLOADING":
                        self.set_state("UNLOADING")
                        self.progress = 10
                        self.unloading_start_time = time.time()
                elif not msg.sensor_a and not msg.sensor_b:
                    self.set_state("BACKWARD")
                    self.progress = 20
                if self.current_status == "UNLOADING" and self.unloading_start_time:
                    elapsed = time.time() - self.unloading_start_time
                    if elapsed >= self.unloading_timer:
                        self.set_state("BACKWARD")
                        self.progress = 20
                        self.unloading_start_time = None

        # 5. REVERSE状态边界检测（保留原有）
        if self.current_status == "REVERSE":
            if msg.sensor_a or msg.sensor_b:
                rospy.logwarn("边界触发，立即STOP")
                self.set_state("STOP")
            return

        # 6. 统一处理自动/手动模式的FORWARD/BACKWARD状态
        self._handle_motion_state(msg, sensor_a_trigger, sensor_b_trigger, sensor_both_trigger)

        # 7. 处理UPSTOP/LOWSTOP状态（等待另一侧传感器触发后反向）
        self._handle_stop_states(msg)

    def _handle_motion_state(self, msg, sensor_a_trigger, sensor_b_trigger, sensor_both_trigger):
        """处理FORWARD/BACKWARD状态的传感器逻辑（自动/手动统一）"""
        with self.state_switch_lock:
            # 前进状态
            if self.current_status == self.status_list[1]:  # FORWARD
                # 双侧传感器触发：完成任务，反向
                if sensor_both_trigger:
                    self._complete_motion_and_reverse("FORWARD")
                # 单侧传感器触发：进入对应停止状态
                elif sensor_a_trigger and not msg.sensor_b:
                    self.set_state("UPSTOP")
                    self.sensor_triggered["a"] = True
                elif sensor_b_trigger and not msg.sensor_a:
                    self.set_state("LOWSTOP")
                    self.sensor_triggered["b"] = True

            # 后退状态
            elif self.current_status == self.status_list[2]:  # BACKWARD
                # 双侧传感器触发：完成任务，反向
                if sensor_both_trigger:
                    self._complete_motion_and_reverse("BACKWARD")
                # 单侧传感器触发：进入对应停止状态
                elif sensor_a_trigger and not msg.sensor_b:
                    self.set_state("UPSTOP")
                    self.sensor_triggered["a"] = True
                elif sensor_b_trigger and not msg.sensor_a:
                    self.set_state("LOWSTOP")
                    self.sensor_triggered["b"] = True

    def _complete_motion_and_reverse(self, current_motion):
        """完成运动并反向切换"""
        current_time = time.time()  # 获取当前时间戳
    
        # 核心：判断是否超过延时阈值
        if current_time - self.last_switch_time < self.SWITCH_DELAY:
            rospy.logwarn("反向切换触发间隔过短，忽略本次触发")
            return
        rospy.loginfo(f"{current_motion}状态下双侧传感器触发，开始反向切换")
        self.set_state("LOADING")
        time.sleep(1.5)
        
        # 清空状态
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
        
        # 重置传感器触发标记
        self.sensor_triggered = {"a": False, "b": False}

    def _handle_stop_states(self, msg):
        """处理UPSTOP/LOWSTOP状态：等待另一侧传感器触发后反向"""
        with self.state_switch_lock:
            # LOWSTOP：等待sensor_a触发（另一侧）
            if self.current_status == self.status_list[7]:  # LOWSTOP
                if msg.sensor_a and not self.sensor_triggered["a"]:
                    self._switch_from_stop_state("LOWSTOP")
            
            # UPSTOP：等待sensor_b触发（另一侧）
            elif self.current_status == self.status_list[6]:  # UPSTOP
                if msg.sensor_b and not self.sensor_triggered["b"]:
                    self._switch_from_stop_state("UPSTOP")

    def _switch_from_stop_state(self, stop_state):
        """从停止状态切换为反向运动"""
        rospy.loginfo(f"在{stop_state}执行，对侧传感器触发，开始反向切换")
        self.initial_yaw = None
        self.set_state("LOADING")
        time.sleep(3)
        
        # 根据之前的运动状态反向切换
        if self.prev_motion_state == "FORWARD":
            self.set_state("BACKWARD")
        elif self.prev_motion_state == "BACKWARD":
            self.set_state("FORWARD")
        else:
            # 默认前进
            self.set_state("FORWARD")
        
        self.progress = 60
        # 重置传感器触发标记
        self.sensor_triggered = {"a": False, "b": False}

    def pid_correction(self, current_yaw):
        error = self.target_yaw - current_yaw
        if abs(error) < 0.05:
            return 0
        
        if abs(error) > 0.3:
            self.pid_integral = 0
        
        self.pid_integral += error
        derivative = error - self.pid_last_error
        
        integral_max = 30
        self.pid_integral = max(min(self.pid_integral, integral_max), -integral_max)
        
        correction = (self.pid_kp * error +
                    self.pid_ki * self.pid_integral +
                    self.pid_kd * derivative)
        
        self.pid_last_error = error
        return max(min(-correction, self.pid_correction_max), -self.pid_correction_max)

    def execute_state(self, event=None):
        if self.enable_drive_flag and (self.current_status =="START" or self.current_status =="CHARGE_OUT" or self.current_status =="RETURN_DOCK"):
            if self.battery_remaining is not None and self.battery_remaining < self.LOW_BATTERY_THRESHOLD and (self.current_status =="START" or self.current_status =="STOP"):
                rospy.logerr("电池电量过低，无法启动电机！请充电后重试!")
                self.enable_drive_flag = False
                self.main_board = False
                self.set_state("STOP")
                return

            if self.elevator_stage == 0:
                rospy.loginfo("电缸抬起...")
                self.motor_cmd_pub.publish(Int8(data=1))
                self.elevator_start_time = rospy.get_time()
                self.elevator_stage = 1
                
            elif self.elevator_stage == 1:
                elapsed = rospy.get_time() - self.elevator_start_time
                if elapsed >= 0.1:
                    rospy.loginfo("开始配置电机速度模式...")
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
                    rospy.loginfo("速度模式初始化完成")
                    self.enable_drive_flag = False
                    self.elevator_stage = 2
                    self.start_time = rospy.get_time()
                    if self.auto_mode and self.auto_step and self.current_status =="START":
                        rospy.loginfo(f"初始化完成，恢复自动流程: {self.auto_step}")
                        self.set_state(self.auto_step)

        if self.current_status in ["FORWARD", "BACKWARD"]:
            correction = self.pid_correction(self.imu_yaw) * rate
            left_speed = int(self.status_config[self.current_status]["velocity_up"] + correction)
            right_speed = int(self.status_config[self.current_status]["velocity_low"] + correction)
            brush_speed = self.status_config[self.current_status]["velocity_brush"]
            right_speed = max(min(right_speed, self.speed_pluse_max), -self.speed_pluse_max)
            left_speed = max(min(left_speed, self.speed_pluse_max), -self.speed_pluse_max)

            if (self.last_left_speed != left_speed or
                self.last_right_speed != right_speed or
                self.last_brush_speed != brush_speed):
                self.set_target_velocity(3, left_speed)
                self.set_target_velocity(2, right_speed)
                self.set_target_velocity(4, brush_speed)
                self.last_left_speed = left_speed
                self.last_right_speed = right_speed
                self.last_brush_speed = brush_speed

            angle_condition_met = (-5 < self.imu_yaw < -2.5 or 2.5 < self.imu_yaw < 5)
            if angle_condition_met:
                if self.reversed_start_time is None:
                    self.reversed_start_time = rospy.get_time()
                    rospy.logwarn(f"检测到角度偏差: {self.imu_yaw:.2f}度，开始计时...")
                elapsed = rospy.get_time() - self.reversed_start_time
                if elapsed >= self.REVERSE_TIME_THRESHOLD:
                    rospy.logwarn(f"角度偏差已持续{elapsed:.1f}秒，进入REVERSE状态")
                    self.set_state("REVERSE")
                    self.reversed_start_time = None
            else:
                if self.reversed_start_time is not None:
                    rospy.loginfo(f"角度偏差消失({self.imu_yaw:.2f}°)，重置计时器")
                    self.reversed_start_time = None

        elif self.current_status == "REVERSE":
            self.reversed_start_time = None
            if not self.has_reverse_flag:
                self.has_reverse_counter += 1
                if self.has_reverse_counter > 10:
                    rospy.logwarn("连续后退10次，可能需要手动干预")
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
                if (self.last_left_speed != left_speed or
                self.last_right_speed != right_speed or
                self.last_brush_speed != brush_speed):
                    rospy.loginfo(f"IMU矫正: yaw={self.imu_yaw:.2f}")
                    rospy.loginfo(f"后退上轮速度: {left_speed}, 下轮速度: {right_speed}")
                    self.set_target_velocity(3, left_speed)
                    self.set_target_velocity(2, right_speed)
                    self.set_target_velocity(4, brush_speed)
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
                    right_speed = left_speed = int (-self.base_speed * 0.8 * self.flag)
                else:
                    right_speed = left_speed = int(-self.base_speed * 0.6 * self.flag)
                right_speed = max(min(right_speed, self.speed_pluse_max), -self.speed_pluse_max)
                left_speed = max(min(left_speed, self.speed_pluse_max), -self.speed_pluse_max)
                brush_speed = self.last_brush_speed
                if (self.last_left_speed != left_speed or
                self.last_right_speed != right_speed or
                self.last_brush_speed != brush_speed):
                    rospy.loginfo(f"IMU矫正: yaw={self.imu_yaw:.2f}")
                    rospy.loginfo(f"后退完毕上轮速度: {left_speed}, 下轮速度: {right_speed}")
                    self.set_target_velocity(3, left_speed)
                    self.set_target_velocity(2, right_speed)
                    self.set_target_velocity(4, brush_speed)
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
            if (self.last_left_speed != left_speed or
                self.last_right_speed != right_speed or
                self.last_brush_speed != brush_speed):
                self.set_target_velocity(3, left_speed)
                self.set_target_velocity(2, right_speed)
                self.set_target_velocity(4, brush_speed)
                self.last_left_speed = left_speed
                self.last_right_speed = right_speed
                self.last_brush_speed = brush_speed

        elif self.current_status == "LOWSTOP":
            left_speed = self.last_left_speed
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

        elif self.current_status == "STOP" or not -5 < self.imu_yaw < 5:
            if (self.last_left_speed != 0 or
                self.last_right_speed != 0 or
                self.last_brush_speed != 0):
                self.has_reverse_counter = 0
                self.set_target_velocity(2, 0)
                self.set_target_velocity(3, 0)
                self.set_target_velocity(4, 0)
                self.disable_drive(2)
                self.disable_drive(3)
                self.disable_drive(4)
                self.last_left_speed = 0
                self.last_right_speed = 0
                self.last_brush_speed = 0
                self.need_speed_mode_init = True

        elif self.current_status == "UNLOADING":
            correction = self.pid_correction(self.imu_yaw) * rate
            left_speed = int(self.status_config[self.current_status]["velocity_up"] + correction)
            right_speed = int(self.status_config[self.current_status]["velocity_low"] + correction)
            brush_speed = self.status_config[self.current_status]["velocity_brush"]
            if (self.last_left_speed != left_speed or
                self.last_right_speed != right_speed or
                self.last_brush_speed != brush_speed):
                self.set_target_velocity(3, left_speed)
                self.set_target_velocity(2, right_speed)
                self.set_target_velocity(4, brush_speed)
                self.last_left_speed = left_speed
                self.last_right_speed = right_speed
                self.last_brush_speed = brush_speed

        elif self.current_status == "LOADING":
            left_speed = int(self.status_config[self.current_status]["velocity_up"])
            right_speed = int(self.status_config[self.current_status]["velocity_low"])
            brush_speed = self.status_config[self.current_status]["velocity_brush"]
            if (self.last_left_speed != left_speed or
                self.last_right_speed != right_speed or
                self.last_brush_speed != brush_speed):
                self.set_target_velocity(3, left_speed)
                self.set_target_velocity(2, right_speed)
                self.set_target_velocity(4, brush_speed)
                self.last_left_speed = left_speed
                self.last_right_speed = right_speed
                self.last_brush_speed = brush_speed
    
    def delayed_publish_freq_switch(self, delay_sec=3):
        time.sleep(delay_sec)
        if self.current_status == "STOP":
            if self.publish_timer is not None:
                self.publish_timer.shutdown()
            self.publish_timer = rospy.Timer(rospy.Duration(1800), lambda event: self.publish_state())
            if self.fault_check_timer is not None:
                self.fault_check_timer.shutdown()
            self.fault_check_timer = rospy.Timer(rospy.Duration(7200), lambda event: self.check_and_clear_faults())
            
    def get_max_torque(self, motor_id):
        self.send_command(motor_id, [0x40, 0x72, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        start_time = time.time()
        while time.time() - start_time < 0.5:
            msg = self.bus.recv(0.1)
            if msg and msg.arbitration_id == (0x580 + motor_id):
                if len(msg.data) >= 6 and msg.data[0] in [0x4B, 0x43]:
                    max_torque = msg.data[4] | (msg.data[5] << 8)
                    return max_torque
        rospy.logwarn(f"读取电机 {motor_id} 最大转矩超时")
        return None

    def get_actual_torque(self, motor_id):
        self.send_command(motor_id, [0x40, 0x77, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        start_time = time.time()
        while time.time() - start_time < 0.5:
            msg = self.bus.recv(0.1)
            if msg and msg.arbitration_id == (0x580 + motor_id):
                if len(msg.data) >= 6 and msg.data[0] == 0x4B and msg.data[1] == 0x77 and msg.data[2] == 0x60:
                    torque = msg.data[4] | (msg.data[5] << 8)
                    if torque > 0x7FFF:
                        torque -= 0x10000
                    rospy.logwarn(f"读取电机 {motor_id} 实际转矩为 {torque/1000} 额定转矩")
                    return torque
        rospy.logwarn(f"读取电机 {motor_id} 实际转矩超时")
        return None
        
    def get_max_current(self, motor_id):
        self.send_command(motor_id, [0x40, 0x73, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        start_time = time.time()
        while time.time() - start_time < 0.5:
            msg = self.bus.recv(0.1)
            if msg and msg.arbitration_id == (0x580 + motor_id):
                if len(msg.data) >= 6 and msg.data[0] == 0x43:
                    current = msg.data[4] | (msg.data[5] << 8)
                    rospy.logwarn(f"读取电机 {motor_id} 最大电流: {current/1000}额定电流")
                    return current
        rospy.logwarn(f"读取电机 {motor_id} 最大电流超时")
        return None
        
    def get_actual_current(self, motor_id):
        self.send_command(motor_id, [0x40, 0x78, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        start_time = time.time()
        while time.time() - start_time < 0.5:
            msg = self.bus.recv(0.1)
            if msg and msg.arbitration_id == (0x580 + motor_id):
                if len(msg.data) >= 6 and msg.data[0] == 0x4B and msg.data[1] == 0x78 and msg.data[2] == 0x60:
                    current = msg.data[4] | (msg.data[5] << 8)
                    if current > 0x7FFF:
                        current -= 0x10000
                    rospy.logwarn(f"读取电机 {motor_id} 实际电流: {current/1000}额定电流")
                    return current
        rospy.logwarn(f"读取电机 {motor_id} 实际电流超时")
        return None
        
    def read_fault_code(self, motor_id):
        self.send_command(motor_id, [0x40, 0x3F, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        start_time = time.time()
        while time.time() - start_time < 0.5:
            msg = self.bus.recv(0.1)
            if msg and msg.arbitration_id == (0x580 + motor_id):
                if len(msg.data) >= 6 and msg.data[1] == 0x3F and msg.data[2] == 0x60:
                    fault_code = msg.data[4] | (msg.data[5] << 8)
                    if fault_code != 0:
                        rospy.logwarn(f"电机 {motor_id} 故障码: 0x{fault_code:04X} ({fault_code})")
                    else:
                        rospy.loginfo(f"电机 {motor_id} 无故障")
                        self.motor_driver = True
                    return fault_code
        rospy.logwarn(f"读取电机 {motor_id} 故障码超时")
        return None

    def clear_fault(self, motor_id):
        rospy.loginfo(f"尝试清除电机 {motor_id} 故障...")
        self.send_command(motor_id, [0x2B, 0x40, 0x60, 0x00, 0x8F, 0x00, 0x00, 0x00])
        time.sleep(0.1)
        self.send_command(motor_id, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00])
        rospy.loginfo(f"电机 {motor_id} 故障复位指令已发送")
        fault_code = self.read_fault_code(motor_id)
        if fault_code == 0 or fault_code is None:
            rospy.loginfo(f"电机 {motor_id} 故障已清除")
        else:
            rospy.logerr(f"电机 {motor_id} 故障清除失败, 代码: 0x{fault_code:04X}")

    def check_and_clear_faults(self):
        for motor_id in [2, 3, 4]:
        # for motor_id in [4]:
            fault_code = self.read_fault_code(motor_id)
            actual_velocity = self.get_actual_velocity(motor_id)
            actual_current = self.get_actual_current(motor_id)
            if fault_code and fault_code != 0:
                rospy.logerr(f"电机 {motor_id} 检测到故障! 代码: 0x{fault_code:04X}")
                actual_torque = self.get_actual_torque(motor_id)
                if actual_torque is not None:
                    rospy.loginfo(f"故障时转矩: {actual_torque/1000} 额定转矩")
                if actual_velocity is not None:
                    rospy.loginfo(f"故障时速度: {actual_velocity/68/20} rpm")
                self.set_state("STOP")
                self.motor_driver = False
                rospy.loginfo("已切换到STOP状态，下次启动后处理故障")

            actual_torque = self.get_actual_torque(motor_id)
            max_torque = self.get_max_torque(motor_id)
            if max_torque is not None and actual_torque is not None:
                rospy.logwarn(f"电机 {motor_id} - 实际转矩: {actual_torque/1000}额定转矩 | 最大限制: {max_torque/1000}额定转矩")
                utilization = abs(actual_torque) / max_torque * 100
                if utilization > 80:
                    rospy.logwarn(f"电机 {motor_id} 转矩利用率过高: {utilization:.1f}%")
                    
    def read_energy_saving_mode(self, motor_id):
        self.send_command(motor_id, [0x40, 0xD0, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00])
        rospy.loginfo(f"已向电机 {motor_id} 发送读取 Fn_1d0 指令")
        start_time = time.time()
        while time.time() - start_time < 0.5:
            msg = self.bus.recv(0.1)
            if msg and msg.arbitration_id == (0x580 + motor_id):
                print("Fn_1d0 读取响应数据:", msg.data)
                if len(msg.data) >= 6 and msg.data[1] == 0xD0 and msg.data[2] == 0x21:
                    fn_1d0_value = msg.data[4] | (msg.data[5] << 8)
                    rospy.loginfo(f"电机 {motor_id} Fn_1d0（省电功能模式）当前值: {fn_1d0_value}")
                    mode_desc = {0: "关闭自动省电功能", 1: "总线指令触发省电", 2: "停机超时触发省电", 3: "指令或超时触发省电"}
                    rospy.loginfo(f"Fn_1d0 模式说明: {mode_desc.get(fn_1d0_value, '未知模式')}")
                    return fn_1d0_value
        rospy.logwarn(f"读取电机 {motor_id} Fn_1d0 超时")
        return None
        
    def set_energy_saving_mode(self, motor_id, target_value=3):
        if target_value not in [0, 1, 2, 3]:
            rospy.logerror(f"Fn_1d0 目标值 {target_value} 非法！仅支持 0（关闭）、1（指令触发）、2（超时触发）、3（指令或超时触发）")
            return False
        low_byte = target_value & 0xFF
        high_byte = (target_value >> 8) & 0xFF
        self.send_command(motor_id, [0x2B, 0xD0, 0x21, 0x00, low_byte, high_byte, 0x00, 0x00])
        rospy.loginfo(f"已向电机 {motor_id} 发送设置 Fn_1d0 指令，目标值: {target_value}")
        start_time = time.time()
        while time.time() - start_time < 0.5:
            msg = self.bus.recv(0.1)
            if msg and msg.arbitration_id == (0x580 + motor_id):
                print("Fn_1d0 设置响应数据:", msg.data)
                if len(msg.data) >= 4 and msg.data[1] == 0xD0 and msg.data[2] == 0x21:
                    rospy.loginfo(f"电机 {motor_id} Fn_1d0 已成功设置为 {target_value}（指令或超时触发省电）")
                    return True
        rospy.logwarn(f"设置电机 {motor_id} Fn_1d0 超时，可能未生效")
        return False
    
    def read_torque_zero_params(self, motor_id):
        param_map = {
            "Fn_04b": {"index": 0x204B, "desc": "零转矩到达门限（额定转矩千分之一）"},
            "Fn_04c": {"index": 0x204C, "desc": "零转矩到达回差值（额定转矩千分之一）"}
        }
        result = {}
        for param_name, info in param_map.items():
            index = info["index"]
            cmd_low_byte = index & 0xFF
            cmd_high_byte = (index >> 8) & 0xFF
            self.send_command(motor_id, [0x40, cmd_low_byte, cmd_high_byte, 0x00, 0x00, 0x00, 0x00, 0x00])
            rospy.loginfo(f"已向电机 {motor_id} 发送读取 {param_name}（索引0x{index:04X}）指令")
            start_time = time.time()
            param_value = None
            while time.time() - start_time < 0.5:
                msg = self.bus.recv(0.1)
                if msg and msg.arbitration_id == (0x580 + motor_id):
                    print(f"{param_name} 读取响应数据:", msg.data)
                    if len(msg.data) >= 6 and msg.data[1] == cmd_low_byte and msg.data[2] == cmd_high_byte:
                        param_value = msg.data[4] | (msg.data[5] << 8)
                        rospy.loginfo(f"电机 {motor_id} {param_name}（{info['desc']}）: {param_value}")
                        break
            if param_value is not None:
                result[param_name] = param_value
            else:
                rospy.logwarn(f"读取电机 {motor_id} {param_name} 超时")
                result[param_name] = None
        if result["Fn_04b"] is not None and result["Fn_04c"] is not None:
            lower_limit = result["Fn_04b"]
            upper_limit = result["Fn_04b"] + result["Fn_04c"]
            if result["Fn_04c"] == 0:
                rospy.logwarn(f"电机 {motor_id} Fn_04c 为0，零转矩判定区间异常（仅[{lower_limit}, {upper_limit}]），易误判未满足条件")
            else:
                rospy.loginfo(f"电机 {motor_id} 零转矩判定区间: [{lower_limit}, {upper_limit}]（额定转矩千分之一）")
        return result
    
    def set_torque_zero_param(self, motor_id, param_name, target_value):
        param_config = {
            "Fn_04b": {"index": 0x204B, "desc": "零转矩到达门限", "min": 0, "max": 1000},
            "Fn_04c": {"index": 0x20C, "desc": "零转矩到达回差值", "min": 0, "max": 500}
        }
        if param_name not in param_config:
            rospy.logerror(f"不支持的参数名 {param_name}！仅支持 {list(param_config.keys())}")
            return False
        info = param_config[param_name]
        if not (info["min"] <= target_value <= info["max"]):
            rospy.logerror(f"{param_name}（{info['desc']}）目标值 {target_value} 非法！需在 [{info['min']}, {info['max']}] 范围内")
            return False
        if param_name == "Fn_04c" and target_value == 0:
            rospy.logwarn(f"警告：{param_name} 设为0会导致零转矩判定异常，建议设为与Fn_04b一致的值（如100）")
        index = info["index"]
        cmd_low_byte = index & 0xFF
        cmd_high_byte = (index >> 8) & 0xFF
        target_low = target_value & 0xFF
        target_high = (target_value >> 8) & 0xFF
        self.send_command(motor_id, [0x2B, cmd_low_byte, cmd_high_byte, 0x00, target_low, target_high, 0x00, 0x00])
        rospy.loginfo(f"已向电机 {motor_id} 发送设置 {param_name} 指令，目标值: {target_value}（{info['desc']}）")
        start_time = time.time()
        while time.time() - start_time < 0.5:
            msg = self.bus.recv(0.1)
            if msg and msg.arbitration_id == (0x580 + motor_id):
                print(f"{param_name} 设置响应数据:", msg.data)
                if len(msg.data) >= 4 and msg.data[1] == cmd_low_byte and msg.data[2] == cmd_high_byte:
                    rospy.loginfo(f"电机 {motor_id} {param_name} 已成功设置为 {target_value}")
                    return True
        rospy.logwarn(f"设置电机 {motor_id} {param_name} 超时，可能未生效")
        return False
    
    def start_imu(self):
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service('/imu_parser_node/start_imu', timeout=5)
                start_srv = rospy.ServiceProxy('/imu_parser_node/start_imu', Trigger)
                resp = start_srv()
                print(resp.message)
                break
            except Exception as e:
                print(f"等待IMU服务中: {e}")
                time.sleep(1)

    def stop_imu(self):
        try:
            rospy.wait_for_service('/imu_parser_node/stop_imu')
            stop_srv = rospy.ServiceProxy('/imu_parser_node/stop_imu', Trigger)
            resp = stop_srv()
            print(resp.message)
        except Exception as e:
            print(f"调用IMU停止服务失败: {e}")

    @staticmethod
    def keyboard_listener(controller):
        rospy.loginfo("按键控制：s=停止, f=前进, b=后退")
        while not rospy.is_shutdown():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.readline().strip()
                if key:
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
            controller.current_status = controller.status_list[0]
        except Exception as e:
            rospy.logerr(f"配置电机 {motor_id} 时出错: {e}")
    rospy.loginfo("电机初始化完成（Ctrl+C 退出）")
    rospy.Subscriber("proximity_sensor_data", Sensors, lambda msg: controller.proximity_callback(msg))
    t = threading.Thread(target=ServoDriveController.keyboard_listener, args=(controller,), daemon=True)
    t.start()
    try:
        rospy.Timer(rospy.Duration(0.05), controller.execute_state)
        controller.set_state("STOP")
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("程序终止")
    finally:
        controller.shutdown()

if __name__ == "__main__":
    main()