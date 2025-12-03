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
import traceback
# import os

rate = 68  # Hz   166.66>> 68.26

class ServoDriveController:
    # def __init__(self, channel='vcan0', interface='socketcan'):
    def __init__(self, channel='can0', interface='socketcan'):
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
        self.motor_driver =True # 电机驱动器状态MQTT
        self.motor_base = 350
        self.base_speed = 17000   #设置后退基础速度值  * 0.8 > * 1
        self.brush_speed = 1600 * rate # 设置滚刷速度
        self.flag = 0  # 用于后退时的速度方向标志，1: IMU>0

        self.speed_pluse_max = 25840  #(380*rate)  #23800 #32467      #23800   # 17000
        # 计时阶段参数
        self.reversed_start_time = None  # 记录首次检测到偏差的时间
        self.REVERSE_TIME_THRESHOLD = 3.0  # 需要持续的时间阈值(秒)
        self.unloading_timer = 20.0
        self.unloading_start_time = None
        self.start_time = 0
        self.elevator_stage = 0  # 电缸升降阶段: 0=待抬升,1=抬升中,2=抬升完成
        self.elevator_start_time = 0
        self.LOW_BATTERY_THRESHOLD = 40  # 电池低电量阈值，单位百分比

        # 添加实时速度发布计数器
        self.velocity_publish_count = 0
        self.velocity_publish_interval = 5  # 每5次发布一次速度数据
            # 添加上次速度值的存储变量
        self.last_velocity_up = 0
        self.last_velocity_low = 0
        self.last_velocity_brush = 0

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
            "START": {  # 速度模式初始化并使能
                # 位置模式未启用
                # "position_left": 65188,  # 左侧电机目标位置 由300cm转换而来  651883
                # "position_right": -65188,  # 右侧电机目标位置              651883
                # "velocity_up": self.motor_base * rate,
                # "velocity_low": self.motor_base * rate, #自动速度无法设置负值，二者速度相同
                # "velocity_brush": -100 * rate #后续添加距离到位后反转的判断
            },

            "STOP": {  # 停止状态
                "velocity_up": 0,
                "velocity_low": 0,
                "velocity_brush": 0
            },
            "FORWARD": {  # 前进状态
                "velocity_up": self.motor_base * rate,
                "velocity_low": -self.motor_base * rate,
                "velocity_brush": -self.brush_speed      #-1000 同向
            },
            "BACKWARD": {  # 后退状态
                "velocity_up": -self.motor_base * rate,
                "velocity_low": self.motor_base * rate,
                "velocity_brush": -self.brush_speed      #1000 同向
            },
            "LOADING": {
                # "velocity_up": self.motor_base *rate,
                # "velocity_low": -self.motor_base *rate,
                # "velocity_brush": 0
                # 测试滚刷
                "velocity_up": 0,
                "velocity_low": 0,
                "velocity_brush": -self.brush_speed   #1600
            },
            "UNLOADING":{
                "velocity_up": -self.motor_base *rate,
                "velocity_low": self.motor_base *rate,
                "velocity_brush": self.brush_speed   #出仓同向
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
            "REVERSE": {  # 后退矫正状态
                "velocity_up": 0,
                "velocity_low": 0,
                "velocity_brush": 0
            },
            "PISTON_OUT":{
                #目前用与自动模式与手动模式的切换
            },
            "PISTON_IN":{
                #目前用于清空自动模式的状态 不符合逻辑，已禁用
                #切换为重置初始偏航角
            },
            "CHARGE_OUT":{
                #出仓充电
            },
            "RETURN_DOCK":{
                #回仓取消充电
            }
            # "FORWARD": {  # 测试电机功耗前进状态
            #     #下发100到电机减速20：1，实际为5RPM ，发self.motor_base最终12.5RPM，速度0.078m/s
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
        self.position_direction = 1  # 1: forward, -1: backward
        self.target_sent_flag = False  # 标记目标指令是否已下发

        self.need_speed_mode_init = False
        self.enable_drive_flag = False # 驱动器是否需要进行速度模式初始化
        self.stop_velocity = 0  # 停止速度
        self.imu_yaw = 0.0  # IMU偏航角 单位度
        self.initial_yaw = None

        self.sensors_status = 0 #表示4个超声波传感器触发状态
        self.complete_state = False
        self.prev_motion_state = None  # 记录进入单侧停止前的运动状态。
        self.is_upstop = False
        self.is_lowstop = False
        self.auto_mode = True #默认自动模式
        self.auto_step = None # 当前自动程序所在状态
        self.count = 1 # 切换自动与手动 
        #控制不同状态下的发布频率,初始化默认为一秒1次
        self.publish_timer = rospy.Timer(rospy.Duration(1.0), lambda event: self.publish_state())
        self.fault_check_timer = rospy.Timer(rospy.Duration(60.0), lambda event: self.check_and_clear_faults())
        # PID参数
        # self.pid_kp = 100.0
        # self.pid_ki = 0.1  # 如果需要加速响应，也可以适当调整积分增益
        # self.pid_kd = 0.5  # 如果系统有震荡，可以调整微分增益来抑制震荡
        self.pid_integral = 0.0
        self.pid_last_error = 0.0
        self.target_yaw = 0.0  # 期望偏航角（可根据需要设定）
        self.pid_kp = 100   # 降低比例增益减少振荡             原100
        self.pid_ki = 0.1  # 提高积分增益增强对持续偏差的纠正   1.5
        self.pid_kd = 10   # 大幅提高微分增益抑制快速变化       20 
        self.pid_correction_max = 150  # 放宽输出限制        200


        self.progress = 0   # 进度百分比，0-100
        self.battery_total_voltage = None  # 电池总电压
        self.battery_current = None  # 电池电流
        self.battery_remaining = None # 电池百分比
        self.battery_temperatures = [] # 电池温度，共3个
        #继电器状态
        self.relay_status = None

        self.state_pub = rospy.Publisher('/robot_state', String, queue_size=10)
        self.motor_cmd_pub = rospy.Publisher('/motor_cmd', Int8, queue_size=10)
        rospy.Subscriber('/robot_cmd', String, self.status_callback)
        rospy.Subscriber('/inspvae_data', INSPVAE, self.imu_callback)
        rospy.Subscriber('/battery_status', BatteryStatus, self.battery_status_callback)
        rospy.Subscriber('/relay_status', Bool, self.relay_callback)

        # self.fault_check_timer = rospy.Timer(rospy.Duration(5.0), lambda event: self.check_and_clear_faults())




    def set_state(self, new_state):
        if new_state not in self.status_config:
            rospy.logwarn(f"尝试设置无效状态: {new_state}")
            return False
        if new_state == self.current_status and new_state != "PISTON_OUT" and new_state != "PISTON_IN" and new_state != "STOP":
            return False  # 状态未改变
        if self.current_status in ["FORWARD", "BACKWARD"] and ( new_state == "CHARGE_OUT" or new_state == "RETURN_DOCK" ):
            return False  # 防止运行时异常状态干扰
            
        # 检查是否从START切换到其他模式
        # if self.current_status == "START" and new_state in ["FORWARD", "BACKWARD", "STOP"]:
        #     self.need_speed_mode_init = True
        # 自动模式记录当前状态，UPSTOP与LOWSTOP待确认
        # if self.auto_mode and new_state in ["FORWARD", "BACKWARD", "LOADING", "UNLOADING"]:
        if self.auto_mode and new_state in ["FORWARD", "BACKWARD"]:
            self.auto_step = new_state
            # print("auto_step:",self.auto_step)
        # 初始化为速度模式，添加恢复状态
        if new_state == "START" or new_state == "CHARGE_OUT" or new_state == "RETURN_DOCK":
            self.complete_state = False
            self.enable_drive_flag = True
            self.progress = 0
            self.motor_driver = True  # 电机驱动器状态
            self.imu_sensor = True  # IMU传感器状态
            self.main_board = True  # 主控板状态

        # STOP时3秒后，降低发布频率为半小时一次（30min*60=1800秒）    
        if new_state == "STOP":
            # self.auto_mode = False
            self.elevator_stage = 0
            if self.publish_timer is not None:
                self.publish_timer.shutdown()
                self.publish_timer = rospy.Timer(rospy.Duration(1.0), lambda event: self.publish_state())
            if self.fault_check_timer is not None:    
                self.fault_check_timer.shutdown()
                self.fault_check_timer = rospy.Timer(rospy.Duration(60.0), lambda event: self.check_and_clear_faults())

            threading.Thread(target=self.delayed_publish_freq_switch,args=(1,),daemon=True).start()
        else: #其他状态保持原频率
            if self.publish_timer is not None:
                self.publish_timer.shutdown()
                self.publish_timer = rospy.Timer(rospy.Duration(1.0), lambda event: self.publish_state())
            if self.fault_check_timer is not None:    
                self.fault_check_timer.shutdown()
                self.fault_check_timer = rospy.Timer(rospy.Duration(60.0), lambda event: self.check_and_clear_faults())

        # 进入运动状态时，只有当前不是REVERSE状态才更新prev_motion_state
        # if new_state in ["FORWARD", "BACKWARD", "LOADING", "UNLOADING"]:
        if new_state in ["FORWARD", "BACKWARD"]:
            if self.current_status != "REVERSE":  # 添加这个条件
                self.prev_motion_state = new_state  # 保存当前要进入的状态，而不是last_state
            # rospy.loginfo(f"保存的运动状态: {self.prev_motion_state}")
        # 进入反向调整、单侧停止时，记录当前运动状态
        if new_state in ["REVERSE", "UPSTOP", "LOWSTOP"]:
            if self.prev_motion_state is None:
                self.prev_motion_state = self.last_state
        
        elif new_state == "PISTON_IN":#清空自动模式重置初始偏航角
            # self.auto_step = None
            self.initial_yaw = None  # 重置初始偏航角与自动模式记录
            self.auto_step = None
        elif new_state == "PISTON_OUT": #切换手动与自动模式
            if( self.count % 2 ):
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
        self.motor_cmd_pub.publish(Int8(data=-1))  # 发布电机控制指令
        self.initial_yaw = None  # 重置初始偏航角


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
        try:
            # 更新计数器
            self.velocity_publish_count += 1
            
            # 只有在达到间隔时才调用get_actual_velocity获取新速度数据
            if self.velocity_publish_count >= self.velocity_publish_interval:
                # 获取新的速度数据
                self.last_velocity_up = self.get_actual_velocity(3)
                self.last_velocity_low = self.get_actual_velocity(2)
                self.last_velocity_brush = self.get_actual_velocity(4)
                self.velocity_publish_count = 0  # 重置计数器
            
            # 始终使用最新的速度值（可能是新获取的，也可能是之前保存的）
            velocity_up = self.last_velocity_up
            velocity_low = self.last_velocity_low
            velocity_brush = self.last_velocity_brush

            state_msg = {
                "status": self.current_status,
                "battery": self.battery_remaining, # 电池百分比,
                "battery_temperatures": self.battery_temperatures, # 电池温度，共3个
                "battery_total_voltage": self.battery_total_voltage, # 电池总电压
                "battery_current": self.battery_current, # 电池电流
                "progress": self.progress,
                "imu_yaw": round(self.imu_yaw, 2) if self.imu_yaw is not None else 0.00,
                "velocity_up": round(velocity_up / rate, 2),  # 保留两位小数，数值类型
                "velocity_low": round(velocity_low / rate, 2),
                "velocity_brush": round(velocity_brush / rate, 2),
                # "velocity_locking": 0,
                "sensors_status": self.sensors_status,  # 超声波传感器状态
                "device_status": {
                "main_board": self.main_board,
                "imu_sensor": self.imu_sensor,
                "motor_driver": self.motor_driver,
                "comm_module": True  },
                "complete_state":self.complete_state, # 任务完成状态
                "auto_mode": self.auto_mode, # 自动模式开关,默认开
                "relay_status": self.relay_status,
                # "auto_step": self.auto_step, # 当前自动程序所在状态
                "timestamp": time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))  # 2025-07-15 14:58:43
            }
            self.state_pub.publish(json.dumps(state_msg))
        except Exception as e:
            error_msg = {
                "status": "ERROR",  # 或者自定义异常内容
                "error_detail": str(e),
                "traceback": traceback.format_exc(),
                "timestamp": time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))}  # 2025-07-15 14:58:43
            self.state_pub.publish(json.dumps(error_msg))

    def status_callback(self, msg):
        """处理状态消息"""
        try:
            cmd_obj = json.loads(msg.data)
            command = cmd_obj.get("command", None)
            if command == "GET_STATUS":
                self.publish_state()
                # return
            elif command in self.status_list:
                # print("cmd:", command)
                self.set_state(command)  
            else:
                rospy.logwarn(f"未找到command字段: {msg.data}")
            
        except Exception as e:
            rospy.logwarn(f"消息解析失败，尝试按字符串处理: {msg.data}, 错误: {e}")
            self.set_state(msg.data)
            self.publish_state()

    def imu_callback(self, msg):
        """处理IMU数据"""
        try:
            self.imu_yaw = msg.yaw if hasattr(msg, "yaw") else msg.get("yaw", 0.0)
            # if self.imu_yaw > 180:
            #     self.imu_yaw -= 360
            if self.initial_yaw is None:
                self.initial_yaw = self.imu_yaw
                rospy.loginfo(f"Initial IMU yaw set to: {self.initial_yaw} degrees")
            
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
    def battery_status_callback(self, msg):

        self.battery_remaining = msg.batttery_remaining  # 电池百分比
        self.battery_total_voltage = round(msg.total_voltage, 2) 
        self.battery_current = round(msg.current, 2)
         # 格式化温度列表，保留一位小数
        self.battery_temperatures = [round(t, 1) for t in msg.temperatures] if hasattr(msg, "temperatures") else []

    def relay_callback(self, msg):
        #继电器状态
        self.relay_status = msg.data


    def create_can_bus(self):
        """创建CAN总线连接，失败时重试"""
        while True:
            try:
                return can.interface.Bus(channel=self.channel, interface=self.interface)
            except (can.CanError, OSError) as e:
                rospy.logerr(f"motor_canopen CAN连接失败: {e}，3秒后重试...")
                time.sleep(3)

    def reconnect_can_bus(self):
        """重连CAN总线"""
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
        # print(f"设置电机 {motor_id} 目标速度: {velocity} RPM")
        self.send_command(motor_id, data)

    def get_actual_velocity(self, motor_id):
        """
        读取电机的实际运行速度 (606Ch, 单位: puu/s)
        :param motor_id: 电机ID
        :return: 实际速度值 (脉冲/秒)，读取失败返回None
        """
        # 发送读取606Ch的指令，16字节响应数据
        self.send_command(motor_id, [0x40, 0x6C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        
        # 等待接收响应
        start_time = time.time()
        while time.time() - start_time < 0.5:  # 超时500ms
            try:
                msg = self.bus.recv(timeout=0.1)
            except (can.CanError, OSError) as e:
                rospy.logerr(f"CAN接收错误: {e}，尝试重连...")
                self.reconnect_can_bus()
                continue
            if msg and msg.arbitration_id == (0x580 + motor_id):
                # 检查是否为有效的606Ch响应
                if len(msg.data) >= 8 and msg.data[0] == 0x43 and msg.data[1] == 0x6C and msg.data[2] == 0x60:
                    # 解析32位速度值 (Int32)
                    velocity = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
                    
                    # 判断数值是否为负数（16位有符号数）
                    if velocity > 0x7FFFFFFF:
                        velocity -= 0x100000000

                    # 转换为速度（RPM）
                    # 注意：1 RPM = 68 pulses per second（因为rate = 68 Hz）
                    rpm = abs(velocity) / 68
                
                    # rospy.loginfo(f"电机 {motor_id} 实际速度: {velocity} puu/s | 约 {rpm} rpm")
                    return velocity  # 返回32位整数形式
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
        # 检查电机故障
        fault_code = self.read_fault_code(motor_id)
        # saving_mode = self.read_energy_saving_mode(motor_id)
        # set_torque_zero_param = self.set_torque_zero_param(motor_id,"Fn_04c",100)
        # max_torque = self.get_max_torque(motor_id)
        # actual_torque = self.get_actual_torque(motor_id)
        if fault_code and fault_code != 0:
            rospy.logwarn(f"电机 {motor_id} 存在故障 (0x{fault_code:04X}), 尝试清除...")
            self.clear_fault(motor_id)
            time.sleep(0.3)  # 等待故障清除

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
        rospy.loginfo(f"接收到按键: {key}")  # 添加这行日志来确认接收到的按键值
        key_mapping = {
            's': "STOP",
            'f': "FORWARD",
            'b': "BACKWARD",
            'a': "START",  # 速度模式初始化并使能
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
            # self.auto_mode = (key == 'a')
        else:
            rospy.loginfo(f"无效按键: {key}")

    def proximity_callback(self, msg):
        """4路接近开关检测回调"""
        if msg.sensor_a:
            self.sensors_status |= 0x01  # 设置传感器A状态
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
        
        if self.auto_mode and self.current_status == "RETURN_DOCK": 
            #回仓
            if self.elevator_stage == 2:
                self.set_state("FORWARD")

        if self.auto_mode and self.current_status == "CHARGE_OUT": 
            if self.elevator_stage == 2:
                if (msg.sensor_a or msg.sensor_c): #仅一个就可以开启自动
                    #定时出仓，等待10秒后进入后退
                    if self.current_status != "UNLOADING":
                        self.set_state("UNLOADING")
                        self.progress = 10
                        self.unloading_start_time = time.time()  # 记录开始时间
                        # print("unloading_start_time:",self.unloading_start_time)
                else:
                    rospy.loginfo("充电状态，等待接近开关触发")
                # 在UNLOADING状态，检查定时器
                if self.current_status == "UNLOADING":
                    while hasattr(self, 'unloading_start_time') and self.unloading_start_time is not None:
                        current_time = time.time()
                        elapsed = current_time - self.unloading_start_time
                        # print(f"已等待: {elapsed:.2f}秒, 目标: {self.unloading_timer}秒")
                        
                        if elapsed >= self.unloading_timer and self.current_status == "UNLOADING":
                            self.set_state("STOP")
                            self.progress = 0
                            self.unloading_start_time = None  # 重置
                            #停止IMU
                            self.stop_imu()

        #自动模式第一步 >> START
        if self.auto_mode and self.current_status == "START": 
            # 检测起始位置，进入第一步动作，有待测试
            if self.elevator_stage == 2:
                if (msg.sensor_a or msg.sensor_c): #仅一个就可以开启自动
                    #定时出仓，等待10秒后进入后退
                    if self.current_status != "UNLOADING":
                        self.set_state("UNLOADING")
                        self.progress = 10
                        self.unloading_start_time = time.time()  # 记录开始时间
                        # print("unloading_start_time:",self.unloading_start_time)
                elif(not msg.sensor_a and not msg.sensor_c):
                    self.set_state("BACKWARD")
                    self.progress = 20
        
                # 在UNLOADING状态，检查定时器
                if self.current_status == "UNLOADING":
                    while hasattr(self, 'unloading_start_time') and self.unloading_start_time is not None:
                        current_time = time.time()
                        elapsed = current_time - self.unloading_start_time
                        # print(f"已等待: {elapsed:.2f}秒, 目标: {self.unloading_timer}秒")
                        
                        if elapsed >= self.unloading_timer and self.current_status == "UNLOADING":
                            self.set_state("BACKWARD")
                            self.progress = 20
                            self.unloading_start_time = None  # 重置

        
        # 在REVERSE状态下检测边界
        if self.current_status == "REVERSE":
            # 左侧前后传感器（a和c）同时触发
            if msg.sensor_a and msg.sensor_c:
                rospy.logwarn("左侧边界全触发，立即STOP")
                self.set_state("STOP")
            # 右侧前后传感器（b和d）同时触发
            elif msg.sensor_b and msg.sensor_d:
                rospy.logwarn("右侧边界全触发，立即STOP")
                self.set_state("STOP")
            # 仅左侧传感器触发
            elif msg.sensor_a or msg.sensor_b:
                rospy.logwarn("下侧边界触发，进入LOWSTOP")
                self.set_state("LOWSTOP")
            # 仅右侧传感器触发
            elif msg.sensor_c or msg.sensor_d:
                rospy.logwarn("上侧边界触发，进入UPSTOP")
                self.set_state("UPSTOP")
            
        # 自动与手动模式下的检测
        if self.auto_mode: # 自动模式开启，完善：第一步START>BACKWARD>FORWARD>STOP 
            if self.current_status == self.status_list[1]:  # FORWARD
                if msg.sensor_a and msg.sensor_c:
                    #清空自动流程状态
                    threading.Timer(5.0, self.lock_motor).start()
                    self.set_state("STOP")
                    self.complete_state = True
                    # self.initial_yaw = None  # 重置初始偏航角
                    self.progress = 100
                    self.auto_step = None
                    self.elevator_stage = 0  # 重置电缸阶段
                    rospy.loginfo("——————————————————————进仓完成——————————————————————")
                    # 完成任务后关闭IMU
                    self.stop_imu()
                    rospy.logwarn("---------- 强制刷新缓冲区开始 ----------")
                    for i in range(50):
                        rospy.logerr("DUMMY MESSAGE %d/5: Flushing buffer before exit." % (i+1))
                    rospy.logwarn("---------- 强制刷新缓冲区结束 ----------")
                elif (msg.sensor_a and not msg.sensor_c):
                    self.set_state("LOWSTOP")
                elif (msg.sensor_c and not msg.sensor_a):
                    self.set_state("UPSTOP")

            if self.current_status == self.status_list[2]:  # BACKWARD
                if (msg.sensor_b and msg.sensor_d):
                    #自动程序：第一步检测接近开关到位>后退>到边缘(可加入对正程序?)自动切换前进>直到进仓>发布完成消息>STOP停止使能。
                    self.initial_yaw = None  # 在对侧执行，重置初始偏航角
                    rospy.loginfo("在对侧执行，重置初始偏航角")
                    self.set_state("LOADING") # 单滚刷运行，之后切换状态
                    time.sleep(3)
                    self.set_state("FORWARD")
                    self.progress = 60
                elif (msg.sensor_b and not msg.sensor_d):
                    self.set_state("LOWSTOP")
                elif (msg.sensor_d and not msg.sensor_b):
                    self.set_state("UPSTOP")

            
        else: # 手动模式，仅在前进与后退中切换
            if self.current_status == self.status_list[1]:  # FORWARD
                if msg.sensor_a and msg.sensor_c:
                    threading.Timer(5.0, self.lock_motor).start()
                    self.set_state("STOP")
                    time.sleep(1)
                    #清空自动流程状态
                    self.complete_state = True
                    # self.initial_yaw = None  # 重置初始偏航角
                    self.progress = 100
                    self.auto_step = None
                    self.elevator_stage = 0  # 重置电缸阶段
                    self.stop_imu()

                elif (msg.sensor_a and not msg.sensor_c):
                    self.set_state("LOWSTOP")

                elif (msg.sensor_c and not msg.sensor_a):
                    self.set_state("UPSTOP")

                # if (msg.sensor_a or msg.sensor_c): # 无仓时不可用
                #     self.set_state("STOP")
                #     self.progress = 0
            elif self.current_status == self.status_list[2]:  # BACKWARD
                if msg.sensor_b and msg.sensor_d:
                    self.set_state("STOP")
                    time.sleep(1)
                    #清空自动流程状态
                    self.complete_state = True
                    # self.initial_yaw = None  # 重置初始偏航角
                    self.progress = 100
                    self.auto_step = None
                    self.elevator_stage = 0  # 重置电缸阶段

                elif (msg.sensor_b and not msg.sensor_d):
                    self.set_state("LOWSTOP")

                elif (msg.sensor_d and not msg.sensor_b):
                    self.set_state("UPSTOP")
        

        #当单侧电机停止时，等待另一侧到位后切换状态
        if self.current_status == self.status_list[7]: #LOWSTOP 
                #确保停到位
            if msg.sensor_c:
                threading.Timer(5.0, self.lock_motor).start()
                self.set_state("STOP")
                # time.sleep(1)
                #清空自动流程状态
                self.complete_state = True
                self.initial_yaw = None  # 重置初始偏航角
                self.progress = 100
                self.auto_step = None
                self.is_lowstop = False
                self.elevator_stage = 0  # 重置电缸阶段
                self.stop_imu()

                rospy.loginfo("——————————————————————由LOWSTOP至进仓完成——————————————————————")
                rospy.logwarn("---------- 强制刷新缓冲区开始 ----------")
                for i in range(50):
                    rospy.logerr("DUMMY MESSAGE %d/5: Flushing buffer before exit." % (i+1))
                rospy.logwarn("---------- 强制刷新缓冲区结束 ----------")

            else:
                self.complete_state = False
            if msg.sensor_d:
                self.initial_yaw = None  # 在对侧执行，重置初始偏航角
                rospy.loginfo("在LOWSTOP执行，对侧重置初始偏航角")
                self.set_state("LOADING") # 单滚刷运行，之后切换状态
                time.sleep(3)
                self.set_state("FORWARD")
                self.progress = 60
        if self.current_status == self.status_list[6]: #UPSTOP 
                #确保停到位
            if msg.sensor_a:
                threading.Timer(5.0, self.lock_motor).start()
                self.set_state("STOP")
                # time.sleep(1)
                #清空自动流程状态
                self.complete_state = True
                self.initial_yaw = None  # 重置初始偏航角
                self.progress = 100
                self.auto_step = None
                self.is_upstop = False
                self.elevator_stage = 0  # 重置电缸阶段
                self.stop_imu()
                rospy.loginfo("——————————————————————由UPSTOP至进仓完成——————————————————————")
                rospy.logwarn("---------- 强制刷新缓冲区开始 ----------")
                for i in range(50):
                    rospy.logerr("DUMMY MESSAGE %d/5: Flushing buffer before exit." % (i+1))
                rospy.logwarn("---------- 强制刷新缓冲区结束 ----------")
            else:
                self.complete_state = False
            if msg.sensor_b:
                self.initial_yaw = None  # 在对侧执行，重置初始偏航角
                rospy.loginfo("在UPSTOP执行，对侧重置初始偏航角")
                self.set_state("LOADING") # 单滚刷运行，之后切换状态
                time.sleep(3)
                self.set_state("FORWARD")
                self.progress = 60

        # if self.current_status == self.status_list[4]:  # LOADING 未使用
        # # if self.current_status == "UNLOADING" and self.side_detected:  # 边缘LOADING、UNLOADING
        #     if msg.sensor_a or msg.sensor_c:
        #         self.set_state("STOP")
        #         time.sleep(1)
        #         #清空自动流程状态
        #         self.complete_state = True
        #         self.initial_yaw = None  # 重置初始偏航角

        #         self.progress = 100
        #         self.auto_step = None
        #     elif (msg.sensor_a and not msg.sensor_c):
        #         self.set_state("LOWSTOP")

        #     elif (msg.sensor_c and not msg.sensor_a):
        #         self.set_state("UPSTOP")
                
        
    # def pid_correction(self, current_yaw):
    #     """根据IMU当前偏航角进行PID矫正，返回速度修正量"""
    #     error = self.target_yaw - current_yaw
    #     self.pid_integral += error
    #     derivative = error - self.pid_last_error
    #     if abs(error) > 0.2:  # 如果误差小于0.2度，则不进行修正
    #         correction = (self.pid_kp * error +
    #                     self.pid_ki * self.pid_integral +
    #                     self.pid_kd * derivative) 
    #         # * 10  # 放大修正量
    #     else:
    #         correction = 0  # 在小范围内不进行调整

    #     self.pid_last_error = error
    #     correction = max(min(correction, self.pid_correction_max), -self.pid_correction_max)  # 限制修正量在-100到100之间
    #     return correction
    #     # return correction if self.current_status in ["FORWARD","LOADING"] else -correction

    def pid_correction(self, current_yaw):
        """改进的PID矫正方法"""
        error = self.target_yaw - current_yaw
        
        # 死区控制 - 增大死区范围
        if abs(error) < 0.05:
            return 0
        
        # 抗积分饱和 - 大偏差时清零积分
        if abs(error) > 0.3: #5
            self.pid_integral = 0
        
        # PID计算
        self.pid_integral += error
        derivative = error - self.pid_last_error
        
        # 积分限幅
        integral_max = 30   #300
        # self.pid_integral = 0
        self.pid_integral = max(min(self.pid_integral, integral_max), -integral_max)
        # print("pid_integral:",self.pid_integral)
        correction = (self.pid_kp * error +
                    self.pid_ki * self.pid_integral +
                    self.pid_kd * derivative)
        
        self.pid_last_error = error
        return max(min(-correction, self.pid_correction_max), -self.pid_correction_max)

    def execute_state(self, event=None):
        # 实时根据当前状态和IMU矫正上下轮速度
        # 1. START状态：速度模式初始化电机
        if self.enable_drive_flag and (self.current_status =="START" or self.current_status =="CHARGE_OUT" or self.current_status =="RETURN_DOCK"):  # START, CHARGE_OUT, RETURN_DOCK
            if self.battery_remaining is not None and self.battery_remaining < self.LOW_BATTERY_THRESHOLD \
                and (self.current_status =="START" or self.current_status =="STOP"):
                rospy.logerr("电池电量过低，无法启动电机！请充电后重试!")
                self.enable_drive_flag = False
                self.main_board = False # 主控板报警表示电量低于阈值无法启动
                self.set_state("STOP")
                self.stop_imu()
                return

            # 阶段0: 开始抬升电缸
            if self.elevator_stage == 0:
                rospy.loginfo("电缸抬起...")
                self.motor_cmd_pub.publish(Int8(data=1))  # 发布电机控制指令
                self.elevator_start_time = rospy.get_time()  # 记录抬升开始时间
                self.elevator_stage = 1  # 进入抬升中阶段
                #同步开启IMU
                self.start_imu()
                
            # 阶段1: 等待电缸完成抬升(非阻塞检查)
            elif self.elevator_stage == 1:
                elapsed = rospy.get_time() - self.elevator_start_time
                
                # 等待20秒完成电缸抬升
                if elapsed >= 20:
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
                    self.elevator_stage = 2  # 完成抬升和初始化
                    
                    # 启动20秒等待检查计时器(代替sleep)
                    self.start_time = rospy.get_time()
                    if self.auto_mode and self.auto_step and self.current_status =="START":
                        rospy.loginfo(f"初始化完成，恢复自动流程: {self.auto_step}")
                        self.set_state(self.auto_step)
                else:
                    # 实时显示剩余时间c
                    remaining = max(0, 20 - elapsed)
                    # rospy.loginfo(f"等待电缸抬起: 还剩 {remaining:.1f}秒")
        
        # 阶段2: 自动恢复：等待20秒后开始自动模式第一步
        # if self.elevator_stage == 2:
           
            # 初始化完成后自动切换到auto_step
            # if self.auto_mode and self.auto_step:
            #     rospy.loginfo(f"初始化完成，恢复自动流程: {self.auto_step}")
            #     self.set_state(self.auto_step)
            # return
            

        # 2. FORWARD/BACKWARD状态：IMU矫正
        if self.current_status in ["FORWARD", "BACKWARD"]:
            correction = self.pid_correction(self.imu_yaw) * rate
            left_speed = int(self.status_config[self.current_status]["velocity_up"] + correction)
            right_speed = int(self.status_config[self.current_status]["velocity_low"] + correction)
            brush_speed = self.status_config[self.current_status]["velocity_brush"]
            right_speed = max(min(right_speed, self.speed_pluse_max), -self.speed_pluse_max)
            left_speed = max(min(left_speed, self.speed_pluse_max), -self.speed_pluse_max)

            # rospy.loginfo(f"IMU矫正: yaw={self.imu_yaw:.2f}, correction={correction:.2f}")
            if (self.last_left_speed != left_speed or
                self.last_right_speed != right_speed or
                self.last_brush_speed != brush_speed):
                # rospy.loginfo(f"IMU矫正: yaw={self.imu_yaw:.2f}, correction={correction:.2f}")
                # rospy.loginfo(f"上轮速度: {left_speed}, 下轮速度: {right_speed}")
                
                self.set_target_velocity(3, left_speed)
                self.set_target_velocity(2, right_speed)
                self.set_target_velocity(4, brush_speed)
                self.last_left_speed = left_speed
                self.last_right_speed = right_speed
                self.last_brush_speed = brush_speed
            # 检查是否需要进入后退矫正状态(根据实际情况调整角度)
            # rospy.loginfo(f"在execute中的：{self.prev_motion_state}")
            # if self.auto_mode:
            # if self.prev_motion_state != "REVERSE":
            #     if -5 < self.imu_yaw < -2 or 2 < self.imu_yaw < 5:
            #         self.set_state("REVERSE")  # 进入后退矫正状态
            # else:
            #     if -5 < self.imu_yaw < -3 or 3 < self.imu_yaw < 5:
            #         self.set_state("REVERSE")  # 放大角度限制，防止再次进入后退矫正状态
            

            angle_condition_met = (-5 < self.imu_yaw < -2.5 or 2.5 < self.imu_yaw < 5)
        
            if angle_condition_met:
                # 第一次检测到角度问题时记录时间
                if self.reversed_start_time is None:
                    self.reversed_start_time = rospy.get_time()
                    rospy.logwarn(f"检测到角度偏差: {self.imu_yaw:.2f}度，开始计时...")
                
                # 检查是否已经达到时间阈值
                elapsed = rospy.get_time() - self.reversed_start_time
                if elapsed >= self.REVERSE_TIME_THRESHOLD:
                    # 满足3秒条件，触发REVERSE状态
                    rospy.logwarn(f"角度偏差已持续{elapsed:.1f}秒，进入REVERSE状态")
                    self.set_state("REVERSE")
                    self.reversed_start_time = None  # 重置计时器
            else:
                # 角度偏差不满足条件，重置计时器
                if self.reversed_start_time is not None:
                    rospy.loginfo(f"角度偏差消失({self.imu_yaw:.2f}°)，重置计时器")
                    self.reversed_start_time = None


            # 实时发布状态
            # self.current_velocity_up = left_speed
            # self.current_velocity_low = right_speed
            # self.current_velocity_brush = brush_speed

        # 3. 反向调整状态
        elif self.current_status == "REVERSE":
            # print("self.prev_motion_state:",self.prev_motion_state)
            self.reversed_start_time = None
            # 执行后退矫正
            if not self.has_reverse_flag:
                self.has_reverse_counter += 1  # 标记后退次数
                if self.has_reverse_counter > 10:  # 连续后退10次后
                    rospy.logwarn("连续后退10次，可能需要手动干预")
                    self.has_reverse_counter = 0
                    self.set_state("STOP")  # 停止后退
                    self.motor_driver = False  # 预警
                    self.imu_sensor = False
                    return
                #考虑使用固定速度
                # correction = self.pid_correction(self.imu_yaw) * rate
                right_speed = -int(self.last_right_speed ) # >> 两轮反向运行进行调整
                left_speed = -int(self.last_left_speed )
                brush_speed = self.last_brush_speed
                right_speed = max(min(right_speed, self.speed_pluse_max), -self.speed_pluse_max)
                left_speed = max(min(left_speed, self.speed_pluse_max), -self.speed_pluse_max)
                if (self.last_left_speed != left_speed or
                self.last_right_speed != right_speed or
                self.last_brush_speed != brush_speed):
                    rospy.loginfo(f"IMU矫正: yaw={self.imu_yaw:.2f}")
                    rospy.loginfo(f"后退上轮速度: {left_speed}, 下轮速度: {right_speed}")
                    # 设置速度
                    self.set_target_velocity(3, left_speed)
                    self.set_target_velocity(2, right_speed)
                    self.set_target_velocity(4, brush_speed)
                # 更新最后速度记录
                self.last_left_speed = left_speed
                self.last_right_speed = right_speed
                self.last_brush_speed = brush_speed
                
                # 标记已执行后退
                self.has_reverse_flag = True
                self.reverse_start_time = time.time()  # 记录后退开始时间
                time.sleep(2.0)
            else:
                if self.imu_yaw >= 0:
                    self.flag = -1
                elif self.imu_yaw < 0:
                    self.flag = 1
                # 使用更平滑的速度调整方式
                if abs(self.imu_yaw) > 1:  # 如果角度偏差较大
                # 根据偏差方向调整轮速
                    right_speed = left_speed = int (-self.base_speed * 0.8 * self.flag)  # 基础后退速度+校正
                else:
                    # 角度接近时减速
                    right_speed = left_speed = int(-self.base_speed * 0.6 * self.flag)
                right_speed = max(min(right_speed, self.speed_pluse_max), -self.speed_pluse_max)
                left_speed = max(min(left_speed, self.speed_pluse_max), -self.speed_pluse_max)
                brush_speed = self.last_brush_speed
                if (self.last_left_speed != left_speed or
                self.last_right_speed != right_speed or
                self.last_brush_speed != brush_speed):
                    rospy.loginfo(f"IMU矫正: yaw={self.imu_yaw:.2f}")
                    rospy.loginfo(f"后退完毕上轮速度: {left_speed}, 下轮速度: {right_speed}")
                
                    # 设置速度
                    self.set_target_velocity(3, left_speed)
                    self.set_target_velocity(2, right_speed)
                    self.set_target_velocity(4, brush_speed)
                
                # 更新最后速度记录
                self.last_left_speed = left_speed
                self.last_right_speed = right_speed
                self.last_brush_speed = brush_speed
                
                # 检查是否满足恢复条件
                current_time = time.time()
                # 条件1: 角度满足要求
                # 条件2: 已经后退了足够时间（例如2秒） 默认2  去除
                # if abs(self.imu_yaw) < 0.2 and (current_time - self.reverse_start_time > 2.5):
                if abs(self.imu_yaw) < 0.05:
                    if self.prev_motion_state:
                        # 保存要恢复的状态，避免在set_state中被重置
                        restore_state = self.prev_motion_state
                        self.prev_motion_state = None  # 先重置，避免在set_state中冲突
                        self.set_state(restore_state)
                    self.is_upstop = False
                    self.is_lowstop = False
                    self.has_reverse_flag = False  # 重置标志位
                    # self.has_reverse_counter = 0  # 重置后退计数器
            
            # self.current_velocity_up = left_speed
            # self.current_velocity_low = right_speed
            # self.current_velocity_brush = brush_speed

        # 4. UPSTOP/LOWSTOP状态：IMU矫正+保持切换前速度
        elif self.current_status == "UPSTOP":
            left_speed = 0 # 上电机停
            # right_speed = int(-self.speed_pluse_max * 1)  # 下轮保持切换前速度
            right_speed = self.last_right_speed  # 下轮保持切换前速度
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

            # self.current_velocity_up = left_speed
            # self.current_velocity_low = right_speed
            # self.current_velocity_brush = brush_speed               
        elif self.current_status == "LOWSTOP":
            # left_speed = int(self.speed_pluse_max * 1) # 上电机保持切换前速度 
            left_speed =  self.last_left_speed  # 上电机保持切换前速度 
            right_speed = 0 # 下电机停
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

           
            # self.current_velocity_up = left_speed
            # self.current_velocity_low = right_speed
            # self.current_velocity_brush = brush_speed

        # 5. STOP状态或IMU角度异常
        elif self.current_status == "STOP" or not -5 < self.imu_yaw < 5:
            if (self.last_left_speed != 0 or
                self.last_right_speed != 0 or
                self.last_brush_speed != 0):
                self.has_reverse_counter = 0  # 重置后退计数器

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


            # self.current_velocity_low = 0
            # self.current_velocity_up = 0
            # self.current_velocity_brush = 0
            # self.publish_state()
        elif self.current_status == "START" or self.current_status == "CHARGE_OUT":
            config = self.status_config["START"]
            

        # 5. LOADING/UNLOADING状态：IMU矫正+边缘检测 未使用
        elif self.current_status == "UNLOADING":
            correction = self.pid_correction(self.imu_yaw) * rate
            left_speed = int(self.status_config[self.current_status]["velocity_up"] + correction)
            right_speed = int(self.status_config[self.current_status]["velocity_low"] + correction)
            brush_speed = self.status_config[self.current_status]["velocity_brush"]
            if (self.last_left_speed != left_speed or
                self.last_right_speed != right_speed or
                self.last_brush_speed != brush_speed):
                # rospy.loginfo(f"IMU矫正: yaw={self.imu_yaw:.2f}, correction={correction:.2f}")
                # rospy.loginfo(f"上轮速度: {left_speed}, 下轮速度: {right_speed}")
                
                self.set_target_velocity(3, left_speed)
                self.set_target_velocity(2, right_speed)
                self.set_target_velocity(4, brush_speed)
                self.last_left_speed = left_speed
                self.last_right_speed = right_speed
                self.last_brush_speed = brush_speed
        # 6.   LOADING 状态仅滚刷运动   
        elif self.current_status == "LOADING":
            left_speed = int(self.status_config[self.current_status]["velocity_up"])
            right_speed = int(self.status_config[self.current_status]["velocity_low"])
            brush_speed = self.status_config[self.current_status]["velocity_brush"]
            if (self.last_left_speed != left_speed or
                self.last_right_speed != right_speed or
                self.last_brush_speed != brush_speed):
                # rospy.loginfo(f"IMU矫正: yaw={self.imu_yaw:.2f}, correction={correction:.2f}")
                # rospy.loginfo(f"上轮速度: {left_speed}, 下轮速度: {right_speed}")
                
                self.set_target_velocity(3, left_speed)
                self.set_target_velocity(2, right_speed)
                self.set_target_velocity(4, brush_speed)
                self.last_left_speed = left_speed
                self.last_right_speed = right_speed
                self.last_brush_speed = brush_speed

        # 实时发布状态
        # self.current_velocity_up = self.get_actual_velocity(3)
        # self.current_velocity_low = self.get_actual_velocity(2)
        # self.current_velocity_brush = self.get_actual_velocity(4)
    
    def delayed_publish_freq_switch(self, delay_sec=3):
        # 延时后切换到低频率
        time.sleep(delay_sec)
        if self.current_status == "STOP":
            if self.publish_timer is not None:
                self.publish_timer.shutdown()
            self.publish_timer = rospy.Timer(rospy.Duration(1800), lambda event: self.publish_state())
            if self.fault_check_timer is not None:
                self.fault_check_timer.shutdown()
            self.fault_check_timer = rospy.Timer(rospy.Duration(7200), lambda event: self.check_and_clear_faults())
    def get_max_torque(self, motor_id):
        """
        读取配置的最大转矩 (6072h)
        :param motor_id: 电机ID
        :return: 最大转矩值 (千分之一额定转矩)，读取失败返回None
        """
        # 发送读取对象字典命令 (索引6072h, 子索引00h)
        self.send_command(motor_id, [0x40, 0x72, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        
        # 等待回复
        start_time = time.time()
        while time.time() - start_time < 0.5:  # 500ms超时
            msg = self.bus.recv(0.1)  # 100ms等待
            if msg and msg.arbitration_id == (0x580 + motor_id):
                # 检查是否是正确的返回数据
                if len(msg.data) >= 6 and msg.data[0] in [0x4B, 0x43]:
                    # 16位返回值 (Uint16)
                    # print(msg)
                    max_torque = msg.data[4] | (msg.data[5] << 8)
                    # rospy.logwarn(f"读取电机最大转矩 {max_torque} ")

                    return max_torque
        # rospy.logwarn(f"读取电机 {motor_id} 最大转矩超时")
        return None

    def get_actual_torque(self, motor_id):
        """
        读取实际转矩 (6077h)
        :param motor_id: 电机ID
        :return: 实际转矩值 (‰额定转矩)，读取失败返回None
        """
        self.send_command(motor_id, [0x40, 0x77, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        
        # 等待回复
        start_time = time.time()
        while time.time() - start_time < 0.5:  # 500ms超时
            msg = self.bus.recv(0.1)  # 100ms等待
            if msg and msg.arbitration_id == (0x580 + motor_id):
                # 验证数据长度和命令字节
                if len(msg.data) >= 6 and msg.data[0] == 0x4B and msg.data[1] == 0x77 and msg.data[2] == 0x60:
                    # 解析16位转矩值 (Int16)
                    torque = msg.data[4] | (msg.data[5] << 8)
                    # 处理有符号数 (16位有符号整数)
                    if torque > 0x7FFF:
                        torque -= 0x10000
                    rospy.logwarn(f"读取电机 {motor_id} 实际转矩为 {torque/1000} 额定转矩")
                    return torque
        rospy.logwarn(f"读取电机 {motor_id} 实际转矩超时")
        return None
    def get_max_current(self, motor_id):
        """
        读取最大允许电流 (6073h)
        :param motor_id: 电机ID
        :return: 驱动器输出最大转矩时的电流 (单位：‰额定电流)，读取失败返回None
        """
        # 发送读取对象字典命令 (索引6073h, 子索引00h)
        self.send_command(motor_id, [0x40, 0x73, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        
        # 等待回复
        start_time = time.time()
        while time.time() - start_time < 0.5:  # 500ms超时
            msg = self.bus.recv(0.1)  # 100ms等待
            if msg and msg.arbitration_id == (0x580 + motor_id):
                # 检查是否正确返回
                if len(msg.data) >= 6 and msg.data[0] == 0x43:
                    # 16位返回值 (Uint16)
                    current = msg.data[4] | (msg.data[5] << 8)
                    rospy.logwarn(f"读取电机 {motor_id} 最大电流: {current/1000}额定电流")
                    return current
        rospy.logwarn(f"读取电机 {motor_id} 最大电流超时")
        return None
    def get_actual_current(self, motor_id):
        """
        读取实际输出电流 (6078h)
        :param motor_id: 电机ID
        :return: 实际电流 (‰额定电流)，读取失败返回None
        """
        # 发送读取对象字典命令 (索引6078h, 子索引00h)
        self.send_command(motor_id, [0x40, 0x78, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        
        # 等待回复
        start_time = time.time()
        while time.time() - start_time < 0.5:  # 500ms超时
            msg = self.bus.recv(0.1)  # 100ms等待
            if msg and msg.arbitration_id == (0x580 + motor_id):
                # 检查是否正确返回
                if len(msg.data) >= 6 and msg.data[0] == 0x4B and msg.data[1] == 0x78 and msg.data[2] == 0x60:
                    # 16位返回值 (Int16)
                    current = msg.data[4] | (msg.data[5] << 8)
                    # 处理有符号数
                    if current > 0x7FFF:
                        current -= 0x10000
                    rospy.logwarn(f"读取电机 {motor_id} 实际电流: {current/1000}额定电流")
                    return current
        rospy.logwarn(f"读取电机 {motor_id} 实际电流超时")
        return None
    def read_fault_code(self, motor_id):
        """读取电机故障码"""
        # 发送读取故障码指令
        self.send_command(motor_id, [0x40, 0x3F, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        # if self.get_actual_velocity(motor_id) != 0:
            # self.motor_driver = True
        # 接收回复
        start_time = time.time()
        while time.time() - start_time < 0.5:  # 500ms超时
            msg = self.bus.recv(0.1)  # 100ms等待
            if msg and msg.arbitration_id == (0x580 + motor_id):
                # print("fault_code_data:",msg.data)
                # 验证 SDO 响应类型（0x43 表示读取成功）
                if len(msg.data) >= 6 and msg.data[1] == 0x3F and msg.data[1] == 0x3F and msg.data[2] == 0x60:
                    fault_code = msg.data[4] | (msg.data[5] << 8)
                    # print("msg.data:", msg.data)
                    if fault_code != 0:
                        rospy.logwarn(f"电机 {motor_id} 故障码: 0x{fault_code:04X} ({fault_code})")
                        self.motor_driver = False   # 提示电机故障
                    else:
                        rospy.loginfo(f"电机 {motor_id} 无故障")
                        self.motor_driver = True
                    return fault_code
        rospy.logwarn(f"读取电机 {motor_id} 故障码超时")
        return None

    def clear_fault(self, motor_id):
        """清除电机故障"""
        rospy.loginfo(f"尝试清除电机 {motor_id} 故障...")
        
        # 步骤1: 发送故障复位激活命令 (bit7=1)
        self.send_command(motor_id, [0x2B, 0x40, 0x60, 0x00, 0x8F, 0x00, 0x00, 0x00])
        time.sleep(0.1)
        
        # 步骤2: 发送故障复位完成命令 (bit7=0)
        self.send_command(motor_id, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00])
        rospy.loginfo(f"电机 {motor_id} 故障复位指令已发送")
        
        # 验证故障是否清除
        fault_code = self.read_fault_code(motor_id)
        if fault_code == 0 or fault_code is None:
            rospy.loginfo(f"电机 {motor_id} 故障已清除")
        else:
            rospy.logerr(f"电机 {motor_id} 故障清除失败, 代码: 0x{fault_code:04X}")

    def check_and_clear_faults(self):
        """定期检查并清除电机故障"""
        # for motor_id in [2, 3, 4]:  # 检查所有电机
        for motor_id in [4]:  # 检查所有电机
            # 1. 检查故障码
            fault_code = self.read_fault_code(motor_id)
            actual_velocity = self.get_actual_velocity(motor_id)
            # max_current = self.get_max_current(motor_id)
            actual_current = self.get_actual_current(motor_id)
            if fault_code and fault_code != 0:  # 非0表示有故障
                rospy.logerr(f"电机 {motor_id} 检测到故障! 代码: 0x{fault_code:04X}")
                # 获取实际转矩帮助诊断
                actual_torque = self.get_actual_torque(motor_id)
                if actual_torque is not None:
                    rospy.loginfo(f"故障时转矩: {actual_torque/1000} 额定转矩")
                
                # 获取实际速度
                if actual_velocity is not None:
                    rospy.loginfo(f"故障时速度: {actual_velocity/68/20} rpm")
                # 设置停止状态
                self.set_state("STOP")
                self.motor_driver = False  # 预警
                rospy.loginfo("已切换到STOP状态，下次启动后处理故障")
                # self.set_state("START")

                # 尝试清除故障
                # self.clear_fault(motor_id)
                # time.sleep(0.5)  # 等待复位完成
        
            # 2. 非故障情况监控
            # 读取当前转矩
            actual_torque = self.get_actual_torque(motor_id)
            # if self.get_actual_velocity(motor_id) == 0:
            #     self.motor_driver = False
            # else:
            #     self.motor_driver = True
            # 获取配置的最大转矩
            max_torque = self.get_max_torque(motor_id)
            if max_torque is not None and actual_torque is not None:
                rospy.logwarn(f"电机 {motor_id} - 实际转矩: {actual_torque/1000}额定转矩 | 最大限制: {max_torque/1000}额定转矩")
                
                # 监控转矩接近阈值
                utilization = abs(actual_torque) / max_torque * 100
                if utilization > 80:
                    rospy.logwarn(f"电机 {motor_id} 转矩利用率过高: {utilization:.1f}%")
    def read_energy_saving_mode(self, motor_id):
        """读取省电功能总开关 Fn_1d0 的参数值（总线零速度指令模式下的自动省电功能模式）"""
        # 发送读取 Fn_1d0 指令：CANopen SDO 读取命令，索引 0x2200（Fn_1d0），子索引 0x00
        # 命令格式：[0x40（读取命令字）, 0x00（索引低字节）, 0x22（索引高字节）, 0x00（子索引）, 0x00, 0x00, 0x00, 0x00]
        self.send_command(motor_id, [0x40, 0xD0, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00])
        rospy.loginfo(f"已向电机 {motor_id} 发送读取 Fn_1d0 指令")

        # 接收回复，超时时间 0.5 秒（与故障码读取保持一致）
        start_time = time.time()
        while time.time() - start_time < 0.5:
            msg = self.bus.recv(0.1)  # 每次等待 100ms
            if msg and msg.arbitration_id == (0x580 + motor_id):  # 验证电机回复的仲裁 ID
                print("Fn_1d0 读取响应数据:", msg.data)
                # 验证响应格式：数据长度≥6，且索引匹配（0x2200，小端模式为 0x00 0x22）
                if len(msg.data) >= 6 and msg.data[1] == 0xD0 and msg.data[2] == 0x21:
                    # 解析 Fn_1d0 值（16 位无符号整数，低字节 data[4]，高字节 data[5]）
                    fn_1d0_value = msg.data[4] | (msg.data[5] << 8)
                    rospy.loginfo(f"电机 {motor_id} Fn_1d0（省电功能模式）当前值: {fn_1d0_value}")
                    # Fn_1d0 取值含义：0=关闭，1=总线指令触发，2=超时触发，3=指令或超时触发
                    mode_desc = {0: "关闭自动省电功能", 1: "总线指令触发省电", 2: "停机超时触发省电", 3: "指令或超时触发省电"}
                    rospy.loginfo(f"Fn_1d0 模式说明: {mode_desc.get(fn_1d0_value, '未知模式')}")
                    return fn_1d0_value  # 返回读取到的 Fn_1d0 值

        # 超时处理
        rospy.logwarn(f"读取电机 {motor_id} Fn_1d0 超时")
        return None  # 超时返回 None
    def set_energy_saving_mode(self, motor_id, target_value=3):
        """修改省电功能总开关 Fn_1d0 为目标值（默认 3：指令或超时触发，假设对应索引 0x2200.00）"""
        # 校验目标值合法性：Fn_1d0 仅支持 0-3（文档规定模式）
        if target_value not in [0, 1, 2, 3]:
            rospy.logerror(f"Fn_1d0 目标值 {target_value} 非法！仅支持 0（关闭）、1（指令触发）、2（超时触发）、3（指令或超时触发）")
            return False

        # 发送修改 Fn_1d0 指令：CANopen SDO 写入命令，索引 0x2200.00，目标值 target_value
        # 命令格式：[0x23（写入命令字）, 0x00（索引低字节）, 0x22（索引高字节）, 0x00（子索引）, 
        #           target_value低字节, target_value高字节, 0x00, 0x00]（16位无符号整数）
        low_byte = target_value & 0xFF  # 目标值低字节
        high_byte = (target_value >> 8) & 0xFF  # 目标值高字节
        self.send_command(motor_id, [0x2B, 0xD0, 0x21, 0x00, low_byte, high_byte, 0x00, 0x00])
        rospy.loginfo(f"已向电机 {motor_id} 发送设置 Fn_1d0 指令，目标值: {target_value}")

        # 接收写入确认，超时时间 0.5 秒
        start_time = time.time()
        while time.time() - start_time < 0.5:
            msg = self.bus.recv(0.1)
            if msg and msg.arbitration_id == (0x580 + motor_id):  # 验证回复 ID
                print("Fn_1d0 设置响应数据:", msg.data)
                # 验证响应：索引匹配（0x2200），且命令字为 0x60（SDO 写入确认，部分电机用 0x43，需根据实际调整）
                if len(msg.data) >= 4 and msg.data[1] == 0xD0 and msg.data[2] == 0x21:
                    # 确认写入成功（部分电机响应中会带回写入值，可额外校验）
                    rospy.loginfo(f"电机 {motor_id} Fn_1d0 已成功设置为 {target_value}（指令或超时触发省电）")
                    return True  # 设置成功返回 True

        # 超时处理
        rospy.logwarn(f"设置电机 {motor_id} Fn_1d0 超时，可能未生效")
        return False  # 超时返回 False
    
    def read_torque_zero_params(self, motor_id):
        """读取零转矩判定参数：Fn_04b（零转矩到达门限）和 Fn_04c（零转矩到达回差值）"""
        # 定义参数与CANopen索引的映射（需根据电机手册确认真实索引）
        param_map = {
            "Fn_04b": {"index": 0x204B, "desc": "零转矩到达门限（额定转矩千分之一）"},
            "Fn_04c": {"index": 0x204C, "desc": "零转矩到达回差值（额定转矩千分之一）"}
        }
        result = {}

        for param_name, info in param_map.items():
            index = info["index"]
            # 构造SDO读取命令：[0x40(读取命令字), 索引低字节, 索引高字节, 0x00(子索引), 0x00*4]
            cmd_low_byte = index & 0xFF
            cmd_high_byte = (index >> 8) & 0xFF
            self.send_command(motor_id, [0x40, cmd_low_byte, cmd_high_byte, 0x00, 0x00, 0x00, 0x00, 0x00])
            rospy.loginfo(f"已向电机 {motor_id} 发送读取 {param_name}（索引0x{index:04X}）指令")

            # 接收该参数的回复
            start_time = time.time()
            param_value = None
            while time.time() - start_time < 0.5:  # 单参数读取超时0.5秒
                msg = self.bus.recv(0.1)
                if msg and msg.arbitration_id == (0x580 + motor_id):  # 验证电机回复ID
                    print(f"{param_name} 读取响应数据:", msg.data)
                    # 验证响应格式：数据长度≥6，且索引匹配（低字节msg.data[1]，高字节msg.data[2]）
                    if len(msg.data) >= 6 and msg.data[1] == cmd_low_byte and msg.data[2] == cmd_high_byte:
                        # 解析16位无符号参数值（低字节data[4]，高字节data[5]）
                        param_value = msg.data[4] | (msg.data[5] << 8)
                        rospy.loginfo(f"电机 {motor_id} {param_name}（{info['desc']}）: {param_value}")
                        break
            
            if param_value is not None:
                result[param_name] = param_value
            else:
                rospy.logwarn(f"读取电机 {motor_id} {param_name} 超时")
                result[param_name] = None

        # 额外输出判定区间分析（基于文档中“配合形成区间”的要求）
        if result["Fn_04b"] is not None and result["Fn_04c"] is not None:
            lower_limit = result["Fn_04b"]
            upper_limit = result["Fn_04b"] + result["Fn_04c"]
            if result["Fn_04c"] == 0:
                rospy.logwarn(f"电机 {motor_id} Fn_04c 为0，零转矩判定区间异常（仅[{lower_limit}, {upper_limit}]），易误判未满足条件")
            else:
                rospy.loginfo(f"电机 {motor_id} 零转矩判定区间: [{lower_limit}, {upper_limit}]（额定转矩千分之一）")
        return result
    
    def set_torque_zero_param(self, motor_id, param_name, target_value):
        """修改零转矩判定参数：支持 Fn_04b（零转矩到达门限）或 Fn_04c（零转矩到达回差值）"""
        # 定义参数与CANopen索引的映射（需根据电机手册确认真实索引）
        param_config = {
            "Fn_04b": {"index": 0x204B, "desc": "零转矩到达门限", "min": 0, "max": 1000},  # 假设范围0-1000（额定转矩千分之一）
            "Fn_04c": {"index": 0x20C, "desc": "零转矩到达回差值", "min": 0, "max": 500}   # 假设范围0-500（额定转矩千分之一）
        }

        # 校验参数名合法性
        if param_name not in param_config:
            rospy.logerror(f"不支持的参数名 {param_name}！仅支持 {list(param_config.keys())}")
            return False
        info = param_config[param_name]

        # 校验目标值范围（基于文档“合理判定区间”要求，Fn_04c建议非0）
        if not (info["min"] <= target_value <= info["max"]):
            rospy.logerror(f"{param_name}（{info['desc']}）目标值 {target_value} 非法！需在 [{info['min']}, {info['max']}] 范围内")
            return False
        if param_name == "Fn_04c" and target_value == 0:
            rospy.logwarn(f"警告：{param_name} 设为0会导致零转矩判定异常，建议设为与Fn_04b一致的值（如100）")

        # 构造SDO写入命令：[0x23(写入命令字), 索引低字节, 索引高字节, 0x00(子索引), 目标值低字节, 目标值高字节, 0x00, 0x00]
        index = info["index"]
        cmd_low_byte = index & 0xFF
        cmd_high_byte = (index >> 8) & 0xFF
        target_low = target_value & 0xFF
        target_high = (target_value >> 8) & 0xFF
        self.send_command(motor_id, [0x2B, cmd_low_byte, cmd_high_byte, 0x00, target_low, target_high, 0x00, 0x00])
        rospy.loginfo(f"已向电机 {motor_id} 发送设置 {param_name} 指令，目标值: {target_value}（{info['desc']}）")

        # 接收写入确认
        start_time = time.time()
        while time.time() - start_time < 0.5:
            msg = self.bus.recv(0.1)
            if msg and msg.arbitration_id == (0x580 + motor_id):
                print(f"{param_name} 设置响应数据:", msg.data)
                # 验证响应索引匹配
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
            # 非阻塞读取键盘
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.readline().strip()
                if key:
                    controller.update_status_by_key(key)    

def main():
    rospy.init_node("motor_canopen_node")
    controller = ServoDriveController()
    # controller.stop_imu()
    controller.start_imu()
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
    # rospy.Subscriber("distance_data", Distances, lambda msg: controller.distance_callback(msg))
    rospy.Subscriber("proximity_sensor_data", Sensors, lambda msg: controller.proximity_callback(msg))
    # 通过检测按键修改运行状态
    # 启动键盘监听线程
    t = threading.Thread(target=ServoDriveController.keyboard_listener, args=(controller,), daemon=True)
    t.start()
    try:
    # 每0.05秒执行一次状态执行器
        rospy.Timer(rospy.Duration(0.05), controller.execute_state)
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("程序终止")
    finally:
        controller.shutdown()

if __name__ == "__main__":
    main()
