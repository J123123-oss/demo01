#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rospy
import json
import yaml
import threading
import sys
import select
from std_msgs.msg import String, Int8, Float32, Bool
from serial_comms.msg import Distances, Sensors, INSPVAE, BatteryStatus
from std_srvs.srv import Trigger
from pymodbus.client import ModbusSerialClient
from pymodbus.payload import BinaryPayloadBuilder, BinaryPayloadDecoder
from pymodbus.constants import Endian as ModbusEndian  
import serial 

# -------------------------- 驱动器核心配置（BL50手册适配） --------------------------
BAUDRATE = 115200  # 手册默认波特率115200（0x2280默认值7）
# SERIAL_PORT = "/dev/Motor-DBLS"  # 根据实际端口修改
SERIAL_PORT = "/dev/ttyv1"  # 根据实际端口修改
STOP_BITS = 1
PARITY = "N"  # 手册默认无校验（0x2283默认值0）
TIMEOUT = 3.0  # 通讯超时
COMMAND_INTERVAL = 0.05  # 指令间隔50ms

# ========== 手册定义核心寄存器地址 ==========
# 控制参数（2000h~2038h）
REG_RUN_CMD = 0x2001          # F00.01 运行指令：0=停止，1=运行（RWN）
REG_SPEED_CMD = 0x2002        # F00.02 速度指令：[-5000,5000] RPM（RWS）
REG_ACCEL = 0x2003            # F00.03 加速度：1~1250（0.1KRPM/S）（RWS）
REG_DECEL = 0x2004            # F00.04 减速度：1~1250（0.1KRPM/S）（RWS）
REG_ENABLE_SOURCE = 0x2005    # F00.05 使能来源：0=IO控制，1=通讯控制（RWS）
REG_STOP_MODE = 0x2009        # F00.08 停车方式：0=惯性停车，1=减速停车（RWS）
REG_MAX_SPEED = 0x2016        # F00.21 最大转速限制：默认3000 RPM（RWS）

# 电机参数（2040h~2047h）
REG_MOTOR_POLE = 0x2043       # F01.03 电机极对数：手册默认5对极（RWS）

# 故障与通讯参数
REG_FAULT_CODE = 0x2080       # 故障代码寄存器（手册5.17）
REG_485_SLAVE_ADDR = 0x2281   # 485从机地址：默认1（RWS）
REG_PARAM_SAVE = 0x2284       # 参数保存：写1保存（RWS）

# ========== 硬件特性配置 ==========
MOTOR_MAX_SPEED = 3000        # 手册默认最高转速3000 RPM
DEFAULT_POLE_PAIRS = 5        # 手册默认极对数5
ACCEL_DEFAULT = 10            # 默认加速度10（1KRPM/S）
DECEL_DEFAULT = 10            # 默认减速度10（1KRPM/S）
STOP_MODE_DEFAULT = 1         # 默认惯性停车0>>>改减速停车1
ENABLE_SOURCE_COMM = 1        # 通讯控制使能

# 故障码映射（手册5.17）
FAULT_MAP = {
    0x0000: "无故障",
    0x0001: "系统参数异常（不可清除）",
    0x0004: "电流零偏检测故障（不可清除）",
    0x0005: "电压零偏检测故障（不可清除）",
    0x0008: "硬件过流故障（不可清除）",
    0x2103: "主回路欠电压（可清除）",
    0x2104: "主回路过电压（可清除）",
    0x2105: "驱动过载（可清除）",
    0x2106: "电机过载（可清除）",
    0x2107: "软件过流故障（可清除）",
    0x2200: "观测器故障（可清除）",
    0x6203: "紧急停止（可清除）",
    0x6205: "驱动器超温故障（可清除）"
}

class ServoDriveController:
    def __init__(self):
        # ROS初始化
        rospy.init_node('motor_modbus_rtu_node', anonymous=True)
        self.rate = rospy.Rate(20)
        
        # RTU客户端初始化
        self.rtu_client = None
        # 扩展为5个电机：电机ID 1~5 对应485从机地址1~5
        self.motor_address_map = {1:1, 2:2, 3:3, 4:4, 5:5}
        self.motor_pole_pairs = {}  # 从寄存器读取极对数
        self.motor_running = {1:False, 2:False, 3:False, 4:False, 5:False}  # 电机运行状态缓存
        self.motor_current_speed = {1:0, 2:0, 3:0, 4:0, 5:0}  # 当前速度缓存
        
        # 核心参数初始化
        self.accel = ACCEL_DEFAULT
        self.decel = DECEL_DEFAULT
        self.stop_mode = STOP_MODE_DEFAULT
        self.LOW_BATTERY_THRESHOLD = 40  # 低电量阈值
        
        # 状态变量（保留原有核心逻辑，适配5电机）
        self.last_left_speed = 0
        self.last_right_speed = 0
        self.last_brush_speed = 0
        self.last_motor4_speed = 0  # 新增电机4速度缓存
        self.last_motor5_speed = 0  # 新增电机5速度缓存
        self.has_reverse_flag = False
        self.has_reverse_counter = 0
        self.reverse_start_time = None
        self.main_board = True
        self.imu_sensor = True
        self.motor_driver = True
        self.motor_base = 1000
        self.brush_base_speed = 1800
        self.brush_forward = rospy.get_param('~brush_forward', False)
        self.flag = 0
        self.speed_pluse_max = MOTOR_MAX_SPEED  # 适配手册最大速度
        self.reversed_start_time = None
        self.REVERSE_TIME_THRESHOLD = 2.0
        self.unloading_timer = 0.1
        self.unloading_start_time = None
        self.start_time = 0
        self.elevator_stage = 0
        self.elevator_start_time = 0
        self.velocity_publish_count = 0
        self.velocity_publish_interval = 2
        self.last_velocity_up = 0
        self.last_velocity_low = 0
        self.last_velocity_brush = 0
        self.last_velocity_motor4 = 0  # 新增电机4速度发布缓存
        self.last_velocity_motor5 = 0  # 新增电机5速度发布缓存
        self.heartbeat_running = False
        self.heartbeat_thread = None
        self.last_sensor_a = False
        self.sensor_a_count = 0
        self.last_sensor_time = 0
        self.last_switch_time = 0
        self.SWITCH_DELAY = 5
        self.PROXIMITY_ENABLE_DELAY = 4.0
        self.startup_time = None
        self.state_change_protect_delay = 5.0
        self.last_state_change_time = 0.0
        self.GLOBAL_REPEAT_DELAY = 5.0
        self.last_critical_switch_time = 0.0
        self.side_duration_time = None
        self.move_duration = None
        self.TIMEOUT_THRESHOLD = 7.0
        self.error_count = 0
        
        # 状态列表（保留原有）
        self.status_list = [
            "STOP", "FORWARD", "BACKWARD", "START", "LOADING", "UNLOADING",
            "UPSTOP", "LOWSTOP", "PISTON_OUT", "PISTON_IN", "CHARGE_OUT",
            "RETURN_DOCK", "PAUSE"
        ]
        
        # 状态速度配置（扩展5电机速度参数）
        self.status_config = {
            "START": {},
            "STOP": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": 0, "velocity_motor4": 0, "velocity_motor5": 0},
            "UNLOADING": {"velocity_up": -self.motor_base, "velocity_low": self.motor_base, "velocity_brush": -self.brush_base_speed, "velocity_motor4": 0, "velocity_motor5": 0},
            "FORWARD": {"velocity_up": self.motor_base, "velocity_low": -self.motor_base, "velocity_brush": -self.brush_base_speed, "velocity_motor4": 0, "velocity_motor5": 0},
            "BACKWARD": {"velocity_up": -self.motor_base, "velocity_low": self.motor_base, "velocity_brush": self.brush_base_speed, "velocity_motor4": 0, "velocity_motor5": 0},
            "LOADING": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": 0, "velocity_motor4": 0, "velocity_motor5": 0},
            "PAUSE": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": -self.brush_base_speed, "velocity_motor4": 0, "velocity_motor5": 0},
            "UPSTOP": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": 0, "velocity_motor4": 0, "velocity_motor5": 0},
            "LOWSTOP": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": 0, "velocity_motor4": 0, "velocity_motor5": 0},
            "REVERSE": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": -self.brush_base_speed, "velocity_motor4": 0, "velocity_motor5": 0},
            "PISTON_OUT": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": 0, "velocity_motor4": 500, "velocity_motor5": 0},  # 电机4示例速度
            "PISTON_IN": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": 0, "velocity_motor4": 0, "velocity_motor5": 0},   # init imu
            "CHARGE_OUT": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": 0, "velocity_motor4": 0, "velocity_motor5": 500},  # 电机5示例速度
            "RETURN_DOCK": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": 0, "velocity_motor4": 0, "velocity_motor5": -500}  # 电机5反向
        }
        
        self.last_state = None
        self.current_status = self.status_list[0]
        self.current_velocity_up = 0
        self.current_velocity_low = 0
        self.current_velocity_brush = 0
        self.current_velocity_motor4 = 0  # 新增电机4当前速度
        self.current_velocity_motor5 = 0  # 新增电机5当前速度
        self.threshold = 30
        self.stop_flag = False
        self.position_engaged = False
        self.position_mode_configured = False
        self.left_position = 0
        self.right_position = 0
        self.position_direction = 1
        self.target_sent_flag = False
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
        
        # PID参数（保留原有）
        self.pid_integral = 0.0
        self.pid_last_error = 0.0
        self.target_yaw = 0.0
        self.pid_kp = 80
        self.pid_ki = 0
        self.pid_kd = 0.5
        self.pid_correction_max = 500
        self.progress = 0
        
        # 电池相关（保留原有）
        self.battery_total_voltage = None
        self.battery_current = None
        self.battery_remaining = None
        self.battery_temperatures = []
        self.relay_status = None
        
        # ROS发布订阅（保留原有）
        self.state_pub = rospy.Publisher('/robot_state', String, queue_size=10)
        self.motor_cmd_pub = rospy.Publisher('/motor_cmd', Int8, queue_size=10)
        rospy.Subscriber('/robot_cmd', String, self.status_callback)
        rospy.Subscriber('/inspvae_data', INSPVAE, self.imu_callback)
        rospy.Subscriber('/battery_status', BatteryStatus, self.battery_status_callback)
        rospy.Subscriber('/relay_status', Bool, self.relay_callback)
        rospy.Subscriber("proximity_sensor_data", Sensors, self.proximity_callback)
        
        self.state_switch_lock = threading.Lock()
        self.sensor_triggered = {"a": False, "b": False}
        
        # 发布定时器
        self.publish_timer = rospy.Timer(rospy.Duration(1.0), self.publish_state)
        
        # 连接RTU客户端
        self.connect_rtu_client_with_timeout()
        
        # 初始化电机参数
        self.init_motor_hardware()

    # -------------------------- RTU连接相关方法（保留原有） --------------------------
    def connect_rtu_client_with_timeout(self):
        max_retry = 5
        retry_count = 0
        
        while not rospy.is_shutdown() and retry_count < max_retry:
            try:
                self.rtu_client = ModbusSerialClient(
                    port=SERIAL_PORT,
                    baudrate=BAUDRATE,
                    stopbits=STOP_BITS,
                    parity=PARITY,
                    timeout=TIMEOUT
                )
                if self.rtu_client.connect():
                    rospy.loginfo(f"✅ RTU客户端连接成功：{SERIAL_PORT}")
                    return True
                else:
                    retry_count += 1
                    rospy.logerr(f"❌ RTU客户端连接失败（重试{retry_count}/{max_retry}），3秒后重试...")
                    time.sleep(3)
            except Exception as e:
                retry_count += 1
                rospy.logerr(f"❌ RTU连接异常：{e}（重试{retry_count}/{max_retry}）")
                time.sleep(3)
        
        rospy.logfatal(f"❌ RTU客户端连接失败，已重试{max_retry}次，请检查串口/接线！")
        return False

    def reconnect_rtu_client(self):
        rospy.logwarn("🔄 尝试重连RTU客户端...")
        if self.rtu_client is not None:
            try:
                self.rtu_client.close()
            except:
                pass
        self.rtu_client = None
        
        try:
            self.rtu_client = ModbusSerialClient(
                port=SERIAL_PORT,
                baudrate=BAUDRATE,
                stopbits=STOP_BITS,
                parity=PARITY,
                timeout=TIMEOUT
            )
            if self.rtu_client.connect():
                rospy.loginfo("✅ RTU客户端重连成功")
                return True
            else:
                rospy.logerr("❌ RTU客户端重连失败")
                return False
        except Exception as e:
            rospy.logerr(f"❌ RTU重连异常：{e}")
            return False

    def rtu_write_register(self, motor_id, reg_addr, value):
        if self.rtu_client is None or not self.rtu_client.is_socket_open():
            rospy.logerr("❌ RTU客户端未连接，无法写寄存器")
            if not self.reconnect_rtu_client():
                return False
        
        rtu_addr = self.motor_address_map.get(motor_id)
        if rtu_addr is None: 
            rospy.logerr(f"❌ 电机ID {motor_id} 无对应RTU站点")
            return False
        
        try:
            time.sleep(COMMAND_INTERVAL)
            response = self.rtu_client.write_register(reg_addr, value, slave=rtu_addr)
            if not response.isError():
                rospy.logdebug(f"✅ 电机{motor_id}写寄存器0x{reg_addr:04X}成功：0x{value:04X}")
                return True
            rospy.logerr(f"❌ RTU写寄存器失败：电机{motor_id}，地址0x{reg_addr:04X}，错误{response}")
            self.reconnect_rtu_client()
            return False
        except Exception as e:
            rospy.logerr(f"❌ RTU写操作异常：{e}")
            self.reconnect_rtu_client()
            return False

    def rtu_read_register(self, motor_id, reg_addr, count=1):
        if self.rtu_client is None or not self.rtu_client.is_socket_open():
            rospy.logerr("❌ RTU客户端未连接，无法读寄存器")
            if not self.reconnect_rtu_client():
                return None
        
        rtu_addr = self.motor_address_map.get(motor_id)
        if rtu_addr is None: 
            rospy.logerr(f"❌ 电机ID {motor_id} 无对应RTU站点")
            return None
        
        try:
            time.sleep(COMMAND_INTERVAL)
            response = self.rtu_client.read_holding_registers(reg_addr, count, slave=rtu_addr)
            if not response.isError():
                rospy.logdebug(f"✅ 电机{motor_id}读寄存器0x{reg_addr:04X}成功：{response.registers}")
                return response.registers
            rospy.logerr(f"❌ RTU读寄存器失败：电机{motor_id}，地址0x{reg_addr:04X}，错误{response}")
            self.reconnect_rtu_client()
            return None
        except Exception as e:
            rospy.logerr(f"❌ RTU读操作异常：{e}")
            self.reconnect_rtu_client()
            return None

    # -------------------------- 电机硬件初始化（适配5电机） --------------------------
    def init_motor_hardware(self):
        if self.rtu_client is None:
            rospy.logerr("❌ RTU未连接，无法初始化电机硬件")
            return
        
        for motor_id in self.motor_address_map.keys():
            slave_addr = self.motor_address_map[motor_id]
            rospy.loginfo(f"⚙️  初始化电机{motor_id}（从机地址{slave_addr}）")
            
            # 1. 设置使能来源为通讯控制（0x2005=1）
            self.rtu_write_register(motor_id, REG_ENABLE_SOURCE, ENABLE_SOURCE_COMM)
            
            # 2. 设置加减速参数（0x2003/0x2004）
            self.rtu_write_register(motor_id, REG_ACCEL, self.accel)
            self.rtu_write_register(motor_id, REG_DECEL, self.decel)
            
            # 3. 设置停车方式（0x2009）
            self.rtu_write_register(motor_id, REG_STOP_MODE, self.stop_mode)
            
            # 4. 设置最大转速限制（0x2016）
            self.rtu_write_register(motor_id, REG_MAX_SPEED, MOTOR_MAX_SPEED)
            
            # 5. 读取极对数（0x2043），默认5
            pole_pairs = self.rtu_read_register(motor_id, REG_MOTOR_POLE)
            self.motor_pole_pairs[motor_id] = pole_pairs[0] if pole_pairs else DEFAULT_POLE_PAIRS
            rospy.loginfo(f"电机{motor_id}极对数：{self.motor_pole_pairs[motor_id]}")
            
            # 6. 保存参数（0x2284=1）
            self.rtu_write_register(motor_id, REG_PARAM_SAVE, 1)
            
            # 7. 初始停止电机（0x2001=0）
            self.rtu_write_register(motor_id, REG_RUN_CMD, 0)
        
        rospy.loginfo("✅ 5个电机硬件初始化完成")

    # -------------------------- 电机控制核心方法（重构，适配5电机） --------------------------
    def motor_start(self, motor_id, target_speed):
        if self.rtu_client is None:
            rospy.logerr(f"❌ 电机{motor_id}：RTU未连接，无法启动")
            return False
        
        # 速度限幅（手册[-5000,5000]，实际限制3000）
        target_speed = max(min(target_speed, MOTOR_MAX_SPEED), -MOTOR_MAX_SPEED)
        if target_speed < 0:
            target_speed = 0x10000 - abs(target_speed)
        try:
            # 1. 写入速度指令（方向通过速度正负实现）
            self.rtu_write_register(motor_id, REG_SPEED_CMD, target_speed)
            
            # 2. 启动电机（0x2001=1）
            self.rtu_write_register(motor_id, REG_RUN_CMD, 1)
            
            # 3. 更新缓存
            self.motor_running[motor_id] = True
            self.motor_current_speed[motor_id] = target_speed
            rospy.loginfo(f"✅ 电机{motor_id}启动：速度{target_speed} RPM（写入指令：0x{target_speed:04X}），{'正转' if target_speed>0 else '反转'}")

            # rospy.loginfo(f"✅ 电机{motor_id}启动：速度{target_speed} RPM，方向{'正转' if target_speed>0 else '反转'}")
            return True
        except Exception as e:
            rospy.logerr(f"❌ 电机{motor_id}启动失败：{e}")
            self.motor_running[motor_id] = False
            return False

    def motor_stop(self, motor_id):
        if self.rtu_client is None:
            rospy.logerr(f"❌ 电机{motor_id}：RTU未连接，无法停止")
            return False
        
        try:
            # 1. 停止运行指令（0x2001=0）
            self.rtu_write_register(motor_id, REG_RUN_CMD, 0)
            
            # 2. 速度清零
            self.rtu_write_register(motor_id, REG_SPEED_CMD, 0)
            
            # 3. 更新缓存
            self.motor_running[motor_id] = False
            self.motor_current_speed[motor_id] = 0
            rospy.loginfo(f"✅ 电机{motor_id}停止（停车方式：{'惯性' if self.stop_mode==0 else '减速'}）")
            return True
        except Exception as e:
            rospy.logerr(f"❌ 电机{motor_id}停止失败：{e}")
            return False

    def motor_adjust_speed(self, motor_id, new_speed):
        if not self.motor_running[motor_id]:
            rospy.logwarn(f"⚠️ 电机{motor_id}未运行，无法调节速度，自动启动")
            return self.motor_start(motor_id, new_speed)
        
        new_speed = max(min(new_speed, MOTOR_MAX_SPEED), -MOTOR_MAX_SPEED)
        if new_speed < 0:
            new_speed = 0x10000 - abs(new_speed)
        try:
            self.rtu_write_register(motor_id, REG_SPEED_CMD, new_speed)
            self.motor_current_speed[motor_id] = new_speed
            rospy.logdebug(f"⚡ 电机{motor_id}速度更新为：{new_speed} RPM（写入指令：0x{new_speed:04X}）")

            # rospy.logdebug(f"⚡ 电机{motor_id}速度更新为：{new_speed} RPM")
            return True
        except Exception as e:
            rospy.logerr(f"❌ 电机{motor_id}速度调节失败：{e}")
            return False

    def read_motor_fault(self, motor_id):
        if self.rtu_client is None:
            rospy.logerr(f"❌ 电机{motor_id}：RTU未连接，无法读取故障")
            return None, "RTU未连接"
        
        registers = self.rtu_read_register(motor_id, REG_FAULT_CODE)
        if not registers or len(registers) != 1:
            rospy.logwarn(f"⚠️ 电机{motor_id}故障码读取失败")
            return None, "读取失败"
        
        fault_code = registers[0]
        fault_desc = FAULT_MAP.get(fault_code, f"未知故障（0x{fault_code:04X}）")
        rospy.loginfo(f"⚠️ 电机{motor_id}故障：{fault_desc}（代码0x{fault_code:04X}）")
        return fault_code, fault_desc

    def clear_motor_fault(self, motor_id):
        fault_code, _ = self.read_motor_fault(motor_id)
        if fault_code is None:
            return False
        
        # 不可清除故障直接返回
        if fault_code in [0x0001, 0x0004, 0x0005, 0x0008]:
            rospy.logerr(f"❌ 电机{motor_id}故障不可清除，需检查硬件")
            return False
        
        # 可清除故障：停止电机→等待→重启
        self.motor_stop(motor_id)
        time.sleep(1.0)
        self.motor_start(motor_id, self.motor_current_speed[motor_id])
        rospy.loginfo(f"✅ 电机{motor_id}故障已清除，恢复运行")
        return True

    # -------------------------- 状态执行（适配5电机控制） --------------------------
    def execute_state(self):
        if self.rtu_client is None:
            rospy.logwarn("⚠️ RTU未连接，跳过电机控制")
            return
        
        # 低电量保护
        if self.battery_remaining is not None and self.battery_remaining < self.LOW_BATTERY_THRESHOLD:
            rospy.logerr("🔋 低电量保护，停止所有电机")
            for motor_id in self.motor_address_map.keys():
                self.motor_stop(motor_id)
            self.set_state("STOP")
            return
        
        # 状态映射执行
        status = self.current_status
        config = self.status_config.get(status, {
            "velocity_up":0, "velocity_low":0, "velocity_brush":0, "velocity_motor4":0, "velocity_motor5":0
        })
        
        # 电机ID映射：2=上电机，1=下电机，3=刷电机，4=新增电机4，5=新增电机5
        motor_up_id = 2
        motor_low_id = 1
        motor_brush_id = 3
        motor4_id = 4
        motor5_id = 5
        
        # 目标速度（含PID矫正）
        correction = self.pid_correction(self.imu_yaw) if hasattr(self, 'pid_correction') else 0
        target_up = int(config["velocity_up"] + correction)
        target_low = int(config["velocity_low"] + correction)
        target_brush = config["velocity_brush"]
        target_motor4 = config["velocity_motor4"]
        target_motor5 = config["velocity_motor5"]
        
        try:
            # 执行对应状态的电机控制
            if status == "STOP":
                # 停止所有5个电机
                self.motor_stop(motor_up_id)
                self.motor_stop(motor_low_id)
                self.motor_stop(motor_brush_id)
                self.motor_stop(motor4_id)
                self.motor_stop(motor5_id)
                self.last_left_speed = 0
                self.last_right_speed = 0
                self.last_brush_speed = 0
                self.last_motor4_speed = 0
                self.last_motor5_speed = 0
            
            elif status in ["FORWARD", "BACKWARD", "UNLOADING"]:
                # 运动状态：调节5电机速度
                self.motor_adjust_speed(motor_up_id, target_up)
                self.motor_adjust_speed(motor_low_id, target_low)
                self.motor_adjust_speed(motor_brush_id, target_brush)
                self.motor_adjust_speed(motor4_id, target_motor4)
                self.motor_adjust_speed(motor5_id, target_motor5)
                
                # 缓存速度
                self.last_left_speed = target_up
                self.last_right_speed = target_low
                self.last_brush_speed = target_brush
                self.last_motor4_speed = target_motor4
                self.last_motor5_speed = target_motor5
            
            elif status in ["PAUSE", "UPSTOP", "LOWSTOP"]:
                # 暂停/停止状态：仅保持刷电机运行，其他电机停止
                self.motor_stop(motor_up_id)
                self.motor_stop(motor_low_id)
                self.motor_stop(motor4_id)
                self.motor_stop(motor5_id)
                self.motor_adjust_speed(motor_brush_id, target_brush)
                self.last_brush_speed = target_brush
            
            elif status == "LOADING":
                # 加载状态：所有电机停止
                self.motor_stop(motor_up_id)
                self.motor_stop(motor_low_id)
                self.motor_stop(motor_brush_id)
                self.motor_stop(motor4_id)
                self.motor_stop(motor5_id)
            
            elif status in ["PISTON_OUT", "PISTON_IN", "CHARGE_OUT", "RETURN_DOCK"]:
                # 特殊状态：控制对应电机
                self.motor_stop(motor_up_id)
                self.motor_stop(motor_low_id)
                self.motor_stop(motor_brush_id)
                self.motor_adjust_speed(motor4_id, target_motor4)
                self.motor_adjust_speed(motor5_id, target_motor5)
                self.last_motor4_speed = target_motor4
                self.last_motor5_speed = target_motor5
            
            elif status == "START":
                # START状态：启动所有电机（按配置速度）
                self.motor_adjust_speed(motor_up_id, target_up)
                self.motor_adjust_speed(motor_low_id, target_low)
                self.motor_adjust_speed(motor_brush_id, target_brush)
                self.motor_adjust_speed(motor4_id, target_motor4)
                self.motor_adjust_speed(motor5_id, target_motor5)
            
            # 5个电机故障检测（心跳检测）
            for motor_id in self.motor_address_map.keys():
                fault_code, _ = self.read_motor_fault(motor_id)
                if fault_code and fault_code != 0x0000:
                    self.clear_motor_fault(motor_id)
        
        except Exception as e:
            rospy.logerr(f"❌ 状态执行异常：{e}")

    # -------------------------- 原有辅助方法（保留并适配5电机） --------------------------
    def is_global_repeat_protected(self):
        current_time = time.time()
        if current_time - self.last_critical_switch_time < self.GLOBAL_REPEAT_DELAY:
            return True
        return False

    def set_state(self, new_state):
        if new_state not in self.status_config:
            rospy.logwarn(f"⚠️ 尝试设置无效状态: {new_state}")
            return False
        if new_state == self.current_status and new_state not in ["PISTON_OUT", "PISTON_IN", "STOP"]:
            return False
        if self.current_status in ["FORWARD", "BACKWARD"] and (new_state == "CHARGE_OUT" or new_state == "RETURN_DOCK"):
            return False
        if self.auto_mode and new_state in ["FORWARD", "BACKWARD"]:
            self.move_duration = time.time()
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
            self.publish_timer.shutdown()
            self.publish_timer = rospy.Timer(rospy.Duration(1.0), self.publish_state)
            self.stop_heartbeat()
            threading.Thread(target=self.delayed_publish_freq_switch, args=(1,), daemon=True).start()
        else:
            self.publish_timer.shutdown()
            self.publish_timer = rospy.Timer(rospy.Duration(1.0), self.publish_state)
        if new_state in ["FORWARD", "BACKWARD"]:
            if self.current_status != "REVERSE":
                self.prev_motion_state = new_state
        if new_state in ["REVERSE", "UPSTOP", "LOWSTOP"]:
            if self.prev_motion_state is None:
                self.prev_motion_state = self.last_state
            if new_state in ["UPSTOP", "LOWSTOP"]:
                self.side_duration_time = time.time()
                self.last_stop_state = new_state
            else:
                if hasattr(self, 'side_duration_time') and self.side_duration_time is not None:
                    self.side_duration_time = None
                self.last_stop_state = None
        elif new_state == "PISTON_IN":
            self.initial_yaw = None
            self.auto_step = None
            self.reversed_start_time = None
            self.has_reverse_flag = False
        elif new_state == "PISTON_OUT":
            if self.count % 2:
                self.auto_mode = False
                rospy.loginfo("🔘 手动模式开")
            else:
                self.auto_mode = True
                rospy.loginfo("🔘 自动模式开")
            self.count += 1
        critical_states = ["FORWARD", "BACKWARD"]
        if new_state in critical_states:
            self.last_critical_switch_time = time.time()
        self.current_status = new_state
        self.last_state = self.current_status
        self.last_state_change_time = time.time()
        rospy.loginfo(f"📌 状态已更新为: {self.current_status}")
        return True

    def lock_motor(self):
        rospy.loginfo("🔒 电机向下转动，锁止")
        self.motor_cmd_pub.publish(Int8(data=-1))
        self.initial_yaw = None

    def publish_state(self, event=None):
        try:
            # 读取5个电机实际转速（按需发布）
            if self.rtu_client is not None:
                self.last_velocity_up = self.get_actual_velocity(2)
                self.last_velocity_low = self.get_actual_velocity(1)
                self.last_velocity_brush = self.get_actual_velocity(3)
                self.last_velocity_motor4 = self.get_actual_velocity(4)  # 新增电机4转速发布
                self.last_velocity_motor5 = self.get_actual_velocity(5)  # 新增电机5转速发布
            
            # 构建状态消息（包含5电机信息）
            state_msg = {
                "status": self.current_status,
                "battery": self.battery_remaining,
                "battery_temperatures": self.battery_temperatures,
                "battery_total_voltage": self.battery_total_voltage,
                "battery_current": self.battery_current,
                "progress": self.progress,
                "imu_yaw": round(self.imu_yaw, 2) if self.imu_yaw is not None else 0.00,
                "velocity_up": round(self.last_velocity_up, 2),
                "velocity_low": round(self.last_velocity_low, 2),
                "velocity_brush": round(self.last_velocity_brush, 2),
                "velocity_motor4": round(self.last_velocity_motor4, 2),  # 新增电机4速度
                "velocity_motor5": round(self.last_velocity_motor5, 2),  # 新增电机5速度
                "sensors_status": self.sensors_status,
                "device_status": {
                    "main_board": self.main_board,
                    "imu_sensor": self.imu_sensor,
                    "motor_driver": self.motor_driver,
                    "comm_module": self.rtu_client.is_socket_open() if self.rtu_client is not None else False
                },
                "complete_state": self.complete_state,
                "auto_mode": self.auto_mode,
                "timestamp": time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
            }
            self.state_pub.publish(json.dumps(state_msg, ensure_ascii=False))
        except Exception as e:
            rospy.logerr(f"❌ 发布状态异常: {e}")
            error_msg = {"status": "ERROR", "error": str(e)}
            self.state_pub.publish(json.dumps(error_msg))

    def get_actual_velocity(self, motor_id):
        if self.rtu_client is None:
            rospy.logerr("❌ RTU客户端未连接，无法读取转速")
            return 0
        
        registers = self.rtu_read_register(motor_id, REG_SPEED_CMD, count=1)
        if not registers or len(registers) != 1:
            rospy.logwarn(f"⚠️ 读取电机{motor_id}转速失败")
            return 0
        speed_cmd = registers[0]
        # 补码还原为原始转速：若指令值>0x8000（32768），则为负数（0x10000 - 指令值）
        if speed_cmd > 0x8000:
            actual_speed = -(0x10000 - speed_cmd)
        else:
            actual_speed = speed_cmd
        
        rospy.logdebug(f"电机{motor_id}实际转速：{actual_speed} RPM（读取指令：0x{speed_cmd:04X}）")
        return round(actual_speed, 2)

    def status_callback(self, msg):
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
                rospy.logwarn(f"⚠️ 未找到command字段: {msg.data}")
        except Exception as e:
            rospy.logwarn(f"⚠️ 消息解析失败: {msg.data}, 错误: {e}")
            if msg.data == "BRUSH_FORWARD":
                self.brush_forward = not self.brush_forward
            self.set_state(msg.data)
            self.publish_state()

    def imu_callback(self, msg):
        try:
            self.imu_yaw = msg.yaw if hasattr(msg, "yaw") else 0.0
            if self.initial_yaw is None:
                self.initial_yaw = self.imu_yaw
                rospy.loginfo(f"🧭 Initial IMU yaw: {self.initial_yaw}°")
            if self.initial_yaw is not None:
                relative_yaw = self.imu_yaw - self.initial_yaw
                if relative_yaw > 180:
                    relative_yaw -= 360
                elif relative_yaw < -180:
                    relative_yaw += 360
                self.imu_yaw = relative_yaw
        except Exception as e:
            rospy.logerr(f"❌ 解析IMU数据失败: {e}")

    def battery_status_callback(self, msg):
        self.battery_remaining = msg.batttery_remaining
        self.battery_total_voltage = round(msg.total_voltage, 2)
        self.battery_current = round(msg.current, 2)
        self.battery_temperatures = [round(t, 1) for t in msg.temperatures] if hasattr(msg, "temperatures") else []

    def relay_callback(self, msg):
        self.relay_status = msg.data

    def delayed_publish_freq_switch(self, delay_sec=3):
        time.sleep(delay_sec)
        if self.current_status == "STOP" and not rospy.is_shutdown():
            self.publish_timer.shutdown()
            self.publish_timer = rospy.Timer(rospy.Duration(1800), self.publish_state)

    def proximity_callback(self, msg):
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
        sensor_a_trigger = msg.sensor_a and not self.sensor_triggered["a"]
        sensor_b_trigger = msg.sensor_b and not self.sensor_triggered["b"]
        sensor_aoth_trigger = msg.sensor_b and msg.sensor_a
        
        # 传感器A计数
        current_time = time.time()
        if not self.last_sensor_a and msg.sensor_a:
            if (current_time - self.last_sensor_time) > 0.1:
                if (current_time - self.last_sensor_time) > 5.0:
                    self.sensor_a_count += 1
                    rospy.loginfo(f"传感器A触发次数: {self.sensor_a_count}")
                    self.last_sensor_time = current_time
        self.last_sensor_a = msg.sensor_a
        
        # RETURN_DOCK/CHARGE_OUT 特殊逻辑
        if self.auto_mode and self.current_status == "RETURN_DOCK":
            if self.elevator_stage == 2:
                self.set_state("FORWARD")
        if self.auto_mode and self.current_status == "CHARGE_OUT":
            if self.elevator_stage == 2:
                if msg.sensor_b or msg.sensor_a:
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
        
        # START状态逻辑
        if self.auto_mode and self.current_status == "START":
            self.startup_time = time.time()
            if self.elevator_stage == 2:
                if self.auto_step is None:
                    self.set_state("BACKWARD")
                self.progress = 20
        
        # REVERSE状态边界检测
        if self.current_status == "REVERSE":
            if msg.sensor_b and msg.sensor_a:
                self.initial_yaw = None
                self.set_state(self.prev_motion_state)
                self.has_reverse_flag = False
            return
        
        # 统一处理自动/手动模式的FORWARD/BACKWARD状态
        self._handle_motion_state(msg, sensor_b_trigger, sensor_a_trigger, sensor_aoth_trigger)
        
        # 处理UPSTOP/LOWSTOP状态
        self._handle_stop_states(msg)

    def _handle_motion_state(self, msg, sensor_b_trigger, sensor_a_trigger, sensor_aoth_trigger):
        with self.state_switch_lock:
            if self.is_global_repeat_protected():
                return
            now = time.time()
            if not hasattr(self, 'move_duration') or self.move_duration is None:
                self.move_duration = now
            elapsed = now - self.move_duration
            if elapsed >= 60:
                if self.current_status != "STOP":
                    rospy.logwarn(f"⚠️ {self.current_status} 状态持续{elapsed:.1f}s, 急停！！！")
                    self.set_state("STOP")
                    self.move_duration = None
                    elapsed = 0
            
            # 前进状态
            if self.current_status == self.status_list[1]:  # FORWARD
                if sensor_aoth_trigger:
                    self._complete_motion_and_reverse("FORWARD")
                    self.last_critical_switch_time = time.time()
                elif sensor_b_trigger and not msg.sensor_a:
                    current_time = time.time()
                    if current_time - self.last_switch_time < self.SWITCH_DELAY:
                        rospy.logwarn("UPSTOP反向切换触发间隔过短，忽略本次触发")
                        return
                    self.set_state("UPSTOP")
                    self.sensor_triggered["b"] = True
                    self.last_switch_time = time.time()
                elif sensor_a_trigger and not msg.sensor_b:
                    current_time = time.time()
                    if current_time - self.last_switch_time < self.SWITCH_DELAY:
                        rospy.logwarn("LOWSTOP反向切换触发间隔过短，忽略本次触发")
                        return
                    self.set_state("LOWSTOP")
                    self.sensor_triggered["a"] = True
                    self.last_switch_time = time.time()
            
            # 后退状态
            elif self.current_status == self.status_list[2]:  # BACKWARD
                if sensor_aoth_trigger:
                    self._complete_motion_and_reverse("BACKWARD")
                    self.last_critical_switch_time = time.time()
                elif sensor_b_trigger and not msg.sensor_a:
                    self.set_state("UPSTOP")
                    self.sensor_triggered["b"] = True
                elif sensor_a_trigger and not msg.sensor_b:
                    self.set_state("LOWSTOP")
                    self.sensor_triggered["a"] = True

    def _complete_motion_and_reverse(self, current_motion):
        current_time = time.time()
        if self.is_global_repeat_protected():
            return
        if current_time - self.last_switch_time < self.SWITCH_DELAY:
            rospy.logwarn("反向切换触发间隔过短，忽略本次触发")
            return
        rospy.loginfo(f"{current_motion}状态下双侧传感器触发，开始反向切换")
        self.brush_forward = not self.brush_forward
        
        # 清空状态
        self.complete_state = True
        self.initial_yaw = None
        self.progress = 100
        self.elevator_stage = 0
        
        # 反向切换
        target_state = "BACKWARD" if current_motion == "FORWARD" else "FORWARD"
        self.last_switch_time = current_time
        self.set_state(target_state)
        self.progress = 10
        self.last_critical_switch_time = current_time
        self.sensor_triggered = {"a": False, "b": False}

    def _handle_stop_states(self, msg):
        with self.state_switch_lock:
            if self.current_status not in ["UPSTOP", "LOWSTOP"]:
                return
            now = time.time()
            if not hasattr(self, 'side_duration_time') or self.side_duration_time is None:
                self.side_duration_time = now
                self.last_stop_state = self.current_status
            elif self.last_stop_state != self.current_status:
                self.side_duration_time = now
                self.last_stop_state = self.current_status
            elapsed = now - self.side_duration_time
            if elapsed >= self.TIMEOUT_THRESHOLD:
                rospy.logwarn(f"⚠️ {self.current_status} 状态持续{elapsed:.1f}s（> {self.TIMEOUT_THRESHOLD}s），强制切换")
                self.side_duration_time = None
                self.last_stop_state = None
                if self.current_status == "LOWSTOP":
                    self._switch_from_stop_state("LOWSTOP")
                elif self.current_status == "UPSTOP":
                    self._switch_from_stop_state("UPSTOP")
                return
            
            # 等待对侧传感器触发
            if self.current_status == "LOWSTOP":
                if msg.sensor_b and not self.sensor_triggered.get("b", False):
                    self._switch_from_stop_state("LOWSTOP")
            elif self.current_status == "UPSTOP":
                if msg.sensor_a and not self.sensor_triggered.get("a", False):
                    self._switch_from_stop_state("UPSTOP")

    def _switch_from_stop_state(self, stop_state):
        if self.is_global_repeat_protected():
            return
        rospy.loginfo(f"在{stop_state}执行，对侧传感器触发，开始反向切换")
        self.initial_yaw = None
        self.brush_forward = not self.brush_forward
        
        # 根据之前的运动状态反向切换
        if self.prev_motion_state == "FORWARD":
            self.set_state("BACKWARD")
        elif self.prev_motion_state == "BACKWARD":
            self.set_state("FORWARD")
        
        self.progress = 60
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
        correction = (self.pid_kp * error + self.pid_ki * self.pid_integral + self.pid_kd * derivative)
        self.pid_last_error = error
        return max(min(-correction, self.pid_correction_max), -self.pid_correction_max)

    def start_heartbeat(self, motor_id):
        self.heartbeat_running = True
        self.heartbeat_thread = threading.Thread(
            target=self._cycle_heartbeat,
            args=(motor_id,),
            daemon=True
        )
        self.heartbeat_thread.start()
        rospy.loginfo(f"❤️ 心跳启动：电机{motor_id}")

    def _cycle_heartbeat(self, motor_id, interval=1.5):
        while self.heartbeat_running and not rospy.is_shutdown():
            if self.rtu_client is None or not self.rtu_client.is_socket_open():
                self.reconnect_rtu_client()
                time.sleep(interval)
                continue
            self.read_motor_fault(motor_id)
            time.sleep(interval)

    def stop_heartbeat(self):
        self.heartbeat_running = False
        if self.heartbeat_thread and self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join(timeout=2)
        rospy.loginfo("❤️ 心跳停止")

    # def load_config(self, config_file="/home/orangepi/demo01/src/motor_can/config/servo_config.yaml"):
    def load_config(self, config_file="/home/ubuntu/demo01/src/motor_can/config/servo_config.yaml"):
        try:
            with open(config_file, 'r') as file:
                config = yaml.safe_load(file)
                return config
        except FileNotFoundError:
            rospy.logerr(f"❌ 配置文件未找到: {config_file}")
            return {}
        except Exception as e:
            rospy.logerr(f"❌ 加载配置出错: {e}")
            return {}

    def shutdown(self):
        rospy.loginfo("🔌 关闭控制器...")
        # 停止心跳
        self.stop_heartbeat()
        # 禁用所有5个电机
        if self.rtu_client is not None:
            for motor_id in self.motor_address_map.keys():
                try:
                    self.motor_stop(motor_id)
                except:
                    pass
            try:
                self.rtu_client.close()
            except:
                pass
        # 停止定时器
        if hasattr(self, 'publish_timer'):
            self.publish_timer.shutdown()
        rospy.loginfo("✅ 控制器已关闭")

    # -------------------------- 键盘监听（适配5电机） --------------------------
    def keyboard_listener(self):
        rospy.loginfo("⌨️  按键控制：s=停止, f=前进, b=后退, a=启动, r=反转, l=加载, p=暂停, u=卸载, 1=上停, 2=下停, 4=电机4启动, 5=电机5启动")
        while not rospy.is_shutdown():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.readline().strip()
                if key:
                    self.update_status_by_key(key)

    def update_status_by_key(self, key):
        key_mapping = {
            's': "STOP", 'f': "FORWARD", 'b': "BACKWARD", 'a': "START",
            'r': "REVERSE", 'l': "LOADING", 'p': "PAUSE", 'u': "UNLOADING",
            '1': "UPSTOP", '2': "LOWSTOP", '4': "PISTON_OUT", '5': "CHARGE_OUT"
        }
        if key in key_mapping:
            if key != 'a':
                self.auto_mode = False
            else:
                self.auto_mode = True
            self.set_state(key_mapping[key])
        else:
            rospy.loginfo(f"⚠️  无效按键: {key}")

def main():
    controller = None
    try:
        controller = ServoDriveController()
        # 加载配置
        config = controller.load_config()
        if not config or "motors" not in config:
            rospy.logerr("❌ 无有效电机配置")
        else:
            rospy.loginfo("⚙️  初始化电机...")
            for motor in config["motors"]:
                motor_id = motor.get("id")
                velocity = motor.get("velocity")
                if None in (motor_id, velocity):
                    rospy.logwarn(f"⚠️  跳过无效配置: {motor}")
                    continue
                try:
                    controller.motor_start(motor_id, int(velocity))
                    controller.current_status = controller.status_list[0]
                except Exception as e:
                    rospy.logerr(f"❌ 配置电机 {motor_id} 出错: {e}")
        
        # 启动键盘监听
        if controller is not None:
            t = threading.Thread(target=controller.keyboard_listener, daemon=True)
            t.start()
        
        # 主循环
        while not rospy.is_shutdown():
            controller.execute_state()
            controller.rate.sleep()
            
    except KeyboardInterrupt:
        rospy.loginfo("🛑 用户终止程序")
    except Exception as e:
        rospy.logfatal(f"💥 节点启动失败: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if controller is not None:
            controller.shutdown()

if __name__ == "__main__":
    main()