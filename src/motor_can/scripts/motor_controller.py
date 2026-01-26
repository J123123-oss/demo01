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

# -------------------------- 驱动器核心配置 --------------------------
BAUDRATE = 38400
# SERIAL_PORT = "/dev/ttyv1"  # 根据实际端口修改
SERIAL_PORT = "/dev/Motor-DBLS"  # 根据实际端口修改
STOP_BITS = 1
PARITY = "N"
TIMEOUT = 3.0  # 通讯超时
COMMAND_INTERVAL = 0.05  # 指令间隔50ms

# 寄存器地址
REG_CONTROL_MODE = 32768  # $8000：控制位+极对数
REG_SPEED_SET = 32773     # $8005：速度设定（RPM）
REG_FAULT_STATUS = 32795  # $801B：故障状态
REG_ACTUAL_SPEED = 32792  # $8018：实际转速
# REG_ACTUAL_SPEED = 32773  # $8018：查询设置8005转速

# 控制位定义
CONTROL_EN = 0x01  # 使能
CONTROL_FR = 0x02  # 正反转
CONTROL_BK = 0x04  # 刹车
CONTROL_NW = 0x08  # 通讯控制

# 速比配置
RATE = 1

# 故障码映射
FAULT_MAP = {
    0x00: "无故障",
    0x01: "堵转",
    0x02: "过流",
    0x08: "母线电压过低",
    0x10: "母线电压过高",
    0x20: "电流峰值报警",
    0x80: "通讯中断报警",
    0x88: "通讯中断报警（扩展码）"
}

class ServoDriveController:
    def __init__(self):
        # ROS初始化
        rospy.init_node('motor_modbus_rtu_node', anonymous=True)
        self.rate = rospy.Rate(20)

        # 初始化RTU客户端为None，避免空值报错
        self.rtu_client = None
        self.motor_address_map = {1:1,2:2,3:3}
        self.motor_pole_pairs = {1:5,2:5,3:5}

        # 状态变量（保留原有）
        self.last_left_speed = 0
        self.last_right_speed = 0
        self.last_brush_speed = 0
        self.has_reverse_flag = False
        self.has_reverse_counter = 0
        self.reverse_start_time = None
        self.main_board = True
        self.imu_sensor = True
        self.motor_driver = True
        self.motor_base = 600
        self.brush_base_speed = 1200
        self.brush_forward = rospy.get_param('~brush_forward', False)# 默认反转 True=正转，False=反转
        self.flag = 0
        self.speed_pluse_max = 1000 * RATE
        self.reversed_start_time = None
        self.REVERSE_TIME_THRESHOLD = 1.0
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
        self.SWITCH_DELAY = 5 # 触发延时阈值，单位：秒
        self.state_change_protect_delay = 5.0
        self.last_state_change_time = 0.0  # 记录上次状态切换时间
        self.GLOBAL_REPEAT_DELAY = 3.5  # 3秒内不重复触发关键状态
        self.last_critical_switch_time = 0.0  # 记录上次关键状态切换时间

        self.is_outside_bin = True  # 默认仓外（BACKWARD）

        self.motor_control_state = {}  # 缓存格式：{motor_id: {"enable": bool, "direction": int, "brake": bool}}
        # 初始化所有电机的默认状态（根据实际场景调整）
        for motor_id in self.motor_pole_pairs.keys():
            self.motor_control_state[motor_id] = {"enable": False, "direction": 0, "brake": False}

        # 状态列表
        self.status_list = [
            "STOP", "FORWARD", "BACKWARD", "START", "LOADING", "UNLOADING",
            "UPSTOP", "LOWSTOP", "PISTON_OUT", "PISTON_IN", "CHARGE_OUT",
            "RETURN_DOCK"
        ]

        # 状态速度配置
        self.status_config = {
            "START": {},
            "STOP": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": 0},
            "UNLOADING": {"velocity_up": -self.motor_base * RATE, "velocity_low": self.motor_base * RATE, "velocity_brush": -self.brush_base_speed},
            "FORWARD": {"velocity_up": self.motor_base * RATE, "velocity_low": -self.motor_base * RATE, "velocity_brush": -self.brush_base_speed},
            "BACKWARD": {"velocity_up": -self.motor_base * RATE, "velocity_low": self.motor_base * RATE, "velocity_brush": self.brush_base_speed},
            "LOADING": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": 0},
            "UPSTOP": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": -self.brush_base_speed},
            "LOWSTOP": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": -self.brush_base_speed},
            "REVERSE": {"velocity_up": 0, "velocity_low": 0, "velocity_brush": -self.brush_base_speed},
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

        # PID参数
        self.pid_integral = 0.0
        self.pid_last_error = 0.0
        self.target_yaw = 0.0
        self.pid_kp = 80
        self.pid_ki = 0
        self.pid_kd = 0.5
        self.pid_correction_max = 200
        self.progress = 0
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

        self.state_switch_lock = threading.Lock()
        self.sensor_triggered = {"a": False, "b": False}  # 传感器触发标记
        # 发布定时器
        self.publish_timer = rospy.Timer(rospy.Duration(1.0), self.publish_state)

        # 连接RTU客户端（增加超时保护）
        self.connect_rtu_client_with_timeout()

        # 启动IMU
        # self.start_imu()

    # -------------------------- 修复RTU连接问题 --------------------------
    def connect_rtu_client_with_timeout(self):
        """连接RTU客户端（增加超时保护，避免无限循环）"""
        max_retry = 5  # 最大重试5次
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
        """重连RTU客户端（增加空值判断）"""
        rospy.logwarn("🔄 尝试重连RTU客户端...")
        # 空值判断：避免关闭None对象
        if self.rtu_client is not None:
            try:
                self.rtu_client.close()
            except:
                pass
        self.rtu_client = None
        # 重新连接（单次尝试，避免无限循环）
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

    # -------------------------- 修复RTU读写操作（增加空值判断）--------------------------
    def rtu_write_register(self, motor_id, reg_addr, value):
        """RTU写寄存器（增加空值判断）"""
        # 先检查客户端是否有效
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
        """RTU读寄存器（增加空值判断）"""
        # 先检查客户端是否有效
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

    # -------------------------- 电机控制方法（增加空值保护）--------------------------
    def is_global_repeat_protected(self):
        """判断是否在5秒全局防重复触发保护期内"""
        current_time = time.time()
        # 判断5秒全局防重复触发期
        if current_time - self.last_critical_switch_time < self.GLOBAL_REPEAT_DELAY:
            rospy.logwarn(f"3秒全局保护期内，距离上次触发还有{self.GLOBAL_REPEAT_DELAY - (current_time - self.last_critical_switch_time):.1f}秒")
            return True
        return False
    
    def set_control_mode(self, motor_id, enable=True, direction=0, brake=False):
        """设置控制模式（增加空值保护+状态缓存）"""
        if self.rtu_client is None:
            rospy.logerr("❌ RTU客户端未连接，无法设置控制模式")
            return False
        
        # 先校验是否需要修改（缓存值与目标值一致则直接返回成功）
        current_state = self.motor_control_state.get(motor_id, {})
        if (current_state.get("enable") == enable and
            current_state.get("direction") == direction and
            current_state.get("brake") == brake):
            rospy.logdebug(f"ℹ️  电机{motor_id}控制模式无需修改：使能={enable}，方向={direction}，刹车={brake}")
            return True
        
        pole_pairs = self.motor_pole_pairs.get(motor_id, 5)
        high_byte = CONTROL_NW | (CONTROL_EN if enable else 0) | (CONTROL_FR if direction else 0) | (CONTROL_BK if brake else 0)
        low_byte = pole_pairs
        control_value = (high_byte << 8) | low_byte
        
        success = self.rtu_write_register(motor_id, REG_CONTROL_MODE, control_value)
        if success:
            # 更新缓存
            self.motor_control_state[motor_id] = {"enable": enable, "direction": direction, "brake": brake}
            rospy.loginfo(f"✅ 电机{motor_id}控制模式设置成功：使能={enable}，方向={direction}，刹车={brake}")
        return success

    def set_target_velocity(self, motor_id, velocity):
        """设置电机速度（适配小端序：将速度值转为小端序后再发送）"""
        if self.rtu_client is None:
            rospy.logerr("❌ RTU客户端未连接，无法设置速度")
            return False
        
        # 1. 基础速度值处理（原有逻辑）
        velocity_abs = abs(velocity)
        velocity_abs = max(min(velocity_abs, 65535), 0)
        direction = 1 if velocity < 0 else 0

        # 2. 将速度值转为小端序（16位整数高低位互换）
        # 原理：大端序是 高位字节<<8 + 低位字节；小端序是 低位字节<<8 + 高位字节
        velocity_low = velocity_abs & 0xFF        # 提取低位字节（0-255）
        velocity_high = (velocity_abs >> 8) & 0xFF # 提取高位字节（0-255）
        velocity_little_endian = (velocity_low << 8) | velocity_high  # 小端序值

        # 3. 发送小端序的速度值（替换原有velocity_abs）
        success = self.rtu_write_register(motor_id, REG_SPEED_SET, velocity_little_endian)
        
        if success:
            self.set_control_mode(motor_id, enable=True, direction=direction)
            # rospy.loginfo(f"✅ 电机{motor_id}速度设置成功：{velocity} ")
        #     current_direction = self.motor_control_state.get(motor_id, {}).get("direction", 0)
        
        # # 4. 方向校验（原有逻辑保留）
        # if direction != current_direction:
        #     mode_success = self.set_control_mode(motor_id, enable=True, direction=direction, brake=False)
        #     if not mode_success:
        #         rospy.logerr(f"❌ 电机{motor_id}方向修改失败")
        #         return False
        
        return success

    def get_actual_velocity(self, motor_id):
        """读取实际转速（增加空值保护）"""
        if self.rtu_client is None:
            rospy.logerr("❌ RTU客户端未连接，无法读取转速")
            return 0
        
        registers = self.rtu_read_register(motor_id, REG_ACTUAL_SPEED, count=1)
        if not registers or len(registers) != 1:
            rospy.logwarn(f" 读取电机{motor_id}转速失败")
            return 0
        
        speed_code_little = registers[0]
    
        # 2. 核心：小端序解析为真实原始值（高低位互换）
        # 原理：驱动器返回的speed_code_little是小端序（低位字节<<8 + 高位字节），需还原为大端序原始值
        speed_high= speed_code_little & 0xFF        # 提取小端序的低位字节
        speed_low= (speed_code_little >> 8) & 0xFF # 提取小端序的高位字节
        speed_code_big = (speed_high << 8) | speed_low  # 还原

        pole_pairs = self.motor_pole_pairs.get(motor_id, 5)
        actual_speed = speed_code_big *20/10
        # actual_speed = max(min(actual_speed, 65535), 0)
        print(f"{motor_id}电机转速：{actual_speed}")
        
        return round(actual_speed, 2)

    def read_fault_code(self, motor_id):
        """读取故障码（增加空值保护）"""
        if self.rtu_client is None:
            rospy.logerr("❌ RTU客户端未连接，无法读取故障码")
            return None, "客户端未连接"
        
        registers = self.rtu_read_register(motor_id, REG_FAULT_STATUS, count=1)
        if not registers:
            rospy.logwarn(f" 读取电机{motor_id}故障码失败")
            return None, "读取失败"
        
        fault_16 = registers[0]
        fault_code = (fault_16 >> 8) & 0xFF
        # print(f"读取电机{motor_id}故障码{fault_code}")
        fault_desc = FAULT_MAP.get(fault_code, f"未知故障（0x{fault_code:02X}）")
        
        if fault_code != 0x00:
            rospy.logwarn(f" 电机{motor_id}故障：{fault_desc}")
            self.motor_driver = False
        else:
            self.motor_driver = True
        
        return fault_code, fault_desc

    def clear_fault(self, motor_id):
        """清除故障（增加空值保护）"""
        if self.rtu_client is None:
            rospy.logerr("❌ RTU客户端未连接，无法清除故障")
            return False
        
        rospy.loginfo(f"🔧 清除电机{motor_id}故障...")
        # self.set_control_mode(motor_id, enable=False, brake=True)
        time.sleep(0.5)
        # success = self.set_control_mode(motor_id, enable=True, brake=False)
        
        # if success:
        #     rospy.loginfo(f"✅ 电机{motor_id}故障清除成功")
        #     self.motor_driver = True
        # return success

    def enable_drive(self, motor_id):
        """使能电机"""
        return self.set_control_mode(motor_id, enable=True, brake=False)

    def disable_drive(self, motor_id):
        """禁用电机（增加空值判断）"""
        if self.rtu_client is None:
            rospy.logerr(f"❌ RTU客户端未连接，无法禁用电机{motor_id}")
            return False
        return self.set_control_mode(motor_id, enable=False, brake=True)

    def start_motor(self, motor_id):
        """启动电机（增加空值保护）"""
        if self.rtu_client is None:
            rospy.logerr("❌ RTU客户端未连接，无法启动电机")
            return False
        
        if motor_id not in self.motor_address_map:
            rospy.logerr(f"❌ 无效电机ID：{motor_id}")
            return False
        
        fault_code, _ = self.read_fault_code(motor_id)
        # if fault_code and fault_code != 0x00:
        #     self.clear_fault(motor_id)
        
        success = self.set_control_mode(motor_id, enable=True)
        if success:
            rospy.loginfo(f"✅ 电机{motor_id}初始化完成")
        return success

    # -------------------------- 修复键盘监听静态方法问题 --------------------------
    def keyboard_listener(self):
        """实例方法：键盘监听（替代静态方法）"""
        rospy.loginfo("⌨️  按键控制：s=停止, f=前进, b=后退, a=启动, r=反转, l=加载, p=暂停, u=卸载, 1=上停, 2=下停")
        while not rospy.is_shutdown():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.readline().strip()
                if key:
                    self.update_status_by_key(key)

    def update_status_by_key(self, key):
        """按键状态更新"""
        key_mapping = {
            's': "STOP", 'f': "FORWARD", 'b': "BACKWARD", 'a': "START",
            'r': "REVERSE", 'l': "LOADING", 'u': "UNLOADING",
            '1': "UPSTOP", '2': "LOWSTOP"
        }
        if key in key_mapping:
            if key != 'a':
                self.auto_mode = False
            else:
                self.auto_mode = True
            self.set_state(key_mapping[key])
        else:
            rospy.loginfo(f"  无效按键: {key}")

    # -------------------------- 其他核心方法（保留并增加保护）--------------------------
    def set_state(self, new_state):
        if new_state not in self.status_config:
            rospy.logwarn(f" 尝试设置无效状态: {new_state}")
            return False
        if new_state == self.current_status and new_state not in ["PISTON_OUT", "PISTON_IN", "STOP"]:
            return False
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

        elif new_state == "PISTON_IN":
            self.initial_yaw = None
            self.auto_step = None
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
        """发布状态（增加空值保护）"""
        try:
            # self.velocity_publish_count += 1
            # if self.velocity_publish_count >= self.velocity_publish_interval and self.rtu_client is not None:
                # self.last_velocity_up = self.get_actual_velocity(2)
                # self.last_velocity_low = self.get_actual_velocity(1)
                # self.last_velocity_brush = self.get_actual_velocity(3)
                # self.velocity_publish_count = 0

            # 构建状态消息
            state_msg = {
                "status": self.current_status,
                "battery": self.battery_remaining,
                "battery_temperatures": self.battery_temperatures,
                "battery_total_voltage": self.battery_total_voltage,
                "battery_current": self.battery_current,
                "progress": self.progress,
                "imu_yaw": round(self.imu_yaw, 2) if self.imu_yaw is not None else 0.00,
                "velocity_up": round(self.last_left_speed *20/15, 2),
                "velocity_low": round(self.last_right_speed*20/15, 2),
                "velocity_brush": round(self.last_brush_speed * 20 / 15, 2),
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

    def status_callback(self, msg):
        """指令回调"""
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
                rospy.logwarn(f" 未找到command字段: {msg.data}")
        except Exception as e:
            rospy.logwarn(f" 消息解析失败: {msg.data}, 错误: {e}")
            if msg.data == "BRUSH_FORWARD":
                self.brush_forward = not self.brush_forward
            self.set_state(msg.data)
            self.publish_state()

    def imu_callback(self, msg):
        """IMU回调"""
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
        """电池回调"""
        self.battery_remaining = msg.batttery_remaining
        self.battery_total_voltage = round(msg.total_voltage, 2)
        self.battery_current = round(msg.current, 2)
        self.battery_temperatures = [round(t, 1) for t in msg.temperatures] if hasattr(msg, "temperatures") else []

    def relay_callback(self, msg):
        """继电器回调"""
        self.relay_status = msg.data

    def delayed_publish_freq_switch(self, delay_sec=3):
        """延迟切换发布频率"""
        time.sleep(delay_sec)
        if self.current_status == "STOP" and not rospy.is_shutdown():
            self.publish_timer.shutdown()
            self.publish_timer = rospy.Timer(rospy.Duration(1800), self.publish_state)

    def proximity_callback(self, msg):
        """核心修改：统一自动/手动模式下的sensor_b/sensor_a处理逻辑"""
        # 仓外：true触发|     a true UPSTOP / b true LOWSTOP     a b  true  LOADING 切换仓内

        # 仓内：false触发|   已经切换FORWARD   a b  false 仓外
        # 1. 更新传感器状态
        if msg.sensor_b:
            self.sensors_status |= 0x01
        else:
            self.sensors_status &= ~0x01
        if msg.sensor_a:
            self.sensors_status |= 0x02
        else:
            self.sensors_status &= ~0x02
        
        # 2. 传感器消抖（仅首次触发时处理）
        sensor_a_trigger = msg.sensor_a and not self.sensor_triggered["a"]
        sensor_b_trigger = msg.sensor_b and not self.sensor_triggered["b"]
        sensor_both_trigger = msg.sensor_b and msg.sensor_a

        # 3. 仓外→仓内 切换逻辑（双传感器触发=到位进仓）
        if self.is_outside_bin and sensor_both_trigger:
            self._switch_to_bin_in()  # 进入仓内
            return  # 仓内逻辑优先，跳过仓外的运动判断

        # 4. 仓内→仓外 切换逻辑（双传感器未触发+停留时间达标）
        if not self.is_outside_bin and not sensor_both_trigger:
            if not msg.sensor_a and not msg.sensor_b:
                self._switch_to_bin_out()  # 退出仓外
                return  # 仓外逻辑优先，跳过仓内的运动判断
                
        # 5. 仓外逻辑（非双触发=仓外，执行原有运动/中间态逻辑）
        if self.is_outside_bin and self.current_status in ["FORWARD", "BACKWARD", "UPSTOP", "LOWSTOP"]:
            self._handle_outside(msg, sensor_b_trigger, sensor_a_trigger)
            # print("仓外逻辑sensor_b_trigger:",sensor_b_trigger)


        # # 3. RETURN_DOCK/CHARGE_OUT 特殊
        # if self.auto_mode and self.current_status == "RETURN_DOCK":
        #     if self.elevator_stage == 2:
        #         self.set_state("FORWARD")

        # if self.auto_mode and self.current_status == "CHARGE_OUT":
        #     if self.elevator_stage == 2:
        #         if msg.sensor_b or msg.sensor_a:
        #             if self.current_status != "UNLOADING":
        #                 self.set_state("UNLOADING")
        #                 self.progress = 10
        #                 self.unloading_start_time = time.time()
        #         if self.current_status == "UNLOADING" and self.unloading_start_time:
        #             elapsed = time.time() - self.unloading_start_time
        #             if elapsed >= self.unloading_timer:
        #                 self.set_state("STOP")
        #                 self.progress = 0
        #                 self.unloading_start_time = None

        # 4. START状态逻辑（保留原有）
        if self.auto_mode and self.current_status == "START":
            if self.elevator_stage == 2:
                
                self.set_state("BACKWARD")
                self.progress = 20
                # self.unloading_start_time = None

        # 5. REVERSE状态边界检测（保留原有）
        if self.current_status == "REVERSE":
            if msg.sensor_b or msg.sensor_a:
                rospy.logwarn("边界触发，立即STOP")
                self.set_state("STOP")
            return

        # 6. 统一处理自动/手动模式的FORWARD/BACKWARD状态
        # self._handle_motion_state(msg, sensor_b_trigger, sensor_a_trigger, sensor_both_trigger)

        # 7. 处理UPSTOP/LOWSTOP状态（等待另一侧传感器触发后反向）
        # self._handle_mid_states(msg)
    def _switch_to_bin_in(self):
        """仓外→仓内：双传感器触发，进入仓内停留"""
        # if self.is_global_repeat_protected():
        #     return
        rospy.loginfo("双传感器触发→进入仓内（LOADING）")
        self.is_outside_bin = False
        self.set_state("LOADING")
        self.progress = 100  # 到位进度
        # 重置传感器标记
        # self.sensor_triggered = {"a": False, "b": False}
        if self.prev_motion_state == "BACKWARD":
            self.set_state("FORWARD")
        elif self.prev_motion_state == "FORWARD":
            self.set_state("BACKWARD")
        elif not self.prev_motion_state:
            self.set_state("BACKWARD")# 启动默认仓外后退
        self.progress = 20
        self.brush_forward = not self.brush_forward  # 反向（原有逻辑保留）
        self.last_switch_time = time.time()
    

    def _switch_to_bin_out(self):
        """仓内→仓外：双传感器未触发,切换回仓外（默认BACKWARD）"""
        rospy.loginfo("切换仓外")
        self.is_outside_bin = True
        self.last_switch_time = time.time()


        

    def _handle_outside(self, msg, sensor_b_trigger, sensor_a_trigger):
        """仓外逻辑：仅处理前进/后退/中间态，不涉及仓内"""
        with self.state_switch_lock:
            # if self.is_global_repeat_protected():
            #     return

            # 仓外主态：FORWARD/BACKWARD
            if self.current_status in ["FORWARD", "BACKWARD"]:
                # 单侧触发→进入中间态（UPSTOP/LOWSTOP）
                if sensor_b_trigger and msg.sensor_b:
                    # if time.time() - self.last_switch_time < self.SWITCH_DELAY:
                    #     rospy.logwarn("UPSTOP触发间隔过短，忽略")
                    #     return
                    self.set_state("UPSTOP")
                    self.sensor_triggered["b"] = True
                    self.last_switch_time = time.time()
                elif sensor_a_trigger and msg.sensor_a:
                    # if time.time() - self.last_switch_time < self.SWITCH_DELAY:
                    #     rospy.logwarn("LOWSTOP触发间隔过短，忽略")
                    #     return
                    self.set_state("LOWSTOP")
                    self.sensor_triggered["a"] = True
                    self.last_switch_time = time.time()
            time.sleep(0.2)

            # # 仓外中间态：UPSTOP/LOWSTOP（仅切换方向，不进仓内）
            # elif self.current_status in ["UPSTOP", "LOWSTOP"]:
            #     self._handle_mid_states(msg)








    # def _handle_motion_state(self, msg, sensor_b_trigger, sensor_a_trigger, sensor_both_trigger):
    #     """处理FORWARD/BACKWARD状态的传感器逻辑（自动/手动统一）"""
    #     with self.state_switch_lock:
    #         # 先判断是否处于状态切换保护期，保护期内直接返回
    #         if self.is_global_repeat_protected():
    #             rospy.logdebug("状态切换保护期")
    #             return
    #         # 前进状态
    #         if self.current_status == self.status_list[1]:  # FORWARD
    #             # 双侧传感器触发：完成任务，反向
    #             if sensor_both_trigger:
    #                 self._complete_motion_and_reverse("FORWARD")
    #                 self.last_critical_switch_time = time.time()
    #             # 单侧传感器触发：进入对应停止状态
    #             elif sensor_b_trigger and not msg.sensor_a:
    #                 current_time = time.time()  # 获取当前时间戳
    #                 # 核心：判断是否超过延时阈值
    #                 if current_time - self.last_switch_time < self.SWITCH_DELAY:
    #                     rospy.logwarn("UPSTOP反向切换触发间隔过短，忽略本次触发")
    #                     return
    #                 self.set_state("UPSTOP")
    #                 self.sensor_triggered["a"] = True
    #                 self.last_switch_time = time.time()
    #             elif sensor_a_trigger and not msg.sensor_b:
    #                 current_time = time.time()  # 获取当前时间戳
    #                 # 核心：判断是否超过延时阈值
    #                 if current_time - self.last_switch_time < self.SWITCH_DELAY:
    #                     rospy.logwarn("LOWSTOP反向切换触发间隔过短，忽略本次触发")
    #                     return
    #                 self.set_state("LOWSTOP")
    #                 self.sensor_triggered["b"] = True
    #                 self.last_switch_time = time.time()
                    

    #         # 后退状态
    #         elif self.current_status == self.status_list[2]:  # BACKWARD
    #             # 双侧传感器触发：完成任务，反向
    #             if sensor_both_trigger:
    #                 self._complete_motion_and_reverse("BACKWARD")
    #                 self.last_critical_switch_time = time.time()
    #             # 单侧传感器触发：进入对应停止状态
    #             elif sensor_b_trigger and not msg.sensor_a:
    #                 self.set_state("UPSTOP")
    #                 self.sensor_triggered["a"] = True
    #             elif sensor_a_trigger and not msg.sensor_b:
    #                 self.set_state("LOWSTOP")
    #                 self.sensor_triggered["b"] = True

    # def _complete_motion_and_reverse(self, current_motion):
    #     """完成运动并反向切换"""
    #     current_time = time.time()  # 获取当前时间戳
    #     if self.is_global_repeat_protected():
    #         return
    #     # 判断是否过启动屏蔽期（3秒内直接返回）
    #     # if not self.is_proximity_sensor_enabled():
    #     #     rospy.logdebug("系统启动未满3秒，屏蔽接近开关触发")
    #     #     return
    #     # 核心：判断是否超过延时阈值
    #     if current_time - self.last_switch_time < self.SWITCH_DELAY:
    #         rospy.logwarn("反向切换触发间隔过短，忽略本次触发")
    #         return
    #     rospy.loginfo(f"{current_motion}状态下双侧传感器触发，开始反向切换")
    #     self.brush_forward = not self.brush_forward
    #     # self.set_state("LOADING")
    #     # time.sleep(1.0)
        
    #     # 清空状态
    #     self.complete_state = True
    #     self.initial_yaw = None
    #     self.progress = 100
    #     self.auto_step = None
    #     self.elevator_stage = 0
        
    #     # 反向切换
    #     target_state = "BACKWARD" if current_motion == "FORWARD" else "FORWARD"
    #     self.last_switch_time = current_time
    #     self.set_state(target_state)
    #     self.progress = 10
    #     self.last_critical_switch_time = current_time
    #     # 重置传感器触发标记
    #     self.sensor_triggered = {"a": False, "b": False}

    def _handle_mid_states(self, msg):
        """处理UPSTOP/LOWSTOP状态：等待另一侧传感器触发后反向"""
        with self.state_switch_lock:
            # self.last_state_change_time = time.time()
            # LOWSTOP：等待sensor_b触发（另一侧）
            if self.current_status == self.status_list[7]:  # LOWSTOP
                # if msg.sensor_b and not self.sensor_triggered["b"]:
                self._switch_from_mid_state("LOWSTOP")
            
            # UPSTOP：等待sensor_a触发（另一侧）
            elif self.current_status == self.status_list[6]:  # UPSTOP
                # if msg.sensor_a and not self.sensor_triggered["a"]:
                self._switch_from_mid_state("UPSTOP")


    def _switch_from_mid_state(self, stop_state):
        """从中间状态切换为反向运动"""
        if self.is_global_repeat_protected():
            return
        rospy.loginfo(f"在{stop_state}执行，对侧传感器触发，开始反向切换")
        self.initial_yaw = None
        self.brush_forward = not self.brush_forward
        # self.set_state("LOADING")
        # time.sleep(1.0)
        
        # 根据之前的运动状态反向切换
        if self.prev_motion_state == "FORWARD":
            self.set_state("BACKWARD")
        elif self.prev_motion_state == "BACKWARD":
            self.set_state("FORWARD")
        # else:
            # 默认BACKWARD
            # self.set_state("FORWARD")
        
        self.progress = 60
        # 重置传感器触发标记
        self.sensor_triggered = {"a": False, "b": False}

    def pid_correction(self, current_yaw):
        """PID矫正"""
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

    def execute_state(self):
        """状态执行"""
        # 客户端未连接时不执行电机控制
        if self.rtu_client is None:
            rospy.logwarn(" RTU客户端未连接，跳过电机控制")
            return

        # START状态
        if self.enable_drive_flag and (self.current_status == "START" or self.current_status == "CHARGE_OUT" or self.current_status == "RETURN_DOCK"):
            if self.battery_remaining is not None and self.battery_remaining < self.LOW_BATTERY_THRESHOLD:
                rospy.logerr("🔋 电量过低，停止启动")
                self.enable_drive_flag = False
                self.main_board = False
                self.set_state("STOP")
                return
            if self.elevator_stage == 0:
                rospy.loginfo("🔼 电缸抬起...")
                self.motor_cmd_pub.publish(Int8(data=1))
                self.elevator_start_time = rospy.get_time()
                self.elevator_stage = 1
            elif self.elevator_stage == 1:
                elapsed = rospy.get_time() - self.elevator_start_time
                if elapsed >= 0.1:
                    rospy.loginfo("⚙️  配置电机...")
                    config = self.load_config()
                    for motor in config.get("motors", []):
                        motor_id = motor.get("id")
                        velocity = motor.get("velocity")
                        if None in (motor_id, velocity):
                            rospy.logwarn(f"  跳过无效配置: {motor}")
                            continue
                        try:
                            self.configure_motor(motor_id=motor_id, velocity=int(velocity * RATE))
                        except Exception as e:
                            rospy.logerr(f"❌ 配置电机 {motor_id} 出错: {e}")
                    rospy.loginfo("✅ 电机配置完成")
                    self.enable_drive_flag = False
                    self.elevator_stage = 2
                    self.start_time = rospy.get_time()
                    if self.auto_mode and self.auto_step and self.current_status == "START":
                        self.set_state(self.auto_step)

        # FORWARD/BACKWARD
        if self.current_status in ["FORWARD", "BACKWARD"]:
            correction = self.pid_correction(self.imu_yaw)
            left_speed = int(self.status_config[self.current_status]["velocity_up"] + correction)
            right_speed = int(self.status_config[self.current_status]["velocity_low"] + correction)
            brush_speed = int(self.status_config[self.current_status]["velocity_brush"])
            right_speed = max(min(right_speed, self.speed_pluse_max), -self.speed_pluse_max)
            left_speed = max(min(left_speed, self.speed_pluse_max), -self.speed_pluse_max)

            if (self.last_left_speed != left_speed or self.last_right_speed != right_speed or self.last_brush_speed != brush_speed):
                self.set_target_velocity(2, left_speed)
                self.set_target_velocity(1, right_speed)
                self.set_target_velocity(3, brush_speed)
                self.last_left_speed = left_speed
                self.last_right_speed = right_speed
                self.last_brush_speed = brush_speed

            # 角度偏差检测
            angle_condition_met = (-7 < self.imu_yaw < -2.0 or 2.0 < self.imu_yaw < 7)
            # angle_condition_met = None
            if angle_condition_met:
                if self.reversed_start_time is None:
                    self.reversed_start_time = rospy.get_time()
                    rospy.logwarn(f"  角度偏差: {self.imu_yaw:.2f}°")
                elapsed = rospy.get_time() - self.reversed_start_time
                if elapsed >= self.REVERSE_TIME_THRESHOLD:
                    rospy.logwarn(f"  进入反转矫正")
                    self.set_state("REVERSE")
                    self.reversed_start_time = None
            else:
                self.reversed_start_time = None

        # REVERSE
        elif self.current_status == "REVERSE":
            self.reversed_start_time = None
            if not self.has_reverse_flag:
                self.has_reverse_counter += 1
                if self.has_reverse_counter > 100:
                    rospy.logwarn("  连续反转100次，停止")
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
                if (self.last_left_speed != left_speed or self.last_right_speed != right_speed or self.last_brush_speed != brush_speed):
                    self.set_target_velocity(2, left_speed)
                    self.set_target_velocity(1, right_speed)
                    self.set_target_velocity(3, brush_speed)
                self.last_left_speed = left_speed
                self.last_right_speed = right_speed
                self.last_brush_speed = brush_speed
                self.has_reverse_flag = True
                self.reverse_start_time = time.time()
                # time.sleep(2.0)
            else:
                self.flag = -1 if self.imu_yaw >= 0 else 1
                if abs(self.imu_yaw) > 1:
                    right_speed = left_speed = int(-self.motor_base * RATE * 0.8 * self.flag)
                else:
                    right_speed = left_speed = int(-self.motor_base * RATE * 0.6 * self.flag)
                right_speed = max(min(right_speed, self.speed_pluse_max), -self.speed_pluse_max)
                left_speed = max(min(left_speed, self.speed_pluse_max), -self.speed_pluse_max)
                brush_speed = self.last_brush_speed
                if (self.last_left_speed != left_speed or self.last_right_speed != right_speed or self.last_brush_speed != brush_speed):
                    self.set_target_velocity(2, left_speed)
                    self.set_target_velocity(1, right_speed)
                    self.set_target_velocity(3, brush_speed)
                self.last_left_speed = left_speed
                self.last_right_speed = right_speed
                self.last_brush_speed = brush_speed
                if abs(self.imu_yaw) < 0.2:
                    if self.prev_motion_state:
                        self.set_state(self.prev_motion_state)
                    self.is_upstop = False
                    self.is_lowstop = False
                    self.has_reverse_flag = False

        # UPSTOP/LOWSTOP
        elif self.current_status == "UPSTOP":
            left_speed = 0
            right_speed = self.last_right_speed
            # brush_speed = -self.brush_base_speed
            # if (self.last_left_speed != left_speed or self.last_right_speed != right_speed ):
            # while True:
            self.set_target_velocity(2, left_speed)
            self.set_target_velocity(1, right_speed)
            # self.set_target_velocity(3, brush_speed)
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            # self.last_brush_speed = brush_speed
        elif self.current_status == "LOWSTOP":
            left_speed = self.last_left_speed
            right_speed = 0
            # brush_speed = -self.brush_base_speed
            # if (self.last_left_speed != left_speed or self.last_right_speed != right_speed ):
            # while True:
            self.set_target_velocity(2, left_speed)
            self.set_target_velocity(1, right_speed)
            # self.set_target_velocity(3, brush_speed)
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed
            # self.last_brush_speed = brush_speed

        # STOP
        elif self.current_status == "STOP" or not -5 < self.imu_yaw < 5:
            if (self.last_left_speed != 0 or self.last_right_speed != 0 or self.last_brush_speed != 0):
                self.has_reverse_counter = 0
                for motor_id in self.motor_address_map.keys():
                    self.disable_drive(motor_id)
                self.last_left_speed = 0
                self.last_right_speed = 0
                self.last_brush_speed = 0

        # UNLOADING
        elif self.current_status == "UNLOADING":
            correction = self.pid_correction(self.imu_yaw)
            left_speed = int(self.status_config[self.current_status]["velocity_up"] + correction)
            right_speed = int(self.status_config[self.current_status]["velocity_low"] + correction)
            brush_speed = int(self.status_config[self.current_status]["velocity_brush"])
            if (self.last_left_speed != left_speed or self.last_right_speed != right_speed or self.last_brush_speed != brush_speed):
                self.set_target_velocity(2, left_speed)
                self.set_target_velocity(1, right_speed)
                self.set_target_velocity(3, brush_speed)
                self.last_left_speed = left_speed
                self.last_right_speed = right_speed
                self.last_brush_speed = brush_speed

        # LOADING
        elif self.current_status in ["LOADING"]:
            left_speed = int(self.status_config[self.current_status]["velocity_up"])
            right_speed = int(self.status_config[self.current_status]["velocity_low"])
            brush_speed = self.status_config[self.current_status]["velocity_brush"]
            # 执行 lambda 表达式获取实时值
            # brush_speed = self.status_config[self.current_status]["velocity_brush"](self)
            if (self.last_left_speed != left_speed or self.last_right_speed != right_speed or self.last_brush_speed != brush_speed):
                self.set_target_velocity(2, left_speed)
                self.set_target_velocity(1, right_speed)
                self.set_target_velocity(3, brush_speed)
                self.last_left_speed = left_speed
                self.last_right_speed = right_speed
                self.last_brush_speed = brush_speed

    def configure_motor(self, motor_id, velocity):
        """配置电机"""
        fault_code, _ = self.read_fault_code(motor_id)
        if fault_code and fault_code != 0x00:
            self.clear_fault(motor_id)
            time.sleep(0.3)
        
        rospy.loginfo(f"⚙️  配置电机 {motor_id}: {int(velocity/RATE)} RPM")
        self.start_motor(motor_id)
        self.set_target_velocity(motor_id, velocity)
        self.enable_drive(motor_id)
        # self.start_heartbeat(motor_id)

    def start_heartbeat(self, motor_id):
        """启动心跳"""
        self.heartbeat_running = True
        self.heartbeat_thread = threading.Thread(
            target=self._cycle_heartbeat,
            args=(motor_id,),
            daemon=True
        )
        self.heartbeat_thread.start()
        rospy.loginfo(f"❤️ 心跳启动：电机{motor_id}")

    def _cycle_heartbeat(self, motor_id, interval=1.5):
        """心跳循环"""
        while self.heartbeat_running and not rospy.is_shutdown():
            if self.rtu_client is None:
                time.sleep(interval)
                self.reconnect_rtu_client()
                continue
            self.read_fault_code(motor_id)
            time.sleep(interval)
            # speed = self.get_actual_velocity(motor_id)
            # if speed is None:
                # rospy.logwarn(f"❤️ 电机 {motor_id} 通讯异常")
                # self.motor_driver = False
                # self.reconnect_rtu_client()
            # else:
                # self.motor_driver = True

    def stop_heartbeat(self):
        """停止心跳"""
        self.heartbeat_running = False
        if self.heartbeat_thread and self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join(timeout=2)
        rospy.loginfo("❤️ 心跳停止")

    def load_config(self, config_file="/home/orangepi/demo01/src/motor_can/config/servo_config.yaml"):
    # def load_config(self, config_file="/home/ubuntu/demo01/src/motor_can/config/servo_config.yaml"):
        """加载配置"""
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

    def start_imu(self):
        """启动IMU"""
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service('/imu_parser_node/start_imu', timeout=5)
                start_srv = rospy.ServiceProxy('/imu_parser_node/start_imu', Trigger)
                resp = start_srv()
                rospy.loginfo(f"🧭 IMU启动成功: {resp.message}")
                break
            except Exception as e:
                rospy.loginfo(f"🧭 等待IMU服务: {e}")
                time.sleep(1)

    def stop_imu(self):
        """停止IMU"""
        try:
            rospy.wait_for_service('/imu_parser_node/stop_imu')
            stop_srv = rospy.ServiceProxy('/imu_parser_node/stop_imu', Trigger)
            resp = stop_srv()
            rospy.loginfo(f"🧭 IMU停止成功: {resp.message}")
        except Exception as e:
            rospy.loginfo(f"🧭 停止IMU失败: {e}")

    def shutdown(self):
        """安全关闭（增加空值判断）"""
        rospy.loginfo("🔌 关闭控制器...")
        # 停止心跳
        self.stop_heartbeat()
        # 禁用电机（增加空值判断）
        if self.rtu_client is not None:
            for motor_id in self.motor_address_map.keys():
                try:
                    self.disable_drive(motor_id)
                except:
                    pass
            # 关闭客户端
            try:
                self.rtu_client.close()
            except:
                pass
        # 停止IMU
        # self.stop_imu()
        # 停止定时器
        if hasattr(self, 'publish_timer'):
            self.publish_timer.shutdown()
        rospy.loginfo("✅ 控制器已关闭")

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
                    rospy.logwarn(f"  跳过无效配置: {motor}")
                    continue
                try:
                    controller.configure_motor(motor_id=motor_id, velocity=int(velocity * RATE))
                    controller.current_status = controller.status_list[0]
                except Exception as e:
                    rospy.logerr(f"❌ 配置电机 {motor_id} 出错: {e}")
        
        # 启动键盘监听（实例方法）
        if controller is not None:
            t = threading.Thread(target=controller.keyboard_listener, daemon=True)
            t.start()
        
        # 主循环
        # controller.set_state("STOP")
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
        # 确保资源释放
        if controller is not None:
            controller.shutdown()

if __name__ == "__main__":
    main()