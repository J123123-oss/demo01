#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
import struct
import time
import datetime
import numpy as np
from std_msgs.msg import Float32, Header, Bool, String
from serial_comms.msg import BatteryStatus
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
import threading

# 状态常量
STATE_READY = 0
STATE_WAITING_BATTERY = 1
STATE_WAITING_RELAY = 3

class BatteryRelayNode:
    def __init__(self):
        rospy.init_node('battery_imu_node')
        
        # 时间阈值管理
        self.last_battery_sent = 0
        self.last_relay_check = 0
        self.battery_buffer = bytearray()
        self.high_temp_triggered = False  # 高温触发标志
        self.low_temp_triggered = False   # 低温触发标志
        
        # 温度控制参数
        self.temperature_threshold_high = rospy.get_param('~temperature_threshold_high', 5.0)  # 高温阈值
        self.temperature_threshold_low = rospy.get_param('~temperature_threshold_low', 3.0)    # 低温阈值
        self.battery_check_interval = rospy.get_param('~battery_check_interval', 5.0)             # 电池检查间隔
        self.relay_check_interval = rospy.get_param('~relay_check_interval', 6.1)             # 继电器检查间隔
        
        # 获取串口参数
        port = rospy.get_param('~serial_port', '/dev/Battery-Relay')
        baudrate = rospy.get_param('~baudrate', 115200)
        self.relay_address = rospy.get_param('~relay_address', 0x02)
        
        # 添加串口访问锁，避免并发访问冲突
        self.serial_lock = threading.Lock()
        self.current_relay_state = False  # 默认初始化为关闭状态
        # 初始化串口
        self.ser = None
        self.init_serial(port, baudrate)
        # 如果失败，循环调用reinit_serial重试
        while not rospy.is_shutdown() and not self.ser:
            rospy.logwarn("relay_battery串口初始化失败，重试...")
            self.reinit_serial()  # 调用重连方法
            rospy.sleep(3)  # 间隔3秒重试，避免频繁尝试

        # 初始化继电器状态为实际状态（加入循环重试机制）
        init_max_retries = 5  # 最大重试次数
        retry_interval = 1  # 重试间隔时间（秒）
        retry_count = 0
        relay_status = None

        while retry_count < init_max_retries:
            relay_status = self.read_relay_status()
            if relay_status is not None:
                break  # 获取到状态则退出循环
            retry_count += 1
            rospy.loginfo(f"第 {retry_count} 次重试读取继电器状态...")
            rospy.sleep(retry_interval)  # 等待重试间隔

        if relay_status is not None:
            self.current_relay_state = relay_status
            rospy.loginfo(f"继电器初始状态: {'开启' if relay_status else '关闭'}")
        else:
            self.current_relay_state = False
            rospy.logwarn(f"达到最大重试次数（{init_max_retries}次），仍无法读取继电器状态")

        # 初始化状态机
        self.current_state = STATE_READY
        self.loop_counter = 0
        self.battery_timeout = 0
        self.relay_timeout = 0
        self.battery_retry_count = 0
        self.max_battery_retry = 3  # 增加重试次数
        
        # 电池03指令请求帧
        self.REQUEST_BASIC_FRAME = bytes.fromhex('DD A5 03 00 FF FD 77')
        
        # 创建发布器
        self.battery_pub = rospy.Publisher('/battery_status', BatteryStatus, queue_size=10)
        self.relay_status_pub = rospy.Publisher('/relay_status', Bool, queue_size=10)
        self.temperature_pub = rospy.Publisher('/control_temperature', Float32, queue_size=10)
        
        # ROS服务
        rospy.Service('~enable_relay', SetBool, self.enable_relay_callback)
        rospy.Service('~get_relay_status', Trigger, self.get_relay_status_callback)
        
        # 设置循环速率
        self.rate = rospy.Rate(100)  # 100Hz
        
        # 电池温度数据
        self.current_temperatures = []
        
        # 添加统计信息
        self.battery_query_count = 0
        self.battery_success_count = 0
        
        rospy.loginfo("电池-继电器集成节点初始化完成")

    def init_serial(self, port, baudrate):
        """串口初始化"""
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05
            )
            rospy.loginfo(f"串口连接成功: {port}")
            return True
        except Exception as e:
            rospy.logerr(f"串口连接失败: {e}")
            return False
    def calculate_crc(self, data):
        """Modbus CRC16校验计算"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return struct.pack('<H', crc)  
    def calculate_checksum(self, check_bytes):
        """
        计算校验码（不包含命令码03）：校验字节总和 → 取反+1（16位）
        :param check_bytes: 校验范围字节列表（如 00 26 12 06 ... 00 F6）
        :return: 16位校验码值
        """
        total = sum(check_bytes)
        checksum = (-total) & 0xFFFF  # 取反+1，限制为16位
        return checksum
    
    def parse_date(self, raw_date):
        """解析电池生产日期"""
        value = (raw_date[0] << 8) | raw_date[1]
        day = value & 0x1F
        month = (value >> 5) & 0x0F
        year = 2000 + (value >> 9)
        return year, month, day

    def parse_current(self, data_bytes):
        """解析电池电流值"""
        value = (data_bytes[0] << 8) | data_bytes[1]
        rospy.logdebug(f"电流原始值: {value}")
        return (value - 65536) * 0.01 if value >= 0x8000 else value * 0.01

    def process_battery_response(self, data_block, check_bytes):
        """
        解析电池响应：分离数据段与校验码，验证后解析
        :param data_block: 数据段（38字节）+ 校验码（2字节）= 40字节
        :param check_bytes: 校验范围字节（不包含命令码03）
        :return: 解析成功返回True，失败返回False
        """
        status_msg = BatteryStatus()
        try:
            # 1. 校验data_block长度（38数据段+2校验码=40字节）
            if len(data_block) != 40:
                rospy.logerr(f"数据块长度错误，预期40字节，实际{len(data_block)}字节")
                return False
            
            # 2. 分离数据段和接收的校验码
            data_segment = data_block[0:38]  # 纯数据段（38字节）
            received_checksum = (data_block[38] << 8) | data_block[39]  # 最后2字节为校验码
            
            # 3. 计算本地校验码并验证（不包含命令码03）
            calc_checksum = self.calculate_checksum(check_bytes)
            if received_checksum != calc_checksum:
                rospy.logerr(f"校验码不匹配：接收0x{received_checksum:04X}，计算0x{calc_checksum:04X}")
                return False
            
            # 4. 基础数据段长度校验（至少23字节）
            if len(data_segment) < 38:
                rospy.logerr(f"数据段长度不足，要求38字节，实际{len(data_segment)}字节")
                return False
            
            # 5. 解析基础电池信息
            status_msg.total_voltage = ((data_segment[0] << 8) | data_segment[1]) * 0.01  # 总电压（10mV/单位）
            status_msg.current = self.parse_current(data_segment[2:4])  # 电流（10mA/单位）
            status_msg.remaining_capacity = ((data_segment[4] << 8) | data_segment[5]) * 0.01  # 剩余容量（10mAh/单位）
            status_msg.nominal_capacity = ((data_segment[6] << 8) | data_segment[7]) * 0.01  # 标称容量（10mAh/单位）
            status_msg.cycle_count = (data_segment[8] << 8) | data_segment[9]  # 循环次数
            
            # 6. 解析生产日期
            year, month, day = self.parse_date(data_segment[10:12])
            status_msg.production_year = year
            status_msg.production_month = month
            status_msg.production_day = day
            
            # 7. 解析均衡状态、保护状态
            status_msg.balance_low = (data_segment[12] << 8) | data_segment[13]  # 低16串均衡
            status_msg.balance_high = (data_segment[14] << 8) | data_segment[15]  # 高16串均衡
            status_msg.protection_status = (data_segment[16] << 8) | data_segment[17]  # 保护状态
            
            # 8. 解析软件版本
            ver_major = data_segment[18] >> 4
            ver_minor = data_segment[18] & 0x0F
            status_msg.software_version = f"{ver_major}.{ver_minor}"
            
            # 9. 解析剩余电量、MOS状态、电池串数
            status_msg.batttery_remaining = float(data_segment[19])  # 剩余电量百分比（0-100%）
            status_msg.mos_state = data_segment[20]  # MOS控制状态
            status_msg.battery_series = data_segment[21]  # 电池串数
            
            # 10. 解析温度数据
            status_msg.ntc_count = data_segment[22]  # 温度探头数量
            required_temp_len = 23 + 2 * status_msg.ntc_count  # 温度数据所需长度
            if len(data_segment) < required_temp_len:
                rospy.logerr(f"温度数据不足，要求{required_temp_len}字节，实际{len(data_segment)}字节")
                return False
            
            self.current_temperatures.clear()
            for i in range(status_msg.ntc_count):
                idx = 23 + i * 2
                raw_temp = (data_segment[idx] << 8) | data_segment[idx + 1]  # 温度原始值（0.1K）
                temp_c = (raw_temp - 2731) / 10.0  # 转换为摄氏度
                status_msg.temperatures.append(round(temp_c, 1))
                self.current_temperatures.append(round(temp_c, 1))
            
            # 11. 发布数据
            if self.current_temperatures:
                avg_temp = sum(self.current_temperatures) / len(self.current_temperatures)
                self.temperature_pub.publish(Float32(data=avg_temp))
            self.battery_pub.publish(status_msg)
            rospy.logdebug("电池数据解析成功并发布")
            return True
            
        except IndexError as e:
            rospy.logerr(f"数据解析索引越界: {e}")
            return False
        except Exception as e:
            rospy.logerr(f"电池数据解析错误: {e}")
            return False
        
    def send_relay_command(self, command_data):
        """发送继电器命令"""
        try:
            # 使用锁保护串口访问
            with self.serial_lock:
                crc = self.calculate_crc(command_data)
                full_command = command_data + crc
                self.ser.write(full_command)
                rospy.loginfo("发送继电器命令: %s", ' '.join(['%02X' % b for b in full_command]))
                
                # 等待响应
                time.sleep(0.05)
                response = self.ser.read(8)
                
            if len(response) > 0:
                rospy.loginfo("接收继电器响应: %s", ' '.join(['%02X' % b for b in response]))
                if len(response) >= 7 and response[:6] == command_data:
                    return True
            return False
        except Exception as e:
            rospy.logerr(f"继电器通信错误: {e}")
            return False

    def enable_relay(self, enable=True, current_temp=None):
        """开启或关闭继电器（仅限白天6:00-17:00可开启）"""
        # 时间限制：只有6:00-17:00允许开启
        if enable:
            now = datetime.datetime.now()
            # 检查是否在允许的常规时段（6:00-17:00）
            in_regular_hours = 6 <= now.hour < 17
            # 极端低温判断（当前温度＜-15℃时忽略时间限制）
            is_extreme_low = current_temp is not None and current_temp < -15

            if not in_regular_hours and not is_extreme_low:
                rospy.logwarn("当前时间不在允许开启继电器的时段（6:00-17:00）且非极端低温（＜-15℃），请求被拒绝")
                return False

        max_retries = 10
        for attempt in range(max_retries):
            if enable:
                command_data = bytes([self.relay_address, 0x05, 0x00, 0x00, 0xFF, 0x00])
                rospy.loginfo("发送继电器开启命令")
            else:
                command_data = bytes([self.relay_address, 0x05, 0x00, 0x00, 0x00, 0x00])
                rospy.loginfo("发送继电器关闭命令")
            
            success = self.send_relay_command(command_data)
            if success:
                self.current_relay_state = enable
                # 发布继电器状态
                status_msg = Bool()
                status_msg.data = bool(self.current_relay_state)
                self.relay_status_pub.publish(status_msg)
                return True
            else:
                rospy.logwarn(f"继电器命令发送失败，重试 {attempt + 1}/{max_retries}")
                time.sleep(0.1)  # 等待后重试
        
        rospy.logerr("继电器命令发送失败，已达到最大重试次数")
        return False

    def read_relay_status(self):
        """读取继电器状态，失败时返回上一次保存的值"""
        command_data = bytes([self.relay_address, 0x01, 0x00, 0x00, 0x00, 0x08])
        # 保存当前状态作为备选
        last_known_state = self.current_relay_state
        
        try:
            # 使用锁保护串口访问
            with self.serial_lock:
                crc = self.calculate_crc(command_data)
                full_command = command_data + crc
                self.ser.write(full_command)
                
                time.sleep(0.01)
                response = self.ser.read(8)
                rospy.loginfo("接收继电器状态响应: %s", ' '.join(['%02X' % b for b in response]))
                
            if response and len(response) >= 6:
                if response[0] == self.relay_address and response[1] == 0x01:
                    byte_count = response[2]
                    if byte_count >= 1:
                        status_byte = response[3]
                        relay_status = (status_byte & 0x01) != 0
                        self.current_relay_state = relay_status
                        return relay_status
            
            # 如果响应解析失败，返回上一次已知状态
            rospy.logwarn("继电器状态解析失败，使用上一次已知状态")
            return last_known_state
        
        except Exception as e:
            rospy.logerr(f"读取继电器状态错误: {e}，使用上一次已知状态")
            # 发生异常时返回上一次已知状态
            return last_known_state

    def temperature_based_control(self):
        """基于温度控制继电器，每种情况只触发一次，温度恢复后可再次触发"""
        if not self.current_temperatures:
            rospy.logwarn("无温度数据，跳过继电器控制")
            return

        min_temp = min(self.current_temperatures)
        max_temp = max(self.current_temperatures)

        rospy.loginfo(f"温度监测 - 最低: {min_temp}°C, 最高: {max_temp}°C, 当前继电器状态: {'开启' if self.current_relay_state else '关闭'}")

        # 高温触发：最低温度超阈值且继电器开启 → 关闭
        if min_temp >= self.temperature_threshold_high and self.current_relay_state:
            rospy.loginfo(f"最低温度 {min_temp}°C 超过阈值 {self.temperature_threshold_high}°C，关闭继电器")
            if self.enable_relay(False):
                rospy.loginfo("继电器已关闭")
            else:
                rospy.logwarn("继电器关闭失败")

        # 低温触发：最低温度低于阈值且继电器关闭 → 开启
        elif min_temp <= self.temperature_threshold_low and not self.current_relay_state:
            rospy.loginfo(f"最低温度 {min_temp}°C 低于阈值 {self.temperature_threshold_low}°C，开启继电器")
            if self.enable_relay(True, min_temp):
                rospy.loginfo("继电器已开启")
            else:
                rospy.logwarn("继电器开启失败")

        # 发布继电器状态
        status_msg = Bool()
        status_msg.data = bool(self.current_relay_state)
        self.relay_status_pub.publish(status_msg)
        
    def run(self):
        """改进的主状态机循环 - 非阻塞版本"""
        rospy.loginfo("节点主循环开始运行")
        while not rospy.is_shutdown():
            current_time = time.time()
            
            # === 优先处理串口数据读取 ===
            self.read_serial_data()
            
            # === 并行处理所有等待状态 ===
            # 1. 处理电池响应（不阻塞其他状态）
            if self.current_state == STATE_WAITING_BATTERY:
                if self.process_battery_buffer():
                    self.current_state = STATE_READY
                    self.battery_success_count += 1
                elif current_time >= self.battery_timeout:
                    rospy.logwarn("电池响应超时，返回就绪状态")
                    self.battery_buffer.clear()
                    self.current_state = STATE_READY
            
            # 2. 处理继电器响应
            if self.current_state == STATE_WAITING_RELAY:
                if current_time >= self.relay_timeout:
                    self.current_state = STATE_READY
                    rospy.logwarn("继电器响应超时")
            
            # === 发送新请求（优化优先级）===
            if self.current_state == STATE_READY:
                # 处理电池查询（低优先级）
                if current_time - self.last_battery_sent >= self.battery_check_interval:
                    if self.send_battery_query():
                        self.last_battery_sent = current_time
                        self.battery_query_count += 1
                        self.current_state = STATE_WAITING_BATTERY
                        self.battery_timeout = current_time + 1.0  # 1秒超时
                    else:
                        rospy.logerr("电池查询发送失败")
                
                # 处理继电器温度控制（最低优先级）
                elif current_time - self.last_relay_check >= self.relay_check_interval:
                    latest_relay_status = self.read_relay_status()
                    # if latest_relay_status is None:
                        # rospy.logwarn("读取继电器状态失败，当前旧状态")
                    self.temperature_based_control()
                    self.last_relay_check = current_time
                    
                    # 定期输出统计信息
                    if self.battery_query_count > 0:
                        battery_success_rate = (self.battery_success_count / self.battery_query_count) * 100
                        rospy.loginfo(f"电池查询成功率: {battery_success_rate:.2f}% ({self.battery_success_count}/{self.battery_query_count})")
            
            self.rate.sleep()

    def send_battery_query(self):
        """发送电池查询指令"""
        try:
            if self.ser and self.ser.is_open:
                # 使用锁保护串口访问
                with self.serial_lock:
                    self.battery_buffer.clear()
                    self.ser.write(b'\x00')  # 清除缓存
                    time.sleep(0.01)
                    self.ser.write(self.REQUEST_BASIC_FRAME)
                rospy.logdebug("电池查询指令发送成功")
                return True
        except serial.SerialException as e:
            rospy.logerr(f"电池查询发送失败（串口异常）: {e}")
            # 尝试重新初始化串口
            if self.reinit_serial():
                return False  # 重新初始化后下次再试
        except Exception as e:
            rospy.logerr(f"电池查询发送失败: {e}")
        return False

    def reinit_serial(self):
        """重新初始化串口"""
        try:
            if self.ser:
                self.ser.close()
        except:
            pass
        
        try:
            port = rospy.get_param('~serial_port', '/dev/Battery-Relay')
            baudrate = rospy.get_param('~baudrate', 115200)
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05
            )
            rospy.loginfo("串口重新初始化成功")
            return True
        except Exception as e:
            rospy.logerr(f"串口重新初始化失败: {e}")
            self.ser = None
            return False

    def read_serial_data(self):
        """改进的串口读取，确保及时处理所有数据"""
        try:
            if self.ser and self.ser.is_open:
                # 使用锁保护串口访问
                with self.serial_lock:
                    # 多次读取直到清空缓冲区
                    for _ in range(3):  # 最多读取3次
                        avail = self.ser.in_waiting
                        if avail == 0:
                            break
                            
                        data = self.ser.read(avail)
                        if data:
                            # 打印原始数据
                            hex_str = ' '.join(['%02X' % b for b in data])
                            rospy.loginfo(f"收到485原始数据: {hex_str}")

                        for byte in data:
                            byte_val = byte if isinstance(byte, int) else ord(byte)

                            # 只保留电池分流逻辑
                            if byte_val == 0xDD:
                                # 1. 若缓冲区有旧数据，先尝试解析
                                if len(self.battery_buffer) > 0:
                                    rospy.loginfo(f"电池缓冲区当前长度: {len(self.battery_buffer)}，尝试解析旧数据")
                                    self.process_battery_buffer()  # 尝试处理旧数据
                                    rospy.logwarn("电池缓冲区已有数据，已尝试解析，现在处理新帧")
                                
                                # 2. 清空缓冲区并添加新帧头
                                self.battery_buffer.clear()
                                self.battery_buffer.append(byte_val)
                            elif len(self.battery_buffer) > 0 and self.battery_buffer[0] == 0xDD and len(self.battery_buffer) < 50:
                                self.battery_buffer.append(byte_val)
                            else:
                                rospy.loginfo(f"收到其他类型数据: {byte_val:02X}")
                        
                        # 短暂休息避免过度占用CPU
                        time.sleep(0.001)
                    
        except serial.SerialException as e:
            rospy.logerr(f"串口读取异常: {e}")
            self.reinit_serial()
        except Exception as e:
            rospy.logwarn(f"串口读取错误: {e}")

    def process_battery_buffer(self):
        """处理电池响应数据：提取完整帧，准备校验范围"""
        MAX_BATTERY_FRAME_LEN = 64
        if len(self.battery_buffer) > MAX_BATTERY_FRAME_LEN:
            rospy.logwarn(f"电池缓冲区过长，清空数据: {len(self.battery_buffer)}字节")
            self.battery_buffer.clear()
            return False
        
        start_idx = 0
        while start_idx < len(self.battery_buffer):
            # 定位帧头（0xDD）
            if self.battery_buffer[start_idx] != 0xDD:
                start_idx += 1
                continue
                
            # 确保帧头后有足够字节（至少4字节：DD 03 00 26）
            if start_idx + 3 >= len(self.battery_buffer):
                rospy.logdebug("帧头后字节不足，无法解析")
                return False
                
            # 校验功能码（03）和状态码（00）
            if self.battery_buffer[start_idx+1] != 0x03:
                rospy.logdebug(f"功能码错误，预期0x03，实际0x{self.battery_buffer[start_idx+1]:02X}")
                start_idx += 1
                continue
            if self.battery_buffer[start_idx+2] != 0x00:
                rospy.logdebug(f"状态码错误，预期0x00，实际0x{self.battery_buffer[start_idx+2]:02X}")
                start_idx += 1
                continue
                
            # 提取数据长度（第4字节），计算完整帧长度
            data_length = self.battery_buffer[start_idx+3]  # 数据段长度（38字节）
            full_frame_len = 4 + data_length + 2 + 1  # 4(帧头) + 38(数据段) + 2(校验码) + 1(帧尾) = 45字节
            total_needed = start_idx + full_frame_len
            
            # 检查帧是否完整
            if len(self.battery_buffer) < total_needed:
                rospy.logdebug(f"帧未完整接收，当前{len(self.battery_buffer)}字节，需{total_needed}字节")
                return False
            
            # 校验帧尾（0x77）
            if self.battery_buffer[total_needed - 1] != 0x77:
                rospy.logwarn(f"帧尾错误，预期0x77，实际0x{self.battery_buffer[total_needed - 1]:02X}，丢弃")
                start_idx += 1
                continue
            
            # 1. 提取校验范围字节（不包含命令码03）：状态码(00) + 数据长度(26) + 数据段(38) + 补充字段(如00 F6)
            check_start = start_idx + 2  # 从状态码（00）开始
            check_end = start_idx + 4 + data_length  # 到数据段结束（不含校验码）
            check_bytes = self.battery_buffer[check_start:check_end]
            
            # 2. 提取数据块（数据段38字节 + 校验码2字节）
            data_start = start_idx + 4  # 跳过帧头4字节（DD 03 00 26）
            data_end = data_start + data_length + 2  # 38+2=40字节
            data_block = self.battery_buffer[data_start:data_end]
            
            # 3. 解析并验证
            success = self.process_battery_response(data_block, check_bytes)
            # 4. 清空已处理的帧，保留后续数据
            self.battery_buffer = self.battery_buffer[total_needed:]
            return success
        
        return False

    def enable_relay_callback(self, req):
        """ROS服务回调: 开启/关闭继电器"""
        success = self.enable_relay(req.data)
        response = SetBoolResponse()
        response.success = success
        if success:
            response.message = "Relay {} successfully".format("enabled" if req.data else "disabled")
        else:
            response.message = "Failed to {} relay".format("enable" if req.data else "disable")
        return response

    def get_relay_status_callback(self, req):
        """ROS服务回调: 获取继电器状态"""
        status = self.read_relay_status()
        response = TriggerResponse()
        if status is not None:
            response.success = True
            response.message = "Relay is {}".format("ON" if status else "OFF")
        else:
            response.success = False
            response.message = "Failed to read relay status"
        return response

if __name__ == '__main__':
    try:
        node = BatteryRelayNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"节点运行异常: {e}")
    finally:
        # 程序退出前，主动关闭继电器
        if 'node' in locals() and node is not None:
            rospy.loginfo("程序退出，主动关闭继电器")
            node.enable_relay(False)