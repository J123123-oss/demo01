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
        # 新增：周期控制参数（核心修改）
        self.cycle_interval = rospy.get_param('~cycle_interval', 10.0)  # 每轮流程间隔时间（秒）
        self.last_cycle_finish = 0  # 上一轮流程完成时间戳
        # 时间管理（仅保留必要时间戳）
        self.last_battery_sent = 0
        self.battery_buffer = bytearray()
        self.high_temp_triggered = False  # 高温触发标志
        self.low_temp_triggered = False   # 低温触发标志
        
        # 温度控制参数
        self.temperature_threshold_high = rospy.get_param('~temperature_threshold_high', 1.0)  # 高温阈值
        self.temperature_threshold_low = rospy.get_param('~temperature_threshold_low', -10.0)    # 低温阈值
        
        # 串行执行状态标记（核心修改）
        self.is_battery_running = False  # True=正在处理电池任务
        self.battery_process_done = False  # 电池处理完成标记（用于触发继电器）
        
        # 获取串口参数
        port = rospy.get_param('~serial_port', '/dev/IMU')
        baudrate = rospy.get_param('~baudrate', 115200)
        self.relay_address = rospy.get_param('~relay_address', 0x02)
        
        # 串口访问锁
        self.serial_lock = threading.Lock()
        
        # 初始化串口
        self.ser = None
        self.init_serial(port, baudrate)
        if not self.ser:
            rospy.signal_shutdown("串口初始化失败")
            return

        # 初始化继电器状态（带重试）
        init_max_retries = 3
        retry_interval = 1
        retry_count = 0
        relay_status = None

        while retry_count < init_max_retries:
            relay_status = self.read_relay_status()
            if relay_status is not None:
                break
            retry_count += 1
            rospy.loginfo(f"第 {retry_count} 次重试读取继电器状态...")
            rospy.sleep(retry_interval)

        if relay_status is not None:
            self.current_relay_state = relay_status
            rospy.loginfo(f"继电器初始状态: {'开启' if relay_status else '关闭'}")
        else:
            self.current_relay_state = None
            rospy.logwarn(f"达到最大重试次数（{init_max_retries}次），仍无法读取继电器状态")

        # 初始化状态机
        self.current_state = STATE_READY
        self.battery_timeout = 0
        self.battery_retry_count = 0
        self.max_battery_retry = 3
        self.batttery_remaining = None
        
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
        
        # 统计信息
        self.battery_query_count = 0
        self.battery_success_count = 0
        
        rospy.loginfo("电池-继电器集成节点初始化完成（串行模式：电池→继电器）")

    def init_serial(self, port, baudrate):
        """串口初始化（延长超时时间）"""
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05  # 延长超时时间至50ms，确保完整接收
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
        return (value - 65536) * 0.01 if value >= 0x8000 else value * 0.01

    def process_battery_response(self, data):
        """解析电池响应数据帧并存储温度数据"""
        status_msg = BatteryStatus()
        try:
            min_data_len = 23
            if len(data) < min_data_len:
                rospy.logerr(f"电池数据长度不足，协议要求至少{min_data_len}字节，实际接收{len(data)}字节")
                return False
            
            # 基础电池信息解析
            status_msg.total_voltage = ((data[0] << 8) | data[1]) * 0.01
            status_msg.current = self.parse_current(data[2:4])
            status_msg.remaining_capacity = ((data[4] << 8) | data[5]) * 0.01
            status_msg.nominal_capacity = ((data[6] << 8) | data[7]) * 0.01
            status_msg.cycle_count = (data[8] << 8) | data[9]
            
            # 生产日期解析
            year, month, day = self.parse_date(data[10:12])
            status_msg.production_year = year
            status_msg.production_month = month
            status_msg.production_day = day
            
            status_msg.balance_low = (data[12] << 8) | data[13]
            status_msg.balance_high = (data[14] << 8) | data[15]
            status_msg.protection_status = (data[16] << 8) | data[17]
            
            # 软件版本解析
            ver_major = data[18] >> 4
            ver_minor = data[18] & 0x0F
            status_msg.software_version = f"{ver_major}.{ver_minor}"
            
            status_msg.batttery_remaining = data[19]
            status_msg.mos_state = data[20]
            status_msg.battery_series = data[21]
            
            # 温度数据解析
            ntc_count = data[22]
            required_data_len = 23 + 2 * ntc_count - 1
            if len(data) < required_data_len:
                rospy.logerr(f"NTC温度数据长度不足，协议要求{required_data_len}字节（NTC个数{ntc_count}），实际接收{len(data)}字节")
                self.current_temperatures = []
                return False
            
            self.current_temperatures = []
            for i in range(ntc_count):
                idx = 23 + i * 2
                if idx + 1 >= len(data):
                    rospy.logerr(f"NTC温度解析索引越界，NTC序号{i}，索引{idx}超出数据长度{len(data)}")
                    break
                raw_temp = (data[idx] << 8) | data[idx + 1]
                temp = (raw_temp - 2731) / 10.0
                rounded_temp = round(temp, 1)
                status_msg.temperatures.append(rounded_temp)
                self.current_temperatures.append(rounded_temp)
            
            # 发布平均温度
            if self.current_temperatures:
                avg_temp = sum(self.current_temperatures) / len(self.current_temperatures)
                temp_msg = Float32()
                temp_msg.data = avg_temp
                self.temperature_pub.publish(temp_msg)
            
            # 发布电池状态
            print("batttery_remaining:",status_msg.batttery_remaining)
            self.battery_pub.publish(status_msg)
            return True
            
        except Exception as e:
            rospy.logerr(f"电池数据解析错误：{e}")
            return False
        
    def send_relay_command(self, command_data):
        """发送继电器命令（增加帧头校验，过滤电池数据）"""
        try:
            with self.serial_lock:
                # 发送前强制清空串口缓冲区，避免残留电池数据
                self.ser.flushInput()
                self.ser.flushOutput()
                
                crc = self.calculate_crc(command_data)
                full_command = command_data + crc
                self.ser.write(full_command)
                rospy.loginfo("发送继电器命令: %s", ' '.join(['%02X' % b for b in full_command]))
                
                # 等待响应（延长超时至100ms，确保继电器有足够时间回复）
                time.sleep(0.1)
                response = self.ser.read(8)  # Modbus响应通常为8字节
                
            if len(response) > 0:
                rospy.loginfo("接收继电器响应: %s", ' '.join(['%02X' % b for b in response]))
                
                # 关键校验：继电器响应必须以自身地址（0x02）和对应功能码开头
                # command_data[0]是继电器地址，command_data[1]是功能码（0x05或0x01）
                if len(response) >= 2 and response[0] == command_data[0] and response[1] == command_data[1]:
                    return True
                else:
                    rospy.logwarn("收到非继电器响应（可能是电池数据），忽略")
                    return False
            return False
        except Exception as e:
            rospy.logerr(f"继电器通信错误: {e}")
            return False

    def enable_relay(self, enable=True):
        """开启或关闭继电器（仅限白天8:00-17:00可开启）"""
        if enable:
            now = datetime.datetime.now()
            if not (8 <= now.hour < 17):
                rospy.logwarn("当前时间不在允许开启继电器的时段（8:00-17:00），请求被拒绝")
                return False

        max_retries = 3
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
                status_msg = Bool()
                status_msg.data = bool(self.current_relay_state) if self.current_relay_state is not None else None
                self.relay_status_pub.publish(status_msg)
                return True
            else:
                rospy.logwarn(f"继电器命令发送失败，重试 {attempt + 1}/{max_retries}")
                time.sleep(0.1)
        
        rospy.logerr("继电器命令发送失败，已达到最大重试次数")
        return False

    def read_relay_status(self):
        """读取继电器状态"""
        command_data = bytes([self.relay_address, 0x01, 0x00, 0x00, 0x00, 0x08])
        rospy.loginfo("发送继电器状态查询")
        
        try:
            with self.serial_lock:
                crc = self.calculate_crc(command_data)
                full_command = command_data + crc
                self.ser.write(full_command)
                
                time.sleep(0.01)
                response = self.ser.read(8)
                print("接收继电器状态响应: %s", ' '.join(['%02X' % b for b in response]))
                
            if response and len(response) >= 6:
                if response[0] == self.relay_address and response[1] == 0x01:
                    byte_count = response[2]
                    if byte_count >= 1:
                        status_byte = response[3]
                        relay_status = (status_byte & 0x08) != 0
                        self.current_relay_state = relay_status
                        return relay_status
            return None
        except Exception as e:
            rospy.logerr(f"读取继电器状态错误: {e}")
            return None

    def temperature_based_control(self):
        """基于温度控制继电器（串行执行，依赖电池数据）"""
        if not self.current_temperatures:
            rospy.logwarn("无温度数据，跳过继电器控制")
            return

        max_temp = max(self.current_temperatures)
        avg_temp = sum(self.current_temperatures) / len(self.current_temperatures)

        rospy.loginfo(f"温度监测 - 最高: {max_temp}°C, 平均: {avg_temp}°C, 当前继电器状态: {'暂无' if self.current_relay_state is None else '开启' if self.current_relay_state else '关闭'}")

        # 高温触发
        if max_temp >= self.temperature_threshold_high and not self.high_temp_triggered:
            rospy.loginfo(f"温度 {max_temp}°C 超过阈值 {self.temperature_threshold_high}°C，关闭继电器")
            if self.enable_relay(False):
                rospy.loginfo("继电器已关闭")
                self.high_temp_triggered = True
                self.low_temp_triggered = False
            else:
                rospy.logwarn("继电器关闭失败")

        # 低温触发
        elif avg_temp <= self.temperature_threshold_low and not self.low_temp_triggered:
            rospy.loginfo(f"温度 {avg_temp}°C 低于阈值 {self.temperature_threshold_low}°C，开启继电器")
            if self.enable_relay(True):
                rospy.loginfo("继电器已开启")
                self.low_temp_triggered = True
                self.high_temp_triggered = False
            else:
                rospy.logwarn("继电器开启失败")

        # 温度恢复正常区间，重置触发标志
        elif self.temperature_threshold_low < avg_temp < self.temperature_threshold_high:
            self.high_temp_triggered = False
            self.low_temp_triggered = False

        # 发布继电器状态
        status_msg = Bool()
        status_msg.data = bool(self.current_relay_state) if self.current_relay_state is not None else False
        self.relay_status_pub.publish(status_msg)
        
    def run(self):
        """串行定时模式主循环：每间隔cycle_interval秒执行一次“电池→继电器”流程"""
        rospy.loginfo("节点主循环开始运行（串行定时模式）")
        while not rospy.is_shutdown():
            current_time = time.time()
            
            # 1. 优先处理串口数据（最高优先级）
            self.read_serial_data()
            
            # 2. 处理电池响应（与之前一致）
            if self.current_state == STATE_WAITING_BATTERY:
                if self.process_battery_buffer():
                    self.current_state = STATE_READY
                    self.battery_success_count += 1
                    self.is_battery_running = False  # 电池处理完成
                    self.battery_process_done = True  # 标记为完成，触发继电器
                elif current_time >= self.battery_timeout:
                    self.battery_buffer.clear()
                    self.current_state = STATE_READY
                    self.is_battery_running = False  # 超时重置状态
            
            # 3. 串行核心：电池完成后自动触发继电器控制
            if self.battery_process_done:
                self.temperature_based_control()  # 执行继电器控制
                rospy.loginfo(f"本轮串行流程完成（耗时{current_time - self.last_battery_sent:.2f}秒）")
                self.battery_process_done = False  # 重置标记
                self.last_cycle_finish = current_time  # 记录本轮完成时间（核心修改）
            
            # 4. 定时触发下一轮：仅当时间间隔满足且无任务运行时
            if self.current_state == STATE_READY and not self.is_battery_running:
                # 检查是否达到间隔时间（核心修改）
                if current_time - self.last_cycle_finish >= self.cycle_interval:
                    if self.send_battery_query():
                        self.last_battery_sent = current_time
                        self.battery_query_count += 1
                        self.current_state = STATE_WAITING_BATTERY
                        self.battery_timeout = current_time + 1.0  # 电池响应超时时间
                        self.is_battery_running = True  # 标记为正在处理电池
                        rospy.loginfo(f"开始新轮串行流程（距离上轮{current_time - self.last_cycle_finish:.2f}秒）")
                    else:
                        rospy.logerr("电池查询发送失败，1秒后重试")
                        time.sleep(1.0)
                else:
                    # 未达到间隔时间，计算剩余等待时间（可选）
                    remaining = self.cycle_interval - (current_time - self.last_cycle_finish)
                    rospy.logdebug(f"等待下一轮流程，剩余{remaining:.2f}秒")
            
            self.rate.sleep()

    def send_battery_query(self):
        """发送电池查询指令（强化缓冲区清空）"""
        try:
            if self.ser and self.ser.is_open:
                with self.serial_lock:
                    # 发送前彻底清空缓冲区，包括输入输出
                    self.ser.flushInput()
                    self.ser.flushOutput()
                    self.battery_buffer.clear()  # 清空电池缓冲区
                    
                    self.ser.write(b'\x00')  # 若此字节无意义可删除，避免干扰
                    time.sleep(0.01)
                    self.ser.write(self.REQUEST_BASIC_FRAME)
                return True
        except serial.SerialException as e:
            rospy.logerr(f"电池查询发送失败（串口异常）: {e}")
            if self.reinit_serial():
                return False
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
            port = rospy.get_param('~serial_port', '/dev/IMU')
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
        """优化的串口读取（严格区分电池数据）"""
        try:
            if self.ser and self.ser.is_open:
                with self.serial_lock:
                    for _ in range(3):
                        avail = self.ser.in_waiting
                        if avail == 0:
                            break
                            
                        data = self.ser.read(avail)
                        if data:
                            hex_str = ' '.join(['%02X' % (b if isinstance(b, int) else ord(b)) for b in data])
                            # rospy.loginfo(f"收到原始数据: {hex_str}")

                        for byte in data:
                            byte_val = byte if isinstance(byte, int) else ord(byte)

                            # 仅处理电池帧（0xDD开头，且长度未超限）
                            if byte_val == 0xDD:
                                if len(self.battery_buffer) > 0:
                                    rospy.logwarn("电池缓冲区已有数据，可能有帧丢失")
                                    rospy.loginfo(f"残留数据: {''.join(['%02X' % b for b in self.battery_buffer])}")
                                self.battery_buffer.clear()
                                self.battery_buffer.append(byte_val)
                            elif self.battery_buffer and self.battery_buffer[0] == 0xDD:
                                # 电池帧最大长度限制（根据实际协议调整，如64字节）
                                if len(self.battery_buffer) < 64:
                                    self.battery_buffer.append(byte_val)
                                else:
                                    rospy.logwarn("电池帧长度超限，丢弃")
                                    self.battery_buffer.clear()
                            # 非电池数据（如继电器响应）不进入电池缓冲区，由继电器逻辑处理
                        
        except serial.SerialException as e:
            rospy.logerr(f"串口读取异常: {e}")
            self.reinit_serial()
        except Exception as e:
            rospy.logwarn(f"串口读取错误: {e}")
            
    def process_battery_buffer(self):
        """处理电池响应数据"""
        MAX_BATTERY_FRAME_LEN = 64
        if len(self.battery_buffer) > MAX_BATTERY_FRAME_LEN:
            rospy.logwarn(f"电池缓冲区过长，清空数据: {len(self.battery_buffer)}字节")
            self.battery_buffer.clear()
            return False
        
        start_idx = 0
        while start_idx < len(self.battery_buffer):
            if self.battery_buffer[start_idx] != 0xDD:
                start_idx += 1
                continue
                
            if start_idx + 3 >= len(self.battery_buffer):
                return False
                
            if self.battery_buffer[start_idx+1] != 0x03:
                start_idx += 1
                continue
                
            if self.battery_buffer[start_idx+2] != 0x00:
                start_idx += 1
                continue
                
            data_length = self.battery_buffer[start_idx+3]
            full_length = data_length + 7
            
            if len(self.battery_buffer) < start_idx + full_length:
                return False
                
            if self.battery_buffer[start_idx + full_length - 1] != 0x77:
                start_idx += 1
                continue
                
            data_start = start_idx + 4
            data_end = data_start + data_length
            data_block = self.battery_buffer[data_start:data_end]
            
            success = self.process_battery_response(data_block)
            self.battery_buffer.clear()
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
    node = None
    try:
        node = BatteryRelayNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"节点运行异常: {e}")
    finally:
        # 程序退出前关闭继电器
        if node is not None:
            rospy.loginfo("程序退出，主动关闭继电器")
            node.enable_relay(False)