#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
import struct
import time
import numpy as np
from std_msgs.msg import Float32, Header, Bool, String
from serial_comms.msg import BatteryStatus, INSPVAE
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
import threading

# 状态常量
STATE_READY = 0
STATE_WAITING_BATTERY = 1
STATE_WAITING_IMU = 2
STATE_WAITING_RELAY = 3

class BatteryIMURelayNode:
    def __init__(self):
        rospy.init_node('battery_imu_relay_node')
        
        # 时间阈值管理
        self.last_battery_sent = 0
        self.last_imu_sent = 0
        self.last_relay_check = 0
        self.battery_buffer = bytearray()
        self.imu_buffer = bytearray()
        
        # 温度控制参数
        self.temperature_threshold_high = rospy.get_param('~temperature_threshold_high', 15.0)  # 高温阈值
        self.temperature_threshold_low = rospy.get_param('~temperature_threshold_low', 10.0)    # 低温阈值
        self.battery_check_interval = rospy.get_param('~battery_check_interval', 60.0)             # 电池检查间隔
        self.relay_check_interval = rospy.get_param('~relay_check_interval', 180.0)             # 继电器检查间隔
        self.current_relay_state = False  # 当前继电器状态
        
        # 获取串口参数
        port = rospy.get_param('~serial_port', '/dev/IMU')
        baudrate = rospy.get_param('~baudrate', 115200)
        self.relay_address = rospy.get_param('~relay_address', 0x02)
        
        # 添加串口访问锁，避免并发访问冲突
        self.serial_lock = threading.Lock()
        
        # 初始化串口
        self.ser = None
        self.init_serial(port, baudrate)
        if not self.ser:
            rospy.signal_shutdown("串口初始化失败")
            return
            
        # 初始化状态机
        self.current_state = STATE_READY
        self.loop_counter = 0
        self.battery_timeout = 0
        self.imu_timeout = 0
        self.relay_timeout = 0
        self.battery_retry_count = 0
        self.max_battery_retry = 3  # 增加重试次数
        
        # 电池03指令请求帧
        self.REQUEST_BASIC_FRAME = bytes.fromhex('DD A5 03 00 FF FD 77')
        
        # IMU查询指令
        self.device_addr = 0x50
        self.rx_frame_length = 7
        self.REQ_IMU_FRAME = self.create_imu_query_frame()
        
        # 创建发布器
        self.battery_pub = rospy.Publisher('/battery_status', BatteryStatus, queue_size=10)
        self.imu_pub = rospy.Publisher('/inspvae_data', INSPVAE, queue_size=1)
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
        self.imu_query_count = 0
        self.imu_success_count = 0
        
        rospy.loginfo("电池-IMU-继电器集成节点初始化完成")

    def init_serial(self, port, baudrate):
        """串口初始化"""
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01
            )
            rospy.loginfo(f"串口连接成功: {port}")
            return True
        except Exception as e:
            rospy.logerr(f"串口连接失败: {e}")
            return False

    def create_imu_query_frame(self):
        """生成IMU查询帧"""
        cmd = bytes([self.device_addr, 0x03, 0x00, 0x3F, 0x00, 0x01])
        crc = self.calculate_crc(cmd)
        return cmd + crc

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
            # 基础电池信息解析
            status_msg.total_voltage = ((data[0] << 8) | data[1]) * 0.01
            status_msg.current = self.parse_current(data[2:4])
            status_msg.remaining_capacity = ((data[4] << 8) | data[5]) * 0.01
            status_msg.nominal_capacity = ((data[6] << 8) | data[7]) * 0.01
            status_msg.cycle_count = (data[8] << 8) | data[9]
            
            year, month, day = self.parse_date(data[10:12])
            status_msg.production_year = year
            status_msg.production_month = month
            status_msg.production_day = day
            
            status_msg.balance_low = (data[12] << 8) | data[13]
            status_msg.balance_high = (data[14] << 8) | data[15]
            status_msg.protection_status = (data[16] << 8) | data[17]
            
            ver_major = data[18] >> 4
            ver_minor = data[18] & 0x0F
            status_msg.software_version = f"{ver_major}.{ver_minor}"
            status_msg.batttery_remaining = data[19]
            
            # 温度数据解析和存储
            ntc_count = data[22]
            self.current_temperatures = []  # 清空旧温度数据
            for i in range(ntc_count):
                idx = 23 + i * 2
                raw_temp = (data[idx] << 8) | data[idx+1]
                temp = (raw_temp - 2731) / 10.0
                rounded_temp = round(temp, 1)
                status_msg.temperatures.append(rounded_temp)
                self.current_temperatures.append(rounded_temp)
            
            # 发布平均温度用于控制
            if self.current_temperatures:
                avg_temp = sum(self.current_temperatures) / len(self.current_temperatures)
                temp_msg = Float32()
                temp_msg.data = avg_temp
                self.temperature_pub.publish(temp_msg)
            
            # 发布电池状态
            self.battery_pub.publish(status_msg)
            rospy.loginfo("电池状态发布完成，温度数据已更新")
            return True
            
        except Exception as e:
            rospy.logerr(f"电池数据解析错误: {e}")
            return False

    def parse_imu_response(self, frame):
        """解析IMU响应数据"""
        try:
            recv_crc = frame[-2:]
            calc_crc = self.calculate_crc(frame[:-2])
            if recv_crc != calc_crc:
                rospy.logwarn("IMU数据CRC校验失败")
                return None
            
            yaw_bytes = frame[3:5]
            yaw = np.int16(struct.unpack('>h', yaw_bytes)[0]) / 32768.0 * 180.0
            return {'yaw': yaw}
        except Exception as e:
            rospy.logerr(f"IMU数据解析错误: {e}")
            return None

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

    def enable_relay(self, enable=True):
        """开启或关闭继电器"""
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
                # 发布继电器状态
                status_msg = Bool()
                status_msg.data = self.current_relay_state
                self.relay_status_pub.publish(status_msg)
                return True
            else:
                rospy.logwarn(f"继电器命令发送失败，重试 {attempt + 1}/{max_retries}")
                time.sleep(0.1)  # 等待后重试
        
        rospy.logerr("继电器命令发送失败，已达到最大重试次数")
        return False

    def read_relay_status(self):
        """读取继电器状态"""
        command_data = bytes([self.relay_address, 0x01, 0x00, 0x00, 0x00, 0x08])
        rospy.loginfo("发送继电器状态查询")
        
        try:
            # 使用锁保护串口访问
            with self.serial_lock:
                crc = self.calculate_crc(command_data)
                full_command = command_data + crc
                self.ser.write(full_command)
                
                time.sleep(0.05)
                response = self.ser.read(8)
                
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
        """基于温度控制继电器"""
        if not self.current_temperatures:
            rospy.logwarn("无温度数据，跳过继电器控制")
            return
        
        max_temp = max(self.current_temperatures)
        avg_temp = sum(self.current_temperatures) / len(self.current_temperatures)
        
        rospy.loginfo(f"温度监测 - 最高: {max_temp}°C, 平均: {avg_temp}°C, 当前继电器状态: {'开启' if self.current_relay_state else '关闭'}")
        
        if max_temp >= self.temperature_threshold_high and self.current_relay_state:
            rospy.loginfo(f"温度 {max_temp}°C 超过阈值 {self.temperature_threshold_high}°C，关闭继电器")
            if self.enable_relay(False):
                rospy.loginfo("继电器已关闭")
            else:
                rospy.logwarn("继电器关闭失败")
                
        elif avg_temp <= self.temperature_threshold_low and not self.current_relay_state:
            rospy.loginfo(f"温度 {avg_temp}°C 低于阈值 {self.temperature_threshold_low}°C，开启继电器")
            if self.enable_relay(True):
                rospy.loginfo("继电器已开启")
            else:
                rospy.logwarn("继电器开启失败")
        # 发布继电器状态
        status_msg = Bool()
        status_msg.data = self.current_relay_state
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
                    # self.battery_retry_count = 0
                    self.battery_success_count += 1
                    # rospy.loginfo("电池数据接收完成")
                elif current_time >= self.battery_timeout:
                    # rospy.logwarn(f"电池响应超时，重试次数: {self.battery_retry_count}")
                    self.battery_buffer.clear()
                    # self.battery_retry_count += 1
                    # if self.battery_retry_count <= self.max_battery_retry:
                        # if self.send_battery_query():
                            # self.battery_timeout = current_time + 1.0  # 1秒超时
                        # else:
                            # rospy.logerr("电池查询发送失败")
                    # else:
                    self.current_state = STATE_READY
                        # self.battery_retry_count = 0
                        # rospy.logwarn("电池查询重试次数用尽，返回就绪状态")
            
            # 2. 处理IMU响应（独立于电池状态）
            if self.current_state == STATE_WAITING_IMU:
                if self.process_imu_buffer():
                    self.current_state = STATE_READY
                    self.imu_success_count += 1
                elif current_time >= self.imu_timeout:
                    # IMU超时不阻塞，直接清除状态
                    self.imu_buffer.clear()
                    self.current_state = STATE_READY
                    rospy.loginfo("IMU响应超时")
            
            # 3. 处理继电器响应
            if self.current_state == STATE_WAITING_RELAY:
                # 继电器响应通常是即时的，短暂等待后返回就绪
                if current_time >= self.relay_timeout:
                    self.current_state = STATE_READY
                    rospy.logwarn("继电器响应超时")
            
            # === 发送新请求（优化优先级）===
            if self.current_state == STATE_READY:
                # 优先处理IMU查询（最高优先级）
                if current_time - self.last_imu_sent >= 0.2:  # 5Hz
                    if self.send_imu_query():
                        self.last_imu_sent = current_time
                        self.imu_query_count += 1
                        self.current_state = STATE_WAITING_IMU
                        self.imu_timeout = current_time + 0.1  # 100ms超时
                    else:
                        rospy.logerr("IMU查询发送失败")
                
                # 其次处理电池查询（低优先级）
                elif current_time - self.last_battery_sent >= self.battery_check_interval:
                    if self.send_battery_query():
                        self.last_battery_sent = current_time
                        self.battery_query_count += 1
                        self.current_state = STATE_WAITING_BATTERY
                        self.battery_timeout = current_time + 1.0  # 1秒超时
                    else:
                        rospy.logerr("电池查询发送失败")
                
                # 最后处理继电器温度控制（最低优先级）
                elif current_time - self.last_relay_check >= self.relay_check_interval:
                    self.temperature_based_control()
                    self.last_relay_check = current_time
                    
                    # 定期输出统计信息
                    if self.battery_query_count > 0:
                        battery_success_rate = (self.battery_success_count / self.battery_query_count) * 100
                        rospy.loginfo(f"电池查询成功率: {battery_success_rate:.2f}% ({self.battery_success_count}/{self.battery_query_count})")
                    
                    if self.imu_query_count > 0:
                        imu_success_rate = (self.imu_success_count / self.imu_query_count) * 100
                        rospy.loginfo(f"IMU查询成功率: {imu_success_rate:.2f}% ({self.imu_success_count}/{self.imu_query_count})")
            
            self.rate.sleep()

    def send_battery_query(self):
        """发送电池查询指令"""
        try:
            if self.ser and self.ser.is_open:
                # 使用锁保护串口访问
                with self.serial_lock:
                    self.battery_buffer.clear()
                    self.ser.write(b'\x00')
                    time.sleep(0.01)
                    self.ser.write(self.REQUEST_BASIC_FRAME)
                return True
        except serial.SerialException as e:
            rospy.logerr(f"电池查询发送失败（串口异常）: {e}")
            # 尝试重新初始化串口
            if self.reinit_serial():
                return False  # 重新初始化后下次再试
        except Exception as e:
            rospy.logerr(f"电池查询发送失败: {e}")
        return False

    def send_imu_query(self):
        """发送IMU查询指令"""
        try:
            if self.ser and self.ser.is_open:
                # 使用锁保护串口访问
                with self.serial_lock:
                    self.ser.write(self.REQ_IMU_FRAME)
                return True
        except serial.SerialException as e:
            rospy.logerr(f"IMU查询发送失败（串口异常）: {e}")
            # 尝试重新初始化串口
            if self.reinit_serial():
                return False  # 重新初始化后下次再试
        except Exception as e:
            rospy.logerr(f"IMU查询发送失败: {e}")
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
                timeout=0.01
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
                            hex_str = ' '.join(['%02X' % (b if isinstance(b, int) else ord(b)) for b in data])
                            # rospy.loginfo(f"收到485原始数据: {hex_str}")

                        for byte in data:
                            if isinstance(byte, int):
                                byte_val = byte
                            else:
                                byte_val = ord(byte)

                            # 分类打印
                            if byte_val == 0xDD:
                                rospy.loginfo("检测到电池数据帧头")
                                self.battery_buffer.clear()
                                self.battery_buffer.append(byte_val)
                            elif byte_val == 0x50:
                                # rospy.loginfo("检测到IMU数据帧头")
                                self.imu_buffer.clear()
                                self.imu_buffer.append(byte_val)
                            elif len(self.battery_buffer) > 0 and len(self.battery_buffer) < 100:
                                self.battery_buffer.append(byte_val)
                            elif len(self.imu_buffer) > 0 and len(self.imu_buffer) < 20:
                                self.imu_buffer.append(byte_val)
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
            self.battery_buffer.clear()  # 清空整个缓冲区
            return success
        
        return False

    def process_imu_buffer(self):
        """处理IMU响应数据"""
        start_idx = 0
        while start_idx < len(self.imu_buffer):
            if self.imu_buffer[start_idx] != 0x50:
                start_idx += 1
                continue
                
            if len(self.imu_buffer) < start_idx + self.rx_frame_length:
                return False
                
            frame_end = start_idx + self.rx_frame_length
            frame = bytes(self.imu_buffer[start_idx:frame_end])
            
            result = self.parse_imu_response(frame)
            if result:
                msg = INSPVAE()
                msg.header = Header(stamp=rospy.Time.now(), frame_id='inspvae')
                msg.yaw = result['yaw'] % 360
                self.imu_pub.publish(msg)
                self.imu_buffer.clear()  # 清空整个缓冲区
                return True
            
            start_idx += 1
            
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
        node = BatteryIMURelayNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"节点运行异常: {e}")