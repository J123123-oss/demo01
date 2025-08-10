#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
import struct
import time
import numpy as np
from std_msgs.msg import Float32, Header
from serial_comms.msg import BatteryStatus, INSPVAE

# 状态常量
STATE_READY = 0
STATE_WAITING_BATTERY = 1
STATE_WAITING_IMU = 2

class BatteryAndIMUNode:
    def __init__(self):
        rospy.init_node('battery_imu_node')
        # 增加时间阈值管理
        self.last_battery_sent = 0
        self.last_imu_sent = 0
        self.battery_buffer = bytearray()
        self.imu_buffer = bytearray()
        
        # 获取串口参数
        port = rospy.get_param('~serial_port', '/dev/IMU')
        baudrate = rospy.get_param('~baudrate', 115200)
        
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
        
        # 电池03指令请求帧 (DDA50300FFFD77)
        self.REQUEST_BASIC_FRAME = bytes.fromhex('DD A5 03 00 FF FD 77')
        
        # IMU查询指令 (50 03 00 3F 00 01 + CRC)
        self.device_addr = 0x50
        self.rx_frame_length = 7
        self.REQ_IMU_FRAME = self.create_imu_query_frame()
        
        # 创建发布器
        self.battery_pub = rospy.Publisher('/battery_status', BatteryStatus, queue_size=10)
        self.imu_pub = rospy.Publisher('/inspvae_data', INSPVAE, queue_size=1)
        
        # 设置循环速率 (100Hz)
        self.rate = rospy.Rate(100)  # 100Hz

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
        """生成IMU查询帧 (带CRC校验)"""
        cmd = bytes([self.device_addr, 0x03, 0x00, 0x3F, 0x00, 0x01])
        crc = self.calculate_crc(cmd)
        return cmd + crc

    def parse_date(self, raw_date):
        """解析电池生产日期 (2字节数据)"""
        value = (raw_date[0] << 8) | raw_date[1]
        day = value & 0x1F
        month = (value >> 5) & 0x0F
        year = 2000 + (value >> 9)
        return year, month, day

    def parse_current(self, data_bytes):
        """解析电池电流值 (2字节数据)"""
        value = (data_bytes[0] << 8) | data_bytes[1]
        return (value - 65536) * 0.01 if value >= 0x8000 else value * 0.01

    def process_battery_response(self, data):
        """解析电池响应数据帧"""
        status_msg = BatteryStatus()
        try:
            # 1. 总电压 (单位10mV转V)
            status_msg.total_voltage = ((data[0] << 8) | data[1]) * 0.01
            
            # 2. 电流 (带符号处理)
            status_msg.current = self.parse_current(data[2:4])
            
            # 3. 容量信息
            status_msg.remaining_capacity = ((data[4] << 8) | data[5]) * 0.01
            status_msg.nominal_capacity = ((data[6] << 8) | data[7]) * 0.01
            
            # 4. 循环次数
            status_msg.cycle_count = (data[8] << 8) | data[9]
            
            # 5. 生产日期
            year, month, day = self.parse_date(data[10:12])
            status_msg.production_year = year
            status_msg.production_month = month
            status_msg.production_day = day
            
            # 6. 均衡状态
            status_msg.balance_low = (data[12] << 8) | data[13]
            status_msg.balance_high = (data[14] << 8) | data[15]
            
            # 7. 保护状态
            status_msg.protection_status = (data[16] << 8) | data[17]
            
            # 8. 软件版本
            ver_major = data[18] >> 4
            ver_minor = data[18] & 0x0F
            status_msg.software_version = f"{ver_major}.{ver_minor}"
            
            # 9. 电量百分比
            status_msg.batttery_remaining = data[19]
            
            # 10. 温度
            ntc_count = data[22]  # 温度传感器数量
            for i in range(ntc_count):
                idx = 23 + i * 2
                raw_temp = (data[idx] << 8) | data[idx+1]
                temp = (raw_temp - 2731) / 10.0
                status_msg.temperatures.append(round(temp, 1))
            
            # 发布电池状态
            self.battery_pub.publish(status_msg)
            rospy.logdebug("Battery status published")
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
                # rospy.logwarn("IMU CRC校验失败")
                return None
            
            # 解析Yaw角度
            yaw_bytes = frame[3:5]
            yaw = np.int16(struct.unpack('>h', yaw_bytes)[0]) / 32768.0 * 180.0
            return {'yaw': yaw}
        except Exception as e:
            rospy.logerr(f"IMU数据解析错误: {e}")
            return None

    def run(self):
        """主状态机循环"""
        while not rospy.is_shutdown():
            current_time = time.time()
            
            # === 处理超时状态 ===
            if self.current_state == STATE_WAITING_BATTERY:
                if self.process_battery_buffer():
                    self.current_state = STATE_READY
                elif current_time >= self.battery_timeout:
                    # rospy.logwarn("电池响应超时")
                    self.battery_buffer.clear()
                    self.current_state = STATE_READY
            
            elif self.current_state == STATE_WAITING_IMU:
                if self.process_imu_buffer():
                    self.current_state = STATE_READY
                elif current_time >= self.imu_timeout:
                    rospy.logdebug("IMU响应超时")
                    self.imu_buffer.clear()
                    self.current_state = STATE_READY
            
            # === 发送新请求 ===
            if self.current_state == STATE_READY:
                # 优先处理电池请求（1Hz）
                if current_time - self.last_battery_sent >= 1.0:
                    if self.send_battery_query():
                        self.last_battery_sent = current_time
                        self.current_state = STATE_WAITING_BATTERY
                        self.battery_timeout = current_time + 0.5  # 100ms超时
                
                # 其次处理IMU请求（50Hz）
                elif current_time - self.last_imu_sent >= 0.02:
                    if self.send_imu_query():
                        self.last_imu_sent = current_time
                        self.current_state = STATE_WAITING_IMU
                        self.imu_timeout = current_time + 0.01
            
            # === 持续读取串口 ===
            self.read_serial_data()
            self.rate.sleep()

    def send_battery_query(self):
        """发送电池查询指令"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(self.REQUEST_BASIC_FRAME)
                rospy.logdebug("已发送电池查询")
                return True
        except Exception as e:
            rospy.logerr(f"电池查询发送失败: {e}")
        return False

    def send_imu_query(self):
        """发送IMU查询指令"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(self.REQ_IMU_FRAME)
                rospy.logdebug("已发送IMU查询")
                return True
        except Exception as e:
            rospy.logerr(f"IMU查询发送失败: {e}")
        return False

    def read_serial_data(self):
        """从串口读取可用数据（分离缓冲区）"""
        try:
            if self.ser and self.ser.is_open:
                avail = self.ser.in_waiting
                if avail > 0:
                    data = self.ser.read(avail)
                    for byte in data:
                        # 1. 电池响应起始标记
                        if byte == 0xDD:
                            self.battery_buffer.append(byte)
                            
                        # 2. IMU响应起始标记
                        elif byte == 0x50:
                            self.imu_buffer.append(byte)
                            
                        # 3. 已有缓冲的延续
                        elif len(self.battery_buffer) > 0:
                            self.battery_buffer.append(byte)
                        elif len(self.imu_buffer) > 0:
                            self.imu_buffer.append(byte)
                            
                        # 4. 无效数据（丢弃）
                        else:
                            rospy.logdebug(f"丢弃无效字节: 0x{byte:02X}")
        except Exception as e:
            rospy.logwarn(f"串口读取错误: {e}")


    def process_battery_buffer(self):
        """处理电池响应数据"""
        # 查找帧头0xDD
        start_idx = 0
        while start_idx < len(self.battery_buffer):
            if self.battery_buffer[start_idx] != 0xDD:
                start_idx += 1
                continue
                
            # 检查指令码和状态码
            if start_idx + 3 >= len(self.battery_buffer):
                return False  # 数据不足
                
            if self.battery_buffer[start_idx+1] != 0x03:
                start_idx += 1
                continue
                
            if self.battery_buffer[start_idx+2] != 0x00:  # 非成功状态
                start_idx += 1
                continue
                
            # 获取数据长度
            data_length = self.battery_buffer[start_idx+3]
            full_length = data_length + 7  # DD + CMD(03) + STATUS(00) + LEN + DATA + CRC(2) + 77
            
            # 检查完整帧是否到达
            if len(self.battery_buffer) < start_idx + full_length:
                return False  # 数据不足
                
            # 检查帧尾0x77
            if self.battery_buffer[start_idx + full_length - 1] != 0x77:
                start_idx += 1
                continue
                
            # 提取数据区块 (跳过DD, CMD, STATUS, LEN)
            data_start = start_idx + 4
            data_end = data_start + data_length
            data_block = self.battery_buffer[data_start:data_end]
            
            # 处理数据区块
            success = self.process_battery_response(data_block)
            
            # 从缓冲区移除已处理数据
            del self.battery_buffer[:start_idx + full_length]
            return success
        
        return False

    def process_imu_buffer(self):
        """处理IMU响应数据"""
        # 查找帧头0x50
        start_idx = 0
        while start_idx < len(self.imu_buffer):
            if self.imu_buffer[start_idx] != 0x50:
                start_idx += 1
                continue
                
            # 检查是否有完整帧
            if len(self.imu_buffer) < start_idx + self.rx_frame_length:
                return False  # 数据不足
                
            # 提取完整帧
            frame_end = start_idx + self.rx_frame_length
            frame = bytes(self.imu_buffer[start_idx:frame_end])
            
            # 解析数据
            result = self.parse_imu_response(frame)
            if result:
                # 发布IMU数据
                msg = INSPVAE()
                msg.header = Header(stamp=rospy.Time.now(), frame_id='inspvae')
                msg.yaw = result['yaw'] % 360  # 规范化到0-360度
                self.imu_pub.publish(msg)
                
                # 从缓冲区移除已处理数据
                del self.imu_buffer[:frame_end]
                return True
            
            start_idx += 1
            
        return False

    @staticmethod
    def calculate_crc(data):
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

if __name__ == '__main__':
    try:
        node = BatteryAndIMUNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
