#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
import struct
import numpy as np
from std_msgs.msg import Header, Float32
from sensor_msgs.msg import Imu
from serial_comms.msg import BatteryStatus, INSPVAE

class CombinedIMUBatteryNode:
    def __init__(self):
        rospy.init_node('combined_imu_battery_node')
        
        # 共享串口参数
        self.port = rospy.get_param('~serial_port', '/dev/IMU')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        
        # 初始化串口
        self.ser = None
        self.init_serial()
        
        # IMU相关参数
        self.imu_device_addr = 0x50
        self.imu_rx_frame_length = 7
        self.last_imu_query_time = 0
        self.imu_query_interval = 0.02  # 50Hz
        
        # 电池相关参数
        self.battery_request_frame = bytes.fromhex('DD A5 03 00 FF FD 77')
        self.last_battery_query_time = 0
        self.battery_query_interval = 1.0  # 1Hz
        
        # 发布器
        self.imu_pub = rospy.Publisher('/inspvae_data', INSPVAE, queue_size=10)
        self.battery_pub = rospy.Publisher('/battery_status', BatteryStatus, queue_size=10)
        
        # 数据缓冲区
        self.buffer = bytearray()
        
        # 定时器
        rospy.Timer(rospy.Duration(0.01), self.run_loop)  # 100Hz主循环

    def init_serial(self):
        """初始化/重新初始化串口连接"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            rospy.loginfo(f"Successfully connected to serial port: {self.port}")
            return True
        except Exception as e:
            rospy.logerr(f"Serial connection failed: {str(e)}")
            return False

    def safe_serial_write(self, data):
        """安全的串口数据写入"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(data)
                return True
            return False
        except Exception as e:
            rospy.logwarn(f"Serial write failed: {str(e)}")
            self.init_serial()  # 尝试重新连接
            return False

    def send_imu_query(self):
        """发送IMU查询指令"""
        cmd = bytes.fromhex(f"{self.imu_device_addr:02X} 03 00 3F 00 01 ")
        crc = self.calculate_crc(cmd)
        full_cmd = cmd + crc
        return self.safe_serial_write(full_cmd)

    def send_battery_query(self):
        rospy.loginfo("Sending battery query frame: DD A5 03 00 FF FD 77")
        if self.safe_serial_write(self.battery_request_frame):
            rospy.loginfo("Battery query sent successfully")
            return True
        rospy.logwarn("Failed to send battery query")
        return False

    def parse_imu_response(self, data):
        """解析IMU返回数据"""
        if len(data) != self.imu_rx_frame_length or data[0] != 0x50:
            return None
        
        # CRC校验
        recv_crc = data[-2:]
        calc_crc = self.calculate_crc(data[:-2])
        if recv_crc != calc_crc:
            return None
        
        # 解析角度数据
        yaw_bytes = data[3:5]
        yaw = np.int16(struct.unpack('>h', yaw_bytes)[0]) / 32768.0 * 180.0
        return {'roll': 0, 'pitch': 0, 'yaw': yaw}

    def parse_battery_response(self, data):
        """解析电池响应数据帧"""
        # 创建自定义消息对象
        status_msg = BatteryStatus()
        
        try:
            # 1. 总电压 (2字节, 单位10mV)
            status_msg.total_voltage = ((data[0] << 8) | data[1]) * 0.01  # 转换为伏特(V)
            
            # 2. 电流 (2字节, 带符号处理)
            value = (data[2] << 8) | data[3]
            if value >= 0x8000:  # 最高位为1表示放电(负电流)
                status_msg.current = (value - 65536) * 0.01  # 转换为安培(A)
            else:
                status_msg.current = value * 0.01  # 转换为安培(A)
            
            # 3. 容量信息 (4字节)
            status_msg.remaining_capacity = ((data[4] << 8) | data[5]) * 0.01  # 转换为安时(Ah)
            status_msg.nominal_capacity = ((data[6] << 8) | data[7]) * 0.01    # 转换为安时(Ah)
            
            # 4. 循环次数 (2字节)
            status_msg.cycle_count = (data[8] << 8) | data[9]
            
            # 5. 生产日期 (2字节)
            raw_date = (data[10] << 8) | data[11]
            day = raw_date & 0x1F
            month = (raw_date >> 5) & 0x0F
            year = 2000 + (raw_date >> 9)
            status_msg.production_year = year
            status_msg.production_month = month
            status_msg.production_day = day
            
            # 6. 均衡状态 (4字节)
            status_msg.balance_low = (data[12] << 8) | data[13]
            status_msg.balance_high = (data[14] << 8) | data[15]
            
            # 7. 保护状态 (2字节)
            status_msg.protection_status = (data[16] << 8) | data[17]
            
            # 8. 软件版本 (1字节)
            version_major = data[18] >> 4
            version_minor = data[18] & 0x0F
            status_msg.software_version = f"{version_major}.{version_minor}"
            
            # 9. 电量百分比 (1字节)
            status_msg.batttery_remaining = data[19]  # 0-100%
            
            # 10. 温度数据解析
            ntc_count = data[22]  # 温度传感器数量
            for i in range(ntc_count):
                index = 23 + i * 2
                raw_temp = (data[index] << 8) | data[index + 1]
                temperature = (raw_temp - 2731) / 10.0
                temperature = round(temperature, 1)  # 保留一位小数
                status_msg.temperatures.append(temperature)
            
            return status_msg
            
        except IndexError as e:
            rospy.logerr(f"Battery data parsing error: insufficient data length ({len(data)} bytes)")
            return None
        except Exception as e:
            rospy.logerr(f"Error processing battery response: {e}")
            return None

    def process_battery_frame(self):
        # 读取帧头
        frame = self.ser.read(1)
        # rospy.loginfo(f"Read header byte: {frame.hex() if frame else 'None'}")
        
        if not frame:
            rospy.loginfo("No data available")
            return False
            
        if frame != b'\xdd':
            # rospy.loginfo(f"Invalid header: {frame.hex()}, expected DD")
            return False
        
        # 命令码 (1字节)
        command = self.ser.read(1)
        if command != b'\x03':  # 只处理基本信息响应
            return False
            
        # 状态码 (1字节)
        status = self.ser.read(1)
        if status != b'\x00':   # 0 表示正确
            rospy.logwarn(f"Battery returned error status: 0x{status.hex()}")
            return False
        
        # 数据长度 (1字节)
        data_length = self.ser.read(1)[0]
        
        # 读取数据块
        data = self.ser.read(data_length)
        
        # 读取校验和 (2字节) 和帧尾 (1字节)
        checksum = self.ser.read(2)
        end_marker = self.ser.read(1)
        
        if end_marker == b'\x77':
            print("data:",data)
            status_msg = self.parse_battery_response(data)
            if status_msg:
                self.battery_pub.publish(status_msg)
                rospy.loginfo_once("Battery status published successfully")
                return True
        
        return False

    def run_loop(self, event):
        """主循环处理"""
        current_time = rospy.Time.now().to_sec()
        
        # 发送IMU查询指令 (50Hz)
        if current_time - self.last_imu_query_time >= self.imu_query_interval:
            if self.send_imu_query():
                self.last_imu_query_time = current_time
        
        # 发送电池查询指令 (1Hz)
        if current_time - self.last_battery_query_time >= self.battery_query_interval:
            if self.send_battery_query():
                self.last_battery_query_time = current_time
                rospy.sleep(0.05)  # 等待电池响应
        
        # 处理串口接收数据
        if self.ser and self.ser.is_open:
            try:
                # 读取所有可用数据
                data = self.ser.read(self.ser.in_waiting or 1)
                if data:
                    self.buffer += data
                
                # 处理IMU数据帧
                while len(self.buffer) >= self.imu_rx_frame_length:
                    # 查找IMU帧头
                    header_pos = self.buffer.find(b'\x50')
                    if header_pos == -1:
                        self.buffer.clear()
                        break
                    
                    # 丢弃帧头前的无效数据
                    if header_pos > 0:
                        self.buffer = self.buffer[header_pos:]
                    
                    # 检查数据长度是否足够
                    if len(self.buffer) < self.imu_rx_frame_length:
                        break
                    
                    # 提取并处理帧
                    frame = self.buffer[:self.imu_rx_frame_length]
                    self.buffer = self.buffer[self.imu_rx_frame_length:]
                    
                    parsed = self.parse_imu_response(frame)
                    if parsed:
                        self.publish_inspvae_data(parsed)
                
                # 处理电池数据帧
                if self.ser.in_waiting > 0:
                    self.process_battery_frame()
                    
            except Exception as e:
                rospy.logerr(f"Data processing error: {str(e)}")
                self.init_serial()  # 尝试重新连接

    def publish_inspvae_data(self, angles):
        """发布INSPVAE数据"""
        msg = INSPVAE()
        msg.header = Header(stamp=rospy.Time.now(), frame_id='inspvae')
        msg.yaw = angles['yaw'] % 360  # 确保角度在0-360范围
        self.imu_pub.publish(msg)

    @staticmethod
    def calculate_crc(data):
        """Modbus CRC16校验"""
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
        node = CombinedIMUBatteryNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass