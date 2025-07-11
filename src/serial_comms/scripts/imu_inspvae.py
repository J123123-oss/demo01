#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
import struct
from sensor_msgs.msg import Imu
from serial_comms.msg import INSPVAE
import numpy as np
from std_msgs.msg import Header

class IMUParser:
    def __init__(self):
        rospy.init_node('imu_parser_node')
        
        # 参数配置
        self.port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.device_addr = 0x50
        self.rx_frame_length = 7
        self.ser = None
        self.reconnect_interval = 1.0  # 重连间隔
        self.last_reconnect_time = 0

        # 初始化串口
        self.init_serial()

        # 发布IMU数据
        self.imu_pub = rospy.Publisher('/inspvae_data', INSPVAE, queue_size=1)
        
        # 定时发送查询指令
        self.timer = rospy.Timer(rospy.Duration(0.02), self.send_query_cmd)

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
            rospy.loginfo(f"Successfully connected to {self.port}")
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

    def send_query_cmd(self, event):
        """发送查询指令"""
        cmd = bytes.fromhex(f"{self.device_addr:02X} 03 00 3F 00 01 ")
        crc = self.calculate_crc(cmd)
        full_cmd = cmd + crc
        self.safe_serial_write(full_cmd)

    def parse_response(self, data):
        """解析返回数据"""
        if len(data) != self.rx_frame_length or data[0] != 0x50:
            return None
        
        # CRC校验
        recv_crc = data[-2:]
        calc_crc = self.calculate_crc(data[:-2])
        if recv_crc != calc_crc:
            return None
        
        # 解析角度数据
        yaw_bytes = data[3:5]
        yaw = np.int16(struct.unpack('>h', yaw_bytes)[0]) / 32768.0 * 180.0
        # if yaw > 180:
            # yaw -= 360
        return {'roll': 0, 'pitch': 0, 'yaw': yaw}

    def run(self):
        """主循环"""
        buffer = bytearray()
        while not rospy.is_shutdown():
            try:
                # 读取串口数据
                if self.ser and self.ser.is_open:
                    data = self.ser.read(self.ser.in_waiting or 1)
                    if data:
                        buffer += data

                    # 处理完整帧
                    while len(buffer) >= self.rx_frame_length:
                        # 查找帧头
                        header_pos = buffer.find(b'\x50')
                        if header_pos == -1:
                            buffer.clear()
                            break
                        
                        # 丢弃帧头前的无效数据
                        if header_pos > 0:
                            buffer = buffer[header_pos:]
                        
                        # 检查数据长度是否足够
                        if len(buffer) < self.rx_frame_length:
                            break
                        
                        # 提取并处理帧
                        frame = buffer[:self.rx_frame_length]
                        buffer = buffer[self.rx_frame_length:]
                        
                        parsed = self.parse_response(frame)
                        if parsed:
                            self.publish_inspvae_data(parsed)
                
                # 检查串口连接状态
                if not self.ser or not self.ser.is_open:
                    if rospy.Time.now().to_sec() - self.last_reconnect_time > self.reconnect_interval:
                        if self.init_serial():
                            self.last_reconnect_time = rospy.Time.now().to_sec()
                        else:
                            rospy.sleep(1)
                
                rospy.sleep(0.001)

            except Exception as e:
                rospy.logerr(f"Main loop error: {str(e)}")
                self.init_serial()
                rospy.sleep(1)

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
        node = IMUParser()
        node.run()
    except rospy.ROSInterruptException:
        pass