#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
import struct
from sensor_msgs.msg import Imu
from serial_comms.msg import INSPVAE
import numpy as np
from std_msgs.msg import Header
import std_srvs.srv

class IMUParser:
    def __init__(self):
        rospy.init_node('imu_parser_node')
        
        # 参数配置
        self.port = rospy.get_param('~serial_port', '/dev/singleIMU')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.device_addr = 0x50
        self.rx_frame_length = 7
        self.ser = None
        self.reconnect_interval = 2.0  # 重连间隔
        self.last_reconnect_time = 0

        self.start_srv = rospy.Service('~start_imu', std_srvs.srv.Trigger, self.handle_start)
        self.stop_srv = rospy.Service('~stop_imu', std_srvs.srv.Trigger, self.handle_stop)
        
        # 等待初始化串口
        self.init_serial_with_retry()

        # 发布IMU数据
        self.imu_pub = rospy.Publisher('/inspvae_data', INSPVAE, queue_size=1)
        
        self.working = False  # IMU工作状态标志
        self.timer = None

    def init_serial(self):
        """单次初始化/重新初始化串口连接"""
        try:
            # 关闭可能存在的旧连接
            if self.ser and self.ser.is_open:
                self.ser.close()
            # 尝试建立新连接
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
            self.ser = None  # 确保连接失败时ser为None
            return False

    def init_serial_with_retry(self):
        """初始化阶段循环重试，直到串口连接成功或节点关闭"""
        rospy.loginfo(f"尝试连接串口 {self.port}...")
        while not rospy.is_shutdown() and not self.init_serial():
            rospy.logwarn(f"IMU串口连接失败，{self.reconnect_interval}秒后重试...")
            rospy.sleep(self.reconnect_interval)
        if rospy.is_shutdown():
            rospy.loginfo("IMU节点已关闭，停止串口初始化")

    def safe_serial_write(self, data):
        """安全的串口数据写入，失败时触发重连"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(data)
                return True
            # 连接未就绪时尝试重连
            self.init_serial()
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
        return {'roll': 0, 'pitch': 0, 'yaw': yaw}

    def start(self):
        """启动IMU工作"""
        if not self.working:
            self.working = True
            # 启动定时器
            self.timer = rospy.Timer(rospy.Duration(0.2), self.send_query_cmd)
            rospy.loginfo("IMU工作已启动(imu_parser_node)")

    def stop(self):
        """停止IMU工作"""
        if self.working:
            self.working = False
            if self.timer:
                self.timer.shutdown()
                self.timer = None
            rospy.loginfo("IMU工作已停止(imu_parser_node)")

    def handle_start(self, req):
        self.start()
        return std_srvs.srv.TriggerResponse(success=True, message="IMU started")

    def handle_stop(self, req):
        self.stop()
        return std_srvs.srv.TriggerResponse(success=True, message="IMU stopped")

    def run(self):
        """主循环"""
        buffer = bytearray()
        while not rospy.is_shutdown():
            if not self.working:
                rospy.sleep(0.1)
                continue
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
                
                # 运行中检查串口连接状态，触发重连
                if not self.ser or not self.ser.is_open:
                    current_time = rospy.Time.now().to_sec()
                    if current_time - self.last_reconnect_time > self.reconnect_interval:
                        if self.init_serial():
                            self.last_reconnect_time = current_time
                        else:
                            rospy.sleep(0.1)  # 短等待避免CPU占用过高
                
                rospy.sleep(0.001)

            except Exception as e:
                rospy.logerr(f"Main loop error: {str(e)}")
                self.init_serial()  # 发生异常时尝试重连
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
        node.start()  # 默认启动IMU工作
        node.run()
    except rospy.ROSInterruptException:
        pass