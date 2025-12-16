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
import atexit  # 新增：注册退出清理函数

class IMUParser:
    def __init__(self):
        rospy.init_node('imu_parser_node')
        
        # 参数配置
        self.port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~baudrate', 9600)
        self.device_addr = 0x50  # 设备地址
        self.query_addr = 0x3D    # 起始寄存器地址(Roll)
        self.query_reg_num = 3    # 读取3个寄存器(Roll/Pitch/Yaw)
        self.rx_frame_length = 11  # 返回帧长度: 50 03 06 + 6字节数据 + 2字节CRC = 11
        self.ser = None
        self.reconnect_interval = 2.0  # 重连间隔
        self.last_reconnect_time = 0

        # ROS服务
        self.start_srv = rospy.Service('~start_imu', std_srvs.srv.Trigger, self.handle_start)
        self.stop_srv = rospy.Service('~stop_imu', std_srvs.srv.Trigger, self.handle_stop)
        
        # 初始化串口
        self.init_serial_with_retry()

        # 发布话题
        self.imu_pub = rospy.Publisher('/inspvae_data', INSPVAE, queue_size=1)
        self.imu_std_pub = rospy.Publisher('/imu/data', Imu, queue_size=1)  # 标准IMU话题
        
        self.working = False  # IMU工作状态标志
        self.timer = None

        # 新增：注册退出清理函数（节点退出时强制关闭串口）
        atexit.register(self.cleanup)
        # 新增：ROS节点关闭回调
        rospy.on_shutdown(self.cleanup)

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
            rospy.loginfo(f"成功连接到串口: {self.port} (波特率: {self.baudrate})")
            return True
        except Exception as e:
            rospy.logerr(f"串口连接失败: {str(e)}")
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
            rospy.logwarn(f"串口写入失败: {str(e)}")
            self.init_serial()  # 尝试重新连接
            return False

    def send_query_cmd(self, event):
        """发送三轴角度查询指令 (0x50 03 00 3D 00 03 + CRC)"""
        # 构造基础指令
        cmd = bytearray([
            self.device_addr,  # 设备地址
            0x03,              # 读寄存器功能码
            (self.query_addr >> 8) & 0xFF,  # 起始寄存器高字节
            self.query_addr & 0xFF,         # 起始寄存器低字节
            (self.query_reg_num >> 8) & 0xFF,  # 寄存器数量高字节
            self.query_reg_num & 0xFF         # 寄存器数量低字节
        ])
        # 计算并添加CRC
        crc = self.calculate_crc(cmd)
        full_cmd = cmd + crc
        # 发送指令
        self.safe_serial_write(full_cmd)

    def parse_response(self, data):
        """解析返回的三轴角度数据"""
        # 1. 基础校验
        if len(data) != self.rx_frame_length:
            rospy.logdebug(f"数据长度错误: 实际{len(data)}，期望{self.rx_frame_length}")
            return None
        
        if data[0] != self.device_addr:
            rospy.logdebug(f"设备地址不匹配: 实际{data[0]}，期望{self.device_addr}")
            return None
        
        if data[1] != 0x03:
            rospy.logdebug(f"功能码错误: 实际{data[1]}，期望0x03")
            return None
        
        if data[2] != 0x06:  # 3个寄存器 * 2字节 = 6字节数据
            rospy.logdebug(f"数据长度字段错误: 实际{data[2]}，期望0x06")
            return None

        # 2. CRC校验
        recv_crc = data[-2:]
        calc_crc = self.calculate_crc(data[:-2])
        if recv_crc != calc_crc:
            rospy.logdebug(f"CRC校验失败: 接收{recv_crc.hex()}，计算{calc_crc.hex()}")
            return None

        # 3. 解析三轴角度
        try:
            # Roll: 第3-4字节 (高字节在前)
            roll_raw = np.int16((data[3] << 8) | data[4])
            roll = roll_raw / 32768.0 * 180.0
            
            # Pitch: 第5-6字节
            pitch_raw = np.int16((data[5] << 8) | data[6])
            pitch = pitch_raw / 32768.0 * 180.0
            
            # Yaw: 第7-8字节
            yaw_raw = np.int16((data[7] << 8) | data[8])
            yaw = yaw_raw / 32768.0 * 180.0
            
            return {
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw,
                'roll_raw': roll_raw,
                'pitch_raw': pitch_raw,
                'yaw_raw': yaw_raw
            }
        except Exception as e:
            rospy.logerr(f"数据解析失败: {str(e)}")
            return None

    def publish_inspvae_data(self, angles):
        """发布自定义INSPVAE消息和标准IMU消息"""
        # 1. 发布自定义INSPVAE消息
        inspvae_msg = INSPVAE()
        inspvae_msg.header = Header(stamp=rospy.Time.now(), frame_id='imu_link')
        inspvae_msg.roll = angles['roll']
        inspvae_msg.pitch = angles['pitch']
        inspvae_msg.yaw = angles['yaw'] % 360  # 归一化到0-360度
        self.imu_pub.publish(inspvae_msg)

        # 2. 发布标准sensor_msgs/Imu消息 (补充角度数据)
        imu_msg = Imu()
        imu_msg.header = Header(stamp=rospy.Time.now(), frame_id='imu_link')
        
        # 角度转四元数 (roll/pitch/yaw -> quaternion)
        roll_rad = np.radians(angles['roll'])
        pitch_rad = np.radians(angles['pitch'])
        yaw_rad = np.radians(angles['yaw'])
        
        # 四元数计算 (Z-Y-X欧拉角转四元数)
        cy = np.cos(yaw_rad * 0.5)
        sy = np.sin(yaw_rad * 0.5)
        cp = np.cos(pitch_rad * 0.5)
        sp = np.sin(pitch_rad * 0.5)
        cr = np.cos(roll_rad * 0.5)
        sr = np.sin(roll_rad * 0.5)
        
        imu_msg.orientation.w = cy * cp * cr + sy * sp * sr
        imu_msg.orientation.x = cy * cp * sr - sy * sp * cr
        imu_msg.orientation.y = sy * cp * sr + cy * sp * cr
        imu_msg.orientation.z = sy * cp * cr - cy * sp * sr
        
        # 标记协方差为未知 (根据实际情况可调整)
        imu_msg.orientation_covariance = [-1.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0]
        imu_msg.angular_velocity_covariance = [-1.0, 0.0, 0.0,
                                               0.0, 0.0, 0.0,
                                               0.0, 0.0, 0.0]
        imu_msg.linear_acceleration_covariance = [-1.0, 0.0, 0.0,
                                                  0.0, 0.0, 0.0,
                                                  0.0, 0.0, 0.0]
        
        self.imu_std_pub.publish(imu_msg)

        # 日志输出 (可选，调试用)
        rospy.logdebug(f"解析结果 - Roll: {angles['roll']:.2f}°, Pitch: {angles['pitch']:.2f}°, Yaw: {angles['yaw']:.2f}°")

    def start(self):
        """启动IMU工作"""
        if not self.working:
            self.working = True
            # 启动定时器 (50Hz，可根据需求调整)
            self.timer = rospy.Timer(rospy.Duration(0.2), self.send_query_cmd)
            rospy.loginfo("IMU工作已启动 - 开始读取三轴角度数据")

    def stop(self):
        """停止IMU工作（修改：新增关闭串口逻辑）"""
        if self.working:
            self.working = False
            # 停止定时器
            if self.timer:
                self.timer.shutdown()
                self.timer = None
            # 关闭串口（新增核心逻辑）
            self.close_serial()
            rospy.loginfo("IMU工作已停止 - 停止读取三轴角度数据，串口已关闭")

    # 新增：独立的串口关闭函数
    def close_serial(self):
        """安全关闭串口"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                rospy.loginfo(f"串口 {self.port} 已成功关闭")
            self.ser = None
        except Exception as e:
            rospy.logerr(f"关闭串口失败: {str(e)}")

    # 新增：全局清理函数（定时器+串口+状态）
    def cleanup(self):
        """节点退出/停止时的全局清理"""
        rospy.loginfo("执行IMU节点清理操作...")
        # 1. 停止工作状态
        self.working = False
        # 2. 关闭定时器
        if self.timer:
            self.timer.shutdown()
            self.timer = None
        # 3. 关闭串口
        self.close_serial()
        rospy.loginfo("IMU节点清理完成")

    def handle_start(self, req):
        """服务回调：启动IMU"""
        self.start()
        return std_srvs.srv.TriggerResponse(success=True, message="IMU已启动，开始读取三轴角度数据")

    def handle_stop(self, req):
        """服务回调：停止IMU"""
        self.stop()
        return std_srvs.srv.TriggerResponse(success=True, message="IMU已停止，串口已关闭")

    def run(self):
        """主循环：读取并处理串口数据"""
        buffer = bytearray()
        while not rospy.is_shutdown():
            if not self.working:
                rospy.sleep(0.1)
                continue
            
            try:
                # 读取串口数据
                if self.ser and self.ser.is_open:
                    # 读取所有可用数据
                    data = self.ser.read(self.ser.in_waiting or 1)
                    if data:
                        buffer += data

                    # 处理缓冲区中的完整帧
                    while len(buffer) >= self.rx_frame_length:
                        # 查找帧头(设备地址)
                        header_pos = buffer.find(bytes([self.device_addr]))
                        if header_pos == -1:
                            buffer.clear()  # 无帧头，清空缓冲区
                            break
                        
                        # 丢弃帧头前的无效数据
                        if header_pos > 0:
                            buffer = buffer[header_pos:]
                        
                        # 检查剩余数据是否足够
                        if len(buffer) < self.rx_frame_length:
                            break
                        
                        # 提取一帧数据并处理
                        frame = buffer[:self.rx_frame_length]
                        buffer = buffer[self.rx_frame_length:]
                        
                        # 解析数据并发布
                        parsed_data = self.parse_response(frame)
                        if parsed_data:
                            self.publish_inspvae_data(parsed_data)
                
                # 运行中检查串口连接状态，按需重连
                if not self.ser or not self.ser.is_open:
                    current_time = rospy.Time.now().to_sec()
                    if current_time - self.last_reconnect_time > self.reconnect_interval:
                        rospy.logwarn("串口连接断开，尝试重连...")
                        if self.init_serial():
                            self.last_reconnect_time = current_time
                        else:
                            rospy.sleep(0.1)  # 短等待避免CPU占用过高
                
                rospy.sleep(0.001)  # 降低CPU占用

            except Exception as e:
                rospy.logerr(f"主循环异常: {str(e)}")
                self.init_serial()  # 异常时尝试重连串口
                rospy.sleep(1)

    @staticmethod
    def calculate_crc(data):
        """Modbus CRC16校验计算 (返回小端序的2字节CRC)"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        # 转换为小端序字节
        return struct.pack('<H', crc)

if __name__ == '__main__':
    try:
        imu_node = IMUParser()
        imu_node.start()  # 默认启动IMU
        imu_node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("IMU节点被中断，正在退出...")
        # 显式触发清理
        imu_node.cleanup()
    except Exception as e:
        rospy.logfatal(f"IMU节点启动失败: {str(e)}")
        # 异常退出也清理串口
        if 'imu_node' in locals():
            imu_node.cleanup()