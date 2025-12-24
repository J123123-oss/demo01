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
import atexit
# 新增导入（放在现有导入后）
from serial_comms.srv import SetGyroCalib, SetGyroCalibResponse
import time

class IMUParser:
    def __init__(self):
        rospy.init_node('imu_parser_node')
        
        # 参数配置
        self.port = rospy.get_param('~serial_port', '/dev/ttyACM0')
        self.baudrate = rospy.get_param('~baudrate', 9600)
        self.device_addr = 0x50  # 设备地址 (0x50 = 'P')
        
        # 寄存器配置：分3条指令查询
        self.sensor_configs = {
            'acc': {
                'addr': 0x34, 
                'reg_num': 3, 
                'frame_len': 11,
                'cmd': self.build_query_cmd(0x34, 3),
                'last_sent': 0
            },
            'gyro': {
                'addr': 0x37, 
                'reg_num': 3, 
                'frame_len': 11,
                'cmd': self.build_query_cmd(0x37, 3),
                'last_sent': 0
            },
            'rpy': {
                'addr': 0x3D, 
                'reg_num': 3, 
                'frame_len': 11,
                'cmd': self.build_query_cmd(0x3D, 3),
                'last_sent': 0
            }
        }
        
        # 记录最后发送的指令类型，用于匹配返回数据
        self.last_sent_sensor = None
        self.sent_cmd_timestamp = {}
        
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
        
        # 数据缓存
        self.acc_data = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'updated': False, 'timestamp': 0}
        self.gyro_data = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'updated': False, 'timestamp': 0}
        self.rpy_data = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'updated': False, 'timestamp': 0}
        
        self.working = False  # IMU工作状态标志
        self.query_timer = None
        self.query_state = 0  # 0:acc, 1:gyro, 2:rpy
        self.data_buffer = bytearray()  # 数据缓冲区
        self.query_interval = 0.1  # 指令发送间隔（100ms）
        
        # 陀螺仪自动校准指令配置
        self.calib_cmds = {
            'unlock': bytearray([0x50, 0x06, 0x00, 0x69, 0xB5, 0x88, 0x22, 0xA1]),  # 解锁指令
            'disable': bytearray([0x50, 0x06, 0x00, 0x61, 0x00, 0x01, 0x14, 0x55]), # 关闭自动校准
            'enable': bytearray([0x50, 0x06, 0x00, 0x61, 0x00, 0x00, 0xD5, 0x95]),  # 开启自动校准
            'save': bytearray([0x50, 0x06, 0x00, 0x00, 0x00, 0x00, 0x84, 0x4B])     # 保存指令
        }
        # 校准服务
        self.calib_srv = rospy.Service('~set_gyro_calib', SetGyroCalib, self.handle_gyro_calib)

        # 注册退出清理函数
        atexit.register(self.cleanup)
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
                timeout=0.3,
                write_timeout=0.3
            )
            rospy.loginfo(f"成功连接到串口: {self.port} (波特率: {self.baudrate})")
            return True
        except Exception as e:
            rospy.logerr(f"串口连接失败: {str(e)}")
            self.ser = None
            return False

    def init_serial_with_retry(self):
        """初始化阶段循环重试，直到串口连接成功或节点关闭"""
        rospy.loginfo(f"尝试连接串口 {self.port}...")
        while not rospy.is_shutdown() and not self.init_serial():
            rospy.logwarn(f"IMU串口连接失败，{self.reconnect_interval}秒后重试...")
            rospy.sleep(self.reconnect_interval)
        if rospy.is_shutdown():
            rospy.loginfo("IMU节点已关闭，停止串口初始化")

    def build_query_cmd(self, reg_addr, reg_num):
        """构建查询指令并返回字节数组"""
        cmd = bytearray([
            self.device_addr,          # 设备地址
            0x03,                      # 读寄存器功能码
            (reg_addr >> 8) & 0xFF,    # 起始寄存器高字节
            reg_addr & 0xFF,           # 起始寄存器低字节
            (reg_num >> 8) & 0xFF,     # 寄存器数量高字节
            reg_num & 0xFF             # 寄存器数量低字节
        ])
        # 计算并添加CRC
        crc = self.calculate_crc(cmd)
        full_cmd = cmd + crc
        return full_cmd

    def calculate_crc(self, data):
        """Modbus CRC16校验计算"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return struct.pack('<H', crc)

    def safe_serial_write(self, cmd, sensor_type):
        """安全的串口数据写入，记录发送的指令类型"""
        try:
            if self.ser and self.ser.is_open:
                # 清空发送和接收缓冲区
                self.ser.flushOutput()
                self.ser.flushInput()
                # 发送指令
                self.ser.write(cmd)
                # 记录发送信息
                self.last_sent_sensor = sensor_type
                self.sent_cmd_timestamp[sensor_type] = rospy.Time.now().to_sec()
                # rospy.loginfo(f"发送{sensor_type}指令: {[hex(b) for b in cmd]}")
                return True
            self.init_serial()
            return False
        except Exception as e:
            rospy.logwarn(f"串口写入失败: {str(e)}")
            self.init_serial()
            return False

    def send_query_cmd(self, event=None):
        """轮询发送3条查询指令"""
        if not self.working or not self.ser or not self.ser.is_open:
            return
        
        current_time = rospy.Time.now().to_sec()
        
        # 根据状态发送不同指令
        if self.query_state == 0:
            # 发送加速度查询指令
            config = self.sensor_configs['acc']
            if current_time - config['last_sent'] > self.query_interval:
                self.safe_serial_write(config['cmd'], 'acc')
                config['last_sent'] = current_time
            self.query_state = 1
            
        elif self.query_state == 1:
            # 发送角速度查询指令
            config = self.sensor_configs['gyro']
            if current_time - config['last_sent'] > self.query_interval:
                self.safe_serial_write(config['cmd'], 'gyro')
                config['last_sent'] = current_time
            self.query_state = 2
            
        else:
            # 发送RPY查询指令
            config = self.sensor_configs['rpy']
            if current_time - config['last_sent'] > self.query_interval:
                self.safe_serial_write(config['cmd'], 'rpy')
                config['last_sent'] = current_time
            self.query_state = 0

    def extract_complete_frame(self):
        """提取完整的11字节帧，确保以0x50开头"""
        # 查找帧头（0x50）
        frame_start = -1
        for i in range(len(self.data_buffer)):
            if self.data_buffer[i] == 0x50:
                frame_start = i
                break
        
        if frame_start == -1:
            # 未找到帧头，清空缓冲区
            self.data_buffer.clear()
            return None
        
        # 移除非帧头数据
        if frame_start > 0:
            self.data_buffer = self.data_buffer[frame_start:]
        
        # 检查是否有完整的帧
        if len(self.data_buffer) < 11:
            return None
        
        # 提取完整帧
        frame = self.data_buffer[:11]
        # 移除已处理的帧
        self.data_buffer = self.data_buffer[11:]
        
        # 验证帧结构
        if len(frame) != 11 or frame[0] != 0x50 or frame[1] != 0x03 or frame[2] != 0x06:
            rospy.logwarn(f"无效帧结构: {frame.hex()}")
            return None
        
        return frame

    def verify_crc(self, frame):
        """验证CRC"""
        if len(frame) < 11:
            return False
        
        data_part = frame[:-2]
        recv_crc = frame[-2:]
        calc_crc = self.calculate_crc(data_part)
        
        if recv_crc == calc_crc:
            return True
        else:
            rospy.logwarn(f"CRC校验失败 - 接收: {recv_crc.hex()}, 计算: {calc_crc.hex()}, 帧: {frame.hex()}")
            return False

    def parse_acc_frame(self, frame):
        """解析加速度帧"""
        try:
            if not self.verify_crc(frame):
                return False
            
            # 提取原始数据
            ax_h = frame[3]
            ax_l = frame[4]
            ay_h = frame[5]
            ay_l = frame[6]
            az_h = frame[7]
            az_l = frame[8]
            
            # 组合为16位有符号整数
            ax_raw = np.int16((ax_h << 8) | ax_l)
            ay_raw = np.int16((ay_h << 8) | ay_l)
            az_raw = np.int16((az_h << 8) | az_l)
            
            # 转换为实际加速度值 (m/s²)
            g = 9.8
            ax = ax_raw / 32768.0 * 16 * g
            ay = ay_raw / 32768.0 * 16 * g
            az = az_raw / 32768.0 * 16 * g
            
            self.acc_data = {
                'x': ax, 
                'y': ay, 
                'z': az,
                'updated': True,
                'timestamp': rospy.Time.now().to_sec()
            }
            
            # rospy.loginfo(f"【加速度】X: {ax:.2f}, Y: {ay:.2f}, Z: {az:.2f} m/s² (原始: {ax_raw}, {ay_raw}, {az_raw})")
            return True
        except Exception as e:
            rospy.logerr(f"解析加速度失败: {str(e)}, 帧: {frame.hex()}")
            return False

    def parse_gyro_frame(self, frame):
        """解析角速度帧"""
        try:
            if not self.verify_crc(frame):
                return False
            
            # 提取原始数据
            gx_h = frame[3]
            gx_l = frame[4]
            gy_h = frame[5]
            gy_l = frame[6]
            gz_h = frame[7]
            gz_l = frame[8]
            
            # 组合为16位有符号整数
            gx_raw = np.int16((gx_h << 8) | gx_l)
            gy_raw = np.int16((gy_h << 8) | gy_l)
            gz_raw = np.int16((gz_h << 8) | gz_l)
            
            # 转换为实际角速度值 (rad/s)
            gx = gx_raw / 32768.0 * 2000   #* np.pi / 180.0
            gy = gy_raw / 32768.0 * 2000   #* np.pi / 180.0
            gz = gz_raw / 32768.0 * 2000   #* np.pi / 180.0
            
            self.gyro_data = {
                'x': gx, 
                'y': gy, 
                'z': gz,
                'updated': True,
                'timestamp': rospy.Time.now().to_sec()
            }
            
            # rospy.loginfo(f"【角速度】X: {gx:.2f}, Y: {gy:.2f}, Z: {gz:.2f} rad/s (原始: {gx_raw}, {gy_raw}, {gz_raw})")
            return True
        except Exception as e:
            rospy.logerr(f"解析角速度失败: {str(e)}, 帧: {frame.hex()}")
            return False

    def parse_rpy_frame(self, frame):
        """解析RPY帧"""
        try:
            if not self.verify_crc(frame):
                return False
            
            # 提取原始数据
            roll_h = frame[3]
            roll_l = frame[4]
            pitch_h = frame[5]
            pitch_l = frame[6]
            yaw_h = frame[7]
            yaw_l = frame[8]
            
            # 组合为16位有符号整数
            roll_raw = np.int16((roll_h << 8) | roll_l)
            pitch_raw = np.int16((pitch_h << 8) | pitch_l)
            yaw_raw = np.int16((yaw_h << 8) | yaw_l)
            
            # 转换为角度值
            roll = roll_raw / 32768.0 * 180.0
            pitch = pitch_raw / 32768.0 * 180.0
            yaw = yaw_raw / 32768.0 * 180.0
            
            self.rpy_data = {
                'roll': roll, 
                'pitch': pitch, 
                'yaw': yaw,
                'updated': True,
                'timestamp': rospy.Time.now().to_sec()
            }
            
            # rospy.loginfo(f"【姿态角】Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}° (原始: {roll_raw}, {pitch_raw}, {yaw_raw})")
            
            # 发布完整IMU数据
            self.publish_imu_data()
            return True
        except Exception as e:
            rospy.logerr(f"解析RPY失败: {str(e)}, 帧: {frame.hex()}")
            return False

    def publish_imu_data(self):
        """发布完整IMU数据"""
        try:
            # 检查数据有效性（1秒内更新）
            current_time = rospy.Time.now().to_sec()
            if (current_time - self.acc_data['timestamp'] > 1.0 or
                current_time - self.gyro_data['timestamp'] > 1.0 or
                current_time - self.rpy_data['timestamp'] > 1.0):
                rospy.loginfo("部分数据过期，跳过发布")
                return

            # 1. 发布自定义INSPVAE消息
            inspvae_msg = INSPVAE()
            inspvae_msg.header = Header(stamp=rospy.Time.now(), frame_id='imu_link')
            inspvae_msg.roll = self.rpy_data['roll']
            inspvae_msg.pitch = self.rpy_data['pitch']
            inspvae_msg.yaw = self.rpy_data['yaw'] % 360
            self.imu_pub.publish(inspvae_msg)

            # 2. 发布标准sensor_msgs/Imu消息
            imu_msg = Imu()
            imu_msg.header = Header(stamp=rospy.Time.now(), frame_id='imu_link')
            
            # 欧拉角转四元数
            roll_rad = np.radians(self.rpy_data['roll'])
            pitch_rad = np.radians(self.rpy_data['pitch'])
            yaw_rad = np.radians(self.rpy_data['yaw'])
            
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
            
            # 设置角速度
            imu_msg.angular_velocity.x = self.gyro_data['x']
            imu_msg.angular_velocity.y = self.gyro_data['y']
            imu_msg.angular_velocity.z = self.gyro_data['z']
            
            # 设置线加速度
            imu_msg.linear_acceleration.x = self.acc_data['x']
            imu_msg.linear_acceleration.y = self.acc_data['y']
            imu_msg.linear_acceleration.z = self.acc_data['z']
            
            # 设置协方差
            imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            imu_msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            imu_msg.linear_acceleration_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]
            
            self.imu_std_pub.publish(imu_msg)
            
        except Exception as e:
            rospy.logerr(f"发布IMU数据失败: {str(e)}")

    def process_serial_data(self, event=None):
        """处理串口数据"""
        if not self.working or not self.ser or not self.ser.is_open:
            return
        
        try:
            # 读取所有可用数据
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                if data:
                    self.data_buffer += data
                    # rospy.loginfo(f"接收原始数据: {data.hex()} (缓冲区长度: {len(self.data_buffer)})")

            # 循环提取并处理完整帧
            while True:
                frame = self.extract_complete_frame()
                if frame is None:
                    break
                
                # 根据最后发送的指令类型解析对应数据
                if self.last_sent_sensor == 'acc':
                    self.parse_acc_frame(frame)
                elif self.last_sent_sensor == 'gyro':
                    self.parse_gyro_frame(frame)
                elif self.last_sent_sensor == 'rpy':
                    self.parse_rpy_frame(frame)
                # else:
                    # rospy.logwarn(f"未知的返回数据类型: {self.last_sent_sensor}")
                
                # 重置最后发送的指令类型
                self.last_sent_sensor = None
                
            # 限制缓冲区最大长度
            if len(self.data_buffer) > 1024:
                rospy.logwarn("缓冲区溢出，清空数据")
                self.data_buffer.clear()
                
        except Exception as e:
            # rospy.logerr(f"处理串口数据异常: {str(e)}")
            self.data_buffer.clear()
    def send_gyro_calib_cmd(self, enable):
        """发送陀螺仪自动校准指令序列"""
        if not self.ser or not self.ser.is_open:
            rospy.logerr("串口未连接，无法发送校准指令")
            return False
        
        try:
            # 1. 发送解锁指令
            self.safe_serial_write(self.calib_cmds['unlock'], 'calib_unlock')
            rospy.sleep(0.1)  # 延时100ms
            
            # 2. 发送开启/关闭校准指令
            cmd_type = 'enable' if enable else 'disable'
            self.safe_serial_write(self.calib_cmds[cmd_type], f'calib_{cmd_type}')
            rospy.sleep(2.0)  # 延时2s
            
            # 3. 发送保存指令
            self.safe_serial_write(self.calib_cmds['save'], 'calib_save')
            
            rospy.loginfo(f"陀螺仪自动校准{'开启' if enable else '关闭'}指令发送完成")
            return True
        except Exception as e:
            rospy.logerr(f"发送校准指令失败: {str(e)}")
            return False

    def start(self):
        """启动IMU轮询"""
        if not self.working:
            self.working = True
            # 启动指令发送定时器 (10Hz)
            self.query_timer = rospy.Timer(rospy.Duration(0.1), self.send_query_cmd)
            # 启动数据处理定时器 (50Hz)
            self.process_timer = rospy.Timer(rospy.Duration(0.02), self.process_serial_data)
            rospy.loginfo("IMU轮询已启动 - 分3条指令查询加速度、角速度、RPY")

    def stop(self):
        """停止IMU轮询"""
        if self.working:
            self.working = False
            # 停止定时器
            if self.query_timer:
                self.query_timer.shutdown()
            if hasattr(self, 'process_timer') and self.process_timer:
                self.process_timer.shutdown()
            # 清空缓冲区
            self.data_buffer.clear()
            # 关闭串口
            self.close_serial()
            rospy.loginfo("IMU轮询已停止")

    def close_serial(self):
        """安全关闭串口"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                rospy.loginfo(f"串口 {self.port} 已关闭")
            self.ser = None
        except Exception as e:
            rospy.logerr(f"关闭串口失败: {str(e)}")

    def cleanup(self):
        """清理资源"""
        rospy.loginfo("执行IMU节点清理...")
        self.working = False
        # 停止所有定时器
        if self.query_timer:
            self.query_timer.shutdown()
        if hasattr(self, 'process_timer') and self.process_timer:
            self.process_timer.shutdown()
        # 清空缓冲区
        self.data_buffer.clear()
        # 关闭串口
        self.close_serial()
        rospy.loginfo("IMU节点清理完成")

    def handle_start(self, req):
        """服务回调：启动"""
        self.start()
        return std_srvs.srv.TriggerResponse(success=True, message="IMU轮询已启动")

    def handle_stop(self, req):
        """服务回调：停止"""
        self.stop()
        return std_srvs.srv.TriggerResponse(success=True, message="IMU轮询已停止")
    def handle_gyro_calib(self, req):
        """服务回调：设置陀螺仪自动校准状态"""
        # 执行校准指令发送
        success = self.send_gyro_calib_cmd(req.enable)
        if success:
            msg = f"陀螺仪自动校准已{'开启' if req.enable else '关闭'}"
        else:
            msg = f"陀螺仪自动校准{'开启' if req.enable else '关闭'}失败"
        return SetGyroCalibResponse(success=success, message=msg)

    def run(self):
        """主循环"""
        rospy.loginfo("IMU节点主循环启动")
        while not rospy.is_shutdown():
            if self.working:
                self.process_serial_data()
            rospy.sleep(0.005)

if __name__ == '__main__':
    try:
        # 设置日志级别
        rospy.set_param('/rosconsole/logger_levels/rosout', 'INFO')
        
        imu_node = IMUParser()
        imu_node.start()
        imu_node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("IMU节点被中断")
        if 'imu_node' in locals():
            imu_node.cleanup()
    except Exception as e:
        rospy.logfatal(f"IMU节点启动失败: {str(e)}")
        if 'imu_node' in locals():
            imu_node.cleanup()