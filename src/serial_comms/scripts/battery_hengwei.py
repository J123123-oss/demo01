#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
import struct
import csv
import os
from datetime import datetime
from std_msgs.msg import Float32MultiArray

class Sensors_485:
    def __init__(self):
        # 初始化ROS 1节点
        rospy.init_node('Sensors_485_node', anonymous=True)
        
        # ========== CSV配置 ==========
        # CSV保存目录（可通过ROS参数配置）
        self.csv_save_dir = rospy.get_param('~csv_save_dir', '/home/orangepi/demo01/battery_logs')
        # 创建保存目录（如果不存在）
        if not os.path.exists(self.csv_save_dir):
            os.makedirs(self.csv_save_dir)
            rospy.loginfo(f"Created CSV save directory: {self.csv_save_dir}")
        # CSV文件对象和写入器
        self.csv_file = None
        self.csv_writer = None
        self.last_csv_date = None  # 记录上次创建文件的日期（按日切分）
        
        # ========== 串口参数配置 ==========
        self.port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~baudrate', 4800)
        self.battery_ID = 0x01  # 电池设备地址
        
        # 电池查询命令
        self.battery_base_cmd = bytes.fromhex(f"01 04 00 00 00 03 B0 0B")
        self.battery_temp_cmd = bytes.fromhex(f"01 03 00 50 00 01 84 1B")
        
        # 串口相关变量
        self.ser = None
        self.reconnect_interval = 1.0  # 重连间隔
        self.last_reconnect_time = 0.0
        self.buffer = bytearray()  # 串口接收缓冲区

        # 数据更新监控参数
        self.last_data_time = rospy.get_time()  # 上次数据更新时间
        self.data_timeout = 3.0  # 数据超时时间(秒)

        # 轮询状态管理
        self.battery_temp_polling_interval = 1.0    # 温度查询间隔
        self.last_temp_poll_time = 0.0
        self.battery_base_polling_interval = 2.0   # 基础参数查询间隔
        self.last_base_poll_time = 0.0

        # 初始化串口
        try:
            if not self.init_serial():
                rospy.logwarn("Failed to initialize serial connection. Will retry in main loop.")
        except Exception as e:
            rospy.logerr("Exception occurred while initializing serial connection: %s" % str(e))
            rospy.logwarn("Will retry in main loop.")

        # 发布器配置（ROS 1）
        self.battery_pub = rospy.Publisher('/battery_data', Float32MultiArray, queue_size=1)
        
        # 定时器（ROS 1）
        # 轮询定时器（5ms周期）
        self.polling_timer = rospy.Timer(rospy.Duration(0.005), self.polling_callback)
        # 主循环定时器（5ms周期）
        self.main_loop_timer = rospy.Timer(rospy.Duration(0.005), self.main_loop)
        
        # 注册关闭回调
        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        """ROS节点关闭时的清理函数"""
        # 关闭CSV文件
        if self.csv_file:
            self.csv_file.close()
            rospy.loginfo("CSV file closed successfully.")
        # 关闭串口
        if self.ser and self.ser.is_open:
            self.ser.close()
            rospy.loginfo("Serial connection closed.")
        rospy.loginfo("Node shutdown completed.")

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
            rospy.loginfo("Successfully connected to %s" % self.port)
            return True
        except Exception as e:
            rospy.logerr("Serial connection failed: %s" % str(e))
            return False

    def safe_serial_write(self, data):
        """安全的串口数据写入"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(data)
                return True
            return False
        except Exception as e:
            rospy.logwarn("Serial write failed: %s" % str(e))
            self.init_serial()  # 尝试重新连接
            return False

    def polling_callback(self, event):
        """统一的轮询回调函数（ROS 1定时器回调）"""
        current_time = rospy.get_time()
        
        # 轮询电池温度
        if current_time - self.last_temp_poll_time >= self.battery_temp_polling_interval:
            self.safe_serial_write(self.battery_temp_cmd)
            rospy.logdebug("Sent battery temp query command: %s" % ' '.join(format(x, '02x') for x in self.battery_temp_cmd))
            self.last_temp_poll_time = current_time
        
        # 轮询电池基础参数
        if current_time - self.last_base_poll_time >= self.battery_base_polling_interval:
            self.safe_serial_write(self.battery_base_cmd)
            rospy.logdebug("Sent battery base query command: %s" % ' '.join(format(x, '02x') for x in self.battery_base_cmd))
            self.last_base_poll_time = current_time

    def create_csv_file(self):
        """创建/打开CSV文件（按日期命名，避免覆盖）"""
        try:
            # 获取当前日期（用于文件命名）
            current_date = datetime.now().strftime("%Y-%m-%d")
            
            # 如果日期变化或文件未创建，创建新文件
            if current_date != self.last_csv_date or self.csv_file is None:
                # 关闭旧文件
                if self.csv_file:
                    self.csv_file.close()
                
                # 生成文件名：battery_data_YYYY-MM-DD.csv
                filename = f"battery_data_{current_date}.csv"
                self.csv_filepath = os.path.join(self.csv_save_dir, filename)
                
                # 以追加模式打开文件（避免覆盖历史数据）
                self.csv_file = open(self.csv_filepath, 'a', newline='', encoding='utf-8')
                self.csv_writer = csv.writer(self.csv_file)
                self.last_csv_date = current_date
                
                # 如果是新文件，写入表头
                if os.path.getsize(self.csv_filepath) == 0:
                    header = [
                        'timestamp',          # 时间戳（精确到毫秒）
                        'capacity_percent',   # 剩余容量百分比
                        'total_current_A',    # 总电流（A）
                        'total_voltage_V',    # 总电压（V）
                        'temperature_C'       # 温度（℃）
                    ]
                    self.csv_writer.writerow(header)
                    rospy.loginfo(f"Created new CSV file: {self.csv_filepath}")
                else:
                    rospy.loginfo(f"Opened existing CSV file: {self.csv_filepath}")
        except Exception as e:
            rospy.logerr(f"Failed to create CSV file: {str(e)}")

    def save_data_to_csv(self, battery_data):
        """将电池数据保存到CSV（带时间戳）"""
        try:
            # 确保CSV文件已创建
            self.create_csv_file()
            
            # 生成精确到毫秒的时间戳
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # 去掉微秒后三位，保留毫秒
            
            # 整理数据（保留指定精度）
            row_data = [
                timestamp,
                round(battery_data.get('capacity_percent', 0.0), 2),
                round(battery_data.get('total_current', 0.0), 2),
                round(battery_data.get('total_voltage', 0.0), 2),
                round(battery_data.get('temperature', 0.0), 1)
            ]
            
            # 写入CSV并立即刷新缓冲区（避免数据丢失）
            self.csv_writer.writerow(row_data)
            self.csv_file.flush()
            
            # 调试日志（可选）
            rospy.logdebug(f"Data saved to CSV: {row_data}")
            
        except Exception as e:
            rospy.logerr(f"Failed to save data to CSV: {str(e)}")

    def parse_battery_base_response(self, data):
        """解析电池基础参数返回数据"""
        rospy.loginfo("Received battery base data: %s" % ' '.join(format(x, '02x') for x in data))
        
        if len(data) < 9 or data[0] != self.battery_ID or data[1] != 0x04:
            rospy.logdebug("Invalid battery base frame: length=%d, addr=0x%02x, func_code=0x%02x" % 
                          (len(data), data[0] if len(data) > 0 else 0, data[1] if len(data) > 1 else 0))
            return None
        
        if data[2] != 0x06:
            rospy.logdebug("Battery base data length mismatch: expected=6, actual=%d" % data[2])
            return None
        
        expected_frame_length = 11
        if len(data) != expected_frame_length:
            rospy.logdebug("Battery base frame length mismatch: expected=%d, actual=%d" % (expected_frame_length, len(data)))
            return None
        
        # CRC校验
        recv_crc = data[-2:]
        calc_crc = self.calculate_modbus_crc(data[:-2])
        if recv_crc != calc_crc:
            rospy.logwarn("Battery base: CRC check failed")
            return None
        
        # 解析参数
        battery_data = {}
        capacity_raw = (data[3] << 8) | data[4]
        battery_data['capacity_percent'] = capacity_raw * 0.01
        
        current_raw = (data[5] << 8) | data[6]
        if current_raw > 0x7FFF:
            current_raw -= 0x10000
        battery_data['total_current'] = current_raw * 0.01
        
        voltage_raw = (data[7] << 8) | data[8]
        battery_data['total_voltage'] = voltage_raw * 0.01
        if battery_data['total_voltage'] < 10.0 or battery_data['total_voltage'] > 55.0:
            return None
        
        rospy.loginfo(f"Parsed battery base: {battery_data['capacity_percent']:.2f}% | "
                      f"{battery_data['total_current']:.2f}A | {battery_data['total_voltage']:.2f}V")
        return battery_data

    def parse_battery_temp_response(self, data):
        """解析电池温度返回数据（适配7字节帧长度）"""
        rospy.loginfo("=== Battery Temp Response ===")
        rospy.loginfo(f"Raw temp data: {''.join(format(x, '02x') for x in data).strip()}")
        
        # 基础格式校验（7字节长度）
        if len(data) < 7 or data[0] != self.battery_ID or data[1] != 0x03:
            rospy.logdebug("Invalid battery temp frame: length=%d, addr=0x%02x, func_code=0x%02x" % 
                          (len(data), data[0] if len(data) > 0 else 0, data[1] if len(data) > 1 else 0))
            return None
        
        # 数据长度校验
        if data[2] != 0x02:
            rospy.logdebug("Battery temp data length mismatch: expected=2, actual=%d" % data[2])
            return None
        
        # 保留7字节帧长度配置
        expected_frame_length = 7
        if len(data) != expected_frame_length:
            rospy.logdebug("Battery temp frame length mismatch: expected=%d, actual=%d" % (expected_frame_length, len(data)))
            return None
        
        # 提取核心温度数据（只解析字节3-4）
        temp_raw = (data[3] << 8) | data[4]
        
        # 处理负温度补码
        if temp_raw & 0x8000:
            temp_raw = temp_raw - 0x10000
        
        # 温度转换
        temperature = temp_raw * 0.1
        
        # 打印解析结果
        rospy.loginfo(f"Temp raw value (16bit): 0x{temp_raw:04x} ({temp_raw})")
        rospy.loginfo(f"Parsed temperature: {temperature:.1f}℃")
        rospy.loginfo("=============================\n")
        
        return {'temperature': temperature}

    def main_loop(self, event):
        """主循环：处理串口数据，解析电池基础参数和温度帧（ROS 1定时器回调）"""
        try:
            # 检查数据超时
            current_time = rospy.get_time()
            elapsed_time = current_time - self.last_data_time
            if elapsed_time > self.data_timeout:
                rospy.logwarn("Data timeout (%.2f seconds since last data), resetting connection" % elapsed_time)
                self.last_data_time = current_time
                self.init_serial()
            
            # 读取串口数据
            if self.ser and self.ser.is_open:
                # 读取所有可用数据
                bytes_available = self.ser.in_waiting
                if bytes_available > 0:
                    rospy.logdebug("Bytes available: %d" % bytes_available)
                    data = self.ser.read(min(bytes_available, 1024))
                    if data:
                        self.buffer += data
                        rospy.logdebug("Buffer size after read: %d" % len(self.buffer))

                # 处理完整帧
                processed_frames = 0
                while len(self.buffer) >= 5 and processed_frames < 10:
                    # 查找帧头 - 匹配电池设备地址
                    header_pos = -1
                    for i in range(len(self.buffer)):
                        if self.buffer[i] == self.battery_ID:
                            header_pos = i
                            break
                    
                    if header_pos == -1:
                        rospy.logdebug("No frame header found, clearing buffer")
                        self.buffer.clear()
                        break
                    
                    # 丢弃帧头前的无效数据
                    if header_pos > 0:
                        rospy.logdebug("Discarding %d bytes before frame header" % header_pos)
                        del self.buffer[:header_pos]
                    
                    # 检查是否有足够的数据确定帧长度
                    if len(self.buffer) < 3:
                        rospy.logdebug("Insufficient data to determine frame length: %d bytes" % len(self.buffer))
                        break
                    
                    # 根据功能码处理不同电池数据
                    func_code = self.buffer[1]
                    parsed_data = None
                    
                    if func_code == 0x04:  # 电池基础参数响应
                        expected_frame_length = 11
                        if len(self.buffer) >= expected_frame_length:
                            frame = self.buffer[:expected_frame_length]
                            del self.buffer[:expected_frame_length]
                            parsed_data = self.parse_battery_base_response(frame)
                            if parsed_data:
                                # 合并温度数据（如果有）
                                if hasattr(self, 'latest_temp'):
                                    parsed_data['temperature'] = self.latest_temp
                                # 发布完整电池数据
                                self.publish_battery_data(parsed_data)
                                self.last_data_time = current_time
                    
                    elif func_code == 0x03:  # 电池温度响应（7字节）
                        expected_frame_length = 7
                        if len(self.buffer) >= expected_frame_length:
                            frame = self.buffer[:expected_frame_length]
                            del self.buffer[:expected_frame_length]
                            parsed_data = self.parse_battery_temp_response(frame)
                            if parsed_data:
                                # 保存最新温度值
                                self.latest_temp = parsed_data['temperature']
                                self.last_data_time = current_time
                    
                    else:
                        # 未知功能码，跳过一个字节
                        rospy.logdebug("Unknown function code: 0x%02x, skipping byte" % func_code)
                        del self.buffer[0]
                        continue
                    
                    processed_frames += 1
            
            # 检查串口连接状态，自动重连
            if not self.ser or not self.ser.is_open:
                if current_time - self.last_reconnect_time > self.reconnect_interval:
                    rospy.loginfo("Attempting to reconnect to serial port")
                    if self.init_serial():
                        self.last_reconnect_time = current_time
        except Exception as e:
            rospy.logerr("Main loop error: %s" % str(e))
            import traceback
            rospy.logerr(traceback.format_exc())
            self.init_serial()

    def publish_battery_data(self, battery_data):
        """发布完整电池数据（包含温度）+ 保存到CSV"""
        # 1. 发布ROS 1话题
        msg = Float32MultiArray()
        # 数据顺序：[剩余容量百分比, 总电流, 总电压, 温度（如果有）]
        msg.data = [
            battery_data['capacity_percent'],
            battery_data['total_current'],
            battery_data['total_voltage'],
            battery_data.get('temperature', 0.0)  # 温度默认0.0
        ]
        self.battery_pub.publish(msg)
        
        # 2. 保存到CSV文件
        self.save_data_to_csv(battery_data)

    @staticmethod
    def calculate_modbus_crc(data):
        """计算Modbus CRC校验"""
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

def main():
    try:
        # 创建节点实例
        node = Sensors_485()
        # 保持节点运行
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted by user")
    except Exception as e:
        rospy.logerr(f"Node error: {str(e)}")
        import traceback
        rospy.logerr(traceback.format_exc())

if __name__ == '__main__':
    main()