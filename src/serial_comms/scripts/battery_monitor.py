#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
from std_msgs.msg import Float32
from serial_comms.msg import BatteryStatus
import time  # 用于时间戳

class BatteryMonitor:
    def __init__(self, port, baudrate=9600):
        # ===================== 新增：日志文件配置 =====================
        self.log_file = open("/home/orangepi/demo01/serial_log.txt", "w", encoding="utf-8")  # 改成你的用户名路径
        self.log_file.write("=== Serial Communication Log ===\n")
        self.log_file.flush()
        # ==============================================================

        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            rospy.loginfo(f"Connected to serial port: {port}")
        except serial.SerialException as e:
            rospy.logerr(f"串口连接失败: {e}")
            rospy.signal_shutdown("串口错误")
            return
        
        self.status_pub = rospy.Publisher('/battery_status', BatteryStatus, queue_size=10)
        self.REQUEST_BASIC_FRAME = bytes.fromhex('DD A5 03 00 FF FD 77')

    # ===================== 新增：日志记录函数 =====================
    def log_serial(self, direction, data):
        """记录串口收发数据"""
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        hex_str = data.hex(' ')  # 空格分隔更易读
        log_line = f"[{timestamp}] {direction}: {hex_str}\n"
        
        # 打印到终端
        rospy.loginfo(log_line.strip())
        
        # 写入文件
        self.log_file.write(log_line)
        self.log_file.flush()  # 立即写入，不缓存
    # ==============================================================

    def parse_date(self, raw_date):
        value = (raw_date[0] << 8) | raw_date[1]
        day = value & 0x1F
        month = (value >> 5) & 0x0F
        year = 2000 + (value >> 9)
        return year, month, day

    def parse_current(self, data_bytes):
        value = (data_bytes[0] << 8) | data_bytes[1]
        if value >= 0x8000:
            return (value - 65536) * 0.01
        return value * 0.01

    def process_response(self, data):
        status_msg = BatteryStatus()
        try:
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
            version_major = data[18] >> 4
            version_minor = data[18] & 0x0F
            status_msg.software_version = f"{version_major}.{version_minor}"
            status_msg.batttery_remaining = data[19]

            ntc_count = data[22]
            for i in range(ntc_count):
                index = 23 + i * 2
                raw_temp = (data[index] << 8) | data[index + 1]
                temperature = (raw_temp - 2731) / 10.0
                temperature = round(temperature, 1)
                status_msg.temperatures.append(temperature)

            self.status_pub.publish(status_msg)
            rospy.loginfo("Battery status published successfully")
            
        except IndexError as e:
            rospy.logerr(f"数据解析错误: 响应长度不足 ({len(data)} bytes)")
        except Exception as e:
            rospy.logerr(f"处理响应时出错: {e}")

    def read_battery_data(self):
        # 读取帧头
        frame = self.ser.read(1)
        # ===================== 接收日志 =====================
        self.log_serial("RECV", frame)

        if frame != b'\xdd':
            return False
        
        command = self.ser.read(1)
        self.log_serial("RECV", command)

        if command != b'\x03':
            return False
            
        status = self.ser.read(1)
        self.log_serial("RECV", status)

        if status != b'\x00':
            rospy.logwarn(f"设备返回错误状态: 0x{status.hex()}")
            return False
        
        data_length_byte = self.ser.read(1)
        self.log_serial("RECV", data_length_byte)
        data_length = data_length_byte[0]
        
        # 读取数据块
        data = self.ser.read(data_length)
        self.log_serial("RECV", data)
        
        # 校验和 + 帧尾
        checksum = self.ser.read(2)
        end_marker = self.ser.read(1)
        
        self.log_serial("RECV", checksum + end_marker)
        
        if end_marker == b'\x77':
            self.process_response(data)
            return True
        return False

    def run(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            try:
                # 发送指令 + 日志
                # self.log_serial("SEND", self.REQUEST_BASIC_FRAME)
                self.ser.write(self.REQUEST_BASIC_FRAME)
                
                rospy.sleep(0.05)
                
                if self.ser.in_waiting > 0:
                    self.read_battery_data()
                    
            except serial.SerialException as e:
                rospy.logerr(f"串口通信错误: {e}")
                rospy.signal_shutdown("串口故障")
            except Exception as e:
                rospy.logerr(f"运行时错误: {e}")
                
            rate.sleep()

    def __del__(self):
        # 关闭日志文件
        if hasattr(self, 'log_file'):
            self.log_file.close()

if __name__ == '__main__':
    rospy.init_node('battery_monitor')
    port = rospy.get_param('~serial_port', '/dev/Battery-Relay')
    monitor = BatteryMonitor(port)
    
    try:
        monitor.run()
    except rospy.ROSInterruptException:
        pass