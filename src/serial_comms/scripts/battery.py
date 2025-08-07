#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import Float32, Float32MultiArray

class BatteryMonitor:
    def __init__(self, port, baudrate=115200): #待确认，手册默认版本9600
        # 配置串口
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            rospy.loginfo(f"Connected to serial port: {port}")
        except serial.SerialException as e:
            rospy.logerr(f"串口连接失败: {e}")
            rospy.signal_shutdown("串口错误")
            return

        # 创建ROS发布器
        self.battery_pub = rospy.Publisher('/remaining_battery_percentage', Float32, queue_size=10)
        self.temp_pub = rospy.Publisher('/battery_temperatures', Float32MultiArray, queue_size=10)

        # 基本指令0x03 (DDA50300FFFD77)
        self.REQUEST_FRAME = bytes.fromhex('DD A5 03 00 FF FD 77')  # 硬件版本请求帧

    def process_response(self, data_start, length):
        """解析响应数据帧"""
        # 数据段索引: 0-19为指定字段
        ntc_count = data_start[22]  # NTC传感器数量（22字节位置）
        temperatures = []

        # 解析NTC温度值
        ntc_start_index = 23  # NTC数据起始位置
        for i in range(ntc_count):
            raw_index = ntc_start_index + i * 2
            # 合并两个字节为一个整数（高位在前）
            raw_value = (data_start[raw_index] << 8) | data_start[raw_index + 1]
            # 转换为摄氏温度：原始值=2731+温度*10
            temp_c = (raw_value - 2731) / 10.0
            temperatures.append(temp_c)

        # 发布温度数据
        temp_msg = Float32MultiArray(data=temperatures)
        self.temp_pub.publish(temp_msg)

        # 获取电量百分比（19字节位置）
        battery_remain = data_start[19]
        rospy.loginfo(f"电池电量: {battery_remain}% | 温度: {temperatures}°C")

        # 发布电量数据
        battery_level = Float32()
        battery_level.data = battery_remain
        self.battery_pub.publish(battery_level)

    def read_battery_data(self):
        """读取并处理数据帧"""
        frame = self.ser.read(1)  # 读取第一个字节
        if frame != b'\xdd':
            return False

        func = self.ser.read(1)  # 命令码
        status = self.ser.read(1)  # 状态码
        if status != b'\x00':
            return False  # 忽略故障状态

        data_length = ord(self.ser.read(1))  # 数据长度字节（单字节）
        data = self.ser.read(data_length)   # 主数据块
        
        checksum = self.ser.read(2)         # 校验位（2字节）
        end_marker = self.ser.read(1)       # 结束符
        
        if end_marker == b'\x77':  # 验证结束符
            self.process_response(data, data_length)
            return True
        return False

    def run(self):
        rate = rospy.Rate(0.1)  # 0.1Hz刷新率
        while not rospy.is_shutdown():
            self.ser.write(self.REQUEST_FRAME)
            rospy.sleep(0.05)  # 等待设备响应
            
            start_count = self.ser.in_waiting
            if start_count > 0:
                try:
                    self.read_battery_data()
                except Exception as e:
                    rospy.logwarn(f"数据处理异常: {e}")
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('battery_monitor')
    
    # 从参数服务器获取串口路径（与IMU同个接口）
    port = rospy.get_param('~serial_port', '/dev/IMU')
    
    monitor = BatteryMonitor(port)
    try:
        monitor.run()
    except rospy.ROSInterruptException:
        pass
