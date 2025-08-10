#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import Float32
from serial_comms.msg import BatteryStatus  #导入自定义类型

class BatteryMonitor:
    def __init__(self, port, baudrate=115200):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            rospy.loginfo(f"Connected to serial port: {port}")
        except serial.SerialException as e:
            rospy.logerr(f"串口连接失败: {e}")
            rospy.signal_shutdown("串口错误")
            return
        
        # 创建发布器
        self.status_pub = rospy.Publisher('/battery_status', BatteryStatus, queue_size=10)
        
        # 03指令请求帧 (DDA50300FFFD77)
        self.REQUEST_BASIC_FRAME = bytes.fromhex('DD A5 03 00 FF FD 77')

    def parse_date(self, raw_date):
        """解析生产日期 (2字节数据)"""
        value = (raw_date[0] << 8) | raw_date[1]
        day = value & 0x1F
        month = (value >> 5) & 0x0F
        year = 2000 + (value >> 9)
        return year, month, day

    def parse_current(self, data_bytes):
        """解析电流值 (2字节数据)"""
        value = (data_bytes[0] << 8) | data_bytes[1]
        if value >= 0x8000:  # 最高位为1表示放电(负电流)
            return (value - 65536) * 0.01  # 转换为安培(A)
        return value * 0.01  # 转换为安培(A)

    def process_response(self, data):
        """解析响应数据帧并发布完整状态"""
        # 创建自定义消息对象
        status_msg = BatteryStatus()
        
        try:
            # 1. 总电压 (2字节, 单位10mV)
            status_msg.total_voltage = ((data[0] << 8) | data[1]) * 0.01  # 转换为伏特(V)
            
            # 2. 电流 (2字节, 带符号处理)
            status_msg.current = self.parse_current(data[2:4])
            
            # 3. 容量信息 (4字节)
            status_msg.remaining_capacity = ((data[4] << 8) | data[5]) * 0.01  # 转换为安时(Ah)
            status_msg.nominal_capacity = ((data[6] << 8) | data[7]) * 0.01    # 转换为安时(Ah)
            
            # 4. 循环次数 (2字节)
            status_msg.cycle_count = (data[8] << 8) | data[9]
            
            # 5. 生产日期 (2字节)
            year, month, day = self.parse_date(data[10:12])
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
            print("status_msg.batttery_remaining:",status_msg.batttery_remaining)
            
            # 10. 温度数据解析
            ntc_count = data[22]  # 温度传感器数量
            for i in range(ntc_count):
                index = 23 + i * 2
                raw_temp = (data[index] << 8) | data[index + 1]
                # 转换为摄氏度: T = (raw_value - 2731) / 10.0
                temperature = (raw_temp - 2731) / 10.0
                #保留一位小数
                temperature = round(temperature, 1)  
                status_msg.temperatures.append(temperature)
            
            # 发布完整状态消息
            self.status_pub.publish(status_msg)
            rospy.loginfo("Battery status published successfully")
            
        except IndexError as e:
            rospy.logerr(f"数据解析错误: 响应长度不足 ({len(data)} bytes)")
        except Exception as e:
            rospy.logerr(f"处理响应时出错: {e}")

    def read_battery_data(self):
        """读取并处理数据帧"""
        # 读取帧头 (1字节)
        frame = self.ser.read(1)
        if frame != b'\xdd':
            return False
        
        # 命令码 (1字节)
        command = self.ser.read(1)
        if command != b'\x03':  # 只处理基本信息响应
            return False
            
        # 状态码 (1字节)
        status = self.ser.read(1)
        if status != b'\x00':   # 0 表示正确
            rospy.logwarn(f"设备返回错误状态: 0x{status.hex()}")
            return False
        
        # 数据长度 (1字节)
        data_length = self.ser.read(1)[0]
        
        # 读取数据块
        data = self.ser.read(data_length)
        
        # 读取校验和 (2字节) 和帧尾 (1字节)
        checksum = self.ser.read(2)
        end_marker = self.ser.read(1)
        
        if end_marker == b'\x77':
            self.process_response(data)
            return True
        
        return False

    def run(self):
        """主循环"""
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            try:
                # 发送基本信息请求
                self.ser.write(self.REQUEST_BASIC_FRAME)
                rospy.sleep(0.05)  # 等待响应
                
                # 检查并读取响应
                if self.ser.in_waiting > 0:
                    self.read_battery_data()
                    
            except serial.SerialException as e:
                rospy.logerr(f"串口通信错误: {e}")
                rospy.signal_shutdown("串口故障")
            except Exception as e:
                rospy.logerr(f"运行时错误: {e}")
                
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('battery_monitor')
    port = rospy.get_param('~serial_port', '/dev/IMU')
    monitor = BatteryMonitor(port)
    
    try:
        monitor.run()
    except rospy.ROSInterruptException:
        pass
