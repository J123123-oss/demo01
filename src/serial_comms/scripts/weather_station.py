#!/usr/bin/env python3
import rospy
from std_msgs.msg import Time
from serial_comms.msg import Environment  # 替换为你的功能包名
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
import time

class WeatherStationNode:
    def __init__(self):
        # 初始化节点
        rospy.init_node('weather_station_node', anonymous=True)
        self.pub = rospy.Publisher('/environment_data', Environment, queue_size=10)
        
        # Modbus RTU配置（重点：波特率设为4800）
        self.client = ModbusSerialClient(
            method='rtu',
            port='/dev/ttyUSB14',  # 串口设备，根据实际情况修改
            baudrate=9600,        # 波特率：4800（按需求设置）
            parity='N',           # 校验位：无校验
            stopbits=1,           # 停止位：1
            bytesize=8,           # 数据位：8
            timeout=1             # 超时时间（秒）
        )
        
        # 寄存器地址定义（十进制）
        self.REG_WIND_SPEED = 500   # 风速（实际值的10倍）
        self.REG_WIND_DIR = 503     # 风向（实际值）
        self.REG_LUX_HIGH = 510     # 光照高16位
        self.REG_LUX_LOW = 511      # 光照低16位
        self.REG_RAINFALL = 513     # 雨量（实际值的10倍）
        self.SLAVE_ID = 1           # 设备地址
        
        # 连接设备
        if not self.client.connect():
            rospy.logfatal("无法连接到Modbus设备，请检查串口和参数！")
            rospy.signal_shutdown("连接失败")
            return
        
        rospy.loginfo("Modbus设备连接成功，开始发布数据...")

    def read_registers(self, addr, count):
        """读取保持寄存器（功能码03），返回数据列表（失败返回None）"""
        try:
            response = self.client.read_holding_registers(
                address=addr,
                count=count,
                slave=self.SLAVE_ID
            )
            if response.isError():
                rospy.logwarn(f"寄存器读取错误: {response}")
                return None
            return response.registers
        except ModbusException as e:
            rospy.logwarn(f"Modbus通信异常: {str(e)}")
            return None
        except Exception as e:
            rospy.logwarn(f"未知错误: {str(e)}")
            return None

    def run(self):
        rate = rospy.Rate(0.5)  # 10Hz发布频率
        while not rospy.is_shutdown():
            msg = Environment()
            msg.stamp = rospy.Time.now()  # 时间戳
            
            # 1. 读取风速（寄存器500，1个寄存器）
            wind_data = self.read_registers(self.REG_WIND_SPEED, 1)
            if wind_data:
                msg.wind_speed = wind_data[0] / 10.0  # 实际值 = 寄存器值 / 10
            else:
                rospy.logwarn("风速数据读取失败")
            
            # 2. 读取风向（寄存器503，1个寄存器）
            dir_data = self.read_registers(self.REG_WIND_DIR, 1)
            if dir_data:
                msg.wind_direction = dir_data[0]  # 直接为实际角度
            else:
                rospy.logwarn("风向数据读取失败")
            
            # 3. 读取光照强度（高16位+低16位，连续2个寄存器）
            lux_data = self.read_registers(self.REG_LUX_HIGH, 2)  # 从510开始读2个
            if lux_data and len(lux_data) == 2:
                lux_high = lux_data[0]
                lux_low = lux_data[1]
                msg.illuminance = (lux_high << 16) | lux_low  # 拼接32位值
            else:
                rospy.logwarn("光照数据读取失败")
            
            # 4. 读取雨量（寄存器513，1个寄存器）
            rain_data = self.read_registers(self.REG_RAINFALL, 1)
            if rain_data:
                msg.rainfall = rain_data[0] / 10.0  # 实际值 = 寄存器值 / 10
            else:
                rospy.logwarn("雨量数据读取失败")
            
            # 发布消息
            self.pub.publish(msg)
            rate.sleep()
        
        # 关闭连接
        self.client.close()

if __name__ == '__main__':
    try:
        node = WeatherStationNode()
        if rospy.core.is_initialized():  # 确保节点初始化成功
            node.run()
    except rospy.ROSInterruptException:
        pass