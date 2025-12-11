#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time
from datetime import datetime
import crcmod
import rospy
from serial_comms.msg import Sensors

class ModbusRTUSwitchReader:
    def __init__(self, port, baudrate=9600, timeout=1):
        rospy.init_node('proximity_sensor_node')
        self.serial_port = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=8,
            parity='N',
            stopbits=1,
            timeout=timeout
        )
        self.crc16 = crcmod.predefined.mkCrcFun('modbus')
        self.pub = rospy.Publisher('proximity_sensor_data', Sensors, queue_size=10)
        
        # 设置定时器，每0.2秒调用一次查询函数
        rospy.Timer(rospy.Duration(0.2), self.timer_callback)
        
        # 用于统计通信错误
        self.error_count = 0
        self.max_error_count = 5

    def generate_modbus_rtu_frame(self, slave_id, function_code, start_addr, num_registers):
        # 生成Modbus RTU请求帧
        data = bytes([
            slave_id,
            function_code,
            (start_addr >> 8) & 0xFF,
            start_addr & 0xFF,
            (num_registers >> 8) & 0xFF,
            num_registers & 0xFF
        ])
        crc = self.crc16(data)
        return data + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    def parse_switch_status(self, response):
        # 解析Modbus响应，返回4路开关状态
        if len(response) < 5:
            raise ValueError("Invalid response length")
        
        # 检查CRC
        received_crc = response[-2] | (response[-2+1] << 8)
        calculated_crc = self.crc16(response[:-2])
        if received_crc != calculated_crc:
            raise ValueError("CRC check failed")
        
        # 解析数据部分
        data_length = response[2]
        if data_length != 3:  # 根据示例，返回的数据长度是3字节
            raise ValueError("Unexpected data length")
        
        status_bytes = response[3:6]
        # status_bytes = b'\x01\x01\x00' # b 为小端排序
        # print("status_bytes:",status_bytes)
        # 小端序解析：将3字节转换为整数（最低有效字节在前）
        status_value = (status_bytes[2] << 16) | (status_bytes[1] << 8) | status_bytes[0]
        # 00000000  00000001  00000001
        # # 转换为6位二进制字符串（前面补零）
        # binary_str = bin(status_value)[2:].zfill(6)
        # print("Binary representation:", binary_str)  # 输出如 "000001"
        
        # 解析4路开关状态（按位从低到高对应sensor_a到sensor_d）
        return {
            'sensor_a': bool(status_value & 0b000000001), # 检测第1位
            'sensor_b': bool(status_value & 0b100000000), # 检测第7位
            'sensor_c': bool(status_value & 0b000000100), # 检测第3位
            'sensor_d': bool(status_value & 0b000010000)  # 检测第5位
        }
    def timer_callback(self, event):
        try:
            # 发送查询指令
            query = self.generate_modbus_rtu_frame(
                slave_id=0x01,
                function_code=0x02,
                start_addr=0x0000,
                num_registers=0x0014
            )
            
            rospy.logdebug(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}]# SEND HEX/{len(query)} >>>")
            rospy.logdebug(' '.join(f"{b:02X}" for b in query))
            
            self.serial_port.write(query)
            
            # 读取响应
            response = self.serial_port.read(8)
            # print(response)
            
            rospy.logdebug(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}]# RECV HEX/{len(response)} <<<")
            if len(response) > 0:
                rospy.logdebug(' '.join(f"{b:02X}" for b in response))
            else:
                rospy.logwarn("No data received")
                self.error_count += 1
                if self.error_count >= self.max_error_count:
                    rospy.logerr("Maximum error count reached, check device connection")
                return
            
            # 解析并发布数据
            status = self.parse_switch_status(response)
            msg = Sensors()
            msg.sensor_a = status['sensor_a']
            msg.sensor_b = status['sensor_b']
            msg.sensor_c = status['sensor_c']
            msg.sensor_d = status['sensor_d']
            
            self.pub.publish(msg)
            self.error_count = 0  # 重置错误计数
            
            # 打印日志信息
            # rospy.loginfo(f"Published sensor status: "
            #              f"A:{status['sensor_a']} "
            #              f"B:{status['sensor_b']} "
            #              f"C:{status['sensor_c']} "
            #              f"D:{status['sensor_d']}")
            
        except Exception as e:
            rospy.logerr(f"Error in timer_callback: {str(e)}")
            self.error_count += 1
            if self.error_count >= self.max_error_count:
                rospy.logerr("Maximum error count reached, check device connection")

    def close(self):
        self.serial_port.close()

if __name__ == "__main__":
    try:
        reader = ModbusRTUSwitchReader(port='/dev/ProximitySensor')
        rospy.loginfo("Proximity sensor node started")
        rospy.spin()  # 保持节点运行，直到接收到关闭信号
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Failed to initialize node: {str(e)}")
    finally:
        if 'reader' in locals():
            reader.close()
        rospy.loginfo("Proximity sensor node shutdown")