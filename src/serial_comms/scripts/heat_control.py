#!/usr/bin/env python3

import rospy
import serial
import threading
import struct
import time
from std_msgs.msg import Bool, String
# 导入ROS服务消息类型
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse

class RelayController:
    def __init__(self):
        # ROS参数
        self.port = rospy.get_param('~port', '/dev/Battery-Relay')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.slave_address = rospy.get_param('~slave_address', 0x02)
        self.timeout = rospy.get_param('~timeout', 1.0)
        
        # 串口连接
        self.serial_conn = None
        self.lock = threading.Lock()
        
        # ROS发布器和订阅器
        self.status_pub = rospy.Publisher('relay_status', Bool, queue_size=10)
        self.debug_pub = rospy.Publisher('relay_debug', String, queue_size=10)
        
        rospy.Service('~enable_relay', SetBool, self.enable_relay_callback)
        rospy.Service('~get_status', Trigger, self.get_status_callback)
        
        # 初始化串口
        self.init_serial()
        
        # 启动状态监控线程
        self.monitor_thread = threading.Thread(target=self.status_monitor)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
        rospy.loginfo("Relay controller initialized on port: %s", self.port)
    
    def init_serial(self):
        """初始化串口连接"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            rospy.loginfo("Successfully connected to relay on %s", self.port)
        except Exception as e:
            rospy.logerr("Failed to connect to relay: %s", str(e))
            self.serial_conn = None
    
    def calculate_crc(self, data):
        """计算Modbus RTU CRC16校验码"""
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for i in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc
    
    def send_modbus_command(self, command_data):
        """发送Modbus命令并接收响应"""
        if self.serial_conn is None:
            rospy.logwarn("Serial port not available")
            return None
        
        with self.lock:
            try:
                # 添加CRC校验
                crc = self.calculate_crc(command_data)
                command_data += struct.pack('<H', crc)
                
                # 发送命令
                self.serial_conn.write(command_data)
                rospy.logdebug("Sent: %s", ' '.join(['%02X' % b for b in command_data]))
                
                # 接收响应
                response = self.serial_conn.read(8)  # 读取足够长的响应
                if len(response) > 0:
                    rospy.logdebug("Received: %s", ' '.join(['%02X' % b for b in response]))
                    return response
                else:
                    rospy.logwarn("No response from relay")
                    return None
                    
            except Exception as e:
                rospy.logerr("Communication error: %s", str(e))
                return None
    
    def enable_relay(self, enable=True):
        """开启或关闭继电器"""
        # 功能码 0x05: 写单个线圈
        # 地址 0x0000: 线圈地址
        # 值: 0xFF00 开启, 0x0000 关闭
        if enable:
            command_data = bytes([self.slave_address, 0x05, 0x00, 0x00, 0xFF, 0x00])
        else:
            command_data = bytes([self.slave_address, 0x05, 0x00, 0x00, 0x00, 0x00])
        
        response = self.send_modbus_command(command_data)
        
        if response and len(response) >= 7:
            # 验证响应是否正确
            expected_response = command_data  # 写线圈命令的响应应该与发送的命令相同
            if response[:6] == expected_response:
                return True
        
        return False
    
    def read_relay_status(self):
        """读取继电器状态"""
        # 功能码 0x01: 读线圈状态
        # 地址 0x0000: 起始地址
        # 数量 0x0008: 读取8个线圈
        command_data = bytes([self.slave_address, 0x01, 0x00, 0x00, 0x00, 0x08])
        
        response = self.send_modbus_command(command_data)
        
        if response and len(response) >= 5:
            # 响应格式: [地址, 功能码, 字节数, 数据..., CRC]
            if response[0] == self.slave_address and response[1] == 0x01:
                byte_count = response[2]
                if byte_count >= 1:
                    # 第一个字节包含前8个线圈的状态
                    status_byte = response[3]
                    # 我们关心的是第四个线圈 (bit 3, 从0开始计数)
                    relay_status = (status_byte & 0x08) != 0
                    return relay_status
        
        return None
    
    def enable_relay_callback(self, req):
        """ROS服务回调: 开启/关闭继电器"""
        success = self.enable_relay(req.data)
        
        response = SetBoolResponse()
        response.success = success
        if success:
            response.message = "Relay {} successfully".format("enabled" if req.data else "disabled")
        else:
            response.message = "Failed to {} relay".format("enable" if req.data else "disable")
        
        return response
    
    def get_status_callback(self, req):
        """ROS服务回调: 获取继电器状态"""
        status = self.read_relay_status()
        
        response = TriggerResponse()
        if status is not None:
            response.success = True
            response.message = "Relay is {}".format("ON" if status else "OFF")
        else:
            response.success = False
            response.message = "Failed to read relay status"
        
        return response
    
    def status_monitor(self):
        """状态监控线程，定期发布继电器状态"""
        rate = rospy.Rate(1)  # 1Hz
        
        while not rospy.is_shutdown():
            if self.serial_conn:
                status = self.read_relay_status()
                if status is not None:
                    # 发布状态
                    status_msg = Bool()
                    status_msg.data = status
                    self.status_pub.publish(status_msg)
                    
                    # 发布调试信息
                    debug_msg = String()
                    debug_msg.data = "Relay status: {}".format("ON" if status else "OFF")
                    self.debug_pub.publish(debug_msg)
            
            rate.sleep()
    
    def run_test_sequence(self):
        """运行测试序列"""
        while True:
            rospy.loginfo("Starting relay test sequence...")
            
            # 测试1: 读取状态
            rospy.loginfo("Test 1: Reading relay status")
            status = self.read_relay_status()
            rospy.loginfo("Initial relay status: %s", "ON" if status else "OFF")
            
            # 测试2: 开启继电器
            rospy.loginfo("Test 2: Enabling relay")
            if self.enable_relay(True):
                rospy.loginfo("Relay enabled successfully")
                time.sleep(1)
                status = self.read_relay_status()
                rospy.loginfo("Relay status after enable: %s", "ON" if status else "OFF")
            else:
                rospy.logerr("Failed to enable relay")
            
            # 测试3: 关闭继电器
            rospy.loginfo("Test 3: Disabling relay")
            if self.enable_relay(False):
                rospy.loginfo("Relay disabled successfully")
                time.sleep(1)
                status = self.read_relay_status()
                rospy.loginfo("Relay status after disable: %s", "ON" if status else "OFF")
            else:
                rospy.logerr("Failed to disable relay")
            
            rospy.loginfo("Relay test sequence completed")

def main():
    rospy.init_node('relay_controller', anonymous=True)
    
    controller = RelayController()
    
    # 如果指定了测试参数，运行测试序列
    if rospy.get_param('~run_test', True):
        controller.run_test_sequence()
    
    rospy.loginfo("Relay controller node is running")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass