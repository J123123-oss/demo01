#!/usr/bin/env python3

import rospy
import serial
import threading
import struct
import time
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
import datetime


class RelayController:
    def __init__(self):
        # ROS参数
        self.port = rospy.get_param('~port', '/dev/Battery-Relay')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.relay_address = rospy.get_param('~relay_address', 0x02)
        self.timeout = rospy.get_param('~timeout', 1.0)
        
        # 串口连接
        self.serial_conn = None
        self.lock = threading.Lock()
        self.serial_lock = threading.Lock()
        self.current_relay_state = False
        self.current_temperatures = []
        
        # ROS发布器
        self.status_pub = rospy.Publisher('relay_status', Bool, queue_size=10)
        self.debug_pub = rospy.Publisher('relay_debug', String, queue_size=10)
        self.relay_auto_off_pub = rospy.Publisher('relay_auto_off', Bool, queue_size=1)
        
        # ROS服务
        rospy.Service('~enable_relay', SetBool, self.enable_relay_callback)
        rospy.Service('~get_status', Trigger, self.get_status_callback)
        
        # 初始化串口
        self.init_serial()
        
        # 继电器计时
        self.relay_on_time = None
        self.relay_auto_shutdown_flag = False
        
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
        # 返回字节形式的CRC（小端序）
        return struct.pack('<H', crc)
    
    def send_modbus_command(self, command_data):
        """发送Modbus命令并接收响应"""
        if self.serial_conn is None:
            rospy.logwarn("Serial port not available")
            return None
        
        with self.lock:
            try:
                # 添加CRC校验（返回的是字节）
                crc_bytes = self.calculate_crc(command_data)
                command_data += crc_bytes
                
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
    
    def send_relay_command(self, command_data):
        """发送继电器命令（使用serial_conn）"""
        try:
            # 计算CRC并转换为字节
            crc_bytes = self.calculate_crc(command_data)
            full_command = command_data + crc_bytes
            
            self.serial_conn.write(full_command)
            self.serial_conn.flush()
            
            rospy.loginfo("发送继电器命令: %s", ' '.join(['%02X' % b for b in full_command]))
            
            # time.sleep(0.2)
            response = self.serial_conn.read(8)
            
            if len(response) > 0:
                rospy.loginfo("接收继电器响应: %s", ' '.join(['%02X' % b for b in response]))
                if len(response) >= 6 and response[:6] == command_data:
                    return True
            return False
        except Exception as e:
            rospy.logerr(f"继电器通信错误: {e}")
            return False
    
    def enable_relay(self, enable=True, current_temp=None):
        """控制继电器"""
        if enable:
            now = datetime.datetime.now()
            in_regular_hours = 6 <= now.hour < 16
            is_extreme_low = current_temp is not None and current_temp < -15

            if not in_regular_hours and not is_extreme_low:
                rospy.logwarn("当前时间不在允许开启继电器的时段且非极端低温，请求被拒绝")
                return False

        max_retries = 10  # 减少重试次数
        for attempt in range(max_retries):
            with self.serial_lock:
                # 等待总线空闲
                time.sleep(0.1)
                
                if enable:
                    command_data = bytes([self.relay_address, 0x05, 0x00, 0x00, 0xFF, 0x00])
                    rospy.loginfo("发送继电器开启命令")
                else:
                    command_data = bytes([self.relay_address, 0x05, 0x00, 0x00, 0x00, 0x00])
                    rospy.loginfo("发送继电器关闭命令")
                
                success = self.send_relay_command(command_data)
                
                if success:
                    self.current_relay_state = enable
                    self.status_pub.publish(Bool(data=enable))
                    
                    if enable:
                        self.relay_on_time = time.time()
                        self.relay_auto_shutdown_flag = False
                        self.relay_auto_off_pub.publish(Bool(data=False))
                    else:
                        self.relay_on_time = None
                    
                    return True
            
            rospy.logwarn(f"继电器命令发送失败，重试 {attempt + 1}/{max_retries}")
            time.sleep(0.5)
        
        rospy.logerr("继电器命令发送失败，已达到最大重试次数")
        return False
    
    def read_relay_status(self):
        """读取继电器状态"""
        # 功能码 0x01: 读线圈状态
        command_data = bytes([self.relay_address, 0x01, 0x00, 0x00, 0x00, 0x01])  # 只读取1个线圈
        
        response = self.send_modbus_command(command_data)
        
        if response and len(response) >= 5:
            # 响应格式: [地址, 功能码, 字节数, 数据..., CRC]
            if response[0] == self.relay_address and response[1] == 0x01:
                byte_count = response[2]
                if byte_count >= 1:
                    # 第一个线圈状态（bit 0）
                    status_byte = response[3]
                    relay_status = (status_byte & 0x01) != 0
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
        rate = rospy.Rate(0.5)  # 降低频率
        
        while not rospy.is_shutdown():
            if self.serial_conn:
                status = self.read_relay_status()
                if status is not None:
                    self.current_relay_state = status
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
        while not rospy.is_shutdown():
            rospy.loginfo("Starting relay test sequence...")
            
            # 测试1: 读取状态
            rospy.loginfo("Test 1: Reading relay status")
            status = self.read_relay_status()
            rospy.loginfo("Initial relay status: %s", "ON" if status else "OFF")
            
            # 测试2: 关闭继电器
            rospy.loginfo("Test 3: Disabling relay")
            if self.enable_relay(False):
                rospy.loginfo("Relay disabled successfully")
                time.sleep(2)
                status = self.read_relay_status()
                rospy.loginfo("Relay status after disable: %s", "ON" if status else "OFF")
            else:
                rospy.logerr("Failed to disable relay")
            
            rospy.loginfo("Relay test sequence completed")
            time.sleep(5)  # 测试间隔

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