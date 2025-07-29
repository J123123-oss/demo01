#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import time
import can
from std_msgs.msg import Bool, UInt8
from serial_comms.msg import Sensors


class DigitalInputReader:
    def __init__(self):
        rospy.init_node('proximity_input_reader')
        
        # ROS parameters
        self.device_address = rospy.get_param('~device_address', 1)
        self.can_interface = rospy.get_param('~can_interface', 'can0')
        self.update_rate = rospy.get_param('~update_rate', 5.0)  # Hz
        self.baudrate = rospy.get_param('~baudrate', 1000000)  # 默认1Mbps(手册值0x0b)
        self.request_baudrate = rospy.get_param('~request_baudrate', True)  # 是否设置波特率，设置1次，断电5秒后生效
        
        # CAN初始化
        self.bus = None
        self._init_can()

        # 尝试设置波特率为1M(若需要)
        if self.request_baudrate:
            self._set_can_baudrate()

        # 等待设备适应新波特率
        time.sleep(0.1)

        self.all_inputs_pub = rospy.Publisher('proximity_sensor_data', Sensors, queue_size=1)
        
        # Timer for periodic polling
        rospy.Timer(rospy.Duration(1.0 / self.update_rate), self.poll_inputs)

    def _init_can(self):
        """Initialize CAN interface with error handling"""
        try:
            # 波特率映射表(手册表2.6)
            self.baudrate_mapping = {
                20000: 0x02,
                50000: 0x03,
                100000: 0x04,
                125000: 0x05,
                200000: 0x06,
                250000: 0x07,  # 出厂默认
                400000: 0x08,
                500000: 0x09,
                800000: 0x0a,
                1000000: 0x0b  # 1Mbps
            }
            
            # 创建带过滤器的CAN总线
            filters = [
                {"can_id": 0x300 | self.device_address, "can_mask": 0xFFF, "extended": False}
            ]
            
            self.bus = can.interface.Bus(
                interface='socketcan',
                channel=self.can_interface,
                bitrate=self.baudrate,
                receive_own_messages=False,
                can_filters=filters
            )
            rospy.loginfo(f"Successfully connected to CAN interface {self.can_interface} at {self.baudrate} bps")
        except Exception as e:
            rospy.logerr(f"CAN interface error: {str(e)}")
            rospy.signal_shutdown("CAN initialization failed")

    def _set_can_baudrate(self):
        """Set device baudrate to 1Mbps according to the manual"""
        try:
            # 获取波特率码(1M对应0x0b)
            if self.baudrate not in self.baudrate_mapping:
                rospy.logwarn(f"Configured baudrate {self.baudrate} not supported. Using default 1000000 bps (1M)")
                baudrate_code = 0x0b
            else:
                baudrate_code = self.baudrate_mapping[self.baudrate]
            
            rospy.loginfo(f"Setting device baudrate to {self.baudrate} bps (code:0x{baudrate_code:02x})")
            
            # 创建波特率设置帧(功能码0x04, 第一字节0xB2)
            frame_id = (0x04 << 8) | self.device_address
            set_baud_msg = can.Message(
                arbitration_id=frame_id,
                data=[0xB2, baudrate_code, 0, 0, 0, 0, 0, 0],
                is_extended_id=False
            )
            
            # 发送设置命令
            self.bus.send(set_baud_msg)
            rospy.logdebug(f"Sent baudrate setting frame: ID=0x{frame_id:03x}, Data={[hex(x) for x in set_baud_msg.data]}")
            
            # 等待并尝试读取响应
            self._wait_for_baudrate_confirmation()
            
            rospy.loginfo("Baudrate setting command sent. Device requires power cycle to take effect.")
            
        except Exception as e:
            rospy.logerr(f"Failed to set baudrate: {str(e)}")
            # 继续运行，但可能使用原波特率

    def _wait_for_baudrate_confirmation(self, timeout=0.5):
        """Wait for device response after baudrate setting"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            response = self.bus.recv(timeout - (time.time() - start_time))
            if response:
                rospy.logdebug(f"Received baudrate response: ID={hex(response.arbitration_id)}, Data={response.data}")
                # 检查响应格式 (功能码0x04 + 设备地址 + 状态)
                expected_id = (0x04 << 8) | self.device_address
                if response.arbitration_id == expected_id and len(response.data) >= 2:
                    if response.data[1] == 0x0b:
                        rospy.loginfo("Device confirmed baudrate setting successfully")
                        return True
        rospy.logwarn("No confirmation received for baudrate setting. Device may require power cycle.")
        return False

    def _create_query_frame(self):
        """Create CAN frame for digital input query"""
        frame_id = (0x03 << 8) | self.device_address  # 功能码0x03 + 设备地址
        return can.Message(
            arbitration_id=frame_id,
            data=[0, 0, 0, 0, 0, 0, 0, 0],
            is_extended_id=False
        )

    def poll_inputs(self, event):
        """Query and process digital inputs"""
        if self.bus is None:
            rospy.logerr("CAN bus not initialized!")
            return
            
        try:
            # 发送查询请求
            query_frame = self._create_query_frame()
            self.bus.send(query_frame)
            rospy.logdebug(f"Sent query frame: ID=0x{query_frame.arbitration_id:03x}")
            
            # 等待响应(增加超时时间)
            response = self.bus.recv(0.8)
            
            if not response:
                rospy.logwarn("Timeout waiting for digital input response")
                return
                
            rospy.logdebug(f"Received response: ID=0x{response.arbitration_id:03x}, Len={len(response.data)}, Data={response.data}")
            
            # 检查响应ID是否匹配
            expected_id = (0x03 << 8) | self.device_address
            if response.arbitration_id != expected_id:
                rospy.logwarn(f"Unexpected response ID: 0x{response.arbitration_id:03x}, expected 0x{expected_id:03x}")
                return
                
            # 检查数据长度
            if len(response.data) < 1:
                rospy.logerr("Received empty response data")
                return
                
            # 解析输入状态
            input_byte = response.data[0]  # 第一字节包含1-8通道状态
            self._process_input_data(input_byte)
           
        except can.CanError as e:
            rospy.logerr(f"CAN communication error: {str(e)}")
        except Exception as e:
            rospy.logerr(f"Unexpected error: {str(e)}")

    def _process_input_data(self, input_byte):
        """Process input byte and publish sensor data"""
        try:
            msg = Sensors()
            # 根据手册：字节的每一位对应一个输入通道(bit0=通道1, bit7=通道8)
            msg.sensor_a = bool(input_byte & 0b00000001)  # 通道1 (位0)
            msg.sensor_b = bool(input_byte & 0b01000000)  # 通道7 (位6)
            msg.sensor_c = bool(input_byte & 0b00000100)  # 通道3 (位2)
            msg.sensor_d = bool(input_byte & 0b00010000)  # 通道5 (位4)
            msg.raw_value = input_byte
            
            # 发布消息
            self.all_inputs_pub.publish(msg)
            rospy.loginfo(f"Published sensor data: [A:{msg.sensor_a}, B:{msg.sensor_b}, C:{msg.sensor_c}, D:{msg.sensor_d}] Raw:0x{input_byte:02x}")
        except Exception as e:
            rospy.logerr(f"Error processing sensor data: {str(e)}")

    def run(self):
        rospy.spin()

    def shutdown(self):
        if self.bus is not None:
            try:
                self.bus.shutdown()
                rospy.loginfo("CAN bus shutdown complete")
            except Exception as e:
                rospy.logerr(f"Error during bus shutdown: {str(e)}")

if __name__ == '__main__':
    try:
        node = DigitalInputReader()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'node' in locals():
            node.shutdown()
