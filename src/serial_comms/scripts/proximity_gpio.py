#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import time
from std_msgs.msg import Bool, UInt8
from serial_comms.msg import Sensors
# 引入香橙派GPIO库
try:
    import OrangePi.GPIO as GPIO
except ImportError:
    pass
    # 兼容树莓派GPIO或香橙派wiringPi兼容模式
    # import RPi.GPIO as GPIO


class DigitalInputReader:
    def __init__(self):
        rospy.init_node('proximity_input_reader')
        
        # 保留原有ROS参数（兼容原配置，无用参数仅保留不影响逻辑）
        self.device_address = rospy.get_param('~device_address', 1)
        self.can_interface = rospy.get_param('~can_interface', 'can0')
        self.update_rate = rospy.get_param('~update_rate', 20.0)  # Hz
        self.baudrate = rospy.get_param('~baudrate', 1000000)
        self.request_baudrate = rospy.get_param('~request_baudrate', False)

        # ========== GPIO配置（核心修改）==========
        # 定义4个接近开关对应的GPIO引脚（wiringOP编号，可根据实际接线修改）
        self.GPIO_PIN_SENSOR_A = 12  # 对应原sensor_a (通道1)
        self.GPIO_PIN_SENSOR_B = 13  # 对应原sensor_b (通道7)
        self.GPIO_PIN_SENSOR_C = 16  # 对应原sensor_c (通道3)
        self.GPIO_PIN_SENSOR_D = 18  # 对应原sensor_d (通道5)
        
        # 初始化GPIO
        self._init_gpio()

        # 保留原发布器
        self.all_inputs_pub = rospy.Publisher('proximity_sensor_data', Sensors, queue_size=1)
        
        # 保留原定时器轮询逻辑
        rospy.Timer(rospy.Duration(1.0 / self.update_rate), self.poll_inputs)

    def _init_gpio(self):
        """初始化GPIO为输入模式，启用内部上拉电阻"""
        try:
            # 设置GPIO编号模式（wiringOP BOARD模式）
            GPIO.setmode(GPIO.BOARD)
            # 配置所有传感器GPIO为输入，启用上拉电阻
            # 接近开关接线：开关一端接GPIO，另一端接GND
            # 未触发时为高电平（True），触发时为低电平（False）
            gpio_pins = [
                self.GPIO_PIN_SENSOR_A,
                self.GPIO_PIN_SENSOR_B,
                self.GPIO_PIN_SENSOR_C,
                self.GPIO_PIN_SENSOR_D
            ]
            for pin in gpio_pins:
                GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            rospy.loginfo("GPIO初始化成功，引脚配置：")
            rospy.loginfo(f"Sensor A: PIN{self.GPIO_PIN_SENSOR_A}")
            rospy.loginfo(f"Sensor B: PIN{self.GPIO_PIN_SENSOR_B}")
            rospy.loginfo(f"Sensor C: PIN{self.GPIO_PIN_SENSOR_C}")
            rospy.loginfo(f"Sensor D: PIN{self.GPIO_PIN_SENSOR_D}")
            
        except Exception as e:
            rospy.logerr(f"GPIO初始化失败: {str(e)}")
            rospy.signal_shutdown("GPIO initialization failed")

    def poll_inputs(self, event):
        """轮询读取GPIO状态（替代原CAN查询）"""
        try:
            # 读取各传感器GPIO状态（反转：低电平=触发=True）
            sensor_a = not GPIO.input(self.GPIO_PIN_SENSOR_A)
            sensor_b = not GPIO.input(self.GPIO_PIN_SENSOR_B)
            sensor_c = not GPIO.input(self.GPIO_PIN_SENSOR_C)
            sensor_d = not GPIO.input(self.GPIO_PIN_SENSOR_D)
            
            # 处理并发布数据（保留原逻辑）
            self._process_input_data(sensor_a, sensor_b, sensor_c, sensor_d)
            
        except Exception as e:
            rospy.logerr(f"读取GPIO状态出错: {str(e)}")

    def _process_input_data(self, sensor_a, sensor_b, sensor_c, sensor_d):
        """处理GPIO数据并发布（修改适配GPIO输入）"""
        try:
            msg = Sensors()
            # 直接赋值GPIO读取的状态（无需解析字节）
            msg.sensor_a = sensor_a
            msg.sensor_b = sensor_b
            msg.sensor_c = sensor_c
            msg.sensor_d = sensor_d
            
            # 保留原发布逻辑
            self.all_inputs_pub.publish(msg)
            rospy.logdebug(f"发布传感器数据: [A:{msg.sensor_a}, B:{msg.sensor_b}, C:{msg.sensor_c}, D:{msg.sensor_d}]")
            
        except Exception as e:
            rospy.logerr(f"处理传感器数据出错: {str(e)}")

    def run(self):
        """保留原运行逻辑"""
        rospy.spin()

    def shutdown(self):
        """关闭GPIO资源（替代原CAN关闭）"""
        try:
            GPIO.cleanup()  # 释放所有GPIO资源
            rospy.loginfo("GPIO资源已释放")
        except Exception as e:
            rospy.logerr(f"关闭GPIO出错: {str(e)}")

if __name__ == '__main__':
    try:
        node = DigitalInputReader()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'node' in locals():
            node.shutdown()