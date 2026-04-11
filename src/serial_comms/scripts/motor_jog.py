#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import time
import threading
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.constants import Endian
from serial_comms.msg import Environment  # 替换为你的功能包名
from serial.serialutil import SerialException

# 驱动器配置参数（与手册默认一致）
SLAVE_ID = 1  # 从站地址默认1
SERIAL_PORT = "/dev/jog-weather"  # 串口设备（根据实际修改）
BAUDRATE = 38400  # 默认波特率
PARITY = "N"  # 校验位：无（8N1）
STOPBITS = 1  # 停止位
BYTESIZE = 8  # 数据位

# 寄存器地址定义（手册5.3节）
IO_STATUS_REG = 0x0179  # 输入IO状态寄存器（DI1-DI7）
CTRL_WORD_REG = 0x1801  # 控制字寄存器
JOG_SPEED_REG = 0x01E1  # JOG速度寄存器（Pr6.00）

# 控制字定义（手册4.7、5.3.3节）
FORWARD_JOG_CMD = 0x4001  # 正向JOG
REVERSE_JOG_CMD = 0x4002  # 反向JOG
STOP_CMD = 0x0000  # 停止JOG（停止发送指令也可，此处冗余设计）

# 传感器配置（DI2=bit1，DI3=bit2）
SENSOR1_BIT = 1  # 第一个传感器（DI2）
SENSOR2_BIT = 2  # 第二个传感器（DI3）
SENSOR_TRIGGERED_MASK = (1 << SENSOR1_BIT) | (1 << SENSOR2_BIT)  # 双传感器同时触发掩码（0x0006）
# 运行参数
jog_interval = 0.03
sensor_check_interval = 0.5 # 传感器状态检查间隔秒
reconnect_interval = 1  # 重连间隔秒

class JogControlNode:
    def __init__(self):
        rospy.init_node("dm2j_jog_control_node", anonymous=True)
        rospy.loginfo("DM2J JOG控制节点启动...")

        # 全局状态
        self.client = None
        self.is_running = False
        self.is_forward = True

        # 传感器实时状态
        self.sensor1_triggered = False
        self.sensor2_triggered = False
        self.both_sensor_triggered = False

        # 串口连接状态
        self.com_connected = False

        # 初始化串口
        self.reinit_modbus_client()

        # 设置速度
        self.set_jog_speed(60)

        # 启动独立传感器监控线程
        self.start_sensor_monitor_thread()

        # ROS服务
        self.start_service = rospy.Service("/start_forward_jog", Trigger, self.start_forward_jog_callback)
        self.start_rev_service = rospy.Service("/start_reverse_jog", Trigger, self.start_reverse_jog_callback)
        self.stop_service = rospy.Service("/stop_jog", Trigger, self.stop_jog_callback)
        self.status_pub = rospy.Publisher("/jog_status", Bool, queue_size=10)

        rospy.loginfo("服务就绪：正向/反向/停止JOG")
        rospy.spin()

    # ===================== 串口自动重连核心 =====================
    def reinit_modbus_client(self):
        """串口异常后自动重新初始化连接"""
        try:
            if self.client:
                try:
                    self.client.close()
                except:
                    pass

            self.client = ModbusSerialClient(
                port=SERIAL_PORT,
                baudrate=BAUDRATE,
                parity=PARITY,
                stopbits=STOPBITS,
                bytesize=BYTESIZE,
                timeout=0.1
            )

            connected = self.client.connect()
            if connected:
                rospy.loginfo("✅ 串口连接成功")
                self.com_connected = True
                return True
            else:
                rospy.logerr("❌ 串口连接失败")
                self.com_connected = False
                return False
        except Exception as e:
            rospy.logerr(f"❌ 串口初始化异常: {str(e)}")
            self.com_connected = False
            return False

    # ===================== 传感器独立线程 =====================
    def start_sensor_monitor_thread(self):
        thread = threading.Thread(target=self.sensor_monitor_loop)
        thread.daemon = True
        thread.start()

    def sensor_monitor_loop(self):
        while not rospy.is_shutdown():
            if not self.com_connected:
                self.reinit_modbus_client()
                time.sleep(reconnect_interval)
                continue

            try:
                response = self.client.read_holding_registers(
                    address=IO_STATUS_REG, count=1, slave=SLAVE_ID
                )
                if response.isError():
                    rospy.logwarn("传感器读取错误，准备重连串口")
                    self.com_connected = False
                    time.sleep(0.1)
                    continue

                io_status = response.registers[0]
                self.sensor1_triggered = (io_status & (1 << SENSOR1_BIT)) != 0
                self.sensor2_triggered = (io_status & (1 << SENSOR2_BIT)) != 0
                self.both_sensor_triggered = (io_status & SENSOR_TRIGGERED_MASK) == SENSOR_TRIGGERED_MASK

            except (ModbusException, SerialException, Exception) as e:
                rospy.logerr(f"传感器读取异常: {e}")
                self.com_connected = False

            time.sleep(sensor_check_interval)

    # ===================== JOG 控制循环 =====================
    def jog_control_loop(self):
        last_jog_time = time.time()
        while not rospy.is_shutdown() and self.is_running:
            if not self.com_connected:
                time.sleep(0.1)
                continue

            try:
                # 定时发送JOG指令
                if time.time() - last_jog_time >= jog_interval:
                    cmd = FORWARD_JOG_CMD if self.is_forward else REVERSE_JOG_CMD
                    self.client.write_register(CTRL_WORD_REG, cmd, SLAVE_ID)
                    last_jog_time = time.time()

                # 传感器触发 → 自动反向
                if self.both_sensor_triggered and self.is_forward:
                    rospy.loginfo("双传感器触发 → 开始反向")
                    self.is_forward = False

            except:
                self.com_connected = False

            time.sleep(0.005)

        # 停止指令
        try:
            if self.com_connected:
                self.client.write_register(CTRL_WORD_REG, STOP_CMD, SLAVE_ID)
        except:
            pass
        rospy.loginfo("JOG已停止")

    # ===================== JOG 服务 =====================
    def start_forward_jog_callback(self, req):
        if self.is_running:
            return TriggerResponse(success=False, message="JOG正在运行")
        self.is_running = True
        self.is_forward = True
        threading.Thread(target=self.jog_control_loop, daemon=True).start()
        return TriggerResponse(success=True, message="正向JOG已启动")

    def start_reverse_jog_callback(self, req):
        if self.is_running:
            return TriggerResponse(success=False, message="JOG正在运行")
        self.is_running = True
        self.is_forward = False
        threading.Thread(target=self.jog_control_loop, daemon=True).start()
        return TriggerResponse(success=True, message="反向JOG已启动")

    def stop_jog_callback(self, req):
        if not self.is_running:
            return TriggerResponse(success=False, message="JOG未运行")
        self.is_running = False
        return TriggerResponse(success=True, message="JOG已停止")

    # ===================== 设置速度 =====================
    def set_jog_speed(self, speed):
        if not self.com_connected:
            return
        try:
            speed = max(0, min(5000, speed))
            self.client.write_register(JOG_SPEED_REG, speed, SLAVE_ID)
            rospy.loginfo(f"JOG速度已设为: {speed} rpm")
        except:
            self.com_connected = False

if __name__ == "__main__":
    try:
        node = JogControlNode()
    except Exception as e:
        rospy.logerr(f"节点异常: {e}")