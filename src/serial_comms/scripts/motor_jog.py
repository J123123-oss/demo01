#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import time
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.constants import Endian
from serial_comms.msg import Environment  # 替换为你的功能包名

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

# 全局变量
client = None
is_running = False  # 运行状态标志
is_forward = True  # 正向/反向标志
jog_interval = 0.03  # JOG发送周期（30ms，<50ms保证连续运行）
sensor_check_interval = 0.01  # 传感器检测周期

class JogControlNode:
    def __init__(self):
        rospy.init_node("dm2j_jog_control_node", anonymous=True)
        rospy.loginfo("DM2J JOG控制节点启动...")

        # 初始化Modbus客户端
        self.init_modbus_client()

        # 设置JOG速度（默认60rpm，可根据需求修改）
        self.set_jog_speed(60)

        # 创建ROS服务
        self.start_service = rospy.Service("/start_jog", Trigger, self.start_jog_callback)
        self.stop_service = rospy.Service("/stop_jog", Trigger, self.stop_jog_callback)

        # 状态发布器（可选，用于监控）
        self.status_pub = rospy.Publisher("/jog_status", Bool, queue_size=10)

        rospy.loginfo("服务已就绪：/start_jog（启动）、/stop_jog（停止）")
        rospy.spin()

    def init_modbus_client(self):
        """初始化Modbus串口客户端"""
        global client
        client = ModbusSerialClient(
            port=SERIAL_PORT,
            baudrate=BAUDRATE,
            parity=PARITY,
            stopbits=STOPBITS,
            bytesize=BYTESIZE,
            timeout=0.1
        )

        # 连接串口
        if not client.connect():
            rospy.logerr("串口连接失败！请检查设备和权限")
            rospy.signal_shutdown("串口连接失败")

    def set_jog_speed(self, speed):
        """设置JOG速度（rpm）"""
        try:
            # 速度值范围：0-5000rpm（手册5.3.1节）
            if speed < 0 or speed > 5000:
                rospy.logwarn(f"速度超出范围（0-5000），使用默认60rpm")
                speed = 60

            # 写入速度寄存器（功能码0x06：写单个寄存器）
            response = client.write_register(
                address=JOG_SPEED_REG,
                value=speed,
                slave=SLAVE_ID
            )

            if not response.isError():
                rospy.loginfo(f"JOG速度设置成功：{speed}rpm")
            else:
                rospy.logerr(f"速度设置失败：{response}")

        except ModbusException as e:
            rospy.logerr(f"Modbus错误（设置速度）：{e}")
        except Exception as e:
            rospy.logerr(f"未知错误（设置速度）：{e}")

    def read_sensor_status(self):
        """读取双传感器状态（手册4.16、5.3.2节）"""
        try:
            # 读取1个保持寄存器（功能码0x03）
            response = client.read_holding_registers(
                address=IO_STATUS_REG,
                count=1,
                slave=SLAVE_ID
            )

            if response.isError():
                rospy.logerr(f"读取传感器失败：{response}")
                return False

            io_status = response.registers[0]  # 直接获取16位寄存器值，无需解码


            # 检查双传感器是否同时触发
            if (io_status & SENSOR_TRIGGERED_MASK) == SENSOR_TRIGGERED_MASK:
                rospy.loginfo(f"双传感器触发！IO状态值：0x{io_status:04X}")
                return True
            else:
                # 调试信息：显示单个传感器状态
                sensor1_status = (io_status & (1 << SENSOR1_BIT)) != 0
                sensor2_status = (io_status & (1 << SENSOR2_BIT)) != 0
                rospy.loginfo(f"传感器状态 - 传感器1：{sensor1_status}，传感器2：{sensor2_status}")
                return False

        except ModbusException as e:
            rospy.logerr(f"Modbus错误（读取传感器）：{e}")
            return False
        except Exception as e:
            rospy.logerr(f"未知错误（读取传感器）：{e}")
            return False

    def send_jog_command(self, cmd):
        """发送JOG指令（功能码0x06：写单个寄存器）"""
        try:
            response = client.write_register(
                address=CTRL_WORD_REG,
                value=cmd,
                slave=SLAVE_ID
            )
            if response.isError():
                rospy.logerr(f"JOG指令发送失败：{response}")
            return not response.isError()
        except ModbusException as e:
            rospy.logerr(f"Modbus错误（发送JOG）：{e}")
            return False
        except Exception as e:
            rospy.logerr(f"未知错误（发送JOG）：{e}")
            return False

    def jog_control_loop(self):
        """JOG控制主循环"""
        global is_running, is_forward
        last_jog_time = time.time()

        while not rospy.is_shutdown() and is_running:
            current_time = time.time()

            # 定时发送JOG指令（保证连续运行）
            if current_time - last_jog_time >= jog_interval:
                if is_forward:
                    self.send_jog_command(FORWARD_JOG_CMD)
                    rospy.logdebug("发送正向JOG指令")
                else:
                    self.send_jog_command(REVERSE_JOG_CMD)
                    rospy.logdebug("发送反向JOG指令")
                last_jog_time = current_time

            # 检测传感器状态
            if self.read_sensor_status() and is_forward:
                # 双传感器触发：切换为反向归位
                rospy.loginfo("开始反向归位...")
                is_forward = False

            # 发布运行状态
            self.status_pub.publish(Bool(data=is_running))

            # 传感器检测延时
            time.sleep(sensor_check_interval)

        # 停止JOG（发送停止指令）
        self.send_jog_command(STOP_CMD)
        rospy.loginfo("JOG控制循环停止")

    def start_jog_callback(self, req):
        """启动JOG服务回调"""
        global is_running, is_forward
        if not is_running:
            is_running = True
            is_forward = True  # 重置为正向
            rospy.loginfo("启动连续正向JOG，等待双传感器触发...")
            # 启动控制循环（非阻塞）
            import threading
            jog_thread = threading.Thread(target=self.jog_control_loop)
            jog_thread.daemon = True
            jog_thread.start()
            return TriggerResponse(success=True, message="正向JOG已启动")
        else:
            return TriggerResponse(success=False, message="JOG已在运行中")

    def stop_jog_callback(self, req):
        """停止JOG服务回调"""
        global is_running
        if is_running:
            is_running = False
            return TriggerResponse(success=True, message="JOG已停止")
        else:
            return TriggerResponse(success=False, message="JOG未在运行")

    def __del__(self):
        """析构函数：关闭串口连接"""
        if client:
            client.close()
            rospy.loginfo("Modbus串口已关闭")

if __name__ == "__main__":
    try:
        node = JogControlNode()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被中断")
    except Exception as e:
        rospy.logerr(f"节点启动失败：{e}")