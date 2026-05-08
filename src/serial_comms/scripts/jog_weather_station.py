#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import time
import threading
import subprocess
import json
from std_msgs.msg import Bool, String, UInt8
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
from serial_comms.msg import Environment  # 气象站消息类型

# ===================== 公共配置 =====================
SERIAL_PORT = "/dev/jog-weather"  # 共享串口
# SERIAL_PORT = "/dev/ttyUSB0"  # 共享串口
BAUDRATE = 9600  
PARITY = "N"
STOPBITS = 1
BYTESIZE = 8
TIMEOUT = 1  # 匹配气象站超时时间

# ===================== 电机驱动器配置 =====================
MOTOR_SLAVE_ID = 1  # 电机从站地址
# 电机寄存器地址
IO_STATUS_REG = 0x0179    # 输入IO状态寄存器（十进制473）
CTRL_WORD_REG = 0x1801    # 控制字寄存器（十进制6145）
JOG_SPEED_REG = 0x01E1    # JOG速度寄存器（十进制481）
# 电机控制字
FORWARD_JOG_CMD = 0x4001  # 正向JOG
REVERSE_JOG_CMD = 0x4002  # 反向JOG
STOP_CMD = 0x0000         # 停止JOG
# 传感器配置
SENSOR1_BIT = 1
SENSOR2_BIT = 2
SENSOR3_BIT = 3
SENSOR_TRIGGERED_MASK = (1 << SENSOR3_BIT)
SENSOR_LIMITED_MASK = (1 << SENSOR2_BIT)
SENSOR_RESET_MASK = (1 << SENSOR1_BIT)  # 可修改为SENSOR1_BIT
# 电机轮询参数
JOG_INTERVAL = 0.015       # JOG指令发送周期（15ms）
MOTOR_LOOP_INTERVAL = 0.001   # 电机控制循环间隔

# ===================== 气象站配置 =====================
WEATHER_SLAVE_ID = 9           # 气象站从站地址
REG_WIND_SPEED = 500   # 风速（实际值的10倍）
REG_WIND_DIR = 503     # 风向（实际值）
REG_LUX_HIGH = 510     # 光照高16位
REG_LUX_LOW = 511      # 光照低16位
REG_RAINFALL = 513     # 雨量（实际值的10倍）
WEATHER_PUBLISH_RATE = 0.02  # 气象站发布频率（50秒/次）

# ===================== MQTT配置 =====================
MQTT_TOPIC = "robot/HEJIN_Huaxinyuan/status"  # MQTT消息订阅话题（根据实际修改）

# ===================== 全局状态 =====================
client = None
modbus_lock = threading.Lock()  # 总线互斥锁
# 电机状态
motor_is_running = False
motor_current_direction = "STOP"  # STOP/FORWARD/REVERSE
# MQTT状态
mqtt_completed = False
mqtt_running_state = ""

class JogWeatherControlNode:
    def __init__(self):
        rospy.init_node("jog_weather_control_node", anonymous=True)
        rospy.loginfo("电机+气象站+MQTT联合控制节点启动...")

        # 1. 初始化Modbus客户端
        self.init_modbus_client()

        # 2. 初始化发布器
        self.motor_status_pub = rospy.Publisher("/jog_status", Bool, queue_size=10)
        self.weather_pub = rospy.Publisher('/environment_data', Environment, queue_size=10)
        self.proximity_sensors_pub = rospy.Publisher("/sensors", UInt8, queue_size=10)
        # 3. 设置电机初始速度
        self.set_motor_jog_speed(180) # default 60rpm

        # 4. 创建ROS服务（正向/反向/停止独立服务）
        self.start_forward_service = rospy.Service("/start_forward_jog", Trigger, self.start_forward_callback)
        self.start_reverse_service = rospy.Service("/start_reverse_jog", Trigger, self.start_reverse_callback)
        self.stop_jog_service = rospy.Service("/stop_jog", Trigger, self.stop_jog_callback)
        rospy.loginfo("电机服务就绪：/start_forward_jog /start_reverse_jog /stop_jog")
        self.trigger_value = 0
        self.io_status = 0


        # 5. 订阅MQTT JSON消息话题
        # self.mqtt_sub = rospy.Subscriber(MQTT_TOPIC, String, self.mqtt_state_callback, queue_size=10)
        # rospy.loginfo(f"已订阅MQTT话题：{MQTT_TOPIC}")

        # 6. 启动气象站发布线程
        weather_thread = threading.Thread(target=self.weather_publish_loop)
        weather_thread.daemon = True
        weather_thread.start()
        rospy.loginfo(f"气象站发布线程已启动（频率：{WEATHER_PUBLISH_RATE}Hz）")

        # 7. 启动电机控制主线程
        motor_thread = threading.Thread(target=self.motor_control_loop)
        motor_thread.daemon = True
        motor_thread.start()
        rospy.loginfo("电机控制线程已启动")

        rospy.spin()

    def init_modbus_client(self):
        """初始化Modbus客户端"""
        global client
        client = ModbusSerialClient(
            method='rtu',
            port=SERIAL_PORT,
            baudrate=BAUDRATE,
            parity=PARITY,
            stopbits=STOPBITS,
            bytesize=BYTESIZE,
            timeout=TIMEOUT
        )

        if not client.connect():
            rospy.logfatal("无法连接到Modbus设备，请检查串口和参数！")
            rospy.signal_shutdown("连接失败")
            return
        rospy.loginfo("Modbus设备连接成功")

    # ===================== 电机控制核心方法 =====================
    def set_motor_jog_speed(self, speed):
        """设置电机JOG速度"""
        if speed < 0 or speed > 5000:
            rospy.logwarn(f"速度超出范围（0-5000），使用默认60rpm")
            speed = 60

        try:
            with modbus_lock:
                response = client.write_register(
                    address=JOG_SPEED_REG,
                    value=speed,
                    slave=MOTOR_SLAVE_ID
                )
            if not response.isError():
                rospy.loginfo(f"电机速度设置成功：{speed}rpm")
            else:
                rospy.logerr(f"速度设置失败：{response}")
        except ModbusException as e:
            rospy.logerr(f"Modbus错误（设置速度）：{e}")
        except Exception as e:
            rospy.logerr(f"未知错误（设置速度）：{str(e)}")

    def read_motor_sensor(self):
        """读取电机传感器状态，返回传感器触发掩码值（io_status & SENSOR_TRIGGERED_MASK）"""
        try:
            with modbus_lock:
                response = client.read_holding_registers(
                    address=IO_STATUS_REG,
                    count=1,
                    slave=MOTOR_SLAVE_ID
                )
            if response.isError():
                rospy.logerr(f"读取传感器失败：{response}")
                return 0  # 读取失败时返回0
            self.io_status = response.registers[0]
            if motor_current_direction == "FORWARD":
                self.trigger_value = (self.io_status & SENSOR_TRIGGERED_MASK or self.io_status & SENSOR_LIMITED_MASK)  # 计算触发掩码值
            elif motor_current_direction == "REVERSE":
                self.trigger_value = self.io_status & SENSOR_RESET_MASK  # 计算重置掩码值
            else:
                rospy.logdebug("无效的电机方向")
                return 0
            # 日志输出触发状态
            if self.trigger_value == SENSOR_TRIGGERED_MASK:
                rospy.loginfo(f"到位传感器触发！IO状态：0x{self.io_status:04X}，触发掩码值：{self.trigger_value}")
            else:
                s1 = (self.io_status & (1 << SENSOR1_BIT)) != 0
                s2 = (self.io_status & (1 << SENSOR2_BIT)) != 0
                s3 = (self.io_status & (1 << SENSOR3_BIT)) != 0
                rospy.loginfo(f"传感器状态 - 1：{s1}，2：{s2}，3：{s3}，触发掩码值：{self.trigger_value}")
            
            return self.trigger_value  # 返回实际的掩码计算值

        except ModbusException as e:
            rospy.logerr(f"Modbus错误（读取传感器）：{e}")
            return 0  # 异常时返回0
        except Exception as e:
            rospy.logerr(f"未知错误（读取传感器）：{str(e)}")
            return 0  # 异常时返回0

    def send_motor_jog_cmd(self, cmd):
        """发送电机JOG指令"""
        try:
            with modbus_lock:
                response = client.write_register(
                    address=CTRL_WORD_REG,
                    value=cmd,
                    slave=MOTOR_SLAVE_ID
                )
            if response.isError():
                rospy.logerr(f"JOG指令发送失败：{response}")
            return not response.isError()
        except ModbusException as e:
            rospy.logerr(f"Modbus错误（发送JOG）：{e}")
            return False
        except Exception as e:
            rospy.logerr(f"未知错误（发送JOG）：{str(e)}")
            return False

    def motor_control_loop(self):
        """电机控制主循环"""
        global motor_is_running, motor_current_direction
        last_jog_send_time = time.time()

        while not rospy.is_shutdown():
            self.read_motor_sensor()
            if motor_is_running:
                current_time = time.time()

                # 定时发送JOG指令
                if current_time - last_jog_send_time >= JOG_INTERVAL:
                    if motor_current_direction == "FORWARD":
                        self.send_motor_jog_cmd(FORWARD_JOG_CMD)
                        rospy.logdebug("发送正向JOG指令")
                    elif motor_current_direction == "REVERSE":
                        self.send_motor_jog_cmd(REVERSE_JOG_CMD)
                        rospy.logdebug("发送反向JOG指令")
                    last_jog_send_time = current_time
                # 前进到位
                if motor_current_direction == "FORWARD" and (self.read_motor_sensor() == SENSOR_TRIGGERED_MASK or self.read_motor_sensor() == SENSOR_LIMITED_MASK):
                    motor_current_direction = "STOP"
                    rospy.loginfo("前进至传感器触发,STOP")

                # 后退到位
                if motor_current_direction == "REVERSE" and self.read_motor_sensor() == SENSOR_RESET_MASK :
                    motor_current_direction = "STOP"
                    rospy.loginfo("后退至传感器触发,STOP")

                # 发布电机运行状态
                self.motor_status_pub.publish(Bool(data=True))
                
            else:
                # 停止电机
                self.send_motor_jog_cmd(STOP_CMD)
                motor_current_direction = "STOP"
                self.motor_status_pub.publish(Bool(data=False))
                # 2. 读取传感器状态并发布（核心新增逻辑）
                sensor_trigger_value = self.io_status
                

            # 构建UInt8消息并发布
            sensors_msg = UInt8()
            sensors_msg.data = self.io_status
            self.proximity_sensors_pub.publish(sensors_msg)
            time.sleep(MOTOR_LOOP_INTERVAL)

    # ===================== 电机控制服务（正向/反向/停止） =====================
    def start_forward_callback(self, req):
        """启动正向JOG服务"""
        global motor_is_running, motor_current_direction
        if not motor_is_running or motor_current_direction != "FORWARD":
            motor_is_running = True
            motor_current_direction = "FORWARD"
            rospy.loginfo("启动正向JOG运行")
            return TriggerResponse(success=True, message="Forward JOG started")
        else:
            return TriggerResponse(success=False, message="Forward JOG is already running")

    def start_reverse_callback(self, req):
        """启动反向JOG服务"""
        global motor_is_running, motor_current_direction
        if not motor_is_running or motor_current_direction != "REVERSE":
            motor_is_running = True
            motor_current_direction = "REVERSE"
            rospy.loginfo("启动反向JOG运行")
            return TriggerResponse(success=True, message="Reverse JOG started")
        else:
            return TriggerResponse(success=False, message="Reverse JOG is already running")

    def stop_jog_callback(self, req):
        """停止JOG服务"""
        global motor_is_running
        if motor_is_running:
            motor_is_running = False
            rospy.loginfo("停止JOG运行")
            return TriggerResponse(success=True, message="JOG stopped")
        else:
            return TriggerResponse(success=False, message="JOG is not running")

    # ===================== 气象站相关方法 =====================
    def read_registers(self, addr, count, slave_id):
        """读取保持寄存器"""
        try:
            with modbus_lock:
                response = client.read_holding_registers(
                    address=addr,
                    count=count,
                    slave=slave_id
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

    def weather_publish_loop(self):
        """气象站数据发布循环"""
        rate = rospy.Rate(WEATHER_PUBLISH_RATE)
        while not rospy.is_shutdown():
            msg = Environment()
            msg.stamp = rospy.Time.now()
            
            # 读取风速
            wind_data = self.read_registers(REG_WIND_SPEED, 1, WEATHER_SLAVE_ID)
            if wind_data:
                msg.wind_speed = wind_data[0] / 10.0
            else:
                rospy.logwarn("风速数据读取失败")
            
            # 读取风向
            dir_data = self.read_registers(REG_WIND_DIR, 1, WEATHER_SLAVE_ID)
            if dir_data:
                msg.wind_direction = dir_data[0]
            else:
                rospy.logwarn("风向数据读取失败")
            
            # 读取光照
            lux_data = self.read_registers(REG_LUX_HIGH, 2, WEATHER_SLAVE_ID)
            if lux_data and len(lux_data) == 2:
                msg.illuminance = (lux_data[0] << 16) | lux_data[1]
            else:
                rospy.logwarn("光照数据读取失败")
            
            # 读取雨量
            rain_data = self.read_registers(REG_RAINFALL, 1, WEATHER_SLAVE_ID)
            if rain_data:
                msg.rainfall = rain_data[0] / 10.0
            else:
                rospy.logwarn("雨量数据读取失败")
            
            self.weather_pub.publish(msg)
            rate.sleep()

    # ===================== MQTT JSON消息解析与处理 =====================
    def mqtt_state_callback(self, msg):
        """解析MQTT JSON消息并控制电机"""
        global mqtt_completed, mqtt_running_state
        try:
            # 解析JSON字符串
            mqtt_data = json.loads(msg.data)
            
            # 提取关键字段（容错处理，避免字段缺失报错）
            mqtt_completed = mqtt_data.get("complete_state", False)
            mqtt_running_state = mqtt_data.get("status", "")
            
            rospy.logdebug(f"解析MQTT消息：complete_state={mqtt_completed}, status={mqtt_running_state}")

            # 根据状态控制电机
            # 1. complete_state为True：启动正向JOG
            if mqtt_completed:
                rospy.loginfo("MQTT指令：complete_state=True，启动正向JOG")
                self.call_ros_service("/start_forward_jog")
            
            # 2. status为"START"：启动反向JOG
            if mqtt_running_state == "START":
                rospy.loginfo("MQTT指令：status=START，启动反向JOG")
                self.call_ros_service("/start_reverse_jog")

        except json.JSONDecodeError as e:
            rospy.logwarn(f"MQTT消息JSON解析失败：{str(e)}，原始消息：{msg.data}")
        except Exception as e:
            rospy.logwarn(f"MQTT消息处理异常：{str(e)}")

    def call_ros_service(self, service_name):
        """封装ROS服务调用（避免subprocess依赖）"""
        try:
            # 等待服务可用
            rospy.wait_for_service(service_name, timeout=1.0)
            # 创建服务代理
            service_proxy = rospy.ServiceProxy(service_name, Trigger)
            # 调用服务
            response = service_proxy()
            if response.success:
                rospy.loginfo(f"调用服务{service_name}成功：{response.message}")
            else:
                rospy.logwarn(f"调用服务{service_name}失败：{response.message}")
        except rospy.ServiceException as e:
            rospy.logwarn(f"服务{service_name}调用异常：{str(e)}")
        except rospy.ROSException as e:
            rospy.logwarn(f"服务{service_name}不可用：{str(e)}")

    def __del__(self):
        """析构函数：关闭资源"""
        global client
        if client:
            client.close()
            rospy.loginfo("Modbus串口已关闭")

if __name__ == '__main__':
    try:
        node = JogWeatherControlNode()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"节点启动失败：{str(e)}")