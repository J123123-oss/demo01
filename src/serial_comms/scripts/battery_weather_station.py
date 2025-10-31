#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
import struct
import time
from std_msgs.msg import Float32, Bool
from serial_comms.msg import BatteryStatus, Environment  # 新增Environment消息
from std_srvs.srv import Trigger, TriggerResponse
import threading
from pymodbus.client import ModbusSerialClient  # 新增Modbus客户端
from pymodbus.exceptions import ModbusException  # 新增Modbus异常处理

# 状态常量（删除继电器相关状态，仅保留电池状态）
STATE_READY = 0
STATE_WAITING_BATTERY = 1


class BatteryWeatherStationNode:
    def __init__(self):
        rospy.init_node('battery_weather_station_node')
        
        # -------------------------- 1. 基础参数初始化（删除继电器参数）--------------------------
        # 时间阈值管理（新增气象站轮询间隔）
        self.last_battery_sent = 0
        self.last_weather_check = 0  # 气象站上次轮询时间
        self.battery_buffer = bytearray()
        
        # 电池参数（保留原逻辑）
        self.battery_check_interval = rospy.get_param('~battery_check_interval', 5.0)
        self.battery_timeout = 0
        self.battery_retry_count = 0
        self.max_battery_retry = 3
        self.REQUEST_BASIC_FRAME = bytes.fromhex('DD A5 03 00 FF FD 77')  # 电池查询指令
        
        # -------------------------- 2. 气象站Modbus参数（新增）--------------------------
        # 气象站ROS参数（支持外部配置）
        self.weather_port = rospy.get_param('~weather_station_port', '/dev/ttyUSB14')
        self.weather_baudrate = rospy.get_param('~weather_station_baudrate', 9600)
        self.weather_slave_id = rospy.get_param('~weather_station_slave_id', 1)
        self.weather_check_interval = rospy.get_param('~weather_station_check_interval', 6.0)  # 气象站轮询间隔
        
        # 气象站寄存器地址（十进制，与原气象站代码一致）
        self.REG_WIND_SPEED = 500   # 风速（×10）
        self.REG_WIND_DIR = 503     # 风向
        self.REG_LUX_HIGH = 510     # 光照高16位
        self.REG_LUX_LOW = 511      # 光照低16位
        self.REG_RAINFALL = 513     # 雨量（×10）
        
        # -------------------------- 3. 硬件初始化（电池串口 + 气象站Modbus）--------------------------
        # 电池串口初始化（保留原逻辑，添加锁）
        self.serial_lock = threading.Lock()
        self.ser = None
        self.init_battery_serial(
            port=rospy.get_param('~battery_serial_port', '/dev/ttyUSB14'),
            baudrate=rospy.get_param('~battery_baudrate', 9600)
        )
        if not self.ser:
            rospy.signal_shutdown("电池串口初始化失败")
            return
        
        # 气象站Modbus客户端初始化（新增）
        self.weather_client = None
        self.init_weather_modbus()
        if not self.weather_client.is_socket_open():
            rospy.logwarn("气象站Modbus连接失败，将重试")
        
        # -------------------------- 4. 发布器与服务（删除继电器相关）--------------------------
        # 电池发布器（保留）
        self.battery_pub = rospy.Publisher('/battery_status', BatteryStatus, queue_size=10)
        self.temperature_pub = rospy.Publisher('/control_temperature', Float32, queue_size=10)
        
        # 气象站发布器（新增）
        self.weather_pub = rospy.Publisher('/environment_data', Environment, queue_size=10)
        
        # 仅保留电池状态查询服务（删除继电器服务）
        rospy.Service('~get_battery_statistics', Trigger, self.get_battery_stat_callback)
        
        # -------------------------- 5. 其他变量（保留电池统计，新增气象站状态）--------------------------
        self.rate = rospy.Rate(100)  # 100Hz循环
        self.current_temperatures = []
        self.battery_query_count = 0
        self.battery_success_count = 0
        self.current_state = STATE_READY  # 初始状态为就绪
        
        rospy.loginfo("电池-气象站集成节点初始化完成")

    # -------------------------- 电池串口相关（保留原逻辑，仅修改函数名避免混淆）--------------------------
    def init_battery_serial(self, port, baudrate):
        """电池串口初始化（原init_serial修改名）"""
        try:
            self.ser = serial.Serial(
                port=port, baudrate=baudrate, bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.05
            )
            rospy.loginfo(f"电池串口连接成功: {port}")
            return True
        except Exception as e:
            rospy.logerr(f"电池串口连接失败: {e}")
            return False

    def reinit_battery_serial(self):
        """电池串口重连（原reinit_serial修改名）"""
        try:
            if self.ser:
                self.ser.close()
            self.init_battery_serial(
                port=rospy.get_param('~battery_serial_port', '/dev/Battery'),
                baudrate=rospy.get_param('~battery_baudrate', 115200)
            )
            return True
        except Exception as e:
            rospy.logerr(f"电池串口重连失败: {e}")
            return False

    # -------------------------- 气象站Modbus相关（新增完整逻辑）--------------------------
    def init_weather_modbus(self):
        """气象站Modbus RTU客户端初始化"""
        self.weather_client = ModbusSerialClient(
            method='rtu',
            port=self.weather_port,
            baudrate=self.weather_baudrate,
            parity='N',
            stopbits=1,
            bytesize=8,
            timeout=1
        )
        # 尝试连接
        if self.weather_client.connect():
            rospy.loginfo(f"气象站Modbus连接成功: {self.weather_port} (从站ID: {self.weather_slave_id})")
        else:
            rospy.logerr(f"气象站Modbus连接失败: {self.weather_port}")

    def read_weather_registers(self, addr, count):
        """读取气象站Modbus寄存器（原read_registers迁移）"""
        # 检查连接，断开则重连
        if not self.weather_client.is_socket_open():
            rospy.logwarn("气象站Modbus连接断开，尝试重连")
            self.weather_client.connect()
            if not self.weather_client.is_socket_open():
                rospy.logerr("气象站Modbus重连失败")
                return None
        
        try:
            response = self.weather_client.read_holding_registers(
                address=addr, count=count, slave=self.weather_slave_id
            )
            if response.isError():
                rospy.logwarn(f"气象站寄存器读取错误: {response}")
                return None
            return response.registers
        except ModbusException as e:
            rospy.logwarn(f"气象站Modbus通信异常: {str(e)}")
            return None
        except Exception as e:
            rospy.logwarn(f"气象站数据读取未知错误: {str(e)}")
            return None

    def read_weather_data(self):
        """读取并发布气象站数据（整合原run中的气象站逻辑）"""
        msg = Environment()
        msg.stamp = rospy.Time.now()  # 时间戳
        
        # 1. 读取风速（×10 → 实际值）
        wind_speed_data = self.read_weather_registers(self.REG_WIND_SPEED, 1)
        if wind_speed_data:
            msg.wind_speed = wind_speed_data[0] / 10.0
        else:
            rospy.logwarn("气象站风速数据读取失败")
        
        # 2. 读取风向（直接为角度值）
        wind_dir_data = self.read_weather_registers(self.REG_WIND_DIR, 1)
        if wind_dir_data:
            msg.wind_direction = wind_dir_data[0]
        else:
            rospy.logwarn("气象站风向数据读取失败")
        
        # 3. 读取光照（高16位+低16位拼接32位值）
        lux_data = self.read_weather_registers(self.REG_LUX_HIGH, 2)
        if lux_data and len(lux_data) == 2:
            msg.illuminance = (lux_data[0] << 16) | lux_data[1]
        else:
            rospy.logwarn("气象站光照数据读取失败")
        
        # 4. 读取雨量（×10 → 实际值）
        rainfall_data = self.read_weather_registers(self.REG_RAINFALL, 1)
        if rainfall_data:
            msg.rainfall = rainfall_data[0] / 10.0
        else:
            rospy.logwarn("气象站雨量数据读取失败")
        
        # 发布气象站数据
        self.weather_pub.publish(msg)

    # -------------------------- 电池数据处理（保留原逻辑，删除继电器相关调用）--------------------------
    def calculate_checksum(self, check_bytes):
        """电池数据校验码计算（保留）"""
        total = sum(check_bytes)
        checksum = (-total) & 0xFFFF
        return checksum

    def parse_date(self, raw_date):
        """电池生产日期解析（保留）"""
        value = (raw_date[0] << 8) | raw_date[1]
        day = value & 0x1F
        month = (value >> 5) & 0x0F
        year = 2000 + (value >> 9)
        return year, month, day

    def parse_current(self, data_bytes):
        """电池电流解析（保留）"""
        value = (data_bytes[0] << 8) | data_bytes[1]
        return (value - 65536) * 0.01 if value >= 0x8000 else value * 0.01

    def process_battery_response(self, data_block, check_bytes):
        """电池数据解析与发布（保留，删除继电器控制调用）"""
        status_msg = BatteryStatus()
        try:
            if len(data_block) != 40:
                rospy.logerr(f"电池数据块长度错误，预期40字节，实际{len(data_block)}字节")
                return False
            
            # 校验与解析（保留原逻辑）
            data_segment = data_block[0:38]
            received_checksum = (data_block[38] << 8) | data_block[39]
            calc_checksum = self.calculate_checksum(check_bytes)
            if received_checksum != calc_checksum:
                rospy.logerr(f"电池校验码不匹配：接收0x{received_checksum:04X}，计算0x{calc_checksum:04X}")
                return False
            
            # 基础信息解析（保留）
            status_msg.total_voltage = ((data_segment[0] << 8) | data_segment[1]) * 0.01
            status_msg.current = self.parse_current(data_segment[2:4])
            status_msg.remaining_capacity = ((data_segment[4] << 8) | data_segment[5]) * 0.01
            status_msg.nominal_capacity = ((data_segment[6] << 8) | data_segment[7]) * 0.01
            status_msg.cycle_count = (data_segment[8] << 8) | data_segment[9]
            
            # 生产日期与状态解析（保留）
            year, month, day = self.parse_date(data_segment[10:12])
            status_msg.production_year = year
            status_msg.production_month = month
            status_msg.production_day = day
            status_msg.balance_low = (data_segment[12] << 8) | data_segment[13]
            status_msg.balance_high = (data_segment[14] << 8) | data_segment[15]
            status_msg.protection_status = (data_segment[16] << 8) | data_segment[17]
            
            # 版本与电量解析（保留）
            ver_major = data_segment[18] >> 4
            ver_minor = data_segment[18] & 0x0F
            status_msg.software_version = f"{ver_major}.{ver_minor}"
            status_msg.batttery_remaining = float(data_segment[19])
            status_msg.mos_state = data_segment[20]
            status_msg.battery_series = data_segment[21]
            
            # 温度解析与发布（保留）
            status_msg.ntc_count = data_segment[22]
            required_temp_len = 23 + 2 * status_msg.ntc_count
            if len(data_segment) < required_temp_len:
                rospy.logerr(f"电池温度数据不足，要求{required_temp_len}字节，实际{len(data_segment)}字节")
                return False
            
            self.current_temperatures.clear()
            for i in range(status_msg.ntc_count):
                idx = 23 + i * 2
                raw_temp = (data_segment[idx] << 8) | data_segment[idx + 1]
                temp_c = (raw_temp - 2731) / 10.0
                status_msg.temperatures.append(round(temp_c, 1))
                self.current_temperatures.append(round(temp_c, 1))
            
            # 发布电池与温度数据（保留）
            if self.current_temperatures:
                avg_temp = sum(self.current_temperatures) / len(self.current_temperatures)
                self.temperature_pub.publish(Float32(data=avg_temp))
            self.battery_pub.publish(status_msg)
            rospy.logdebug("电池数据解析成功并发布")
            return True
        except Exception as e:
            rospy.logerr(f"电池数据解析错误: {e}")
            return False

    def send_battery_query(self):
        """发送电池查询指令（保留，仅修改串口调用）"""
        try:
            if self.ser and self.ser.is_open:
                with self.serial_lock:
                    self.battery_buffer.clear()
                    self.ser.write(b'\x00')  # 清除缓存
                    time.sleep(0.01)
                    self.ser.write(self.REQUEST_BASIC_FRAME)
                rospy.logdebug("电池查询指令发送成功")
                return True
        except serial.SerialException as e:
            rospy.logerr(f"电池查询发送失败（串口异常）: {e}")
            self.reinit_battery_serial()
        except Exception as e:
            rospy.logerr(f"电池查询发送失败: {e}")
        return False

    def read_battery_serial_data(self):
        """读取电池串口数据（原read_serial_data修改，删除继电器数据处理）"""
        try:
            if self.ser and self.ser.is_open:
                with self.serial_lock:
                    for _ in range(3):  # 最多读取3次清空缓冲区
                        avail = self.ser.in_waiting
                        if avail == 0:
                            break
                        data = self.ser.read(avail)
                        hex_str = ' '.join(['%02X' % b for b in data])
                        rospy.loginfo(f"收到电池串口数据: {hex_str}")
                        
                        # 仅处理电池帧（0xDD开头）
                        for byte in data:
                            byte_val = byte if isinstance(byte, int) else ord(byte)
                            if byte_val == 0xDD:
                                if len(self.battery_buffer) > 0:
                                    self.process_battery_buffer()  # 解析旧数据
                                self.battery_buffer.clear()
                                self.battery_buffer.append(byte_val)
                            elif len(self.battery_buffer) > 0 and self.battery_buffer[0] == 0xDD and len(self.battery_buffer) < 50:
                                self.battery_buffer.append(byte_val)
                        time.sleep(0.001)
        except serial.SerialException as e:
            rospy.logerr(f"电池串口读取异常: {e}")
            self.reinit_battery_serial()
        except Exception as e:
            rospy.logwarn(f"电池串口读取错误: {e}")

    def process_battery_buffer(self):
        """处理电池缓冲区数据（保留原逻辑）"""
        MAX_BATTERY_FRAME_LEN = 64
        if len(self.battery_buffer) > MAX_BATTERY_FRAME_LEN:
            rospy.logwarn(f"电池缓冲区过长，清空数据: {len(self.battery_buffer)}字节")
            self.battery_buffer.clear()
            return False
        
        start_idx = 0
        while start_idx < len(self.battery_buffer):
            if self.battery_buffer[start_idx] != 0xDD:
                start_idx += 1
                continue
            if start_idx + 3 >= len(self.battery_buffer):
                rospy.logdebug("电池帧头后字节不足，无法解析")
                return False
            if self.battery_buffer[start_idx+1] != 0x03 or self.battery_buffer[start_idx+2] != 0x00:
                rospy.logdebug("电池功能码/状态码错误，丢弃")
                start_idx += 1
                continue
            
            data_length = self.battery_buffer[start_idx+3]
            full_frame_len = 4 + data_length + 2 + 1  # 帧头4 + 数据段 + 校验2 + 帧尾1
            total_needed = start_idx + full_frame_len
            if len(self.battery_buffer) < total_needed:
                rospy.logdebug(f"电池帧未完整接收，当前{len(self.battery_buffer)}字节，需{total_needed}字节")
                return False
            if self.battery_buffer[total_needed - 1] != 0x77:
                rospy.logwarn("电池帧尾错误，丢弃")
                start_idx += 1
                continue
            
            # 提取校验范围与数据块
            check_bytes = self.battery_buffer[start_idx+2 : start_idx+4+data_length]
            data_block = self.battery_buffer[start_idx+4 : start_idx+4+data_length+2]
            success = self.process_battery_response(data_block, check_bytes)
            self.battery_buffer = self.battery_buffer[total_needed:]
            return success
        return False

    # -------------------------- 服务回调（仅保留电池统计查询）--------------------------
    def get_battery_stat_callback(self, req):
        """电池查询成功率统计服务（新增）"""
        response = TriggerResponse()
        if self.battery_query_count == 0:
            response.success = True
            response.message = "暂无电池查询数据"
            return response
        success_rate = (self.battery_success_count / self.battery_query_count) * 100
        response.success = True
        response.message = f"电池查询成功率: {success_rate:.2f}% (成功{self.battery_success_count}/总{self.battery_query_count})"
        return response

    # -------------------------- 主循环（修改为电池+气象站轮询）--------------------------
    def run(self):
        rospy.loginfo("节点主循环开始运行")
        while not rospy.is_shutdown():
            current_time = time.time()
            
            # 1. 优先读取电池串口数据
            self.read_battery_serial_data()
            
            # 2. 处理电池等待状态
            if self.current_state == STATE_WAITING_BATTERY:
                if self.process_battery_buffer():
                    self.current_state = STATE_READY
                    self.battery_success_count += 1
                elif current_time >= self.battery_timeout:
                    rospy.logwarn("电池响应超时，返回就绪状态")
                    self.battery_buffer.clear()
                    self.current_state = STATE_READY
            
            # 3. 就绪状态下轮询电池与气象站
            if self.current_state == STATE_READY:
                # 3.1 轮询电池（按间隔）
                if current_time - self.last_battery_sent >= self.battery_check_interval:
                    if self.send_battery_query():
                        self.last_battery_sent = current_time
                        self.battery_query_count += 1
                        self.current_state = STATE_WAITING_BATTERY
                        self.battery_timeout = current_time + 1.0  # 1秒超时
                    else:
                        rospy.logerr("电池查询发送失败")
                
                # 3.2 轮询气象站（按间隔，与电池并行不阻塞）
                elif current_time - self.last_weather_check >= self.weather_check_interval:
                    self.read_weather_data()  # 读取并发布气象站数据
                    self.last_weather_check = current_time
                    
                    # 打印电池统计（每轮询一次气象站输出一次）
                    if self.battery_query_count > 0:
                        success_rate = (self.battery_success_count / self.battery_query_count) * 100
                        rospy.loginfo(f"电池查询成功率: {success_rate:.2f}% ({self.battery_success_count}/{self.battery_query_count})")
            
            self.rate.sleep()
        
        # 节点退出清理
        if self.ser and self.ser.is_open:
            self.ser.close()
        if self.weather_client and self.weather_client.is_socket_open():
            self.weather_client.close()
        rospy.loginfo("节点退出，资源已释放")


if __name__ == '__main__':
    try:
        node = BatteryWeatherStationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"节点运行异常: {e}")