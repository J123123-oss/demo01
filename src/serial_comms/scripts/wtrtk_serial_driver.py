#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
import threading
import time
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix  #GNGGA解析结果
from serial_comms.msg import WTRTK  # 替换为你的功能包名

class WTRTKSerialDriver:
    def __init__(self):
        # 初始化节点
        rospy.init_node('wtrtk_serial_driver', anonymous=True)
        
        # 读取参数（默认端口和波特率）
        self.port = rospy.get_param('~port', '/dev/WTRTK')
        self.baud_rate = rospy.get_param('~baud', 460800)
        
        # 初始化串口
        self.ser = None
        self.connect_serial()
        
        # 新增：GNGGA消息发布者（话题名：/fix）
        self.fix_pub = rospy.Publisher('/fix', NavSatFix, queue_size=10)
        # WTRTK消息发布者（保持不变）
        self.wtrtk_pub = rospy.Publisher('/wtrtk_data', WTRTK, queue_size=10)
        
        self.buffer = ""  # 缓存串口数据，同时用于两种帧的解析
        
        # 线程与事件（保持不变，用于控制发布频率）
        self.publish_event = threading.Event()
        self.publish_thread = threading.Thread(target=self.publish_loop, daemon=True)
        self.publish_thread.start()
        self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.read_thread.start()
        
        rospy.loginfo("GNGGA + WTRTK serial driver started")

    def connect_serial(self):
        """连接串口设备"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=0.05,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            if self.ser.is_open:
                rospy.loginfo(f"Connected to {self.port} at {self.baud_rate} baud")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to open serial port {self.port}: {str(e)}")
            return False
    def parse_gngga(self, frame):
        """解析$GNGGA帧，返回sensor_msgs/NavSatFix消息"""
        if not frame.startswith("$GNGGA"):
            return None
        
        # 分割帧头和校验位（与WTRTK解析逻辑一致）
        star_pos = frame.find('*')
        if star_pos == -1:
            rospy.logwarn("Invalid GNGGA frame (no checksum)")
            return None
        
        # 提取内容字段（$GNGGA,后到*前的部分）
        content = frame[7:star_pos]
        fields = content.split(',')
        
        # GNGGA标准格式包含14个字段（不含帧头和校验位）
        if len(fields) < 14:
            rospy.logwarn(f"Invalid GNGGA fields count: {len(fields)} (expected >=14)")
            return None
        
        # 构造NavSatFix消息
        fix_msg = NavSatFix()
        fix_msg.header = Header()
        fix_msg.header.stamp = rospy.Time.now()
        fix_msg.header.frame_id = "gps"
        
        try:
            # 解析经纬度（度分格式→十进制）
            # 纬度：字段2（如"3032.04204"）+ 字段3（N/S）
            lat_dms = fields[2]
            lat_flag = fields[3]
            latitude = self.dms_to_decimal(lat_dms, is_latitude=True)
            if latitude is not None and lat_flag == 'S':
                latitude = -latitude  # 南纬为负
            
            # 经度：字段4（如"12006.94560"）+ 字段5（E/W）
            lon_dms = fields[4]
            lon_flag = fields[5]
            longitude = self.dms_to_decimal(lon_dms, is_latitude=False)
            if longitude is not None and lon_flag == 'W':
                longitude = -longitude  # 西经为负
            
            # 解析海拔（字段9：海拔值，字段10：单位，通常为M）
            altitude = float(fields[9]) if fields[9] else 0.0
            
            # 定位状态（字段6：0=未定位，1=单点定位，2=差分定位，4=固定解，5=浮点解）
            fix_status = int(fields[6]) if fields[6] else 0
            
            # 填充消息
            fix_msg.latitude = latitude if latitude is not None else 0.0
            fix_msg.longitude = longitude if longitude is not None else 0.0
            fix_msg.altitude = altitude
            fix_msg.status.status = fix_status  # 定位状态
            fix_msg.status.service = 1  # 表示GPS服务
            
            #  covariance（可选，根据实际精度填充）
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            fix_msg.position_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 1.0]  # 示例值
            
        except (ValueError, IndexError) as e:
            rospy.logwarn(f"Failed to parse GNGGA fields: {str(e)}")
            return None
        
        return fix_msg
    def dms_to_decimal(self, dms_str, is_latitude=True):
        """
        将度分格式（DDMM.MMMMM）转换为十进制格式（DD.DDDDD°）
        :param dms_str: 度分字符串（如"3019.26385001"表示30°19.26385001'）
        :param is_latitude: 是否为纬度（用于校验范围）
        :return: 十进制角度（float），转换失败返回None
        """
        try:
            dms = float(dms_str)
            # 提取度（整数部分）和分（小数部分）
            degrees = int(dms // 100)  # 3019.26385 → 30（3019//100=30）
            minutes = dms % 100         # 3019.26385 → 19.26385（3019%100=19.26385）
            # 转换公式：十进制 = 度 + 分/60
            decimal = degrees + minutes / 60.0
            
            # 校验范围（纬度：-90~90，经度：-180~180）
            if is_latitude:
                if not (-90 <= decimal <= 90):
                    rospy.logwarn(f"纬度超出范围: {decimal}")
                    return None
            else:
                if not (-180 <= decimal <= 180):
                    rospy.logwarn(f"经度超出范围: {decimal}")
                    return None
            return decimal
        except (ValueError, TypeError) as e:
            rospy.logwarn(f"经纬度转换失败: {dms_str}, 错误: {e}")
            return None
    def parse_wtrtk(self, frame):
        """解析$WTRTK帧（25个字段，不含帧头和校验位）"""
        if not frame.startswith("$WTRTK"):
            return None
        
        # 分割帧头和校验位
        star_pos = frame.find('*')
        if star_pos == -1:
            rospy.logwarn("Invalid WTRTK frame (no checksum)")
            return None
        
        # 提取内容字段（$WTRTK,后到*前的部分）
        content = frame[7:star_pos]
        fields = content.split(',')
        
        # 检查字段数量是否为25个
        if len(fields) != 25:
            rospy.logwarn(f"Invalid WTRTK fields count: {len(fields)} (expected 25)")
            return None
        
        # 构造消息
        msg = WTRTK()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "wtrtk_link"
        
        try:
            # 按协议顺序填充字段（根据实际25个字段调整索引）
            msg.diff_x = float(fields[0])       # 差分X
            msg.diff_y = float(fields[1])       # 差分Y
            msg.diff_z = float(fields[2])       # 差分Z
            msg.diff_r = float(fields[3])       # 差分R
            msg.angle_x = float(fields[4])      # 角度X
            msg.angle_y = float(fields[5])      # 角度Y
            msg.angle_z = float(fields[6])      # 角度Z
            msg.fix_status = int(fields[7])     # 定向状态
            msg.wireless_status = int(fields[8])# 无线连接状态
            msg.ntrip_status = int(fields[9])   # Ntrip状态
            msg.signal_quality = int(fields[10])# 信号质量
            msg.data_rate = int(fields[11])     # 数据量
            msg.gps_heading = fields[12]        # GPS航向角
            msg.calib_flag = int(fields[13])    # 校准标志
            msg.battery_voltage = float(fields[14])# 电池电压
            msg.temperature = float(fields[15]) # 温度
            msg.base_distance = int(fields[16]) # 基站距离
            msg.ins_flag = int(fields[17])      # 惯导标志
            # 经纬度转换（核心修改）
            # 1. 纬度转换（度分→十进制）
            lat_dms = fields[18]  # 度分格式："3019.26385001"
            msg.ins_latitude = self.dms_to_decimal(lat_dms, is_latitude=True)
            # 2. 纬度标志（N/S）
            msg.lat_flag = fields[19]
            # 3. 经度转换（度分→十进制）
            lon_dms = fields[20]  # 度分格式："12004.23373081"
            msg.ins_longitude = self.dms_to_decimal(lon_dms, is_latitude=False)
            # 4. 经度标志（E/W）
            msg.lon_flag = fields[21]
            msg.ins_speed = float(fields[22])   # 惯导地速
            msg.ins_heading = float(fields[23]) # 惯导航向角
            msg.ins_altitude = float(fields[24])# 惯导高度
        except (ValueError, IndexError) as e:
            rospy.logwarn(f"Failed to parse WTRTK fields: {str(e)}")
            return None
        
        return msg
    def publish_loop(self):
        """1Hz频率发布GNGGA和WTRTK数据"""
        last_fix = None
        last_wtrtk = None
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            self.publish_event.wait(timeout=1.0)
            self.publish_event.clear()
            
            # 更新最新消息缓存
            if hasattr(self, 'latest_fix'):
                last_fix = self.latest_fix
            if hasattr(self, 'latest_wtrtk'):
                last_wtrtk = self.latest_wtrtk
            
            # 发布GNGGA解析结果
            if last_fix:
                last_fix.header.stamp = rospy.Time.now()
                self.fix_pub.publish(last_fix)
                rospy.logdebug(f"Published GNGGA (fix status: {last_fix.status.status})")
            
            # 发布WTRTK解析结果
            if last_wtrtk:
                last_wtrtk.header.stamp = rospy.Time.now()
                self.wtrtk_pub.publish(last_wtrtk)
                rospy.logdebug(f"Published WTRTK (fix status: {last_wtrtk.fix_status})")
            
            rate.sleep()
    def read_serial(self):
        """持续读取串口数据，同时解析GNGGA和WTRTK帧"""
        while not rospy.is_shutdown():
            if not self.ser or not self.ser.is_open:
                rospy.logwarn("Serial port closed, reconnecting...")
                if not self.connect_serial():
                    time.sleep(1)
                    continue
            
            try:
                data = self.ser.read(1024)
                if data:
                    # 解码并缓存数据（保留无效字符替换，避免分割错误）
                    self.buffer += data.decode('utf-8', errors='replace')
                    
                    # 循环处理缓存中的所有完整帧（同时支持GNGGA和WTRTK）
                    while True:
                        # 查找两种帧的起始位置
                        gngga_start = self.buffer.find('$GNGGA')
                        wtrtk_start = self.buffer.find('$WTRTK')
                        
                        # 没有任何帧起始，退出循环
                        if gngga_start == -1 and wtrtk_start == -1:
                            break
                        
                        # 选择最早出现的帧进行处理
                        if gngga_start != -1 and (wtrtk_start == -1 or gngga_start < wtrtk_start):
                            # 处理GNGGA帧
                            start_idx = gngga_start
                            end_idx = self.buffer.find('\r\n', start_idx)
                            if end_idx == -1:
                                break  # 未找到帧尾，等待下一次数据
                            frame = self.buffer[start_idx:end_idx]
                            self.buffer = self.buffer[end_idx+2:]  # 移除已处理部分
                            parsed_fix = self.parse_gngga(frame)
                            if parsed_fix:
                                self.latest_fix = parsed_fix  # 缓存最新GNGGA消息
                                self.publish_event.set()
                        else:
                            # 处理WTRTK帧（复用原有逻辑）
                            start_idx = wtrtk_start
                            end_idx = self.buffer.find('\r\n', start_idx)
                            if end_idx == -1:
                                break
                            frame = self.buffer[start_idx:end_idx]
                            self.buffer = self.buffer[end_idx+2:]
                            parsed_wtrtk = self.parse_wtrtk(frame)
                            if parsed_wtrtk:
                                self.latest_wtrtk = parsed_wtrtk  # 缓存最新WTRTK消息
                                self.publish_event.set()
            
            except Exception as e:
                rospy.logerr(f"Serial read error: {str(e)}")
                self.ser.close()
                time.sleep(1)

    def run(self):
        """保持节点运行"""
        rospy.spin()

if __name__ == '__main__':
    try:
        driver = WTRTKSerialDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass