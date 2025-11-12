#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
import threading
import time
from std_msgs.msg import Header
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
        
        # 初始化消息发布者（话题名：/wtrtk_data）
        self.pub = rospy.Publisher('/wtrtk_data', WTRTK, queue_size=10)
        
        # 缓存串口数据
        self.buffer = ""
        
        # 1Hz发布控制（使用事件锁控制频率）
        self.publish_event = threading.Event()
        self.publish_thread = threading.Thread(target=self.publish_loop, daemon=True)
        self.publish_thread.start()
        
        # 启动串口读取线程
        self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.read_thread.start()
        
        rospy.loginfo("WTRTK serial driver started")

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

    def read_serial(self):
        """持续读取串口数据并缓存，优化帧分割逻辑"""
        while not rospy.is_shutdown():
            if not self.ser or not self.ser.is_open:
                rospy.logwarn("Serial port closed, reconnecting...")
                if not self.connect_serial():
                    time.sleep(1)
                    continue
            
            try:
                # 读取串口数据（非阻塞，一次最多读1024字节）
                data = self.ser.read(1024)
                if data:
                    rospy.logdebug(f"Raw data: {data.hex()}")  # 打印十六进制原始数据（便于观察帧结构）
                    rospy.logdebug(f"Decoded data: {data.decode('utf-8', errors='replace')}")  # 打印解码后的数据
                    # 解码时忽略无效字符，避免乱码导致分割错误
                    self.buffer += data.decode('utf-8', errors='replace')
                    
                    # 只处理包含完整$WTRTK帧的缓存（以$开头，\r\n结尾）
                    while '$WTRTK' in self.buffer and '\r\n' in self.buffer:
                        # 找到当前帧的起始位置
                        start_idx = self.buffer.find('$WTRTK')
                        # 找到当前帧的结束位置（从起始位置后找\r\n）
                        end_idx = self.buffer.find('\r\n', start_idx)
                        if end_idx == -1:
                            break  # 未找到完整帧尾，退出循环等待下一次数据
                        
                        # 提取完整帧（从start_idx到end_idx，包含$和\r\n前的内容）
                        frame = self.buffer[start_idx:end_idx]
                        # 移除已处理的部分（保留剩余缓存）
                        self.buffer = self.buffer[end_idx+2:]  # +2是跳过\r\n
                        
                        # 解析帧
                        parsed_msg = self.parse_wtrtk(frame)
                        if parsed_msg:
                            self.latest_msg = parsed_msg  # 缓存最新消息
                            self.publish_event.set()  # 通知发布线程
            except Exception as e:
                rospy.logerr(f"Serial read error: {str(e)}")
                self.ser.close()
                time.sleep(1)

    def publish_loop(self):
        """1Hz频率发布数据（即使无新数据也保持频率）"""
        last_msg = None
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            # 等待新数据或定时发布
            self.publish_event.wait(timeout=1.0)
            self.publish_event.clear()
            
            # 如果有新数据，更新缓存
            if hasattr(self, 'latest_msg'):
                last_msg = self.latest_msg
            
            # 发布最新数据（确保1Hz频率）
            if last_msg:
                last_msg.header.stamp = rospy.Time.now()  # 更新时间戳
                self.pub.publish(last_msg)
                rospy.logdebug(f"Published WTRTK data (fix status: {last_msg.fix_status})")
            
            rate.sleep()

    def run(self):
        """保持节点运行"""
        rospy.spin()

if __name__ == '__main__':
    try:
        driver = WTRTKSerialDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass