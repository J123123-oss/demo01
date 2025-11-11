#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import threading
import time
from std_msgs.msg import Header
from serial_comms.msg import WTRTK  # 替换为你的功能包名

class WTRTKFileDriver:
    def __init__(self):
        # 初始化节点
        rospy.init_node('wtrtk_file_driver', anonymous=True)
        
        # 读取参数（文本文件路径，默认当前目录下的wtrtk_data.txt）
        # 读取参数（参数名为~file_path，默认值为你的文件路径）
        self.file_path = rospy.get_param('~file_path', '/home/ubuntu/Downloads/室外1107-WTRTK-GNGGA.txt1')
        
        # 初始化消息发布者（话题名：/wtrtk_data）
        self.pub = rospy.Publisher('/wtrtk_data', WTRTK, queue_size=10)
        
        # 从文件读取所有WTRTK帧
        self.frames = self.load_frames_from_file()
        if not self.frames:
            rospy.logwarn("未从文件中读取到有效数据，将使用默认测试帧")
            # 添加默认测试帧（你提供的示例数据）
            self.frames = [
                # "$WTRTK,-0.473,-0.177,0.116,0.519,1.93,4.15,118.45,4,21,5,31,87,--,1,9.3,22.3,114,2,3019.26424643,N,12004.23352394,E,4.52,241.55,18.37*69"
                "$WTRTK,-0.521,-0.188,0.012,0.554,1.82,5.03,104.63,4,21,5,31,0,--,1,9.3,22.4,114,2,3019.26396878,N,12004.23357210,E,0.12,255.37,18.41*54"
                # "$WTRTK,-0.515,-0.179,0.033,0.547,1.82,5.22,112.23,4,21,5,31,1703,--,1,9.3,22.3,114,2,3019.26385001,N,12004.23373081,E,0.19,247.77,18.46*6D"
            ]
        
        # 当前帧索引（循环读取文件中的帧）
        self.frame_index = 0
        
        # 1Hz发布控制
        self.publish_thread = threading.Thread(target=self.publish_loop, daemon=True)
        self.publish_thread.start()
        
        rospy.loginfo("WTRTK文件驱动启动，从文件读取数据: %s", self.file_path)
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
    def load_frames_from_file(self):
        """从文本文件中加载所有$WTRTK帧"""
        frames = []
        try:
            with open(self.file_path, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if line.startswith("$WTRTK"):
                        frames.append(line)
            rospy.loginfo(f"从文件中读取到 {len(frames)} 个WTRTK帧")
        except Exception as e:
            rospy.logwarn(f"读取文件失败: {str(e)}")
        return frames

    def parse_wtrtk(self, frame):
        """解析$WTRTK帧（根据实际字段数量调整）"""
        if not frame.startswith("$WTRTK"):
            return None
        
        # 分割帧头和校验位
        star_pos = frame.find('*')
        if star_pos == -1:
            rospy.logwarn("无效的WTRTK帧（无校验位）")
            return None
        
        # 提取内容字段（$WTRTK,后到*前的部分）
        content = frame[7:star_pos]
        fields = content.split(',')
        
        # 打印字段数量（调试用）
        rospy.logdebug(f"解析到字段数量: {len(fields)}，内容: {fields}")
        
        # 检查字段数量（根据实际文本调整，你的示例数据是25个字段）
        if len(fields) != 25:
            rospy.logwarn(f"字段数量不匹配: {len(fields)}（预期25）")
            return None
        
        # 构造消息
        msg = WTRTK()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "wtrtk_link"
        
        try:
            # 按示例数据顺序填充字段（根据实际字段含义调整索引）
            msg.diff_x = float(fields[0])       # 差分X: -0.515
            msg.diff_y = float(fields[1])       # 差分Y: -0.179
            msg.diff_z = float(fields[2])       # 差分Z: 0.033
            msg.diff_r = float(fields[3])       # 差分R: 0.547
            msg.angle_x = float(fields[4])      # 角度X: 1.82
            msg.angle_y = float(fields[5])      # 角度Y: 5.22
            msg.angle_z = float(fields[6])      # 角度Z: 112.23
            msg.fix_status = int(fields[7])     # 定向状态: 4（固定解）
            msg.wireless_status = int(fields[8])# 无线状态: 21（已连接）
            msg.ntrip_status = int(fields[9])   # Ntrip状态:5（连接成功）
            msg.signal_quality = int(fields[10])# 信号质量:31
            msg.data_rate = int(fields[11])     # 数据量:1703
            msg.gps_heading = fields[12]        # GPS航向角:--
            msg.calib_flag = int(fields[13])    # 校准标志:1
            msg.battery_voltage = float(fields[14])# 电池电压:9.3V
            msg.temperature = float(fields[15]) # 温度:22.3℃
            msg.base_distance = int(fields[16]) # 基站距离:114米
            msg.ins_flag = int(fields[17])      # 惯导标志:2
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
            msg.ins_speed = float(fields[22])   # 惯导地速:0.19km/h
            msg.ins_heading = float(fields[23]) # 惯导航向角:247.77°
            msg.ins_altitude = float(fields[24])# 惯导高度:18.46米
        except (ValueError, IndexError) as e:
            rospy.logwarn(f"解析字段失败: {str(e)}")
            return None
        
        return msg

    def publish_loop(self):
        """1Hz频率发布数据（循环读取文件中的帧）"""
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            # 循环获取帧（索引越界时重置）
            frame = self.frames[self.frame_index]
            self.frame_index = (self.frame_index + 1) % len(self.frames)
            
            # 解析并发布
            parsed_msg = self.parse_wtrtk(frame)
            if parsed_msg:
                parsed_msg.header.stamp = rospy.Time.now()
                self.pub.publish(parsed_msg)
                rospy.loginfo(f"发布WTRTK数据（定向状态: {parsed_msg.fix_status}）")
            else:
                rospy.logwarn("解析失败，跳过当前帧")
            
            rate.sleep()

    def run(self):
        """保持节点运行"""
        rospy.spin()

if __name__ == '__main__':
    try:
        driver = WTRTKFileDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass