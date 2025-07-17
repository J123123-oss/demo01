#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from serial_comms.msg import Distances  
import can

class UltrasonicDistanceNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('distance_can', anonymous=True)
        
        # 创建Publisher
        self.pub = rospy.Publisher('distance_data', Distances, queue_size=10)
        
        # 初始化距离消息
        self.distance_msg = Distances()
        
        # 设置CAN总线
        self.setup_can_bus()
        
        # 设置发布频率
        self.rate = rospy.Rate(10)  # 10Hz
        
    def setup_can_bus(self):
        # 配置CAN总线 (1M波特率)
        try:
            self.bus = can.interface.Bus(
                channel='can0',  # 根据实际情况修改
                interface='socketcan',
                bitrate=1000000
            )
            rospy.loginfo("CAN总线初始化成功")
        except Exception as e:
            rospy.logerr(f"CAN总线初始化失败: {e}")
            raise
            
    def parse_can_message(self, msg):
        # 处理ID为0x101的消息 (包含A,B,C,D距离)
        if msg.arbitration_id == 0x101:
            if len(msg.data) >= 8:
                self.distance_msg.distance_a = (msg.data[0] << 8) | msg.data[1]
                self.distance_msg.distance_b = (msg.data[2] << 8) | msg.data[3]
                self.distance_msg.distance_c = (msg.data[4] << 8) | msg.data[5]
                self.distance_msg.distance_d = (msg.data[6] << 8) | msg.data[7]
        
        # 处理ID为0x102的消息 (包含E,F距离)
        elif msg.arbitration_id == 0x102:
            if len(msg.data) >= 4:
                self.distance_msg.distance_e = (msg.data[0] << 8) | msg.data[1]
                self.distance_msg.distance_f = (msg.data[2] << 8) | msg.data[3]
    
    def run(self):
        while not rospy.is_shutdown():
            try:
                # 接收CAN消息 (超时0.1秒)
                msg = self.bus.recv(timeout=0.5)
                
                if msg is not None:
                    self.parse_can_message(msg)
                    # 发布距离消息
                    self.pub.publish(self.distance_msg)
                    rospy.logdebug("发布距离数据: %s", self.distance_msg)
                
                self.rate.sleep()
                
            except can.CanError as e:
                rospy.logwarn(f"CAN通信错误: {e}")
            except rospy.ROSInterruptException:
                rospy.loginfo("ROS中断，退出节点")
                break
            except Exception as e:
                rospy.logerr(f"意外错误: {e}")
                break
        
        # 关闭CAN总线
        self.bus.shutdown()

if __name__ == '__main__':
    try:
        node = UltrasonicDistanceNode()
        node.run()
    except rospy.ROSInterruptException:
        pass