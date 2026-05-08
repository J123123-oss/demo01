#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import can
import time
import rospy
import json
import threading
import sys
import select
from std_msgs.msg import String, UInt8
from std_srvs.srv import Trigger
from serial_comms.msg import Environment

class ServoDriveController:
    def __init__(self, channel='vcan0', interface='socketcan'):
        # CAN 基础
        self.channel = channel
        self.interface = interface
        self.bus = self.create_can_bus()

        # 状态
        self.current_status = "STOP"
        self.last_mqtt_completed = False
        self.last_mqtt_status = ""
        self.last_mqtt_full_charge = False
        self.last_sensors = 0

        # 传感器与环境
        self.sensors = 0
        self.wind_speed = None
        self.wind_direction = None
        self.illuminance = None
        self.rainfall = None

        # 发布频率定时器
        self.publish_timer = None
        self.set_publish_freq(fast=True)  # 初始快速发布

        # ROS 订阅与发布
        self.state_pub = rospy.Publisher('/robot_state', String, queue_size=10)
        rospy.Subscriber('/robot_cmd', String, self.mqtt_state_callback)
        rospy.Subscriber('/sensors', UInt8, self.sensors_callback)
        rospy.Subscriber('/environment_data', Environment, self.environment_data_callback)

        # MQTT 重发定时器
        self.mqtt_state_timer = rospy.Timer(rospy.Duration(10.0), self.mqtt_state_timer_callback)
        self.last_mqtt_msg_data = None

    # ------------------------------
    # 环境/传感器回调（保留基础）
    # ------------------------------
    def environment_data_callback(self, msg):
        self.wind_speed = msg.wind_speed
        self.wind_direction = msg.wind_direction
        self.illuminance = msg.illuminance
        self.rainfall = msg.rainfall

    def sensors_callback(self, msg):
        self.sensors = msg.data
        # 触发停机
        if (self.sensors & 0x02 and not self.last_sensors & 0x02) or \
           (self.sensors & 0x08 and not self.last_sensors & 0x08):
            self.set_state("STOP")
        self.last_sensors = self.sensors

    # ------------------------------
    # MQTT 状态回调 + JOG 控制（核心）
    # ------------------------------
    def mqtt_state_callback(self, msg):
        self.last_mqtt_msg_data = msg.data
        self.process_mqtt_state(msg.data)

    def mqtt_state_timer_callback(self, event):
        if self.last_mqtt_msg_data:
            rospy.loginfo("定时器重处理MQTT状态")
            self.process_mqtt_state(self.last_mqtt_msg_data)

    def process_mqtt_state(self, msg_data):
        try:
            data = json.loads(msg_data)
            if "complete_state" not in data and "status" not in data:
                return

            completed = data.get("complete_state", False)
            status = data.get("status", "START")
            full_charge = data.get("full_charge", False)

            # 上升沿
            complete_rising = completed and not self.last_mqtt_completed
            status_changed = status != self.last_mqtt_status
            full_charge_rising = full_charge and not self.last_mqtt_full_charge

            # 正向 JOG
            if complete_rising and status != "START":
                rospy.loginfo("MQTT：正向JOG")
                self.call_ros_service("/start_forward_jog")
                self.set_state("START")

            # 反向 JOG
            elif (status == "START" and status_changed) or \
                 (status == "UNLOADING" and status_changed) or \
                 (status == "LOADING" and status_changed) or \
                 full_charge_rising:
                rospy.loginfo("MQTT：反向JOG")
                self.call_ros_service("/start_reverse_jog")
                self.set_state("STOP")

            # 更新历史
            self.last_mqtt_completed = completed
            self.last_mqtt_status = status
            self.last_mqtt_full_charge = full_charge

        except json.JSONDecodeError:
            rospy.logwarn("MQTT JSON解析失败")
        except Exception as e:
            rospy.logwarn(f"MQTT处理异常: {e}")

    def call_ros_service(self, srv_name):
        try:
            rospy.wait_for_service(srv_name, timeout=1.0)
            proxy = ro.ServiceProxy(srv_name, Trigger)
            res = proxy()
            rospy.loginfo(f"服务{srv_name}调用成功" if res.success else f"服务{srv_name}失败")
        except Exception as e:
            rospy.logwarn(f"服务调用失败: {e}")

    # ------------------------------
    # 状态设置 + 发布频率切换（核心）
    # ------------------------------
    def set_state(self, new_state):
        if new_state == self.current_status:
            return
        self.current_status = new_state
        rospy.loginfo(f"状态: {new_state}")

        # 频率切换：停止→慢速；运动→快速
        if new_state == "STOP":
            self.set_publish_freq(fast=True)
            # 180秒后再切更慢
            threading.Thread(target=self.delayed_slow_freq, daemon=True).start()
        else:
            self.set_publish_freq(fast=True)

    def set_publish_freq(self, fast=True):
        if self.publish_timer:
            self.publish_timer.shutdown()
        # 运行/刚停止：5秒；停止长时间后：300秒
        interval = 5.0 if fast else 300.0
        self.publish_timer = rospy.Timer(rospy.Duration(interval), self.publish_state)

    def delayed_slow_freq(self, delay=180):
        time.sleep(delay)
        if self.current_status == "STOP":
            self.set_publish_freq(fast=False)

    # ------------------------------
    # 状态发布
    # ------------------------------
    def publish_state(self, event=None):
        try:
            msg = {
                "sensors": self.sensors,
                "wind_speed": self.wind_speed,
                "wind_direction": self.wind_direction,
                "illuminance": self.illuminance,
                "rainfall": self.rainfall,
                "status": self.current_status
            }
            self.state_pub.publish(json.dumps(msg))
        except Exception:
            self.state_pub.publish(json.dumps({"status": "ERROR"}))

    # ------------------------------
    # CAN 基础（最简保留）
    # ------------------------------
    def create_can_bus(self):
        while True:
            try:
                return can.interface.Bus(channel=self.channel, interface=self.interface)
            except Exception as e:
                rospy.logerr(f"CAN连接失败: {e}，重试中")
                time.sleep(3)

    def shutdown(self):
        rospy.loginfo("关闭控制器")
        if self.bus:
            self.bus.shutdown()

# ------------------------------
# 主函数
# ------------------------------
def main():
    rospy.init_node("motor_can_simple")
    ctrl = ServoDriveController()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        ctrl.shutdown()

if __name__ == "__main__":
    main()