#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import time
import subprocess
from std_msgs.msg import Int8

class MotorController:
    def __init__(self):
        # 配置参数 - 电机1和电机2的GPIO引脚
        self.pin_map = {
            'motor1': [23, 25],  # 电机1的正反转引脚
            'motor2': [20, 22],   # 电机2的正反转引脚
        }
        
        # 初始化标志位
        self.gpio_initialized = False
        
        # 尝试初始化GPIO（最多重试3次）
        for attempt in range(3):
            try:
                self._init_gpio()
                self.gpio_initialized = True
                break
            except Exception as e:
                rospy.logwarn(f"GPIO初始化尝试 {attempt+1}/3 失败: {str(e)}")
                time.sleep(1)
        
        if not self.gpio_initialized:
            rospy.logerr("无法初始化GPIO，设备可能被占用")
            raise RuntimeError("GPIO初始化失败")

        # ROS订阅 - 现在只订阅一个话题
        rospy.Subscriber('motor_cmd', Int8, self.motor_cb)
        rospy.loginfo("电机控制器初始化完成")

    def _init_gpio(self):
        """初始化GPIO资源，使用 subprocess 设置引脚模式"""
        # 初始化所有电机引脚
        for motor, pins in self.pin_map.items():
            for pin in pins:
                self._set_gpio_mode(pin)

    def _set_gpio_mode(self, pin):
        """使用 subprocess 设置 GPIO 引脚模式"""
        try:
            # 设置引脚为输出模式
            subprocess.run(['gpio', 'mode', str(pin), 'out'], check=True)
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"无法设置GPIO {pin} 模式: {str(e)}")

    def set_motors(self, direction):
        """同时设置两个电机的转动方向"""
        if not self.gpio_initialized:
            return
            
        try:
            # 电机1的引脚
            m1_fwd, m1_rev = self.pin_map['motor1']
            # 电机2的引脚
            m2_fwd, m2_rev = self.pin_map['motor2']
            
            # 设置两个电机转动方向
            if direction == -1:    # 正转，放下去
                # 电机1
                subprocess.run(['gpio', 'write', str(m1_fwd), '1'], check=True)
                subprocess.run(['gpio', 'write', str(m1_rev), '0'], check=True)
                # 电机2
                subprocess.run(['gpio', 'write', str(m2_fwd), '1'], check=True)
                subprocess.run(['gpio', 'write', str(m2_rev), '0'], check=True)
                rospy.loginfo("双电机反转")
            elif direction == 1: # 反转，抬起来
                # 电机1
                subprocess.run(['gpio', 'write', str(m1_fwd), '0'], check=True)
                subprocess.run(['gpio', 'write', str(m1_rev), '1'], check=True)
                # 电机2
                subprocess.run(['gpio', 'write', str(m2_fwd), '0'], check=True)
                subprocess.run(['gpio', 'write', str(m2_rev), '1'], check=True)
                rospy.loginfo("双电机正转")
            else:                # 停止== 脱机
                # 电机1
                subprocess.run(['gpio', 'write', str(m1_fwd), '1'], check=True)
                subprocess.run(['gpio', 'write', str(m1_rev), '1'], check=True)
                # 电机2
                subprocess.run(['gpio', 'write', str(m2_fwd), '1'], check=True)
                subprocess.run(['gpio', 'write', str(m2_rev), '1'], check=True)
                rospy.loginfo("双电机停止")
        except Exception as e:
            rospy.logerr(f"设置电机状态失败: {str(e)}")

    def motor_cb(self, msg):
        """回调函数，同时控制两个电机"""
        self.set_motors(msg.data)

    def cleanup(self):
        """安全释放所有GPIO资源"""
        if self.gpio_initialized:
            for motor, pins in self.pin_map.items():
                for pin in pins:
                    try:
                        subprocess.run(['gpio', 'write', str(pin), '1'], check=True)
                    except Exception as e:
                        rospy.logerr(f"清理GPIO {pin} 失败: {str(e)}")
        rospy.loginfo("GPIO资源已清理")

if __name__ == '__main__':
    rospy.init_node('motor_controller')
    mc = None
    try:
        mc = MotorController()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"节点运行错误: {str(e)}")
    finally:
        if mc:
            mc.cleanup()
        rospy.loginfo("电机控制器已关闭")