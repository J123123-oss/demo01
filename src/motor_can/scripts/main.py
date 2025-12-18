#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import threading
import sys
import select
from motor_driver import MotorDriverManager
from motor_controller import MotorController
import yaml

# def load_motor_config(config_file="/home/orangepi/demo01/src/motor_can/config/servo_config.yaml"):
def load_motor_config(config_file="/home/ubuntu/demo01/src/motor_can/config/servo_config.yaml"):
    """加载电机配置文件"""
    try:
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)
            return config.get("motors", [])
    except FileNotFoundError:
        rospy.logerr(f"❌ 配置文件未找到: {config_file}")
        return [
            {"motor_id": 1, "rtu_addr": 1, "pole_pairs": 5},
            {"motor_id": 2, "rtu_addr": 2, "pole_pairs": 5},
            {"motor_id": 3, "rtu_addr": 3, "pole_pairs": 5}
        ]
    except Exception as e:
        rospy.logerr(f"❌ 加载配置失败: {e}")
        return [
            {"motor_id": 1, "rtu_addr": 1, "pole_pairs": 5},
            {"motor_id": 2, "rtu_addr": 2, "pole_pairs": 5},
            {"motor_id": 3, "rtu_addr": 3, "pole_pairs": 5}
        ]

def keyboard_listener(controller):
    """键盘监听线程（独立于控制逻辑）"""
    rospy.loginfo("⌨️  按键控制：s=停止, f=前进, b=后退, a=启动, r=反转, l=加载, p=暂停, u=卸载, 1=上停, 2=下停")
    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0.1)[0]:
            key = sys.stdin.readline().strip()
            if key:
                update_status_by_key(controller, key)

def update_status_by_key(controller, key):
    """按键状态更新"""
    key_mapping = {
        's': "STOP", 'f': "FORWARD", 'b': "BACKWARD", 'a': "START",
        'r': "REVERSE", 'l': "LOADING", 'p': "PAUSE", 'u': "UNLOADING",
        '1': "UPSTOP", '2': "LOWSTOP"
    }
    if key in key_mapping:
        if key != 'a':
            controller.auto_mode = False
        else:
            controller.auto_mode = True
        controller.set_state(key_mapping[key])
    else:
        rospy.loginfo(f"⚠️  无效按键: {key}")

def main():
    rospy.init_node('motor_modbus_rtu_node', anonymous=True)
    rospy.loginfo("🚀 启动电机控制节点...")

    try:
        # 1. 加载电机配置
        motor_config = load_motor_config()
        if not motor_config:
            rospy.logfatal("❌ 无有效电机配置，退出程序")
            return

        # 2. 初始化驱动管理器
        driver_manager = MotorDriverManager(motor_config)
        if not driver_manager.connect_all():
            rospy.logfatal("❌ 部分电机驱动连接失败，退出程序")
            return

        # 3. 初始化控制器（注入驱动管理器）
        controller = MotorController(driver_manager)

        # 4. 启动键盘监听线程
        keyboard_thread = threading.Thread(target=keyboard_listener, args=(controller,), daemon=True)
        keyboard_thread.start()

        # 5. 主循环
        rospy.loginfo("✅ 系统初始化完成，进入主循环")
        while not rospy.is_shutdown():
            controller.execute_state()
            controller.rate.sleep()

    except KeyboardInterrupt:
        rospy.loginfo("🛑 用户终止程序")
    except Exception as e:
        rospy.logfatal(f"💥 程序异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 安全关闭
        if 'controller' in locals():
            controller.shutdown()
        if 'driver_manager' in locals():
            driver_manager.close_all()
        rospy.loginfo("✅ 程序已安全退出")

if __name__ == "__main__":
    main()