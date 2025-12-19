#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import threading
from motor_controller import MotorController  # 导入控制类

def main():
    controller = None
    try:
        # 初始化控制器
        controller = MotorController()
        rospy.loginfo("🚀 机器人控制器初始化完成")
        
        # 加载电机配置
        rospy.loginfo("⚙️  加载电机配置...")
        controller._configure_motors_from_config()
        
        # 启动键盘监听线程
        rospy.loginfo("⌨️  启动键盘监听...")
        keyboard_thread = threading.Thread(target=controller.keyboard_listener, daemon=True)
        keyboard_thread.start()
        
        # 主循环
        rospy.loginfo("🔄 进入主控制循环...")
        while not rospy.is_shutdown():
            controller.execute_state()
            controller.rate.sleep()
            
    except KeyboardInterrupt:
        rospy.loginfo("🛑 用户终止程序")
    except Exception as e:
        rospy.logfatal(f"💥 程序启动失败：{e}")
        import traceback
        traceback.print_exc()
    finally:
        # 安全关闭
        if controller is not None:
            controller.shutdown()

if __name__ == "__main__":
    main()