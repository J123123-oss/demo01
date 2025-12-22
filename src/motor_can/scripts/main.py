#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import threading
from motor_driver import CanMotorDriver
from motor_controller import RobotController

def main():
    try:
        # 初始化ROS节点
        rospy.init_node("motor_canopen_node")
        rospy.loginfo("🚀 启动机器人控制系统...")
        
        # 1. 初始化CAN电机驱动
        motor_driver = CanMotorDriver(channel='can0')
        
        # 2. 初始化机器人控制器（注入驱动实例）
        controller = RobotController(motor_driver=motor_driver)
        
        # 3. 加载电机配置
        config = RobotController.load_config()
        if not config or "motors" not in config or not config["motors"]:
            rospy.logerr("❌ 未找到有效电机配置，请检查配置文件")
            return
        
        # 4. 自动配置电机
        rospy.loginfo("⚙️  配置电机...")
        for motor in config["motors"]:
            motor_id = motor.get("id")
            velocity = motor.get("velocity")
            acceleration = motor.get("acceleration")
            deceleration = motor.get("deceleration")
            if None in (motor_id, velocity, acceleration, deceleration):
                rospy.logwarn(f"⚠️  跳过无效配置: {motor}")
                continue
            try:
                controller.configure_motor(
                    motor_id=motor_id,
                    velocity=int(velocity * 24),  # RATE=24
                    acceleration=int(acceleration * 24),
                    deceleration=int(deceleration * 24)
                )
                controller.current_status = controller.status_list[0]  # 初始为STOP
            except Exception as e:
                rospy.logerr(f"❌ 配置电机{motor_id}失败: {e}")
        
        # 5. 启动键盘监听线程
        rospy.loginfo("⌨️  启动键盘监听...")
        keyboard_thread = threading.Thread(
            target=RobotController.keyboard_listener,
            args=(controller,),
            daemon=True
        )
        keyboard_thread.start()
        
        # 6. 启动状态执行定时器
        rospy.loginfo("🔄 进入主控制循环...")
        rospy.Timer(rospy.Duration(0.05), controller.execute_state)
        controller.set_state("STOP")  # 初始状态
        rospy.spin()
        
    except KeyboardInterrupt:
        rospy.loginfo("🛑 用户终止程序")
    except Exception as e:
        rospy.logfatal(f"💥 程序启动失败: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 安全关闭
        if 'controller' in locals():
            controller.shutdown()

if __name__ == "__main__":
    main()