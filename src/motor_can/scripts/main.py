#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import threading
from motor_can import RobotController  # 导入机器人控制类

def main():
    try:
        # 初始化机器人控制器
        rospy.init_node("motor_canopen_node")
        controller = RobotController()
        rospy.loginfo("🚀 机器人控制器初始化完成")
        
        # 加载电机配置
        config = RobotController.load_config()
        if not config or "motors" not in config or not config["motors"]:
            rospy.logerr("❌ 未找到有效电机配置，请检查配置文件")
            return
        
        # 自动配置电机
        rospy.loginfo("⚙️  开始自动配置电机...")
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
                    velocity=int(velocity * 68),  # RATE=68
                    acceleration=int(acceleration * 68),
                    deceleration=int(deceleration * 68)
                )
                controller.current_status = controller.status_list[0]  # 重置为STOP状态
            except Exception as e:
                rospy.logerr(f"❌ 配置电机{motor_id}失败: {e}")
        
        # 启动键盘监听线程
        rospy.loginfo("⌨️  启动键盘监听...")
        keyboard_thread = threading.Thread(
            target=RobotController.keyboard_listener,
            args=(controller,),
            daemon=True
        )
        keyboard_thread.start()
        
        # 启动状态执行定时器
        rospy.loginfo("🔄 进入主控制循环...")
        rospy.Timer(rospy.Duration(0.05), controller.execute_state)
        controller.set_state("STOP")  # 初始状态设为STOP
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