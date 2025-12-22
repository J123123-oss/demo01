#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import can
import time
import rospy
import threading

class CanMotorDriver:
    def __init__(self, channel='can0', interface='socketcan'):
        self.channel = channel
        self.interface = interface
        self.bus = self.create_can_bus()
        self.motor_driver_status = True  # 驱动状态标记
        # 心跳线程控制
        self.heartbeat_running = False
        self.heartbeat_thread = None

    # -------------------------- CAN总线连接 --------------------------
    def create_can_bus(self):
        """创建CAN总线连接（自动重试）"""
        while True:
            try:
                return can.interface.Bus(channel=self.channel, interface=self.interface)
            except (can.CanError, OSError) as e:
                rospy.logerr(f"CAN连接失败: {e}，3秒后重试...")
                time.sleep(3)

    def reconnect_can_bus(self):
        """重连CAN总线"""
        rospy.logwarn("🔄 重连CAN总线...")
        try:
            if self.bus is not None:
                self.bus.shutdown()
        except Exception:
            pass
        self.bus = self.create_can_bus()

    # -------------------------- CAN指令发送 --------------------------
    def send_command(self, motor_id, command_data):
        """发送CAN指令（3次重试）"""
        if self.bus is None:
            rospy.logerr("❌ CAN总线未初始化")
            return False
        # 校验socket有效性
        try:
            sock = getattr(self.bus, 'socket', None)
            if sock is None or not hasattr(sock, 'fileno') or sock.fileno() < 0:
                rospy.logerr("❌ CAN socket无效，尝试重连...")
                self.reconnect_can_bus()
                return False
        except Exception as e:
            rospy.logwarn(f"⚠️ 检查socket异常: {e}")
        
        frame_id = 0x64 + motor_id
        msg = can.Message(arbitration_id=frame_id, data=command_data, is_extended_id=False)
        for attempt in range(3):
            try:
                self.bus.send(msg)
                time.sleep(0.05)
                return True
            except (can.CanError, OSError, ValueError) as e:
                rospy.logerr(f"⚠️ 电机{motor_id}发送失败（{attempt+1}/3）: {e}")
                self.reconnect_can_bus()
                time.sleep(0.05)
        rospy.logerr(f"❌ 电机{motor_id}发送指令失败")
        return False

    # -------------------------- 电机基础控制 --------------------------
    def set_velocity_mode(self, motor_id):
        """设置电机为速度模式"""
        return self.send_command(motor_id, [motor_id, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF])

    def set_target_velocity(self, motor_id, velocity):
        """设置电机目标速度（脉冲/秒）"""
        data = [
            motor_id, 0x20, 0x00,
            velocity & 0xFF,
            (velocity >> 8) & 0xFF,
            (velocity >> 16) & 0xFF,
            (velocity >> 24) & 0xFF,
            0xFF
        ]
        return self.send_command(motor_id, data)

    def get_actual_velocity(self, motor_id):
        """读取电机实际速度（脉冲/秒）"""
        self.send_command(motor_id, [motor_id, 0x12, 0xFC, 0x00, 0x00, 0x00, 0x00, 0xFF])
        start_time = time.time()
        while time.time() - start_time < 0.5:
            try:
                msg = self.bus.recv(timeout=0.1)
            except (can.CanError, OSError) as e:
                rospy.logerr(f"⚠️ 接收失败: {e}")
                self.reconnect_can_bus()
                continue
            if msg and msg.arbitration_id == (0x64 + motor_id):
                if len(msg.data) >= 8 and msg.data[0] == motor_id and msg.data[1] == 0x12 and msg.data[2] == 0xFC:
                    velocity = msg.data[3] | (msg.data[4] << 8) | (msg.data[5] << 16) | (msg.data[6] << 24)
                    if velocity > 0x7FFFFFFF:
                        velocity -= 0x100000000
                    return velocity
        rospy.logwarn(f"⚠️ 读取电机{motor_id}速度超时")
        return 0

    def enable_drive(self, motor_id):
        """使能电机"""
        return self.send_command(motor_id, [motor_id, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF])

    def disable_drive(self, motor_id):
        """禁用电机"""
        return self.send_command(motor_id, [motor_id, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF])

    def start_motor(self, motor_id):
        """初始化电机（配置参数+开启心跳）"""
        if not 0 <= motor_id <= 255:
            rospy.logerr(f"❌ 电机ID{motor_id}无效（0-255）")
            return False
        # 配置电机参数
        self.send_command(motor_id, [motor_id, 0x15, 0x01, 0x04, 0x00, 0x08, 0x00, 0xFF])
        self.send_command(motor_id, [motor_id, 0x15, 0x01, 0x4A, 0x00, 0x01, 0x00, 0xFF])
        self.send_command(motor_id, [motor_id, 0x15, 0x01, 0xB1, 0x00, 0xE8, 0x03, 0xFF])
        return True

    # -------------------------- 心跳检测 --------------------------
    def _send_heartbeat_0x10(self, motor_id):
        """发送初始化心跳帧"""
        data = [motor_id, 0x10, 0x88, 0x00, 0x00, 0x00, 0x00, 0xFF]
        self.send_command(motor_id, data)

    def _cycle_send_heartbeat_0x08(self, motor_id, interval=1.5):
        """周期发送保活心跳帧"""
        data = [motor_id, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF]
        while self.heartbeat_running:
            self.send_command(motor_id, data)
            time.sleep(interval)

    def start_heartbeat(self, motor_id):
        """启动电机心跳（独立线程）"""
        if self.bus is None:
            rospy.logerr("❌ CAN未就绪，无法启动心跳")
            return
        self.stop_heartbeat()  # 防止重复启动
        self._send_heartbeat_0x10(motor_id)
        self.heartbeat_running = True
        self.heartbeat_thread = threading.Thread(
            target=self._cycle_send_heartbeat_0x08,
            args=(motor_id,),
            daemon=True
        )
        self.heartbeat_thread.start()
        rospy.loginfo(f"❤️ 电机{motor_id}心跳启动（1.5秒/次）")

    def stop_heartbeat(self):
        """停止心跳线程"""
        self.heartbeat_running = False
        if self.heartbeat_thread and self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join(timeout=2)
        rospy.loginfo("❤️ 心跳停止")

    # -------------------------- 故障处理 --------------------------
    def read_fault_code(self, motor_id):
        """读取故障码"""
        self.send_command(motor_id, [0x40, 0x3F, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        start_time = time.time()
        while time.time() - start_time < 0.5:
            msg = self.bus.recv(0.1)
            if msg and msg.arbitration_id == (0x580 + motor_id):
                if len(msg.data) >= 6 and msg.data[1] == 0x3F and msg.data[2] == 0x60:
                    fault_code = msg.data[4] | (msg.data[5] << 8)
                    if fault_code != 0:
                        rospy.logwarn(f"⚠️ 电机{motor_id}故障码: 0x{fault_code:04X}")
                        self.motor_driver_status = False
                    else:
                        self.motor_driver_status = True
                    return fault_code
        rospy.logwarn(f"⚠️ 读取电机{motor_id}故障码超时")
        return None

    def clear_fault(self, motor_id):
        """清除故障"""
        rospy.loginfo(f"🔧 清除电机{motor_id}故障...")
        self.send_command(motor_id, [0x2B, 0x40, 0x60, 0x00, 0x8F, 0x00, 0x00, 0x00])
        time.sleep(0.1)
        self.send_command(motor_id, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00])
        fault_code = self.read_fault_code(motor_id)
        if fault_code == 0 or fault_code is None:
            rospy.loginfo(f"✅ 电机{motor_id}故障清除成功")
            return True
        rospy.logerr(f"❌ 故障清除失败，码: 0x{fault_code:04X}")
        return False

    # -------------------------- 资源释放 --------------------------
    def shutdown(self):
        """关闭驱动"""
        rospy.loginfo("🔌 关闭CAN电机驱动...")
        self.stop_heartbeat()
        for motor_id in [1,2,3,4]:
            self.disable_drive(motor_id)
        if self.bus is not None:
            self.bus.shutdown()
        rospy.loginfo("✅ 驱动关闭完成")

# 单独测试入口
if __name__ == "__main__":
    rospy.init_node("can_driver_test", anonymous=True)
    # 初始化驱动
    driver = CanMotorDriver(channel='can0')
    try:
        # 测试电机2：初始化+速度设置
        motor_id = 2
        rospy.loginfo(f"=== 测试电机{motor_id} ===")
        driver.start_motor(motor_id)
        driver.set_velocity_mode(motor_id)
        driver.enable_drive(motor_id)
        driver.start_heartbeat(motor_id)
        
        # 设置速度并读取
        driver.set_target_velocity(motor_id, 1000)
        time.sleep(1)
        actual_speed = driver.get_actual_velocity(motor_id)
        rospy.loginfo(f"目标速度: 1000 脉冲/秒，实际速度: {actual_speed} 脉冲/秒")
        
        # 停止电机
        time.sleep(3)
        driver.set_target_velocity(motor_id, 0)
        driver.disable_drive(motor_id)
        rospy.loginfo("=== 测试结束 ===")
    except Exception as e:
        rospy.logerr(f"测试异常: {e}")
    finally:
        driver.shutdown()