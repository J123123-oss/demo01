#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import can
import time
import rospy

class CanMotorDriver:
    def __init__(self, channel='can0', interface='socketcan'):
        self.channel = channel
        self.interface = interface
        self.bus = self.create_can_bus()
        self.motor_driver_status = True  # 驱动状态标记

    # -------------------------- CAN总线连接 --------------------------
    def create_can_bus(self):
        """创建CAN总线连接（自动重试）"""
        while True:
            try:
                return can.interface.Bus(channel=self.channel, interface=self.interface)
            except (can.CanError, OSError) as e:
                rospy.logerr(f"CAN总线连接失败: {e}，3秒后重试...")
                time.sleep(3)

    def reconnect_can_bus(self):
        """重连CAN总线"""
        rospy.logwarn("🔄 尝试重连CAN总线...")
        try:
            if self.bus is not None:
                self.bus.shutdown()
        except Exception:
            pass
        self.bus = self.create_can_bus()

    # -------------------------- CAN指令发送 --------------------------
    def send_command(self, motor_id, command_data):
        """发送CAN指令（自动重试3次）"""
        frame_id = 0x600 + motor_id
        msg = can.Message(arbitration_id=frame_id, data=command_data, is_extended_id=False)
        for attempt in range(3):
            try:
                self.bus.send(msg)
                time.sleep(0.05)
                return True
            except (can.CanError, OSError) as e:
                rospy.logerr(f"CAN发送失败（电机{motor_id}）: {e}，第{attempt+1}次重试...")
                self.reconnect_can_bus()
        rospy.logerr(f"❌ 电机{motor_id} CAN指令发送失败，已重试3次")
        return False

    # -------------------------- 电机基础控制 --------------------------
    def set_velocity_mode(self, motor_id):
        """设置电机为速度模式"""
        return self.send_command(motor_id, [0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00])

    def set_target_velocity(self, motor_id, velocity):
        """设置电机目标速度"""
        data = [
            0x23, 0xFF, 0x60, 0x00,
            velocity & 0xFF,
            (velocity >> 8) & 0xFF,
            (velocity >> 16) & 0xFF,
            (velocity >> 24) & 0xFF
        ]
        return self.send_command(motor_id, data)

    def get_actual_velocity(self, motor_id):
        """读取电机实际速度"""
        # 发送读取指令
        self.send_command(motor_id, [0x40, 0x6C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        start_time = time.time()
        # 等待响应（超时0.5秒）
        while time.time() - start_time < 0.5:
            try:
                msg = self.bus.recv(timeout=0.1)
            except (can.CanError, OSError) as e:
                rospy.logerr(f"CAN接收失败（电机{motor_id}）: {e}，尝试重连...")
                self.reconnect_can_bus()
                continue
            # 校验响应帧
            if msg and msg.arbitration_id == (0x580 + motor_id):
                if len(msg.data) >= 8 and msg.data[0] == 0x43 and msg.data[1] == 0x6C and msg.data[2] == 0x60:
                    velocity = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
                    # 处理负速度
                    if velocity > 0x7FFFFFFF:
                        velocity -= 0x100000000
                    return velocity
        rospy.logwarn(f"⚠️ 读取电机{motor_id}实际速度超时")
        return 0

    def set_acceleration(self, motor_id, acceleration):
        """设置电机加速度"""
        data = [
            0x23, 0x83, 0x60, 0x00,
            acceleration & 0xFF,
            (acceleration >> 8) & 0xFF,
            (acceleration >> 16) & 0xFF,
            (acceleration >> 24) & 0xFF
        ]
        return self.send_command(motor_id, data)

    def set_deceleration(self, motor_id, deceleration):
        """设置电机减速度"""
        data = [
            0x23, 0x84, 0x60, 0x00,
            deceleration & 0xFF,
            (deceleration >> 8) & 0xFF,
            (deceleration >> 16) & 0xFF,
            (deceleration >> 24) & 0xFF
        ]
        return self.send_command(motor_id, data)

    def enable_drive(self, motor_id):
        """使能电机"""
        return self.send_command(motor_id, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00])

    def disable_drive(self, motor_id):
        """禁用电机"""
        return self.send_command(motor_id, [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00])

    def start_motor(self, motor_id):
        """启动电机（初始化）"""
        if not 1 <= motor_id <= 127:
            rospy.logerr(f"❌ 电机ID{motor_id}超出有效范围（1-127）")
            return False
        return self.send_command(motor_id, [0x01, motor_id & 0xFF])

    # -------------------------- 故障处理 --------------------------
    def read_fault_code(self, motor_id):
        """读取电机故障码"""
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
        """清除电机故障"""
        rospy.loginfo(f"🔧 清除电机{motor_id}故障...")
        self.send_command(motor_id, [0x2B, 0x40, 0x60, 0x00, 0x8F, 0x00, 0x00, 0x00])
        time.sleep(0.1)
        self.send_command(motor_id, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00])
        # 校验故障是否清除
        fault_code = self.read_fault_code(motor_id)
        if fault_code == 0 or fault_code is None:
            rospy.loginfo(f"✅ 电机{motor_id}故障已清除")
            return True
        else:
            rospy.logerr(f"❌ 电机{motor_id}故障清除失败，故障码: 0x{fault_code:04X}")
            return False

    # -------------------------- 高级参数读取/设置 --------------------------
    def get_max_torque(self, motor_id):
        """读取电机最大转矩"""
        self.send_command(motor_id, [0x40, 0x72, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        start_time = time.time()
        while time.time() - start_time < 0.5:
            msg = self.bus.recv(0.1)
            if msg and msg.arbitration_id == (0x580 + motor_id):
                if len(msg.data) >= 6 and msg.data[0] in [0x4B, 0x43]:
                    max_torque = msg.data[4] | (msg.data[5] << 8)
                    return max_torque
        rospy.logwarn(f"⚠️ 读取电机{motor_id}最大转矩超时")
        return None

    def get_actual_torque(self, motor_id):
        """读取电机实际转矩"""
        self.send_command(motor_id, [0x40, 0x77, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        start_time = time.time()
        while time.time() - start_time < 0.5:
            msg = self.bus.recv(0.1)
            if msg and msg.arbitration_id == (0x580 + motor_id):
                if len(msg.data) >= 6 and msg.data[0] == 0x4B and msg.data[1] == 0x77 and msg.data[2] == 0x60:
                    torque = msg.data[4] | (msg.data[5] << 8)
                    if torque > 0x7FFF:
                        torque -= 0x10000
                    return torque
        rospy.logwarn(f"⚠️ 读取电机{motor_id}实际转矩超时")
        return None

    def get_actual_current(self, motor_id):
        """读取电机实际电流"""
        self.send_command(motor_id, [0x40, 0x78, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        start_time = time.time()
        while time.time() - start_time < 0.5:
            msg = self.bus.recv(0.1)
            if msg and msg.arbitration_id == (0x580 + motor_id):
                if len(msg.data) >= 6 and msg.data[0] == 0x4B and msg.data[1] == 0x78 and msg.data[2] == 0x60:
                    current = msg.data[4] | (msg.data[5] << 8)
                    if current > 0x7FFF:
                        current -= 0x10000
                    return current
        rospy.logwarn(f"⚠️ 读取电机{motor_id}实际电流超时")
        return None

    def read_energy_saving_mode(self, motor_id):
        """读取省电模式（Fn_1d0）"""
        self.send_command(motor_id, [0x40, 0xD0, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00])
        start_time = time.time()
        while time.time() - start_time < 0.5:
            msg = self.bus.recv(0.1)
            if msg and msg.arbitration_id == (0x580 + motor_id):
                if len(msg.data) >= 6 and msg.data[1] == 0xD0 and msg.data[2] == 0x21:
                    fn_1d0_value = msg.data[4] | (msg.data[5] << 8)
                    mode_desc = {0: "关闭自动省电", 1: "总线指令触发", 2: "停机超时触发", 3: "指令或超时触发"}
                    rospy.loginfo(f"电机{motor_id}省电模式: {fn_1d0_value} - {mode_desc.get(fn_1d0_value, '未知')}")
                    return fn_1d0_value
        rospy.logwarn(f"⚠️ 读取电机{motor_id}省电模式超时")
        return None

    def set_energy_saving_mode(self, motor_id, target_value=3):
        """设置省电模式（Fn_1d0）"""
        if target_value not in [0, 1, 2, 3]:
            rospy.logerr(f"❌ 省电模式值{target_value}非法，仅支持0-3")
            return False
        low_byte = target_value & 0xFF
        high_byte = (target_value >> 8) & 0xFF
        self.send_command(motor_id, [0x2B, 0xD0, 0x21, 0x00, low_byte, high_byte, 0x00, 0x00])
        start_time = time.time()
        while time.time() - start_time < 0.5:
            msg = self.bus.recv(0.1)
            if msg and msg.arbitration_id == (0x580 + motor_id):
                if len(msg.data) >= 4 and msg.data[1] == 0xD0 and msg.data[2] == 0x21:
                    rospy.loginfo(f"✅ 电机{motor_id}省电模式设置为{target_value}")
                    return True
        rospy.logwarn(f"⚠️ 设置电机{motor_id}省电模式超时")
        return False

    # -------------------------- 资源释放 --------------------------
    def shutdown(self):
        """关闭驱动（禁用电机+关闭CAN总线）"""
        rospy.loginfo("🔌 关闭CAN电机驱动...")
        try:
            # 禁用常见电机（2/3/4）
            for motor_id in [2, 3, 4]:
                self.disable_drive(motor_id)
            if self.bus is not None:
                self.bus.shutdown()
        except Exception as e:
            rospy.logerr(f"❌ 关闭驱动异常: {e}")
        rospy.loginfo("✅ CAN电机驱动已关闭")