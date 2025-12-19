#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rospy
from pymodbus.client import ModbusSerialClient

# -------------------------- 驱动器核心配置 --------------------------
BAUDRATE = 38400
SERIAL_PORT = "/dev/Motor-DBLS"
STOP_BITS = 1
PARITY = "N"
TIMEOUT = 3.0
COMMAND_INTERVAL = 0.05

# 寄存器地址
REG_CONTROL_MODE = 32768  # $8000：控制位+极对数
REG_SPEED_SET = 32773     # $8005：速度设定（RPM）
REG_FAULT_STATUS = 32795  # $801B：故障状态
REG_ACTUAL_SPEED = 32792  # $8018：实际转速

# 控制位定义
CONTROL_EN = 0x01  # 使能
CONTROL_FR = 0x02  # 正反转
CONTROL_BK = 0x04  # 刹车
CONTROL_NW = 0x08  # 通讯控制

# 速比配置
RATE = 1

# 故障码映射
FAULT_MAP = {
    0x00: "无故障",
    0x01: "堵转",
    0x02: "过流",
    0x08: "母线电压过低",
    0x10: "母线电压过高",
    0x20: "电流峰值报警",
    0x80: "通讯中断报警",
    0x88: "通讯中断报警（扩展码）"
}

class MotorDriver:
    def __init__(self):
        self.rtu_client = None
        self.motor_address_map = {1:1, 2:2, 3:3}
        self.motor_pole_pairs = {1:5, 2:5, 3:5}
        self.motor_control_state = {}  # 缓存：{motor_id: {"enable": bool, "direction": int, "brake": bool}}
        self.motor_driver = True  # 驱动状态标记
        # 初始化电机控制状态缓存
        for motor_id in self.motor_pole_pairs.keys():
            self.motor_control_state[motor_id] = {"enable": False, "direction": 0, "brake": False}
        # 连接RTU客户端
        self.connect_rtu_client_with_timeout()

    # -------------------------- RTU连接相关 --------------------------
    def connect_rtu_client_with_timeout(self):
        """连接RTU客户端（带超时重试）"""
        max_retry = 5
        retry_count = 0
        while not rospy.is_shutdown() and retry_count < max_retry:
            try:
                self.rtu_client = ModbusSerialClient(
                    port=SERIAL_PORT, baudrate=BAUDRATE, stopbits=STOP_BITS, parity=PARITY, timeout=TIMEOUT
                )
                if self.rtu_client.connect():
                    rospy.loginfo(f"✅ RTU客户端连接成功：{SERIAL_PORT}")
                    return True
                retry_count += 1
                rospy.logerr(f"❌ RTU连接失败（重试{retry_count}/{max_retry}），3秒后重试...")
                time.sleep(3)
            except Exception as e:
                retry_count += 1
                rospy.logerr(f"❌ RTU连接异常：{e}（重试{retry_count}/{max_retry}）")
                time.sleep(3)
        rospy.logfatal(f"❌ RTU连接失败，已重试{max_retry}次")
        return False

    def reconnect_rtu_client(self):
        """重连RTU客户端"""
        rospy.logwarn("🔄 尝试重连RTU客户端...")
        if self.rtu_client is not None:
            try:
                self.rtu_client.close()
            except:
                pass
        self.rtu_client = None
        try:
            self.rtu_client = ModbusSerialClient(
                port=SERIAL_PORT, baudrate=BAUDRATE, stopbits=STOP_BITS, parity=PARITY, timeout=TIMEOUT
            )
            if self.rtu_client.connect():
                rospy.loginfo("✅ RTU重连成功")
                return True
            rospy.logerr("❌ RTU重连失败")
            return False
        except Exception as e:
            rospy.logerr(f"❌ RTU重连异常：{e}")
            return False

    # -------------------------- RTU读写操作 --------------------------
    def rtu_write_register(self, motor_id, reg_addr, value):
        """写寄存器（带空值判断和重连）"""
        if self.rtu_client is None or not self.rtu_client.is_socket_open():
            rospy.logerr("❌ RTU未连接，无法写寄存器")
            if not self.reconnect_rtu_client():
                return False
        rtu_addr = self.motor_address_map.get(motor_id)
        if rtu_addr is None:
            rospy.logerr(f"❌ 电机ID {motor_id} 无对应RTU站点")
            return False
        try:
            time.sleep(COMMAND_INTERVAL)
            response = self.rtu_client.write_register(reg_addr, value, slave=rtu_addr)
            if not response.isError():
                rospy.logdebug(f"✅ 电机{motor_id}写寄存器0x{reg_addr:04X}：0x{value:04X}")
                return True
            rospy.logerr(f"❌ 电机{motor_id}写寄存器失败：{response}")
            self.reconnect_rtu_client()
            return False
        except Exception as e:
            rospy.logerr(f"❌ 电机{motor_id}写操作异常：{e}")
            self.reconnect_rtu_client()
            return False

    def rtu_read_register(self, motor_id, reg_addr, count=1):
        """读寄存器（带空值判断和重连）"""
        if self.rtu_client is None or not self.rtu_client.is_socket_open():
            rospy.logerr("❌ RTU未连接，无法读寄存器")
            if not self.reconnect_rtu_client():
                return None
        rtu_addr = self.motor_address_map.get(motor_id)
        if rtu_addr is None:
            rospy.logerr(f"❌ 电机ID {motor_id} 无对应RTU站点")
            return None
        try:
            time.sleep(COMMAND_INTERVAL)
            response = self.rtu_client.read_holding_registers(reg_addr, count, slave=rtu_addr)
            if not response.isError():
                rospy.logdebug(f"✅ 电机{motor_id}读寄存器0x{reg_addr:04X}：{response.registers}")
                return response.registers
            rospy.logerr(f"❌ 电机{motor_id}读寄存器失败：{response}")
            self.reconnect_rtu_client()
            return None
        except Exception as e:
            rospy.logerr(f"❌ 电机{motor_id}读操作异常：{e}")
            self.reconnect_rtu_client()
            return None

    # -------------------------- 电机基础控制 --------------------------
    def set_control_mode(self, motor_id, enable=True, direction=0, brake=False):
        """设置电机控制模式（使能/方向/刹车）"""
        if self.rtu_client is None:
            rospy.logerr("❌ RTU未连接，无法设置控制模式")
            return False
        # 状态缓存校验，无需修改则直接返回
        current_state = self.motor_control_state.get(motor_id, {})
        if (current_state.get("enable") == enable and
            current_state.get("direction") == direction and
            current_state.get("brake") == brake and
            self.motor_driver):
            rospy.logdebug(f"ℹ️  电机{motor_id}控制模式无需修改：使能={enable}，方向={direction}，刹车={brake}")
            return True
        # 构建控制字
        pole_pairs = self.motor_pole_pairs.get(motor_id, 5)
        high_byte = CONTROL_NW | (CONTROL_EN if enable else 0) | (CONTROL_FR if direction else 0) | (CONTROL_BK if brake else 0)
        control_value = (high_byte << 8) | pole_pairs
        # 写入寄存器并更新缓存
        success = self.rtu_write_register(motor_id, REG_CONTROL_MODE, control_value)
        if success:
            self.motor_control_state[motor_id] = {"enable": enable, "direction": direction, "brake": brake}
            rospy.loginfo(f"✅ 电机{motor_id}控制模式设置成功：使能={enable}，方向={direction}，刹车={brake}")
        return success

    def set_target_velocity(self, motor_id, velocity):
        """设置电机速度（小端序适配）"""
        if self.rtu_client is None:
            rospy.logerr("❌ RTU未连接，无法设置速度")
            return False
        # 速度值处理
        velocity_abs = abs(velocity)
        velocity_abs = max(min(velocity_abs, 65535), 0)
        direction = 1 if velocity < 0 else 0
        # 转换为小端序
        velocity_low = velocity_abs & 0xFF
        velocity_high = (velocity_abs >> 8) & 0xFF
        velocity_little_endian = (velocity_low << 8) | velocity_high
        # 写入速度寄存器
        success = self.rtu_write_register(motor_id, REG_SPEED_SET, velocity_little_endian)
        if success:
            self.set_control_mode(motor_id, enable=True, direction=direction)
            # 方向校验（确保方向一致）
            current_direction = self.motor_control_state.get(motor_id, {}).get("direction", 0)
            if direction != current_direction:
                mode_success = self.set_control_mode(motor_id, enable=True, direction=direction, brake=False)
                if not mode_success:
                    rospy.logerr(f"❌ 电机{motor_id}方向修改失败")
                    return False
            rospy.loginfo(f"✅ 电机{motor_id}速度设置成功：{velocity} RPM")
        return success

    def get_actual_velocity(self, motor_id):
        """读取电机实际转速"""
        if self.rtu_client is None:
            rospy.logerr("❌ RTU未连接，无法读取转速")
            return 0
        registers = self.rtu_read_register(motor_id, REG_ACTUAL_SPEED, count=1)
        if not registers or len(registers) != 1:
            rospy.logwarn(f"⚠️ 读取电机{motor_id}转速失败")
            return 0
        # 小端序解析
        speed_code_little = registers[0]
        speed_high = speed_code_little & 0xFF
        speed_low = (speed_code_little >> 8) & 0xFF
        speed_code_big = (speed_high << 8) | speed_low
        # 计算实际转速
        pole_pairs = self.motor_pole_pairs.get(motor_id, 5)
        actual_speed = speed_code_big * 20 / 10
        print(f"{motor_id}电机实际转速：{actual_speed} RPM")
        return round(actual_speed, 2)

    def read_fault_code(self, motor_id):
        """读取电机故障码"""
        if self.rtu_client is None:
            rospy.logerr("❌ RTU未连接，无法读取故障码")
            return None, "客户端未连接"
        registers = self.rtu_read_register(motor_id, REG_FAULT_STATUS, count=1)
        if not registers:
            rospy.logwarn(f"⚠️ 读取电机{motor_id}故障码失败")
            return None, "读取失败"
        fault_16 = registers[0]
        fault_code = (fault_16 >> 8) & 0xFF
        fault_desc = FAULT_MAP.get(fault_code, f"未知故障（0x{fault_code:02X}）")
        # 更新驱动状态
        if fault_code != 0x00:
            rospy.logwarn(f"⚠️ 电机{motor_id}故障：{fault_desc}")
            self.motor_driver = False
            self.enable_drive(motor_id)
            rospy.logwarn(f"⚠️ 电机{motor_id}重新使能")
        else:
            self.motor_driver = True
        return fault_code, fault_desc

    def clear_fault(self, motor_id):
        """清除电机故障"""
        if self.rtu_client is None:
            rospy.logerr("❌ RTU未连接，无法清除故障")
            return False
        rospy.loginfo(f"🔧 清除电机{motor_id}故障...")
        time.sleep(0.5)
        return True

    def enable_drive(self, motor_id):
        """使能电机"""
        return self.set_control_mode(motor_id, enable=True, brake=False)

    def disable_drive(self, motor_id):
        """禁用电机（带刹车）"""
        if self.rtu_client is None:
            rospy.logerr(f"❌ RTU未连接，无法禁用电机{motor_id}")
            return False
        return self.set_control_mode(motor_id, enable=False, brake=True)

    def start_motor(self, motor_id):
        """启动电机（初始化+使能）"""
        if self.rtu_client is None:
            rospy.logerr("❌ RTU未连接，无法启动电机")
            return False
        if motor_id not in self.motor_address_map:
            rospy.logerr(f"❌ 无效电机ID：{motor_id}")
            return False
        # 读取故障码（如有故障自动清除）
        fault_code, _ = self.read_fault_code(motor_id)
        if fault_code and fault_code != 0x00:
            self.clear_fault(motor_id)
        # 使能电机
        success = self.set_control_mode(motor_id, enable=True)
        if success:
            rospy.loginfo(f"✅ 电机{motor_id}初始化完成")
        return success

    def close(self):
        """关闭驱动（禁用电机+关闭RTU）"""
        rospy.loginfo("🔌 关闭电机驱动...")
        if self.rtu_client is not None:
            # 禁用所有电机
            for motor_id in self.motor_address_map.keys():
                try:
                    self.disable_drive(motor_id)
                except:
                    pass
            # 关闭RTU客户端
            try:
                self.rtu_client.close()
            except:
                pass
        rospy.loginfo("✅ 电机驱动已关闭")