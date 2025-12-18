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
COMMAND_INTERVAL = 0.05  # 指令间隔50ms

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
    """电机驱动类：仅负责Modbus RTU硬件交互，不包含业务逻辑"""
    def __init__(self, motor_id, rtu_addr, pole_pairs=5):
        self.motor_id = motor_id  # 电机ID（1/2/3）
        self.rtu_addr = rtu_addr  # RTU站点地址
        self.pole_pairs = pole_pairs  # 电机极对数
        self.rtu_client = None  # Modbus客户端
        self.connection_status = False  # 连接状态

    def connect(self, max_retry=5):
        """连接RTU客户端，带重试机制"""
        retry_count = 0
        while not rospy.is_shutdown() and retry_count < max_retry:
            try:
                self.rtu_client = ModbusSerialClient(
                    port=SERIAL_PORT,
                    baudrate=BAUDRATE,
                    stopbits=STOP_BITS,
                    parity=PARITY,
                    timeout=TIMEOUT
                )
                if self.rtu_client.connect():
                    self.connection_status = True
                    rospy.loginfo(f"✅ 电机{self.motor_id}驱动连接成功：{SERIAL_PORT}")
                    return True
                else:
                    retry_count += 1
                    rospy.logerr(f"❌ 电机{self.motor_id}驱动连接失败（重试{retry_count}/{max_retry}）")
                    time.sleep(3)
            except Exception as e:
                retry_count += 1
                rospy.logerr(f"❌ 电机{self.motor_id}驱动连接异常：{e}（重试{retry_count}/{max_retry}）")
                time.sleep(3)
        self.connection_status = False
        rospy.logfatal(f"❌ 电机{self.motor_id}驱动连接失败，已重试{max_retry}次")
        return False

    def reconnect(self):
        """重连RTU客户端"""
        rospy.logwarn(f"🔄 电机{self.motor_id}驱动尝试重连...")
        if self.rtu_client:
            try:
                self.rtu_client.close()
            except:
                pass
        return self.connect(max_retry=1)

    def _check_connection(self):
        """检查连接状态，断开则自动重连"""
        if not self.rtu_client or not self.rtu_client.is_socket_open():
            self.connection_status = False
            return self.reconnect()
        return True

    def write_register(self, reg_addr, value):
        """写寄存器（硬件层面）"""
        if not self._check_connection():
            return False
        try:
            time.sleep(COMMAND_INTERVAL)
            response = self.rtu_client.write_register(reg_addr, value, slave=self.rtu_addr)
            if not response.isError():
                rospy.logdebug(f"✅ 电机{self.motor_id}写寄存器0x{reg_addr:04X}：0x{value:04X}")
                return True
            rospy.logerr(f"❌ 电机{self.motor_id}写寄存器失败：地址0x{reg_addr:04X}，错误{response}")
            self.reconnect()
            return False
        except Exception as e:
            rospy.logerr(f"❌ 电机{self.motor_id}写操作异常：{e}")
            self.reconnect()
            return False

    def read_register(self, reg_addr, count=1):
        """读寄存器（硬件层面）"""
        if not self._check_connection():
            return None
        try:
            time.sleep(COMMAND_INTERVAL)
            response = self.rtu_client.read_holding_registers(reg_addr, count, slave=self.rtu_addr)
            if not response.isError():
                rospy.logdebug(f"✅ 电机{self.motor_id}读寄存器0x{reg_addr:04X}：{response.registers}")
                return response.registers
            rospy.logerr(f"❌ 电机{self.motor_id}读寄存器失败：地址0x{reg_addr:04X}，错误{response}")
            self.reconnect()
            return None
        except Exception as e:
            rospy.logerr(f"❌ 电机{self.motor_id}读操作异常：{e}")
            self.reconnect()
            return None

    def set_control_mode(self, enable=True, direction=0, brake=False):
        """设置控制模式（使能/方向/刹车）"""
        high_byte = CONTROL_NW | (CONTROL_EN if enable else 0) | (CONTROL_FR if direction else 0) | (CONTROL_BK if brake else 0)
        low_byte = self.pole_pairs
        control_value = (high_byte << 8) | low_byte
        return self.write_register(REG_CONTROL_MODE, control_value)

    def set_speed(self, velocity):
        """设置电机速度（RPM）"""
        velocity_abs = abs(velocity)
        velocity_abs = max(min(velocity_abs, 65535), 0)  # 限制范围
        direction = 1 if velocity < 0 else 0
        # 先写速度寄存器，再设置方向
        if self.write_register(REG_SPEED_SET, velocity_abs):
            return self.set_control_mode(enable=True, direction=direction, brake=False)
        return False

    def get_actual_speed(self):
        """获取实际转速（RPM）"""
        registers = self.read_register(REG_ACTUAL_SPEED, count=1)
        if not registers or len(registers) != 1:
            rospy.logwarn(f"⚠️ 电机{self.motor_id}转速读取失败")
            return 0.0
        speed_code = registers[0]
        actual_speed = (speed_code * 20) / self.pole_pairs
        return round(max(min(actual_speed, 65535), 0), 2)

    def get_fault_info(self):
        """获取故障信息"""
        registers = self.read_register(REG_FAULT_STATUS, count=1)
        if not registers:
            return None, "读取失败"
        fault_16 = registers[0]
        fault_code = (fault_16 >> 8) & 0xFF
        fault_desc = FAULT_MAP.get(fault_code, f"未知故障（0x{fault_code:02X}）")
        return fault_code, fault_desc

    def enable(self):
        """使能电机"""
        return self.set_control_mode(enable=True, brake=False)

    def disable(self):
        """禁用电机（刹车）"""
        return self.set_control_mode(enable=False, brake=True)

    def close(self):
        """关闭驱动连接"""
        if self.rtu_client:
            try:
                self.disable()  # 禁用电机后再关闭连接
                self.rtu_client.close()
                rospy.loginfo(f"✅ 电机{self.motor_id}驱动已关闭")
            except Exception as e:
                rospy.logerr(f"❌ 电机{self.motor_id}驱动关闭异常：{e}")
        self.connection_status = False

class MotorDriverManager:
    """电机驱动管理器：管理多个电机驱动实例"""
    def __init__(self, motor_config):
        """
        Args:
            motor_config: 电机配置列表，格式：
            [
                {"motor_id": 1, "rtu_addr": 1, "pole_pairs": 5},
                {"motor_id": 2, "rtu_addr": 2, "pole_pairs": 5},
                ...
            ]
        """
        self.drivers = {}
        # 初始化所有电机驱动
        for config in motor_config:
            motor_id = config["motor_id"]
            driver = MotorDriver(
                motor_id=motor_id,
                rtu_addr=config["rtu_addr"],
                pole_pairs=config.get("pole_pairs", 5)
            )
            self.drivers[motor_id] = driver

    def connect_all(self):
        """连接所有电机驱动"""
        all_success = True
        for motor_id, driver in self.drivers.items():
            if not driver.connect():
                all_success = False
        return all_success

    def get_driver(self, motor_id):
        """获取指定电机的驱动实例"""
        return self.drivers.get(motor_id)

    def close_all(self):
        """关闭所有电机驱动"""
        for driver in self.drivers.values():
            driver.close()