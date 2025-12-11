#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import sys
import threading
from pymodbus.client import ModbusSerialClient
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.constants import Endian as ModbusEndian

# -------------------------- 调试配置项（根据实际硬件修改） --------------------------
SERIAL_PORT = "/dev/ttyUSB0"  # 串口端口（Windows为COMx）
BAUDRATE = 38400              # 波特率（匹配手册）
STOP_BITS = 1                 # 停止位
PARITY = "N"                  # 校验位（无校验）
TIMEOUT = 3.0                 # 通讯超时（手册3秒）
COMMAND_INTERVAL = 0.05       # 指令间隔（50ms，手册要求）
QUERY_INTERVAL = 1.0          # 循环查询间隔（1秒/次）
MONITOR_MOTORS = [1]          # 仅监控电机1
INIT_SPEED = {1: 2000}        # 初始速度1000 RPM

# 寄存器地址（十六进制转十进制）
REG_CONTROL_MODE = 32768  # $8000：控制位+极对数
REG_SPEED_SET = 32773     # $8005：速度设定（RPM）
REG_FAULT_STATUS = 32795  # $801B：故障状态（16位，高字节=故障码）
REG_ACTUAL_SPEED = 32792  # $8018：实际转速（16位编码值）
REG_SAVE_PARAM = 32792    # $8018：参数保存指令地址

# 控制位定义（$8000高位）
CONTROL_EN = 0x01  # 使能（1=有效）
CONTROL_FR = 0x02  # 正反转（1=逆时针，0=顺时针）
CONTROL_BK = 0x04  # 刹车（1=有效）
CONTROL_NW = 0x08  # 控制模式（1=通讯控制，0=外部控制）

# 电机ID与RTU站点地址映射
MOTOR_MAP = {1: 1}
# 电机极对数（默认5对，匹配手册）
MOTOR_POLE_PAIRS = {1:5}

# 故障码映射（补充实际通讯中发现的0x88）
FAULT_MAP = {
    0x00: "无故障",
    0x01: "堵转",
    0x02: "过流",
    0x08: "母线电压过低",
    0x10: "母线电压过高",
    0x20: "电流峰值报警",
    0x80: "通讯中断报警",
    0x88: "通讯中断报警（扩展码）"  # 适配实际返回的0x88
}

class RTUMonitor:
    def __init__(self):
        self.client = None
        self.is_running = True  # 运行状态标记
        self.connect()
        # 启动键盘监听线程（用于退出）
        self.exit_thread = threading.Thread(target=self.listen_exit, daemon=True)
        self.exit_thread.start()

    def connect(self):
        """建立RTU连接"""
        print(f"[INFO] 尝试连接RTU串口 {SERIAL_PORT} (波特率{BAUDRATE})...")
        self.client = ModbusSerialClient(
            port=SERIAL_PORT,
            baudrate=BAUDRATE,
            stopbits=STOP_BITS,
            parity=PARITY,
            timeout=TIMEOUT
        )
        if self.client.connect():
            print("[SUCCESS] RTU连接成功！")
        else:
            print("[ERROR] RTU连接失败，请检查串口/波特率/接线！")
            sys.exit(1)

    def reconnect(self):
        """重连RTU"""
        print("[WARN] 尝试重连RTU...")
        self.client.close()
        self.connect()

    def write_register(self, motor_id, reg_addr, value):
        """写单个寄存器（06H功能码）"""
        slave_id = MOTOR_MAP.get(motor_id)
        if slave_id is None:
            print(f"[ERROR] 电机ID {motor_id} 无对应站点地址！")
            return False
        
        time.sleep(COMMAND_INTERVAL)
        try:
            resp = self.client.write_register(reg_addr, value, slave=slave_id)
            if resp.isError():
                print(f"[ERROR] 写寄存器失败：电机{motor_id} 地址{reg_addr} → {resp}")
                self.reconnect()
                return False
            print(f"[SUCCESS] 写寄存器：电机{motor_id} 地址{reg_addr} = 0x{value:04X}（十进制{value}）")
            return True
        except Exception as e:
            print(f"[ERROR] 写寄存器异常：{e}")
            self.reconnect()
            return False

    def read_register(self, motor_id, reg_addr, count=1):
        """读寄存器（03H功能码）"""
        slave_id = MOTOR_MAP.get(motor_id)
        if slave_id is None:
            print(f"[ERROR] 电机ID {motor_id} 无对应站点地址！")
            return None
        
        time.sleep(COMMAND_INTERVAL)
        try:
            resp = self.client.read_holding_registers(reg_addr, count, slave=slave_id)
            if resp.isError():
                print(f"[ERROR] 读寄存器失败：电机{motor_id} 地址{reg_addr} → {resp}")
                self.reconnect()
                return None
            # 打印原始通讯帧（模拟）
            print(f"[DEBUG] 电机{motor_id}读寄存器{hex(reg_addr)}返回：{resp.registers}")
            return resp.registers
        except Exception as e:
            print(f"[ERROR] 读寄存器异常：{e}")
            self.reconnect()
            return None

    def set_control_mode(self, motor_id, enable=True, direction=0, brake=False):
        """配置控制模式（通讯控制+使能+转向）"""
        pole_pairs = MOTOR_POLE_PAIRS.get(motor_id, 5)
        # 高位：控制位（NW=1通讯控制），低位：极对数
        high_byte = CONTROL_NW | (CONTROL_EN if enable else 0) | (CONTROL_FR if direction else 0) | (CONTROL_BK if brake else 0)
        control_value = (high_byte << 8) | pole_pairs
        print(f"[INFO] 配置电机{motor_id}控制模式：控制位=0x{high_byte:02X} 极对数={pole_pairs} → 数值=0x{control_value:04X}")
        return self.write_register(motor_id, REG_CONTROL_MODE, control_value)

    def enable_motor(self, motor_id):
        """使能电机（通讯控制模式）"""
        print(f"\n[INFO] 开始使能电机{motor_id}...")
        # 1. 先禁用+刹车（复位状态）
        self.set_control_mode(motor_id, enable=False, brake=True)
        time.sleep(0.5)
        # 2. 配置通讯控制模式+使能
        success = self.set_control_mode(motor_id, enable=True, brake=False)
        if success:
            print(f"[SUCCESS] 电机{motor_id}使能成功（通讯控制模式）")
        else:
            print(f"[ERROR] 电机{motor_id}使能失败")
        return success

    def set_motor_speed(self, motor_id, speed):
        """设置电机速度（$8005寄存器）"""
        speed = max(min(abs(speed), 65535), 0)
        print(f"[INFO] 设置电机{motor_id}初始速度：{speed} RPM（十六进制0x{speed:04X}）")
        success = self.write_register(motor_id, REG_SPEED_SET, speed)
        if success:
            print(f"[SUCCESS] 电机{motor_id}速度设置完成（对应通讯帧：01 06 80 05 {speed:02X} {speed>>8:02X} ...）")
        else:
            print(f"[ERROR] 电机{motor_id}速度设置失败")
        return success

    def init_motors(self):
        """初始化所有监控电机（使能+设置初始速度）"""
        print("\n" + "="*60)
        print("[INFO] 开始初始化电机配置...")
        print("="*60)
        for motor_id in MONITOR_MOTORS:
            # 1. 使能电机
            if not self.enable_motor(motor_id):
                continue
            # 2. 设置初始速度
            init_speed = INIT_SPEED.get(motor_id, 0)
            self.set_motor_speed(motor_id, init_speed)
            time.sleep(0.5)  # 间隔避免通讯冲突
        print("="*60)
        print("[INFO] 所有电机初始化完成！")
        print("="*60 + "\n")

    def read_actual_speed(self, motor_id):
        """读取实际转速（精准适配通讯帧：16位编码值）"""
        # 实际通讯中读1个寄存器（count=1），而非2个
        regs = self.read_register(motor_id, REG_ACTUAL_SPEED, count=1)
        if not regs or len(regs) != 1:
            return 0.0
        
        # 解析16位编码值（通讯帧返回0x0007=7）
        speed_code = regs[0]
        print(f"[DEBUG] 电机{motor_id}转速编码值：0x{speed_code:04X}（十进制{speed_code}）")
        
        # 手册公式：实际转速 = 编码值 ×20 / 电机极数
        pole_pairs = MOTOR_POLE_PAIRS.get(motor_id, 5)
        actual_speed = (speed_code * 20) / pole_pairs
        # 容错：限制合理范围（0~65535）
        actual_speed = max(min(actual_speed, 65535), 0)
        
        return round(actual_speed, 2)

    def read_fault(self, motor_id):
        """读取故障码（精准适配通讯帧：16位高字节）"""
        # 实际通讯中读1个寄存器（count=1），而非2个
        regs = self.read_register(motor_id, REG_FAULT_STATUS, count=1)
        if not regs:
            return None, "读取失败"
        
        # 解析16位值，故障码=高字节（通讯帧返回0x8800→高字节0x88）
        fault_16 = regs[0]
        fault_code = (fault_16 >> 8) & 0xFF  # 取高8位
        print(f"[DEBUG] 电机{motor_id}故障16位值：0x{fault_16:04X} → 故障码0x{fault_code:02X}")
        
        fault_desc = FAULT_MAP.get(fault_code, f"未知故障（0x{fault_code:02X}）")
        return fault_code, fault_desc

    def monitor_loop(self):
        """先初始化电机，再循环监控故障码和转速"""
        # 第一步：初始化电机（使能+设置速度）
        self.init_motors()
        
        # 第二步：进入循环监控
        print(f"\n[INFO] 开始循环监控（间隔{QUERY_INTERVAL}秒），输入 'q' 退出...")
        print("-" * 100)
        print(f"{'时间':<20} {'电机ID':<10} {'设定速度(RPM)':<15} {'实际转速(RPM)':<15} {'故障码':<10} {'故障描述':<20}")
        print("-" * 100)

        while self.is_running:
            current_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
            for motor_id in MONITOR_MOTORS:
                # 读取设定速度
                set_speed = INIT_SPEED.get(motor_id, 0)
                # 读取实际转速
                speed = self.read_actual_speed(motor_id)
                # 读取故障码
                fault_code, fault_desc = self.read_fault(motor_id)
                
                # 格式化输出
                set_speed_str = f"{set_speed}"
                speed_str = f"{speed}" if speed > 0 else "0.00"
                fault_code_str = f"0x{fault_code:02X}" if fault_code is not None else "----"
                fault_desc_str = fault_desc if fault_desc else "读取失败"
                
                print(f"{current_time:<20} {motor_id:<10} {set_speed_str:<15} {speed_str:<15} {fault_code_str:<10} {fault_desc_str:<20}")
                sys.stdout.flush()  # 强制刷新输出
            
            # 等待查询间隔
            time.sleep(QUERY_INTERVAL)

    def listen_exit(self):
        """监听退出指令（输入q退出）"""
        while self.is_running:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                input_str = sys.stdin.readline().strip()
                if input_str.lower() == 'q':
                    self.is_running = False
                    # 退出前禁用所有电机
                    print("\n[INFO] 退出前禁用所有电机...")
                    for motor_id in MONITOR_MOTORS:
                        self.set_control_mode(motor_id, enable=False, brake=True)
                    print("[INFO] 接收到退出指令，正在停止监控...")
                    break

    def close(self):
        """关闭连接"""
        if self.client:
            self.client.close()
            print("[INFO] RTU连接已关闭")
            print("[INFO] 监控程序已退出")

# 兼容Windows/Linux的select模块（Windows需额外处理）
try:
    import select
except ImportError:
    # Windows下简化处理
    import msvcrt
    def listen_exit(self):
        while self.is_running:
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8').lower()
                if key == 'q':
                    self.is_running = False
                    # 退出前禁用所有电机
                    print("\n[INFO] 退出前禁用所有电机...")
                    for motor_id in MONITOR_MOTORS:
                        self.set_control_mode(motor_id, enable=False, brake=True)
                    print("[INFO] 接收到退出指令，正在停止监控...")
                    break
    # 替换原有listen_exit方法
    RTUMonitor.listen_exit = listen_exit

def main():
    # 初始化监控器
    monitor = RTUMonitor()
    try:
        # 启动循环监控（包含初始化）
        monitor.monitor_loop()
    except KeyboardInterrupt:
        monitor.is_running = False
        # Ctrl+C退出时禁用电机
        print("\n[INFO] 用户终止程序，禁用所有电机...")
        for motor_id in MONITOR_MOTORS:
            monitor.set_control_mode(motor_id, enable=False, brake=True)
    except Exception as e:
        print(f"[FATAL] 程序异常：{e}")
    finally:
        # 关闭连接
        monitor.close()

if __name__ == "__main__":
    main()