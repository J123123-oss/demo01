#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class BatteryStatus:
    def __init__(self):
        self.total_voltage = 0.0          # 总电压（V）
        self.current = 0.0                # 总电流（A）：正数充电，负数放电
        self.remaining_capacity = 0.0     # 剩余容量（Ah）
        self.nominal_capacity = 0.0       # 标称容量（Ah）
        self.battery_remaining = 0.0      # 剩余电量百分比（%）
        self.temperatures = []            # 温度列表（℃）：包含电池温度、PCB温度、环境温度
        self.cycle_count = 0              # 循环次数
        self.charge_mos_state = 0         # 充电MOS管状态（0关闭，1开启）
        self.discharge_mos_state = 0      # 放电MOS管状态（0关闭，1开启）
        self.battery_series = 0           # 电池串数
        self.battery_type = "未知"         # 电池类型（磷酸铁锂/三元锂）

    def __str__(self):
        mos_info = f"充电MOS: {'开启' if self.charge_mos_state == 1 else '关闭'} | 放电MOS: {'开启' if self.discharge_mos_state == 1 else '关闭'}"
        temp_info = " | ".join([f"温度{i+1}: {t:.1f}℃" for i, t in enumerate(self.temperatures)]) if self.temperatures else "无"
        return (
            f"===== 电池状态信息（Modbus RTU协议V1.0）=====\n"
            f"电池类型: {self.battery_type}\n"
            f"电池串数: {self.battery_series}\n"
            f"总电压: {self.total_voltage:.2f}V\n"
            f"总电流: {self.current:.2f}A（正数=充电，负数=放电）\n"
            f"剩余容量: {self.remaining_capacity:.2f}Ah\n"
            f"标称容量: {self.nominal_capacity:.2f}Ah\n"
            f"剩余电量百分比: {self.battery_remaining:.2f}%\n"
            f"循环次数: {self.cycle_count}\n"
            f"MOS状态: {mos_info}\n"
            f"温度信息: {temp_info}\n"
            f"=========================================="
        )

class BatteryParser:
    def __init__(self):
        # 测试帧：模拟读取寄存器0000H-0061H（包含核心参数：电量、电流、电压、温度等）
        # 帧格式：地址(01) + 功能码(03) + 起始地址(0000) + 寄存器个数(0062) + CRC校验(940B) + 帧尾(77)
        # 数据段长度：0062个寄存器 × 2字节 = 194字节（0xC2）
        self.test_frame = bytes.fromhex(
            "01 03 00 00 00 62 94 0B "  # 帧头+CRC
            # 数据段：寄存器0000H-0061H（共98个寄存器，196字节）- 模拟数据
            "03 E8 "  # 0000H: 剩余电量百分比（1000 → 1000×0.01% = 10.00%）
            "04 D2 "  # 0001H: 总电流（1234 → 1234×0.01A = 12.34A 充电）
            "27 10 "  # 0002H: 总电压（10000 → 10000×0.01V = 100.00V）
            "07 D0 "  # 0003H: 剩余容量（2000 → 2000×0.1Ah = 200.0Ah）
            "0BB8 "  # 0004H: 标称容量（3000 → 3000×0.1Ah = 300.0Ah）
            "03 E8 "  # 0005H: 循环容量（1000 → 1000×0.1Ah = 100.0Ah）
            "00 64 "  # 0006H: 循环次数（100次）
            "FFFF "  # 0007H: 剩余放电时间（非放电状态）
            "00 3C "  # 0008H: 剩余充电时间（60分钟）
            "00 02 "  # 0009H: 容量学习状态（已完成）
            "00 01 "  # 000AH: 充电MOS状态（开启）
            "00 01 "  # 000BH: 放电MOS状态（开启）
            "00 00 " * 14  # 000CH-001FH: 均衡状态位（未均衡）
            "0C 80 " * 32  # 0020H-003FH: 32节电池电压（3200mV/节）
            "00 00 " * 16  # 0040H-004FH: 49-64节电池电压（未使用）
            "01 2C "  # 0050H: 第1路电池温度（300 → 300×0.1℃ = 30.0℃）
            "01 38 "  # 0051H: 第2路电池温度（312 → 31.2℃）
            "01 44 "  # 0052H: 第3路电池温度（324 → 32.4℃）
            "FFFF " * 13  # 0053H-005EH: 4-16路温度（未使用）
            "01 50 "  # 005FH: 第16路电池温度（336 → 33.6℃）
            "01 5A "  # 0060H: PCB温度（346 → 34.6℃）
            "01 64 "  # 0061H: 环境温度（356 → 35.6℃）
            "77"  # 帧尾
        )
        self.battery_buffer = bytearray()
        self.register_map = {
            "remaining_percent": 0x0000,    # 剩余电量百分比（0.01%）
            "total_current": 0x0001,        # 总电流（0.01A，有符号）
            "total_voltage": 0x0002,        # 总电压（0.01V）
            "remaining_capacity": 0x0003,   # 剩余容量（0.1Ah）
            "nominal_capacity": 0x0004,     # 标称容量（0.1Ah）
            "cycle_count": 0x0006,          # 循环次数
            "charge_mos": 0x000A,           # 充电MOS状态
            "discharge_mos": 0x000B,        # 放电MOS状态
            "battery_series": 0x0063,       # 电池串数
            "battery_type": 0x0066,         # 电池类型
            "temp_1": 0x0050,               # 第1路电池温度（0.1℃）
            "temp_2": 0x0051,               # 第2路电池温度
            "temp_3": 0x0052,               # 第3路电池温度
            "temp_16": 0x005F,              # 第16路电池温度
            "pcb_temp": 0x0060,             # PCB温度
            "env_temp": 0x0061              # 环境温度
        }

    def modbus_crc16(self, data):
        """计算Modbus RTU CRC16校验码（低位在前，高位在后）"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    def parse_uint16(self, data_bytes):
        """解析无符号16位数据（高位在前）"""
        return (data_bytes[0] << 8) | data_bytes[1]

    def parse_int16(self, data_bytes):
        """解析有符号16位数据（高位在前，补码）"""
        value = self.parse_uint16(data_bytes)
        return value - 65536 if value >= 0x8000 else value

    def get_register_value(self, data_segment, start_addr, reg_addr):
        """从数据段中获取指定寄存器的值"""
        # 寄存器偏移量 = 目标寄存器地址 - 起始寄存器地址
        offset = reg_addr - start_addr
        if offset < 0:
            return None
        # 数据段中位置 = 偏移量 × 2字节/寄存器
        pos = offset * 2
        if pos + 1 >= len(data_segment):
            return None
        return data_segment[pos:pos+2]

    def process_battery_response(self, frame):
        """解析Modbus RTU响应帧"""
        status = BatteryStatus()
        try:
            # 帧结构校验：最小长度=地址(1)+功能码(1)+数据长度(1)+数据段(n)+CRC(2)+帧尾(1)
            if len(frame) < 8:
                print("[错误] 帧长度过短")
                return None

            # 提取帧头信息
            addr = frame[0]
            func_code = frame[1]
            data_len = frame[2]
            data_segment = frame[3:3+data_len]
            crc_recv = frame[3+data_len:5+data_len]
            frame_tail = frame[5+data_len] if len(frame) > 5+data_len else None

            # 校验功能码、帧尾、CRC
            if func_code != 0x03:
                print(f"[错误] 不支持的功能码: 0x{func_code:02X}")
                return None
            if frame_tail != 0x77:
                print(f"[错误] 帧尾错误，预期0x77，实际0x{frame_tail:02X}")
                return None
            crc_calc = self.modbus_crc16(frame[0:3+data_len])
            if crc_recv != crc_calc:
                print(f"[错误] CRC校验失败：接收0x{crc_recv.hex()}，计算0x{crc_calc.hex()}")
                return None

            # 提取起始寄存器地址（从帧头的第3-4字节获取，测试帧中为0000H）
            start_addr = 0x0000  # 实际场景需从主机下发命令中解析，此处简化为协议默认起始地址

            # 1. 解析剩余电量百分比（0000H，无符号，0.01%）
            percent_bytes = self.get_register_value(data_segment, start_addr, self.register_map["remaining_percent"])
            if percent_bytes:
                status.battery_remaining = self.parse_uint16(percent_bytes) * 0.01

            # 2. 解析总电流（0001H，有符号，0.01A）
            current_bytes = self.get_register_value(data_segment, start_addr, self.register_map["total_current"])
            if current_bytes:
                status.current = self.parse_int16(current_bytes) * 0.01

            # 3. 解析总电压（0002H，无符号，0.01V）
            voltage_bytes = self.get_register_value(data_segment, start_addr, self.register_map["total_voltage"])
            if voltage_bytes:
                status.total_voltage = self.parse_uint16(voltage_bytes) * 0.01

            # 4. 解析剩余容量（0003H，无符号，0.1Ah）
            remain_cap_bytes = self.get_register_value(data_segment, start_addr, self.register_map["remaining_capacity"])
            if remain_cap_bytes:
                status.remaining_capacity = self.parse_uint16(remain_cap_bytes) * 0.1

            # 5. 解析标称容量（0004H，无符号，0.1Ah）
            nom_cap_bytes = self.get_register_value(data_segment, start_addr, self.register_map["nominal_capacity"])
            if nom_cap_bytes:
                status.nominal_capacity = self.parse_uint16(nom_cap_bytes) * 0.1

            # 6. 解析循环次数（0006H，无符号）
            cycle_bytes = self.get_register_value(data_segment, start_addr, self.register_map["cycle_count"])
            if cycle_bytes:
                status.cycle_count = self.parse_uint16(cycle_bytes)

            # 7. 解析MOS管状态（000AH/000BH）
            charge_mos_bytes = self.get_register_value(data_segment, start_addr, self.register_map["charge_mos"])
            if charge_mos_bytes:
                status.charge_mos_state = self.parse_uint16(charge_mos_bytes)
            discharge_mos_bytes = self.get_register_value(data_segment, start_addr, self.register_map["discharge_mos"])
            if discharge_mos_bytes:
                status.discharge_mos_state = self.parse_uint16(discharge_mos_bytes)

            # 8. 解析电池串数（0063H，无符号）
            series_bytes = self.get_register_value(data_segment, start_addr, self.register_map["battery_series"])
            if series_bytes:
                status.battery_series = self.parse_uint16(series_bytes)

            # 9. 解析电池类型（0066H，0=磷酸铁锂，1=三元锂）
            type_bytes = self.get_register_value(data_segment, start_addr, self.register_map["battery_type"])
            if type_bytes:
                type_val = self.parse_uint16(type_bytes)
                status.battery_type = "磷酸铁锂" if type_val == 0 else "三元锂" if type_val == 1 else "未知"

            # 10. 解析温度（0050H-0061H，有符号，0.1℃）
            temp_regs = [
                self.register_map["temp_1"],
                self.register_map["temp_2"],
                self.register_map["temp_3"],
                self.register_map["temp_16"],
                self.register_map["pcb_temp"],
                self.register_map["env_temp"]
            ]
            temp_names = ["电池1", "电池2", "电池3", "电池16", "PCB", "环境"]
            for i, reg in enumerate(temp_regs):
                temp_bytes = self.get_register_value(data_segment, start_addr, reg)
                if temp_bytes:
                    temp_val = self.parse_int16(temp_bytes) * 0.1
                    status.temperatures.append(temp_val)

            return status

        except Exception as e:
            print(f"[错误] 解析失败: {e}")
            return None

    def process_battery_buffer(self):
        """从缓冲区提取完整帧并解析"""
        MAX_LEN = 256  # Modbus RTU最大帧长（含帧尾）
        if len(self.battery_buffer) > MAX_LEN:
            self.battery_buffer.clear()
            return None

        start_idx = 0
        while start_idx < len(self.battery_buffer):
            # 查找帧头（地址字节，默认1-252，此处简化为任意非0x77字节）
            if self.battery_buffer[start_idx] == 0x77:
                start_idx += 1
                continue

            # 查找帧尾（0x77）
            end_idx = self.battery_buffer.find(0x77, start_idx)
            if end_idx == -1:
                return None  # 未找到完整帧

            # 提取完整帧（包含帧尾）
            frame = self.battery_buffer[start_idx:end_idx+1]
            # 移除已解析的帧
            self.battery_buffer = self.battery_buffer[end_idx+1:]

            # 解析帧
            result = self.process_battery_response(frame)
            if result:
                return result

            start_idx += 1
        return None

    def simulate_receive(self):
        """模拟接收并解析测试数据"""
        print(f"原始数据帧: {self.test_frame.hex().upper()}")
        for byte in self.test_frame:
            self.battery_buffer.append(byte)
            result = self.process_battery_buffer()
            if result:
                print("\n===== 解析成功 =====")
                print(result)
                return
        print("\n解析失败")

if __name__ == "__main__":
    parser = BatteryParser()
    parser.simulate_receive()