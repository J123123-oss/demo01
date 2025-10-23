#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class BatteryStatus:
    def __init__(self):
        self.total_voltage = 0.0
        self.current = 0.0
        self.remaining_capacity = 0.0
        self.mos_state = 0
        self.battery_series = 0
        self.ntc_count = 0
        self.nominal_capacity = 0.0
        self.cycle_count = 0
        self.production_year = 0
        self.production_month = 0
        self.production_day = 0
        self.balance_low = 0
        self.balance_high = 0
        self.protection_status = 0
        self.software_version = ""
        self.batttery_remaining = 0.0
        self.temperatures = []

    def __str__(self):
        return (
            f"===== 电池状态信息 =====\n"
            f"总电压: {self.total_voltage:.2f}V\n"
            f"电流: {self.current:.2f}A\n"
            f"剩余容量: {self.remaining_capacity:.2f}Ah\n"
            f"标称容量: {self.nominal_capacity:.2f}Ah\n"
            f"循环次数: {self.cycle_count}\n"
            f"生产日期: {self.production_year}-{self.production_month:02d}-{self.production_day:02d}\n"
            f"电量百分比: {self.batttery_remaining:.1f}%\n"
            f"MOS状态: 0x{self.mos_state:02X}\n"
            f"电池串数: {self.battery_series}\n"
            f"温度列表: {self.temperatures}°C\n"
            f"软件版本: {self.software_version}"
        )

class BatteryParser:
    def __init__(self):
        # 你的测试数据帧
        self.test_frame = bytes.fromhex(
            "DD 03 00 26 14 DE 00 00 06 60 07 D0 00 10 33 0F 00 00 00 00 00 00 A1 52 03 0D 03 0B 2B 0B 26 0B 17 00 00 00 07 D0 06 60 00 00 FA 8D 77"
            # "DD 03 00 26 14 DE 00 00 06 60 07 D0 00 10 33 0F 00 00 00 00 00 00 A1 52 03 0D 03 0B 2B 0B 26 0B 17 00 00 00 07 D0 06 60 00 00 FA 8D 77"
            # "DD 03 00 26 13 72 00 00 03 90 07 D0 00 0F 33 0F 00 00 00 00 00 20 A1 2E 02 0D 03 0A 83 0A 7D 0A 72 00 00 00 07 D0 03 90 00 00 F9 9F 77"
            # "DD 03 00 26 12 CC FF E5 02 A1 07 D0 00 0E 33 0F 00 00 00 00 00 00 A1 22 03 0D 03 0A E3 0A DD 0A D4 00 00 00 07 D0 02 A1 00 18 F6 34 77"
            # "DD 03 00 26 12 06 00 43 01 A5 07 D0 00 0E 33 0F 00 00 00 00 00 00 A1 15 03 0D 03 0A E6 0A E6 0A D3 00 00 00 07 D0 01 A5 00 F6 F7 B9 77"
        )
        self.battery_buffer = bytearray()

    def calculate_checksum(self, check_bytes):
        """
        计算校验码：校验范围字节总和 → 取反+1（16位），高位在前
        :param check_bytes: 校验范围字节列表（不包含命令码03）
        :return: 校验码（高位字节, 低位字节）和16位值
        """
        total = sum(check_bytes)
        checksum = (-total) & 0xFFFF  # 取反+1，限制16位
        high_byte = (checksum >> 8) & 0xFF
        low_byte = checksum & 0xFF
        return (high_byte, low_byte), checksum

    def parse_date(self, raw_date):
        value = (raw_date[0] << 8) | raw_date[1]
        day = value & 0x1F
        month = (value >> 5) & 0x0F
        year = 2000 + (value >> 9)
        return year, month, day

    def parse_current(self, data_bytes):
        value = (data_bytes[0] << 8) | data_bytes[1]
        return (value - 65536) * 0.01 if value >= 0x8000 else value * 0.01

    def process_battery_response(self, data_segment, received_checksum, check_bytes):
        """解析数据并验证校验码"""
        status = BatteryStatus()
        try:
            # 验证校验码
            (calc_high, calc_low), calc_checksum = self.calculate_checksum(check_bytes)
            recv_high = (received_checksum >> 8) & 0xFF
            recv_low = received_checksum & 0xFF

            if (calc_high, calc_low) != (recv_high, recv_low):
                print(f"[错误] 校验码不匹配！")
                print(f"接收: 0x{recv_high:02X}{recv_low:02X}")
                print(f"计算: 0x{calc_high:02X}{calc_low:02X}")
                return None

            # 解析数据段（38字节）
            status.total_voltage = ((data_segment[0] << 8) | data_segment[1]) * 0.01
            status.current = self.parse_current(data_segment[2:4])
            status.remaining_capacity = ((data_segment[4] << 8) | data_segment[5]) * 0.01
            status.nominal_capacity = ((data_segment[6] << 8) | data_segment[7]) * 0.01
            status.cycle_count = (data_segment[8] << 8) | data_segment[9]

            year, month, day = self.parse_date(data_segment[10:12])
            status.production_year = year
            status.production_month = month
            status.production_day = day

            status.balance_low = (data_segment[12] << 8) | data_segment[13]
            status.balance_high = (data_segment[14] << 8) | data_segment[15]
            status.protection_status = (data_segment[16] << 8) | data_segment[17]

            ver_major = data_segment[18] >> 4
            ver_minor = data_segment[18] & 0x0F
            status.software_version = f"{ver_major}.{ver_minor}"

            status.batttery_remaining = float(data_segment[19])
            status.mos_state = data_segment[20]
            status.battery_series = data_segment[21]

            status.ntc_count = data_segment[22]
            required_len = 23 + 2 * status.ntc_count
            if len(data_segment) < required_len:
                print(f"[错误] 温度数据不足，需{required_len}字节")
                return status

            for i in range(status.ntc_count):
                idx = 23 + i * 2
                raw_temp = (data_segment[idx] << 8) | data_segment[idx + 1]
                status.temperatures.append(round((raw_temp - 2731)/10.0, 1))

            return status

        except Exception as e:
            print(f"[错误] 解析失败: {e}")
            return None

    def process_battery_buffer(self):
        """提取完整帧并解析"""
        MAX_LEN = 64
        if len(self.battery_buffer) > MAX_LEN:
            self.battery_buffer.clear()
            return None

        start_idx = 0
        while start_idx < len(self.battery_buffer):
            if self.battery_buffer[start_idx] != 0xDD:
                start_idx += 1
                continue

            # 校验帧头后字段（至少4字节：DD 03 00 26）
            if start_idx + 3 >= len(self.battery_buffer):
                return None
            command_code = self.battery_buffer[start_idx + 1]  # 0x03
            status_code = self.battery_buffer[start_idx + 2]   # 0x00
            length_byte = self.battery_buffer[start_idx + 3]   # 0x26（38字节数据段）

            if command_code != 0x03:
                start_idx += 1
                continue

            # 完整帧长度：4(帧头) + 38(数据段) + 2(校验码) + 1(帧尾) = 45
            full_len = 4 + length_byte + 2 + 1
            if len(self.battery_buffer) < start_idx + full_len:
                return None

            # 校验帧尾
            if self.battery_buffer[start_idx + full_len - 1] != 0x77:
                start_idx += 1
                continue

            # 提取关键字段
            data_start = start_idx + 4
            data_end = data_start + length_byte
            data_segment = self.battery_buffer[data_start:data_end]  # 数据段（38字节）

            checksum_start = data_end
            received_checksum = (self.battery_buffer[checksum_start] << 8) | self.battery_buffer[checksum_start + 1]

            # 校验范围：命令码(03) + 状态码(00) + 长度字节(26) + 数据段 + 补充字段(00 F6)
            # 对应原始帧索引1~40：03 00 26 12 06 ... 00 F6（共40字节）
            check_bytes = self.battery_buffer[start_idx + 2 : checksum_start]

            # 解析
            result = self.process_battery_response(data_segment, received_checksum, check_bytes)
            self.battery_buffer.clear()
            return result

        return None

    def simulate_receive(self):
        """模拟接收并解析你的测试数据"""
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