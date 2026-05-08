import serial
import serial.tools.list_ports

# ====================== 协议固定参数 ======================
DEFAULT_ADDR = 0x01        # 从机地址
BAUDRATE = 19200           # 波特率
TIMEOUT = 1                # 超时秒

# ====================== CRC16 计算（协议原版） ======================
def crc16_modbus(buffer: bytes) -> bytes:
    wcrc = 0xFFFF
    for b in buffer:
        wcrc ^= b
        for _ in range(8):
            if wcrc & 0x0001:
                wcrc >>= 1
                wcrc ^= 0xA001
            else:
                wcrc >>= 1
    crc_l = wcrc & 0xFF
    crc_h = (wcrc >> 8) & 0xFF
    return bytes([crc_l, crc_h])

# ====================== 指令构造 ======================
def make_cmd_write(addr, reg, value):
    buf = bytes([addr, 0x06, (reg >> 8) & 0xFF, reg & 0xFF, (value >> 8) & 0xFF, value & 0xFF])
    return buf + crc16_modbus(buf)

def make_cmd_read(addr, start_reg, count):
    buf = bytes([addr, 0x03, (start_reg >> 8) & 0xFF, start_reg & 0xFF, (count >> 8) & 0xFF, count & 0xFF])
    return buf + crc16_modbus(buf)

# ====================== 3大核心功能 ======================
def start_charge(ser):
    cmd = make_cmd_write(DEFAULT_ADDR, 0x00, 0x00B1)
    ser.write(cmd)
    resp = ser.read(8)
    print("[开始充电] 指令:", cmd.hex(' ').upper())
    print("[开始充电] 应答:", resp.hex(' ').upper() if resp else "无应答")
    return resp

def stop_charge(ser):
    cmd = make_cmd_write(DEFAULT_ADDR, 0x00, 0x00B2)
    ser.write(cmd)
    resp = ser.read(8)
    print("[停止充电] 指令:", cmd.hex(' ').upper())
    print("[停止充电] 应答:", resp.hex(' ').upper() if resp else "无应答")
    return resp

def query_status(ser):
    # 读 0x01~0x05：工作模式、电压、电流、充电状态、故障码
    cmd = make_cmd_read(DEFAULT_ADDR, 0x01, 5)
    ser.write(cmd)
    resp = ser.read(15)
    print("[查询状态] 指令:", cmd.hex(' ').upper())
    if not resp or len(resp) < 9:
        print("[查询状态] 无有效应答")
        return
    # 解析
    byte_cnt = resp[2]
    data = resp[3:3+byte_cnt]
    mode      = (data[0]<<8) | data[1]
    volt      = (data[2]<<8) | data[3]
    curr      = (data[4]<<8) | data[5]
    charge_st = (data[6]<<8) | data[7]
    err_code  = (data[8]<<8) | data[9]
    print("===== 设备状态 =====")
    print(f"工作模式    : {mode:04X}")
    print(f"输出电压    : {volt*0.01:.2f} V")
    print(f"输出电流    : {curr*0.01:.2f} A")
    print(f"充电状态    : {charge_st:04X} (0=未充电 1=涓流 2=恒流 3=恒压)")
    print(f"故障码      : {err_code:04X}")
    print("====================")

# ====================== 扫描串口 ======================
def scan_ports():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    print("可用串口:", ports)
    return ports

# ====================== 主菜单 ======================
def main():
    scan_ports()
    port = input("请输入串口号(如COM3 / /dev/ttyUSB0): ").strip()
    try:
        ser = serial.Serial(
            port=port, baudrate=BAUDRATE,
            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS, timeout=TIMEOUT
        )
        print(f"\n✅ 串口 {port} 已打开")
        while True:
            print("\n===== 充电控制工具 =====")
            print("1 → 开始充电")
            print("2 → 停止充电")
            print("3 → 查询状态(电压/电流/模式/故障)")
            print("0 → 退出")
            opt = input("请选择: ").strip()
            if opt == "1":
                start_charge(ser)
            elif opt == "2":
                stop_charge(ser)
            elif opt == "3":
                query_status(ser)
            elif opt == "0":
                print("退出...")
                break
            else:
                print("输入无效")
        ser.close()
    except Exception as e:
        print(f"异常: {e}")

if __name__ == "__main__":
    main()