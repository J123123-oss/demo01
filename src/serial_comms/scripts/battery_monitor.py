#!/usr/bin/env python3
import rospy
import time
from pymodbus.client import ModbusSerialClient
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.constants import Endian
from serial_comms.msg import BatteryStatus  # 导入自定义msg

class BatteryModbusROS:
    """ROS版电池Modbus RTU监控节点"""
    def __init__(self):
        # 1. 初始化ROS节点
        rospy.init_node('battery_modbus_node', anonymous=True)
        
        # 2. 获取ROS参数（支持launch文件配置）
        self.port = rospy.get_param('~serial_port', '/dev/ttyUSB0')  # 默认Linux串口
        self.baudrate = rospy.get_param('~baudrate', 9600)
        self.slave_id = rospy.get_param('~slave_id', 1)
        self.query_freq = rospy.get_param('~query_frequency', 0.2)  # 0.2Hz = 5秒/次
        
        # 3. 初始化Modbus客户端
        self.client = ModbusSerialClient(
            port=self.port,
            baudrate=self.baudrate,
            parity='N',
            stopbits=1,
            bytesize=8,
            timeout=1
        )
        
        # 4. 创建发布者，话题名/battery_status，队列大小10
        self.pub = rospy.Publisher('/battery_status', BatteryStatus, queue_size=10)
        
        # 5. 连接Modbus设备
        if not self.client.connect():
            rospy.logerr("Modbus RTU连接失败！请检查串口/硬件连接")
            rospy.signal_shutdown("连接失败")
        
        rospy.loginfo("电池监控节点初始化完成，开始5秒周期查询...")

    def decode_mos_state(self, charge_mos, discharge_mos):
        """
        编码MOS状态到uint8：
        bit0: 充电MOS（0=关闭，1=开启）
        bit1: 放电MOS（0=关闭，1=开启）
        示例：充电开+放电关 → 0b01 → 1；充电关+放电开 → 0b10 → 2；都开 → 0b11 →3
        """
        mos_state = 0
        if charge_mos == 1:
            mos_state |= 0x01  # 置位bit0
        if discharge_mos == 1:
            mos_state |= 0x02  # 置位bit1
        return mos_state

    def read_all_data(self):
        """读取所有Modbus数据并封装为ROS msg"""
        msg = BatteryStatus()
        
        # ========== 1. 读取电量信息 ==========
        try:
            # 读取0000H-0002H（地址0-2）
            resp_power = self.client.read_holding_registers(0, 3, self.slave_id)
            if not resp_power.isError():
                decoder = BinaryPayloadDecoder.fromRegisters(
                    resp_power.registers, Endian.Big, Endian.Big
                )
                # 剩余电量百分比（0.01%单位）
                msg.batttery_remaining = decoder.decode_16bit_uint() * 0.01
                # 总电流（0.01A单位，有符号）
                msg.current = decoder.decode_16bit_int() * 0.01
                # 总电压（0.01V单位）
                msg.total_voltage = decoder.decode_16bit_uint() * 0.01
            else:
                rospy.logwarn("读取电量信息失败: %s", resp_power)
        except Exception as e:
            rospy.logerr("电量信息读取异常: %s", e)
        
        # ========== 2. 读取MOS管状态 ==========
        try:
            # 读取000AH-000BH（地址10-11）
            resp_mos = self.client.read_holding_registers(10, 2, self.slave_id)
            if not resp_mos.isError():
                charge_mos = resp_mos.registers[0]
                discharge_mos = resp_mos.registers[1]
                # 编码MOS状态到uint8
                msg.mos_state = self.decode_mos_state(charge_mos, discharge_mos)
            else:
                rospy.logwarn("读取MOS状态失败: %s", resp_mos)
        except Exception as e:
            rospy.logerr("MOS状态读取异常: %s", e)
        
        # ========== 3. 读取温度信息 ==========
        try:
            # 读取005FH-0061H（地址95-97）
            resp_temp = self.client.read_holding_registers(95, 3, self.slave_id)
            if not resp_temp.isError():
                decoder = BinaryPayloadDecoder.fromRegisters(
                    resp_temp.registers, Endian.Big, Endian.Big
                )
                # 温度数组：[第16路电池温度, PCB温度, 环境温度]
                temp_16 = decoder.decode_16bit_int() * 0.1
                temp_pcb = decoder.decode_16bit_int() * 0.1
                temp_env = decoder.decode_16bit_int() * 0.1
                msg.temperatures = [temp_16, temp_pcb, temp_env]
            else:
                rospy.logwarn("读取温度信息失败: %s", resp_temp)
        except Exception as e:
            rospy.logerr("温度信息读取异常: %s", e)
        
        # 剩余容量（协议未提供原始值，暂设为0，可根据实际寄存器补充）
        msg.remaining_capacity = 0.0
        
        return msg

    def run(self):
        """主循环：5秒发布一次数据"""
        rate = rospy.Rate(self.query_freq)  # 0.2Hz = 5秒/次
        while not rospy.is_shutdown():
            # 读取所有数据
            battery_msg = self.read_all_data()
            # 发布消息
            self.pub.publish(battery_msg)
            # 打印日志（可选）
            rospy.loginfo("\n发布电池状态：")
            rospy.loginfo(f"剩余电量: {battery_msg.batttery_remaining:.2f}%")
            rospy.loginfo(f"总电压: {battery_msg.total_voltage:.2f}V")
            rospy.loginfo(f"总电流: {battery_msg.current:.2f}A")
            rospy.loginfo(f"温度: {battery_msg.temperatures}°C")
            rospy.loginfo(f"MOS状态: 0x{battery_msg.mos_state:02x}")
            
            # 等待周期
            rate.sleep()
        
        # 关闭连接
        self.client.close()
        rospy.loginfo("节点关闭，Modbus连接已断开")

if __name__ == '__main__':
    try:
        # 安装pymodbus（若未安装）
        # import subprocess
        # import sys
        # subprocess.check_call([sys.executable, "-m", "pip", "install", "pymodbus"])
        
        # 启动节点
        battery_node = BatteryModbusROS()
        battery_node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被中断")
    except Exception as e:
        rospy.logerr("节点启动失败: %s", e)