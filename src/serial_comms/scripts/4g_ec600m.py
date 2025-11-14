#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial
import time

class EC600MDialer:
    def __init__(self, port='/dev/ttyS3', baudrate=115200):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=3  # 超时时间3秒
        )
        self.connected = False

    def send_at_command(self, cmd, timeout=2, expect_response=None):
        """发送AT指令并返回响应"""
        if not self.ser.is_open:
            self.ser.open()
        # 清空缓冲区
        self.ser.flushInput()
        self.ser.flushOutput()
        # 发送指令（末尾加\r\n，部分模块仅需\r，可根据实际调整）
        self.ser.write((cmd + '\r\n').encode('utf-8'))
        time.sleep(timeout)
        # 读取响应
        response = b''
        while self.ser.in_waiting:
            response += self.ser.read(self.ser.in_waiting)
            time.sleep(0.1)
        response_str = response.decode('utf-8', errors='ignore').strip()
        print(f"发送指令: {cmd}")
        print(f"响应: {response_str}\n")
        # 检查是否包含预期响应（如OK、CONNECT）
        if expect_response and expect_response not in response_str:
            raise Exception(f"指令 {cmd} 执行失败，响应不包含 {expect_response}")
        return response_str

    def dial(self, apn='CMNET'):
        """拨号上网流程"""
        try:
            # 1. 检查模块是否响应
            self.send_at_command('AT', expect_response='OK')
            
            # 2. 检查SIM卡状态（0=就绪）
            self.send_at_command('AT+CPIN?', expect_response='READY')
            
            # 3. 配置APN（PDP上下文1）
            self.send_at_command(f'AT+CGDCONT=1,"IP","{apn}"', expect_response='OK')
            
            # 4. 激活PDP上下文（建立连接）
            self.send_at_command('AT+CGACT=1,1', expect_response='OK')
            
            # 5. 获取分配的IP地址
            ip_response = self.send_at_command('AT+CGPADDR=1')
            if '10.' in ip_response or '192.' in ip_response or '172.' in ip_response:
                print(f"拨号成功！IP地址: {ip_response.split(':')[-1].strip()}")
                self.connected = True
            else:
                raise Exception("未获取到IP地址，连接失败")
                
        except Exception as e:
            print(f"拨号失败: {str(e)}")
        finally:
            if self.ser.is_open:
                self.ser.close()
    def ping_test(self, target='8.8.8.8', count=3):
        try:
            # 发送ping指令（EC600M支持AT+QPING命令）
            self.send_at_command(f'AT+QPING=1,"{target}",{count}', expect_response='OK')
        except Exception as e:
            print(f"Ping测试失败: {str(e)}")


    def hangup(self):
        """断开连接"""
        if self.connected and self.ser.is_open:
            self.send_at_command('AT+CGACT=0,1', expect_response='OK')
            print("已断开连接")
        self.ser.close()

if __name__ == "__main__":
    dialer = EC600MDialer(port='/dev/ttyS3', baudrate=115200)
    # 注意：APN需根据SIM卡运营商修改，例如：
    # 中国移动：CMNET
    # 中国联通：UNINET
    # 中国电信：CTNET
    dialer.dial(apn='CTNET')  # 替换为你的运营商APN
    if dialer.connected:
        dialer.ping_test(target='121.40.57.48') 