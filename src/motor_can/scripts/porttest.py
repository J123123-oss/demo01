from pymodbus.client import ModbusSerialClient
client = ModbusSerialClient(port='/dev/ttyUSB0', baudrate=38400, stopbits=1, parity='N', timeout=3)
if client.connect():
    print("连接成功")
else:
    print("连接失败")