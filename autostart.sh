#!/bin/bash

# 启动或重启slcand，并等待can0设备就绪
# 1. 加载vcan模块
sudo modprobe vcan
# 2. 创建vCAN接口
sudo ip link add dev vcan0 type vcan
# 3. 配置vCAN接口
sudo ip link set vcan0 up