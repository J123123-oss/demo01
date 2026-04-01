#!/bin/bash

# 启动或重启slcand，并等待can0设备就绪
restart_can() {
    # 先停止可能残留的slcand进程，避免冲突
    sudo pkill slcand 2>/dev/null
    # 启动slcand（-o后台运行，-c使能，-s8波特率800000）
    sudo slcand -o -c -s8 /dev/CAN0 can0
    # 等待can0设备创建（最多等5秒，每0.5秒检查一次）
    for i in {1..10}; do
        if ip link show can0 >/dev/null 2>&1; then
            break
        fi
        sleep 0.5
    done
    # 启动can0接口
    sudo ifconfig can0 up
    # 检查最终状态
    if ip link show can0 >/dev/null 2>&1; then
        echo "can0接口已正常启动"
        return 0
    else
        echo "can0接口启动失败"
        return 1
    fi
}

# 初始启动
restart_can

# 后台循环检测（每5秒）
(
    while true; do
        sleep 5
        if ! ip link show can0 >/dev/null 2>&1; then
            echo "检测到can0掉线，正在重启..."
            restart_can
        fi
    done
) &