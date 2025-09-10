#!/bin/bash

# 等待网络连接
wait_for_network() {
    echo "等待网络连接..."
    while ! ping -c 1 -W 1 121.40.57.48 > /dev/null 2>&1; do
        sleep 2
    done
    echo "网络已连接"
}

# 主循环
while true; do
    wait_for_network
    
    echo "启动主程序..."
    # 这里启动您的程序
    source /home/orangepi/demo01/devel/setup.bash && roslaunch motor_can run.launch
    # 如果程序退出，等待一段时间后重启
    echo "主程序退出, 10秒后重启..."
    sleep 10
done


# sleep 60
# source /home/orangepi/demo01/devel/setup.bash && roslaunch motor_can run.launch
