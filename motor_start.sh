#!/bin/bash

# 设置需要检测的目标IP和端口（如果需要）
IP="121.40.57.48"
RETRY_INTERVAL=15  # 重试时间间隔（秒）

# 网络连接检测函数
check_network() {
    ping -c 2 -W 3 $IP > /dev/null 2>&1
    return $?
}

# 循环检测网络连接并运行指令
while true; do
    # 检测网络是否连接
    if check_network; then
        # 如果网络连接正常，执行指令
        echo "网络连接正常，正在运行指令..."
        source /home/ubuntu/demo01/devel/setup.bash && roslaunch motor_can run.launch
        # 检查网络断开，断开后会退出当前运行并继续重试
        while check_network; do
            sleep 30  # 每30秒检查一次
        done
        echo "网络已断开，重新连接中..."
    else
        # 网络连接失败，等待重试
        echo "网络连接失败，正在重试..."
        sleep $RETRY_INTERVAL
    fi
done
