#!/bin/bash

sudo slcand -o -c -s8 /dev/CAN0 can0
sudo ifconfig can0 up

# 检查can0状态
can_status=$(ip -details link show can0 2>/dev/null)
if [[ -z "$can_status" ]]; then
    echo "can0接口未正常启动，正在重试..."
    exec "$0"
else
    echo "can0接口已正常启动"
fi

# 每30秒检测一次can0状态，掉线则重启slcand和ifconfig
(
    while true; do
        sleep 30
        can_status=$(ip -details link show can0 2>/dev/null)
        if [[ -z "$can_status" ]]; then
            echo "检测到can0掉线，正在重启..."
            sudo slcand -o -c -s8 /dev/CAN0 can0
            sudo ifconfig can0 up
        fi
    done
) &


