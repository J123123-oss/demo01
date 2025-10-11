#!/bin/bash
# 网络时间校准脚本，仅在拨号成功后运行一次

# 配置参数
TTY_DEVICE="/dev/ttyUSB3"
BAUDRATE=115200
MAX_RETRY=3  # 获取时间最大重试次数
DIAL_CHECK_INTERVAL=5  # 拨号检查间隔(秒)

# 等待拨号成功
wait_for_dial() {
    echo "等待拨号成功..."
    while true; do
        # 检查是否有有效的网络连接（可根据实际拨号方式调整）
        if ping -c 1 121.40.57.48 >/dev/null 2>&1; then
            echo "拨号成功，开始时间校准"
            return 0
        fi
        sleep $DIAL_CHECK_INTERVAL
    done
}

# 通过AT指令获取网络时间（使用文件描述符直接操作串口）
get_network_time() {
    local retry=0
    local time_str=""
    
    # 确保串口权限
    sudo chmod 666 $TTY_DEVICE 2>/dev/null

    while [ $retry -lt $MAX_RETRY ]; do
        # 直接操作串口设备
        exec 3<> $TTY_DEVICE
        stty -F $TTY_DEVICE $BAUDRATE cs8 -cstopb -parenb raw

        # 发送AT指令
        echo -e "AT+QLTS=2\r" >&3
        sleep 1  # 等待响应

        # 读取响应（最多读取512字节）
        response=$(dd if=$TTY_DEVICE bs=512 count=1 2>/dev/null)
        exec 3>&-  # 关闭文件描述符

        # 解析响应
        if echo "$response" | grep -q "+QLTS:"; then
            time_str=$(echo "$response" | grep "+QLTS:" | awk -F'"' '{print $2}' | cut -d',' -f1,2)
            formatted_time=$(echo "$time_str" | sed 's/\//-/g; s/,/ /g')
            echo "$formatted_time"
            return 0
        fi

        retry=$((retry + 1))
        echo "获取网络时间失败，重试第$retry次..."
        sleep 2
    done

    echo ""  # 返回空值表示失败
    return 1
}


# 主流程
main() {
    # 等待拨号成功
    wait_for_dial || exit 1
    
    # 获取网络时间
    local net_time=$(get_network_time)
    if [ -z "$net_time" ]; then
        exit 1
    fi
    
    # 校准系统时间
    echo "校准系统时间为: $net_time"
    if sudo date -s "$net_time"; then
        echo "时间校准成功"
        # 同步硬件时钟（可选）
        sudo hwclock -w
    else
        echo "时间校准失败"
        exit 1
    fi
}

# 确保脚本只运行一次
LOCK_FILE="/var/lock/ntp_calibration.lock"
if [ -f "$LOCK_FILE" ]; then
    echo "脚本已运行过，退出"
    exit 0
fi

main && touch "$LOCK_FILE"