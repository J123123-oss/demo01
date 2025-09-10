#!/bin/bash
PPP_IFACE="ens33"  # ppp接口名称

# 检查4G连接状态
check_4g_connection() {
    if ping -c 2 -W 3 121.40.57.48 > /dev/null 2>&1; then
        if ip link show $PPP_IFACE > /dev/null 2>&1; then
            echo "连接正常"
            return 0
        else
            echo "4G接口不存在"
            return 1
        fi
    else
        echo "网络无法通信"
        return 2
    fi
}

# 建立4G连接
connect_4g() {
    echo "尝试建立4G连接..."
    
    # 首先检查并杀死可能存在的pppd进程
    pkill -f pppd
    
    # 等待一段时间让系统清理
    sleep 3
    
    # 建立新的ppp连接
    if pon > /dev/null 2>&1; then
        echo "4G连接命令已发送，等待连接建立..."
        
        # 等待连接建立
        local timeout=30
        while [ $timeout -gt 0 ]; do
            if check_4g_connection; then
                echo "4G连接成功建立"
                return 0
            fi
            sleep 2
            timeout=$((timeout-2))
        done
        
        echo "4G连接超时"
        return 1
    else
        echo "无法启动4G连接"
        return 1
    fi
}

# 断开4G连接
disconnect_4g() {
    echo "断开4G连接..."
    poff > /dev/null 2>&1
    pkill -f pppd
    sleep 3
}

# 重启4G连接
restart_4g() {
    echo "重启4G连接..."
    disconnect_4g
    connect_4g
}

# 等待网络连接
wait_for_network() {
    echo "等待网络连接..."
    local max_attempts=5000000
    local attempts=0
    
    while [ $attempts -lt $max_attempts ]; do
        if check_4g_connection; then
            echo "4G网络已连接"
            return 0
        else
            echo "尝试重新连接4G ($((attempts+1))/$max_attempts)..."
            restart_4g
            attempts=$((attempts+1))
            sleep 5
        fi
    done
    
    echo "无法建立4G连接，最大重试次数已达"
    return 1
}

# 主循环
while true; do
    if wait_for_network; then
        echo "启动主程序..."
        # 这里启动您的程序
        source /home/orangepi/demo01/devel/setup.bash && roslaunch motor_can run.launch
        
        echo "主程序退出, 10秒后重启..."
        sleep 10
    else
        echo "网络连接失败，30秒后重试..."
        sleep 30
    fi
done


# sleep 60
# source /home/orangepi/demo01/devel/setup.bash && roslaunch motor_can run.launch
