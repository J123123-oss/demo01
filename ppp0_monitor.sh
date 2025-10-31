#!/bin/bash

PPP_IFACE="ppp0"
PING_TARGET="121.40.57.48"
CHECK_INTERVAL=300          # 正常连接时的检查间隔（秒）
MAX_FAILURES=3              # 网络不通的最大连续失败次数
INTERFACE_POLL_INTERVAL=5   # 接口状态轮询间隔（秒）
MAX_PPP_CREATION_FAILURES=5 # ppp0创建失败的最大次数（超过则重启）
PPP_CREATION_RETRY_DELAY=20 # 每次ppp0创建失败后的重试间隔（秒）

consecutive_failures=0
ppp_creation_failures=0     # 记录ppp0接口创建失败的次数

# 检查ppp0接口是否存在
check_ppp_interface() {
    ip link show "$PPP_IFACE" > /dev/null 2>&1
    return $?  # 0=存在，1=不存在
}

# 执行拨号操作（封装为函数，便于复用）
dial_up() {
    echo "$(date): 开始拨号..."
    # 清理残留进程
    poff > /dev/null 2>&1
    pkill -f pppd > /dev/null 2>&1
    sleep 3
    # 执行拨号（后台运行，不阻塞脚本）
    # sudo wvdial > /dev/null 2>&1 &
}

while true; do
    # 第一步：检测ppp0接口是否存在（优先于ping检测）
    if ! check_ppp_interface; then
        echo "$(date): ppp0接口掉线，尝试重新拨号..."
        sudo wvdial > /dev/null 2>&1 &
        
        # 等待接口重新创建（最多60秒，期间高频检测）
        wait_time=0
        while ! check_ppp_interface && [ $wait_time -lt 60 ]; do
            sleep 1
            wait_time=$((wait_time + 1))
        done
        
        if check_ppp_interface; then
            echo "$(date): ppp0接口已重建，开始检测网络连通性..."
            consecutive_failures=0          # 重置网络失败计数
            ppp_creation_failures=0         # 重置接口创建失败计数
        else
            # 接口创建失败，累加计数并检查是否达到重启阈值
            ppp_creation_failures=$((ppp_creation_failures + 1))
            echo "$(date): 拨号后ppp0仍未出现（第${ppp_creation_failures}/${MAX_PPP_CREATION_FAILURES}次失败）"
            
            # 达到最大创建失败次数，执行重启
            if [ $ppp_creation_failures -ge $MAX_PPP_CREATION_FAILURES ]; then
                echo "$(date): ppp0接口多次创建失败（超过${MAX_PPP_CREATION_FAILURES}次），执行重启..."
                reboot
                sleep 5  # 确保日志输出
                exit 0
            fi
            
            # 未达阈值，等待一段时间后重试
            echo "$(date): ${PPP_CREATION_RETRY_DELAY}秒后再次尝试创建ppp0..."
            sleep $PPP_CREATION_RETRY_DELAY
            consecutive_failures=$MAX_FAILURES  # 强制进入重拨循环
        fi
    fi

    # 第二步：仅当接口存在时，检测网络连通性
    if check_ppp_interface; then
        if ping -c 3 -W 2 $PING_TARGET > /dev/null 2>&1; then
            if [ $consecutive_failures -gt 0 ]; then
                echo "$(date): 网络连接恢复！"
                consecutive_failures=0
            fi
            # 正常连接时，轮询接口状态（避免中途掉线未检测）
            for ((i=0; i<CHECK_INTERVAL; i+=INTERFACE_POLL_INTERVAL)); do
                sleep $INTERFACE_POLL_INTERVAL
                if ! check_ppp_interface; then
                    echo "$(date): 等待期间检测到ppp0掉线！"
                    break
                fi
            done
        else
            consecutive_failures=$((consecutive_failures + 1))
            echo "$(date): 网络连接中断（${consecutive_failures}/${MAX_FAILURES}）"
            
            if [ $consecutive_failures -ge $MAX_FAILURES ]; then
                # 无限重拨循环（期间仍会检测接口创建失败次数）
                while true; do
                    echo "$(date): 尝试重新拨号..."
                    dial_up
                    
                    # 等待30秒检查是否成功（接口存在且网络通）
                    timeout=30
                    connected=false
                    while [ $timeout -gt 0 ]; do
                        if check_ppp_interface && ping -c 1 -W 2 $PING_TARGET > /dev/null 2>&1; then
                            echo "$(date): 重新拨号成功！跳出重拨循环"
                            consecutive_failures=0
                            ppp_creation_failures=0  # 重置创建失败计数
                            break 2  # 跳出两层循环
                        fi
                        sleep 1
                        timeout=$((timeout - 1))
                    done
                    
                    # 拨号失败，累加接口创建失败计数
                    ppp_creation_failures=$((ppp_creation_failures + 1))
                    echo "$(date): 拨号失败（第${ppp_creation_failures}/${MAX_PPP_CREATION_FAILURES}次接口创建失败）"
                    
                    # 达到最大失败次数，执行重启
                    if [ $ppp_creation_failures -ge $MAX_PPP_CREATION_FAILURES ]; then
                        echo "$(date): ppp0接口多次创建失败，执行重启..."
                        reboot
                        sleep 5
                        exit 0
                    fi
                    
                    echo "$(date): 10秒后再次尝试拨号..."
                    sleep 10
                done
            else
                sleep $INTERFACE_POLL_INTERVAL  # 短间隔重试
            fi
        fi
    fi
done