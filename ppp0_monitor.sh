#!/bin/bash

PPP_IFACE="ppp0"
PING_TARGET="121.40.57.48"
CHECK_INTERVAL=300
MAX_FAILURES=3

consecutive_failures=0

while true; do
    if ping -c 3 -W 2 $PING_TARGET > /dev/null 2>&1; then
        if [ $consecutive_failures -gt 0 ]; then
            echo "$(date): 连接恢复！"
            consecutive_failures=0
        fi
        sleep $CHECK_INTERVAL
    else
        consecutive_failures=$((consecutive_failures + 1))
        echo "$(date): 连接中断 ($consecutive_failures/$MAX_FAILURES)"
        
        if [ $consecutive_failures -ge $MAX_FAILURES ]; then
            # 无限重拨循环
            while true; do
                echo "$(date): 尝试重新拨号..."
                
                # 清理并重拨
                poff > /dev/null 2>&1
                pkill -f pppd > /dev/null 2>&1
                sleep 3
                # pon > /dev/null 2>&1
                
                # 等待30秒看是否连接成功
                timeout=30
                connected=false
                while [ $timeout -gt 0 ]; do
                    if ping -c 1 -W 2 $PING_TARGET > /dev/null 2>&1; then
                        echo "$(date): 重新拨号成功！跳出重拨循环"
                        connected=true
                        consecutive_failures=0
                        break 2  # 跳出两层循环
                    fi
                    sleep 1
                    timeout=$((timeout-1))
                done
                reboot
                echo "$(date): 拨号失败，10秒后再次尝试..."
                sleep 10
            done
        else
            sleep 5
        fi
    fi
done