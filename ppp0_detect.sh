#!/bin/bash

# 配置参数
INTERFACE="ppp0"          # 要检测的接口
CHECK_INTERVAL=20         # 检测间隔（秒）
TIMEOUT=60                # 超时时间（秒）- 连续检测失败达到此时长则重启

# 初始化变量
fail_count=0
max_fail=$((TIMEOUT / CHECK_INTERVAL))

echo "开始监控 $INTERFACE 接口状态，超时时间: $TIMEOUT 秒"

while true; do
    # 检查接口是否存在且有IP地址
    if ip addr show "$INTERFACE" | grep -q "inet "; then
        echo "$(date +'%Y-%m-%d %H:%M:%S') - $INTERFACE 接口正常"
        fail_count=0  # 重置失败计数
    else
        echo "$(date +'%Y-%m-%d %H:%M:%S') - $INTERFACE 接口异常"
        ((fail_count++))
        
        # 检查是否达到超时阈值
        if [ $fail_count -ge $max_fail ]; then
            echo "$(date +'%Y-%m-%d %H:%M:%S') - $INTERFACE 已掉线超过 $TIMEOUT 秒，执行重启..."
            
            # 尝试重启接口（先关闭再启动）
            reboot
            # 记录重启日志
            echo "$(date +'%Y-%m-%d %H:%M:%S') - 已尝试重启 $INTERFACE" >> ./ppp0_monitor.log
            
            # 重置失败计数
            fail_count=0
        fi
    fi
    
    sleep $CHECK_INTERVAL
done
