#!/bin/bash
export LC_ALL=zh_CN.UTF-8
export LANG=zh_CN.UTF-8
export LANGUAGE=zh_CN.UTF-8
# 配置参数
INTERFACE="ppp0"          # 要检测的接口
CHECK_INTERVAL=300         # 检测间隔（秒）
TIMEOUT=1800                # 超时时间（秒）- 连续检测失败达到此时长则重启

# 初始化变量
fail_count=0
max_fail=$((TIMEOUT / CHECK_INTERVAL))

echo "开始监控 $INTERFACE 接口状态，超时时间: $TIMEOUT 秒"

while true; do
    # 检查接口是否存在且有IP地址
    # if ip addr show "$INTERFACE" | grep -q "inet "; then
    if ip addr show "$INTERFACE" ; then
        echo "$(date +'%Y-%m-%d %H:%M:%S') - $INTERFACE 接口正常" >> /home/orangepi/demo01/ppp0_log.log
        fail_count=0  # 重置失败计数
    else
        echo "$(date +'%Y-%m-%d %H:%M:%S') - $INTERFACE 接口异常" >> /home/orangepi/demo01/ppp0_log.log
        ((fail_count++))
        
        # 检查是否达到超时阈值
        if [ $fail_count -ge $max_fail ]; then
            # 记录重启日志
            echo "$(date +'%Y-%m-%d %H:%M:%S') - $INTERFACE 已掉线超过 $TIMEOUT 秒，执行重启..."
            sleep 1
            reboot
        else
            echo "$(date +'%Y-%m-%d %H:%M:%S') - 尝试重新拨号..." >> /home/orangepi/demo01/ppp0_log.log
            pkill -f pppd > /dev/null 2>&1
            pkill -f wvdial > /dev/null 2>&1
            sleep 3
            sudo wvdial >> /home/orangepi/demo01/ppp0_log.log 2>&1
            sleep 30
            if ip addr show "$INTERFACE" | grep -q "inet "; then
                echo "$(date +'%Y-%m-%d %H:%M:%S') - 重新拨号成功！" >> /home/orangepi/demo01/ppp0_log.log
                fail_count=0  # 重置失败计数
            else
                echo "$(date +'%Y-%m-%d %H:%M:%S') - 重新拨号失败！" >> /home/orangepi/demo01/ppp0_log.log
            fi
        fi
    fi
    
    sleep $CHECK_INTERVAL
done
