#!/bin/bash
# conservative_network.sh

CHECK_INTERVAL=600  # 更长的检测间隔
TARGET_IP="121.40.57.48"

echo "保守网络切换脚本启动..."
echo "检测间隔: ${CHECK_INTERVAL}秒"

# 初始设置为4G（因为WiFi没网）
sudo ip route del default 2>/dev/null
sudo ip route add default dev ppp0
CURRENT_NETWORK="cellular"

echo "初始设置为4G网络"

while true; do
    # 检查WiFi是否真的可用
    if ip link show wlan0 | grep -q "state UP"; then
        echo "检测WiFi网络..."
        # 给WiFi更严格的测试
        if ping -I wlan0 -c 4 -W 3 $TARGET_IP &>/dev/null; then
            # WiFi真正可用，才切换
            if [ "$CURRENT_NETWORK" != "wifi" ]; then
                echo "WiFi网络可用，切换到WiFi"
                sudo ip route del default 2>/dev/null
                sudo ip route add default via 192.168.200.1 dev wlan0
                CURRENT_NETWORK="wifi"
            fi
        else
            # WiFi不可用，确保使用4G
            if [ "$CURRENT_NETWORK" != "cellular" ]; then
                echo "WiFi不可用，切换回4G"
                sudo ip route del default 2>/dev/null
                sudo ip route add default dev ppp0
                CURRENT_NETWORK="cellular"
            fi
        fi
    else
        # 没有WiFi连接，确保使用4G
        if [ "$CURRENT_NETWORK" != "cellular" ]; then
            echo "WiFi未连接，使用4G"
            sudo ip route del default 2>/dev/null
            sudo ip route add default dev ppp0
            CURRENT_NETWORK="cellular"
        fi
    fi
    
    echo "当前网络: $CURRENT_NETWORK"
    echo "---"
    sleep $CHECK_INTERVAL
done