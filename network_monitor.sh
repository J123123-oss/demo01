#!/bin/bash
# conservative_network.sh（优先4G版本）

CHECK_INTERVAL=600  # 主检测间隔（秒）
TARGET_IP="121.40.57.48"
WIFI_GATEWAY="192.168.200.1"  # WiFi网关
WIFI_INTERFACE="wlan0"        # WiFi接口
CELLULAR_INTERFACE="ppp0"     # 4G接口
CELL_RECONNECT_CHECK=1        # WiFi状态下检测4G恢复（1=开启）  # 新增：4G恢复检测开关
# WiFi热点信息
WIFI_SSID="磅房1"
WIFI_PASSWORD="88888888"

echo "保守网络切换脚本启动（优先4G）..."
echo "检测间隔: ${CHECK_INTERVAL}秒 | 目标IP: ${TARGET_IP}"
echo "4G接口: ${CELLULAR_INTERFACE} | WiFi热点: ${WIFI_SSID}"

# 初始优先尝试4G（替换原优先WiFi逻辑）
sudo ip route del default 2>/dev/null
# 先检查4G接口是否就绪并尝试拨号（假设已配置自动拨号，若未配置需添加拨号命令）
if ip link show ${CELLULAR_INTERFACE} >/dev/null 2>&1 && ping -I ${CELLULAR_INTERFACE} -c 2 -W 2 ${TARGET_IP} &>/dev/null; then
    sudo ip route add default dev ${CELLULAR_INTERFACE}
    CURRENT_NETWORK="cellular"
else
    # 4G不可用时，尝试WiFi
    echo "4G初始连接失败，尝试WiFi..."
    sudo ip link set ${WIFI_INTERFACE} up 2>/dev/null
    nmcli device wifi connect "${WIFI_SSID}" password "${WIFI_PASSWORD}" ifname ${WIFI_INTERFACE} 2>/dev/null
    sleep 5
    if ip link show ${WIFI_INTERFACE} | grep -v "NO-CARRIER" && ping -I ${WIFI_INTERFACE} -c 2 -W 2 ${TARGET_IP} &>/dev/null; then
        sudo ip route add default via ${WIFI_GATEWAY} dev ${WIFI_INTERFACE}
        CURRENT_NETWORK="wifi"
    else
        CURRENT_NETWORK="none"  # 均不可用
    fi
fi
echo "初始网络状态: $CURRENT_NETWORK"

while true; do
    # 情况1：当前是4G状态——检测4G是否可用（原WiFi检测逻辑反转）
    if [ "$CURRENT_NETWORK" = "cellular" ]; then
        echo "检测4G网络可用性..."
        # 检查4G接口是否存在
        if ! ip link show ${CELLULAR_INTERFACE} >/dev/null 2>&1; then
            echo "4G接口消失，尝试重新等待接口..."
            PPP_WAIT=0
            while ! ip link show ${CELLULAR_INTERFACE} >/dev/null 2>&1; do
                echo "等待${CELLULAR_INTERFACE}接口（${PPP_WAIT}秒）..."
                sleep 2
                PPP_WAIT=$((PPP_WAIT + 2))
                [ $PPP_WAIT -ge 30 ] && { echo "${CELLULAR_INTERFACE}未就绪，准备切换WiFi"; break; }
            done
        fi
        # 临时路由检测4G连通性
        sudo ip route add ${TARGET_IP} dev ${CELLULAR_INTERFACE} 2>/dev/null  # 4G无需网关，直接走接口
        
        if ping -I ${CELLULAR_INTERFACE} -c 4 -W 3 ${TARGET_IP} &>/dev/null; then
            echo "4G网络正常，保持连接"
        else
            echo "4G不可用，切换到WiFi..."
            # 尝试连接WiFi
            sudo ip link set ${WIFI_INTERFACE} up 2>/dev/null
            nmcli device wifi connect "${WIFI_SSID}" password "${WIFI_PASSWORD}" ifname ${WIFI_INTERFACE} 2>/dev/null
            sleep 5
            # 检测WiFi是否可用
            if ip link show ${WIFI_INTERFACE} | grep -v "NO-CARRIER" && ping -I ${WIFI_INTERFACE} -c 2 -W 2 ${TARGET_IP} &>/dev/null; then
                sudo ip route del default 2>/dev/null
                sudo ip route add default via ${WIFI_GATEWAY} dev ${WIFI_INTERFACE}
                CURRENT_NETWORK="wifi"
                echo "已切换到WiFi网络"
            else
                echo "WiFi也不可用，维持4G状态重试"
            fi
        fi
        # 清理4G临时路由
        sudo ip route del ${TARGET_IP} dev ${CELLULAR_INTERFACE} 2>/dev/null

    # 情况2：当前是WiFi状态——检测4G是否恢复（原逻辑反转，优先切回4G）
    elif [ "$CURRENT_NETWORK" = "wifi" ] && [ $CELL_RECONNECT_CHECK -eq 1 ]; then
        echo "当前为WiFi状态，检测4G是否恢复..."
        # 步骤1：检查4G接口是否就绪
        if ip link show ${CELLULAR_INTERFACE} >/dev/null 2>&1; then
            echo "4G接口已就绪，检测连通性..."
            sudo ip route add ${TARGET_IP} dev ${CELLULAR_INTERFACE} 2>/dev/null
            if ping -I ${CELLULAR_INTERFACE} -c 4 -W 3 ${TARGET_IP} &>/dev/null; then
                echo "4G已恢复，切换回4G..."
                sudo ip route del default 2>/dev/null
                sudo ip route add default dev ${CELLULAR_INTERFACE}
                CURRENT_NETWORK="cellular"
                echo "已切换到4G网络"
            else
                echo "4G接口存在但无法连通，继续使用WiFi"
                sudo ip route del ${TARGET_IP} dev ${CELLULAR_INTERFACE} 2>/dev/null
            fi
        else
            echo "4G接口未就绪，继续使用WiFi"
        fi

    # 情况3：均不可用状态
    else
        echo "所有网络均不可用，尝试重新连接4G..."
        # 优先重试4G
        if ip link show ${CELLULAR_INTERFACE} >/dev/null 2>&1 && ping -I ${CELLULAR_INTERFACE} -c 2 -W 2 ${TARGET_IP} &>/dev/null; then
            sudo ip route add default dev ${CELLULAR_INTERFACE}
            CURRENT_NETWORK="cellular"
            echo "4G重新连接成功"
        else
            # 4G失败再试WiFi
            echo "4G重试失败，尝试WiFi..."
            nmcli device wifi connect "${WIFI_SSID}" password "${WIFI_PASSWORD}" ifname ${WIFI_INTERFACE} 2>/dev/null
            sleep 5
            if ip link show ${WIFI_INTERFACE} | grep -v "NO-CARRIER" && ping -I ${WIFI_INTERFACE} -c 2 -W 2 ${TARGET_IP} &>/dev/null; then
                sudo ip route add default via ${WIFI_GATEWAY} dev ${WIFI_INTERFACE}
                CURRENT_NETWORK="wifi"
                echo "WiFi重新连接成功"
            fi
        fi
    fi
    
    echo "当前网络: $CURRENT_NETWORK"
    echo "---"
    sleep $CHECK_INTERVAL
done