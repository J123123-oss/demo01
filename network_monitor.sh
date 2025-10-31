#!/bin/bash
# conservative_network.sh

CHECK_INTERVAL=600  # 主检测间隔（秒）
TARGET_IP="121.40.57.48"
WIFI_GATEWAY="192.168.200.1"  # WiFi网关
WIFI_INTERFACE="wlan0"        # WiFi接口
CELLULAR_INTERFACE="ppp0"     # 4G接口
WIFI_RECONNECT_CHECK=1        # 4G状态下检测WiFi恢复（1=开启）
# 新增：WiFi热点信息（替换为你的WiFi名称和密码）
# WIFI_SSID="磅房1"       # 例如 "MyHomeWiFi"
WIFI_SSID="GiiFen-5G"       # 例如 "MyHomeWiFi"
# WIFI_PASSWORD="88888888"   # 例如 "12345678"
WIFI_PASSWORD="giifen666666"   # 例如 "12345678"

echo "保守网络切换脚本启动..."
echo "检测间隔: ${CHECK_INTERVAL}秒 | 目标IP: ${TARGET_IP}"
echo "自动连接WiFi热点: ${WIFI_SSID}"

# 初始优先尝试WiFi（含自动连接）
sudo ip route del default 2>/dev/null
# 先确保WiFi接口启动并尝试连接
sudo ip link set ${WIFI_INTERFACE} up 2>/dev/null
nmcli device wifi connect "${WIFI_SSID}" password "${WIFI_PASSWORD}" ifname ${WIFI_INTERFACE} 2>/dev/null
# 检测是否连接成功
if ip link show ${WIFI_INTERFACE} | grep -v "NO-CARRIER" && ping -I ${WIFI_INTERFACE} -c 2 -W 2 ${TARGET_IP} &>/dev/null; then
    sudo ip route add default via ${WIFI_GATEWAY} dev ${WIFI_INTERFACE}
    CURRENT_NETWORK="wifi"
else
    sudo ip route add default dev ${CELLULAR_INTERFACE}
    CURRENT_NETWORK="cellular"
fi
echo "初始网络状态: $CURRENT_NETWORK"

while true; do
    # 情况1：当前是WiFi状态——检测WiFi是否可用
    if [ "$CURRENT_NETWORK" = "wifi" ]; then
        echo "检测WiFi网络可用性..."
        # 检查是否有载波（是否关联热点）
        if ip link show ${WIFI_INTERFACE} | grep -q "NO-CARRIER"; then
            echo "WiFi接口无载波（未关联热点），尝试重新连接..."
            nmcli device wifi connect "${WIFI_SSID}" password "${WIFI_PASSWORD}" ifname ${WIFI_INTERFACE} 2>/dev/null
            sleep 5  # 等待连接生效
        fi
        # 临时路由检测连通性
        sudo ip route add ${TARGET_IP} via ${WIFI_GATEWAY} dev ${WIFI_INTERFACE} 2>/dev/null
        
        if ping -I ${WIFI_INTERFACE} -c 4 -W 3 ${TARGET_IP} &>/dev/null; then
            echo "WiFi网络正常，保持连接"
        else
            echo "WiFi不可用，切换到4G..."
            # 等待4G接口就绪
            PPP_WAIT=0
            while ! ip link show ${CELLULAR_INTERFACE} >/dev/null 2>&1; do
                echo "等待${CELLULAR_INTERFACE}接口（${PPP_WAIT}秒）..."
                sleep 2
                PPP_WAIT=$((PPP_WAIT + 2))
                [ $PPP_WAIT -ge 30 ] && { echo "${CELLULAR_INTERFACE}未就绪，放弃切换"; break; }
            done
            # 切换4G
            if ip link show ${CELLULAR_INTERFACE} >/dev/null 2>&1; then
                sudo ip route del default 2>/dev/null
                sudo ip route add default dev ${CELLULAR_INTERFACE}
                CURRENT_NETWORK="cellular"
                echo "已切换到4G网络"
            fi
        fi
        # 清理临时路由
        sudo ip route del ${TARGET_IP} via ${WIFI_GATEWAY} dev ${WIFI_INTERFACE} 2>/dev/null

    # 情况2：当前是4G状态——检测WiFi是否恢复（含自动连接）
    elif [ "$CURRENT_NETWORK" = "cellular" ] && [ $WIFI_RECONNECT_CHECK -eq 1 ]; then
        echo "当前为4G状态，检测WiFi是否恢复..."
        # 步骤1：确保WiFi接口启动
        if ! ip link show ${WIFI_INTERFACE} | grep -q "state UP"; then
            echo "WiFi接口未启动，尝试启动..."
            sudo ip link set ${WIFI_INTERFACE} up 2>/dev/null
            sleep 2
        fi
        # 步骤2：检查是否关联热点（无载波则连接）
        if ip link show ${WIFI_INTERFACE} | grep -q "NO-CARRIER"; then
            echo "WiFi接口未关联热点，自动连接 ${WIFI_SSID}..."
            nmcli device wifi connect "${WIFI_SSID}" password "${WIFI_PASSWORD}" ifname ${WIFI_INTERFACE} 2>/dev/null
            sleep 5  # 等待连接生效
        fi
        # 步骤3：检测WiFi连通性
        if ip link show ${WIFI_INTERFACE} | grep -v "NO-CARRIER"; then
            sudo ip route add ${TARGET_IP} via ${WIFI_GATEWAY} dev ${WIFI_INTERFACE} 2>/dev/null
            if ping -I ${WIFI_INTERFACE} -c 4 -W 3 ${TARGET_IP} &>/dev/null; then
                echo "WiFi已恢复，切换回WiFi..."
                sudo ip route del default 2>/dev/null
                sudo ip route add default via ${WIFI_GATEWAY} dev ${WIFI_INTERFACE}
                CURRENT_NETWORK="wifi"
                echo "已切换到WiFi网络"
            else
                echo "WiFi接口已关联，但无法连通公网，继续使用4G"
            fi
            # 清理临时路由
            sudo ip route del ${TARGET_IP} via ${WIFI_GATEWAY} dev ${WIFI_INTERFACE} 2>/dev/null
        else
            echo "WiFi接口未关联热点，继续使用4G"
        fi
    fi
    
    echo "当前网络: $CURRENT_NETWORK"
    echo "---"
    sleep $CHECK_INTERVAL
done