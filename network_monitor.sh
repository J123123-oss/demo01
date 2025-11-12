#!/bin/bash
export LC_ALL=zh_CN.UTF-8
export LANG=zh_CN.UTF-8
export LANGUAGE=zh_CN.UTF-8

# 日志配置
LOG_FILE="/home/orangepi/demo01/network_monitor.log"  # 日志文件路径
mkdir -p "$(dirname "$LOG_FILE")"  # 确保日志目录存在

# 网络参数配置
CHECK_INTERVAL=60  # 主检测间隔（秒）
TARGET_IP="121.40.57.48"
WIFI_GATEWAY="192.168.200.1"
WIFI_INTERFACE="wlan0"
CELLULAR_INTERFACE="ppp0"
CELL_RECONNECT_CHECK=1
# WIFI_SSID="磅房1"       # 例如 "MyHomeWiFi"
WIFI_SSID="GiiFen-5G"       # 例如 "MyHomeWiFi"
# WIFI_PASSWORD="88888888"   # 例如 "12345678"
WIFI_PASSWORD="giifen666666"   # 例如 "12345678"

# 启动日志
echo "$(date +'%Y-%m-%d %H:%M:%S') - 保守网络切换脚本启动（优先4G）..." >> "$LOG_FILE"
echo "$(date +'%Y-%m-%d %H:%M:%S') - 检测间隔: ${CHECK_INTERVAL}秒 | 目标IP: ${TARGET_IP}" >> "$LOG_FILE"
echo "$(date +'%Y-%m-%d %H:%M:%S') - 4G接口: ${CELLULAR_INTERFACE} | WiFi热点: ${WIFI_SSID}" >> "$LOG_FILE"

# 初始网络检测（优先4G）
sudo ip route del default 2>/dev/null
if ip link show ${CELLULAR_INTERFACE} >/dev/null 2>&1 && ping -I ${CELLULAR_INTERFACE} -c 2 -W 2 ${TARGET_IP} &>/dev/null; then
    sudo ip route add default dev ${CELLULAR_INTERFACE}
    CURRENT_NETWORK="cellular"
else
    echo "$(date +'%Y-%m-%d %H:%M:%S') - 4G初始连接失败，尝试WiFi..." >> "$LOG_FILE"
    sudo ip link set ${WIFI_INTERFACE} up 2>/dev/null
    nmcli device wifi connect "${WIFI_SSID}" password "${WIFI_PASSWORD}" ifname ${WIFI_INTERFACE} 2>/dev/null
    sleep 5
    if ip link show ${WIFI_INTERFACE} | grep -v "NO-CARRIER" && ping -I ${WIFI_INTERFACE} -c 2 -W 2 ${TARGET_IP} &>/dev/null; then
        sudo ip route add default via ${WIFI_GATEWAY} dev ${WIFI_INTERFACE}
        CURRENT_NETWORK="wifi"
    else
        CURRENT_NETWORK="none"
    fi
fi
echo "$(date +'%Y-%m-%d %H:%M:%S') - 初始网络状态: $CURRENT_NETWORK" >> "$LOG_FILE"

# 主循环检测
while true; do
    if [ "$CURRENT_NETWORK" = "cellular" ]; then
        echo "$(date +'%Y-%m-%d %H:%M:%S') - 检测4G网络可用性..." >> "$LOG_FILE"
        if ! ip link show ${CELLULAR_INTERFACE} >/dev/null 2>&1; then
            echo "$(date +'%Y-%m-%d %H:%M:%S') - 4G接口消失，尝试重新等待接口..." >> "$LOG_FILE"
            PPP_WAIT=0
            while ! ip link show ${CELLULAR_INTERFACE} >/dev/null 2>&1; do
                echo "$(date +'%Y-%m-%d %H:%M:%S') - 等待${CELLULAR_INTERFACE}接口（${PPP_WAIT}秒）..." >> "$LOG_FILE"
                sleep 2
                PPP_WAIT=$((PPP_WAIT + 2))
                [ $PPP_WAIT -ge 30 ] && { echo "$(date +'%Y-%m-%d %H:%M:%S') - ${CELLULAR_INTERFACE}未就绪，准备切换WiFi" >> "$LOG_FILE"; break; }
            done
        fi
        sudo ip route add ${TARGET_IP} dev ${CELLULAR_INTERFACE} 2>/dev/null
        
        if ping -I ${CELLULAR_INTERFACE} -c 4 -W 3 ${TARGET_IP} &>/dev/null; then
            echo "$(date +'%Y-%m-%d %H:%M:%S') - 4G网络正常，保持连接" >> "$LOG_FILE"
        else
            echo "$(date +'%Y-%m-%d %H:%M:%S') - 4G不可用，切换到WiFi..." >> "$LOG_FILE"
            sudo ip link set ${WIFI_INTERFACE} up 2>/dev/null
            sleep 2
        fi
        if ip link show ${WIFI_INTERFACE} | grep -q "NO-CARRIER"; then
            echo "$(date +'%Y-%m-%d %H:%M:%S') - WiFi接口未关联热点，自动连接 ${WIFI_SSID}..." >> "$LOG_FILE"
            nmcli device wifi connect "${WIFI_SSID}" password "${WIFI_PASSWORD}" ifname ${WIFI_INTERFACE} 2>/dev/null
            sleep 5
        fi
        if ip link show ${WIFI_INTERFACE} | grep -v "NO-CARRIER"; then
            sudo ip route add ${TARGET_IP} via ${WIFI_GATEWAY} dev ${WIFI_INTERFACE} 2>/dev/null
            if ping -I ${WIFI_INTERFACE} -c 4 -W 3 ${TARGET_IP} &>/dev/null; then
                echo "$(date +'%Y-%m-%d %H:%M:%S') - WiFi已恢复，切换回WiFi..." >> "$LOG_FILE"
                sudo ip route del default 2>/dev/null
                sudo ip route add default via ${WIFI_GATEWAY} dev ${WIFI_INTERFACE}
                CURRENT_NETWORK="wifi"
                echo "$(date +'%Y-%m-%d %H:%M:%S') - 已切换到WiFi网络" >> "$LOG_FILE"
            else
                echo "$(date +'%Y-%m-%d %H:%M:%S') - WiFi也不可用，维持4G状态重试" >> "$LOG_FILE"
            fi
        fi
        sudo ip route del ${TARGET_IP} dev ${CELLULAR_INTERFACE} 2>/dev/null

    elif [ "$CURRENT_NETWORK" = "wifi" ] && [ $CELL_RECONNECT_CHECK -eq 1 ]; then
        echo "$(date +'%Y-%m-%d %H:%M:%S') - 当前为WiFi状态，检测4G是否恢复..." >> "$LOG_FILE"
        if ip link show ${CELLULAR_INTERFACE} >/dev/null 2>&1; then
            echo "$(date +'%Y-%m-%d %H:%M:%S') - 4G接口已就绪，检测连通性..." >> "$LOG_FILE"
            sudo ip route add ${TARGET_IP} dev ${CELLULAR_INTERFACE} 2>/dev/null
            if ping -I ${CELLULAR_INTERFACE} -c 4 -W 3 ${TARGET_IP} &>/dev/null; then
                echo "$(date +'%Y-%m-%d %H:%M:%S') - 4G已恢复，切换回4G..." >> "$LOG_FILE"
                sudo ip route del default 2>/dev/null
                sudo ip route add default dev ${CELLULAR_INTERFACE}
                CURRENT_NETWORK="cellular"
                echo "$(date +'%Y-%m-%d %H:%M:%S') - 已切换到4G网络" >> "$LOG_FILE"
            else
                echo "$(date +'%Y-%m-%d %H:%M:%S') - 4G接口存在但无法连通，继续使用WiFi" >> "$LOG_FILE"
                sudo ip route del ${TARGET_IP} dev ${CELLULAR_INTERFACE} 2>/dev/null
            fi
        else
            echo "$(date +'%Y-%m-%d %H:%M:%S') - 4G接口未就绪，继续使用WiFi" >> "$LOG_FILE"
        fi

    else
        echo "$(date +'%Y-%m-%d %H:%M:%S') - 所有网络均不可用，尝试重新连接4G..." >> "$LOG_FILE"
        if ip link show ${CELLULAR_INTERFACE} >/dev/null 2>&1 && ping -I ${CELLULAR_INTERFACE} -c 2 -W 2 ${TARGET_IP} &>/dev/null; then
            sudo ip route add default dev ${CELLULAR_INTERFACE}
            CURRENT_NETWORK="cellular"
            echo "$(date +'%Y-%m-%d %H:%M:%S') - 4G重新连接成功" >> "$LOG_FILE"
        else
            echo "$(date +'%Y-%m-%d %H:%M:%S') - 4G重试失败，尝试WiFi..." >> "$LOG_FILE"
            nmcli device wifi connect "${WIFI_SSID}" password "${WIFI_PASSWORD}" ifname ${WIFI_INTERFACE} 2>/dev/null
            sleep 5
            if ip link show ${WIFI_INTERFACE} | grep -v "NO-CARRIER" && ping -I ${WIFI_INTERFACE} -c 2 -W 2 ${TARGET_IP} &>/dev/null; then
                sudo ip route add default via ${WIFI_GATEWAY} dev ${WIFI_INTERFACE}
                CURRENT_NETWORK="wifi"
                echo "$(date +'%Y-%m-%d %H:%M:%S') - WiFi重新连接成功" >> "$LOG_FILE"
            fi
        fi
    fi
    
    echo "$(date +'%Y-%m-%d %H:%M:%S') - 当前网络: $CURRENT_NETWORK" >> "$LOG_FILE"
    echo "$(date +'%Y-%m-%d %H:%M:%S') - ---" >> "$LOG_FILE"
    sleep $CHECK_INTERVAL
done