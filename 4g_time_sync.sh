#!/bin/bash
SERIAL_DEV="/dev/ttyUSB3"
BAUD_RATE="115200"
TIMEOUT=5

# 检查串口设备
if [ ! -c "$SERIAL_DEV" ]; then
    echo "错误：串口设备 $SERIAL_DEV 不存在！"
    exit 1
fi

# 清空历史日志
> /tmp/4g_time.log

# 发送AT指令并读取响应
screen -L -Logfile /tmp/4g_time.log -d -m -S 4g_time $SERIAL_DEV $BAUD_RATE \
&& sleep 1 \
&& screen -S 4g_time -X stuff "AT+QLTS=2$(printf '\r')" \
&& sleep $TIMEOUT \
&& screen -S 4g_time -X quit

# 提取最新响应
RAW_RESPONSE=$(grep "+QLTS:" /tmp/4g_time.log | tail -1)

# 核心修改：彻底替换所有"/"为"-"，并替换","为空格
# 步骤1：提取引号内内容 → 2025/10/13,20:56:31+32,0
# 步骤2：移除时区及后缀 → 2025/10/13,20:56:31
# 步骤3：将所有"/"替换为"-"，","替换为空格 → 2025-10-13 20:56:31
TIME_STR=$(echo "$RAW_RESPONSE" | awk -F'"' '{print $2}' | cut -d'+' -f1 | sed 's/\//-/g; s/,/ /')

# 验证时间格式
if ! date -d "$TIME_STR" > /dev/null 2>&1; then
    echo "错误：获取的时间格式无效！原始响应：$RAW_RESPONSE"
    echo "解析后的时间：$TIME_STR"
    exit 1
fi

# 校准系统时间
echo "正在校准系统时间：$TIME_STR"
sudo date -s "$TIME_STR"
sudo hwclock -w

echo "时间校准完成！当前系统时间：$(date)"