#!/bin/bash
SERIAL_DEV="/dev/4G-time-sync"
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

echo "原始响应: $RAW_RESPONSE"

# 提取时间数据
TIME_DATA=$(echo "$RAW_RESPONSE" | grep -o '"[^"]*"')
TIME_DATA=${TIME_DATA//\"/}

echo "提取的时间数据: $TIME_DATA"

# 解析时间数据
IFS=',' read -r DATE TIME_PART DST <<< "$TIME_DATA"
IFS='+' read -r TIME TIMEZONE <<< "$TIME_PART"

echo "解析结果:"
echo "日期: $DATE"
echo "时间: $TIME" 
echo "时区: $TIMEZONE"
echo "夏令时标志: $DST"

# 组合标准时间格式
FORMATTED_TIME="${DATE//\//-} $TIME"

echo "格式化时间: $FORMATTED_TIME"

sudo date +"%Y%m%d %H:%M:%S" -s "$FORMATTED_TIME"
sudo hwclock -w

echo "时间校准完成！"
echo "当前系统时间: $(date)"
echo "当前UTC时间: $(date -u)"
echo "当前硬件时间: $(sudo hwclock -r)"