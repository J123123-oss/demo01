#!/bin/bash
SERIAL_DEV="/dev/4G-time-sync"
BAUD_RATE="115200"
TIMEOUT=5
PROCESS_DELAY=7  # 处理延迟补偿秒数
ERROR_THRESHOLD=10  # 时间误差阈值（秒）

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

# 检查原始响应是否为空
if [ -z "$RAW_RESPONSE" ]; then
    echo "错误：未获取到有效响应数据，终止同步操作"
    exit 1
fi

# 提取时间数据
TIME_DATA=$(echo "$RAW_RESPONSE" | grep -o '"[^"]*"')
TIME_DATA=${TIME_DATA//\"/}

echo "提取的时间数据: $TIME_DATA"

# 检查时间数据是否为空
if [ -z "$TIME_DATA" ]; then
    echo "错误：未提取到有效时间数据，终止同步操作"
    exit 1
fi

# 解析时间数据
IFS=',' read -r DATE TIME_PART DST <<< "$TIME_DATA"
IFS='+' read -r TIME TIMEZONE <<< "$TIME_PART"

echo "解析结果:"
echo "日期: $DATE"
echo "时间: $TIME" 
echo "时区: $TIMEZONE"
echo "夏令时标志: $DST"

# 组合标准时间格式并添加处理延迟补偿
FORMATTED_TIME="${DATE//\//-} $TIME"
echo "原始获取时间: $FORMATTED_TIME"

# 将获取的时间转换为时间戳并添加延迟补偿
REMOTE_TIMESTAMP=$(date -d "$FORMATTED_TIME" +%s)
if [ -z "$REMOTE_TIMESTAMP" ]; then
    echo "错误：无法解析获取的时间"
    exit 1
fi

# 计算补偿后的时间（加上处理延迟）
COMPENSATED_TIMESTAMP=$((REMOTE_TIMESTAMP + PROCESS_DELAY))
COMPENSATED_TIME=$(date -d @$COMPENSATED_TIMESTAMP +"%Y-%m-%d %H:%M:%S")
echo "补偿后时间（+${PROCESS_DELAY}秒）: $COMPENSATED_TIME"

# 获取当前系统时间戳
LOCAL_TIMESTAMP=$(date +%s)

# 计算时间误差
TIME_DIFF=$((REMOTE_TIMESTAMP - LOCAL_TIMESTAMP))
TIME_DIFF=${TIME_DIFF#-}  # 取绝对值

echo "本地时间与原始获取时间误差: $TIME_DIFF 秒"

# 判断是否需要同步时间（误差大于阈值）
if [ $TIME_DIFF -gt $ERROR_THRESHOLD ]; then
    echo "时间误差超过${ERROR_THRESHOLD}秒，执行同步..."
    sudo date +"%Y%m%d %H:%M:%S" -s "$COMPENSATED_TIME"
    sudo hwclock -w
    echo "时间校准完成！"
else
    echo "时间误差在${ERROR_THRESHOLD}秒以内，无需同步"
fi

echo "当前系统时间: $(date)"
echo "当前UTC时间: $(date -u)"
echo "当前硬件时间: $(sudo hwclock -r)"