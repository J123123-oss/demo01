#!/bin/bash

# [ -e /dev/ttyACM0 ] && sudo chmod 666 /dev/IMU
# [ -e /dev/ttyACM1] && sudo chmod 666 /dev/CAN0


# [ -e /dev/ttyUSB1 ] && sudo chmod 777 /dev/ProximitySensor
# [ -e /dev/ttyACM1 ] && sudo chmod 777 /dev/SENSOR0


sudo slcand -o -c -s8 /dev/CAN0 can0


sudo ifconfig can0 up

# 检查can0状态
can_status=$(ip -details link show can0 2>/dev/null)
if [[ -z "$can_status" ]]; then
    echo "can0接口未正常启动，正在重试..."
    exec "$0"
else
    echo "can0接口已正常启动"
fi


