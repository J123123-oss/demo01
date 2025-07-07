#!/bin/bash
#开启CAN转USB
sudo chmod 777 /dev/ttyACM3
sudo slcand -o -c -s8 /dev/ttyACM3 can0
sudo ip link set can0 type can bitrate 1000000
sudo ifconfig can0 up

sudo chmod 777 /dev/ttyUSB0
# sudo chmod 777 /dev/ttyACM3

#香橙派设置
# cd /home/orangepi/demo01 
#虚拟机设置
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
# roscore &

#进入工作空间启动launch
cd /home/ubuntu/demo01
source devel/setup.bash
roslaunch motor_can run.launch &



#结束后需要关闭CAN
# sudo ifconfig can0 down