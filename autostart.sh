#!/bin/bash

[ -e /dev/ttyACM1 ] && sudo chmod 777 /dev/IMU
# [ -e /dev/ttyUSB1 ] && sudo chmod 777 /dev/ProximitySensor
[ -e /dev/ttyACM0 ] && sudo chmod 777 /dev/CAN0
# [ -e /dev/ttyACM1 ] && sudo chmod 777 /dev/SENSOR0


sudo slcand -o -c -s8 /dev/CAN0 can0


sudo ifconfig can0 up


