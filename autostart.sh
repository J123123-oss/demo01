#!/bin/bash

[ -e /dev/ttyUSB0 ] && sudo chmod 777 /dev/ttyUSB0
[ -e /dev/ttyACM0 ] && sudo chmod 777 /dev/ttyACM0
[ -e /dev/ttyACM1 ] && sudo chmod 777 /dev/ttyACM1


sudo slcand -o -c -s8 /dev/ttyACM0 can0


sudo ifconfig can0 up


