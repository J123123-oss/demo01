#!/bin/bash

[ -e /dev/ttyUSB0 ] && sudo chmod 777 /dev/ttyUSB0
[ -e /dev/ttyACM0 ] && sudo chmod 777 /dev/CAN0
[ -e /dev/ttyACM1 ] && sudo chmod 777 /dev/SENSOR0


<<<<<<< HEAD
sudo slcand -o -c -s8 /dev/ttyACM0 can0
=======
sudo slcand -o -c -s8 /dev/CAN0 can0
>>>>>>> fbe820bd8f01db9f73a0a09e671f86a56f59ea46


sudo ifconfig can0 up


