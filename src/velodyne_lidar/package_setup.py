#!bin/bash

import os

# Installing ROS dependencies for Velodyne Drivers
os.system('echo "\e[5m \e[91m*You might need to enter the password \
for sudo operations*\e[39m \e[25m" ')
os.system('sudo apt-get install  ros-humble-velodyne -y')
os.system('while true;')
os.system('printf "\r< Loading..."')
os.system('sleep 0.75')
os.system('printf "\r> Loading..."')
os.system('sleep 0.75')
os.system('echo "\n\e[7mDone installing the necessary drivers for Velodyne\e[27m\n" ') 
os.system('echo "-----------------------------------------\n" ')

# Installing rviz2 for ros2


