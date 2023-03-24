#!/bin/bash
cd ~
cd Desktop/Zima/catkin_ws
catkin_make
source devel/setup.bash

sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1


roslaunch d_hospital_navigation bloco_c.launch &
#roslaunch d_hospital_avigation bloco_c_dysplay.launch &
rosrun d_hospital_navigation goal_from_app.py &
cd src/d_hospital_navigation/src/server_test/ && npm start &
