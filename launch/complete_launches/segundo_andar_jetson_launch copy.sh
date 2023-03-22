#! /home/Desktop/Zima/catkin_ws/
catkin_make
source devel/setup.bash

sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1


roslaunch d_hospital_navigation segundo_andar.launch &
roslaunch d_hospital_avigation segundo_andar_dysplay.launch &
cd src/d_hospital_navigation/src/server_test/ && npm start &
