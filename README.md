# d_hospital_navigation
> algoritmos de controle do robô hospitalar

## 💥 How to Run Simulation

```
roslaunch d_hospital_gazebo spawn_in_hospital.launch
```

```
roslaunch d_hospital_gazebo display.launch
```

Then
```
roslaunch d_hospital_navigation d_hospital_willow_navigation.launch
```
Or
```
 rosrun joy joy_node
```
```
rosrun d_hospital_navigation joy_xbox_nav.py
```
Or
```
rosrun d_hospital_navigation keyboard_nav.py
```



## 📦 Requeriments

```
sudo apt-get install ros-melodic-slam-gmapping
```

```
sudo apt-get install ros-melodic-map-server
```

```
sudo apt-get install ros-melodic-move-base
```

```
sudo apt-get install ros-melodic-amcl
```

```
sudo apt-get install ros-melodic-dwa-local-planner
```

```
sudo apt-get install ros-melodic-joy
```
