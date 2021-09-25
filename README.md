

<!--
 * @Author       : Jingsheng Lyu
 * @Date         : 2021-07-03 18:34:33
 * @LastEditors  : Jingsheng Lyu
 * @LastEditTime : 2021-09-25 16:15:02
 * @FilePath     : /undefined/home/jingsheng/Trajectory-Estimation/README.md
 * @Github       : https://github.com/jingshenglyu
 * @Web          : https://jingshenglyu.github.io/
 * @E-Mail       : jingshenglyu@gmail.com
-->
# Trajectory-Estimation
* Master Thesis: Trajectory Estimation in Different Scenarioswith A Monolithic Senso
* Author: Jingsheng Lyu
* Supervisor: M.Sc. Philipp Kremer
* Examiners: Prof. Dr.-Ing. Sangyoung Park
* Faculty: Faculty V Fachgebiet Smart Mobility Systems at TU Berlin

This is the project about trajectory estimation for the master's thesis at TU Berlin. 

## Platforms

* Ubuntu 18.04 LTS
* ROS Melodic
* Rviz
* Gazebo

## Preparation

1. Sofortware Installation
    1. [ROS Melodic](http://wiki.ros.org/melodic/Installation) installation
    2. rviz installation
        * Please execute `rosrun rviz rviz` to confirm 
    3. Gazebo installation
        * Please execute `rosrun gazebo_ros gazebo` to confirm
2. Download the gazebo model library with the followed link and place it in `/.gazebo/models`
```
https://bitbucket.org/osrf/gazebo_models/downloads/
```

3. Install ROS Package
    1. Mapping
        * Cartographer 
            ```
            sudo apt-get install ros-melodic-cartographer-*
            ```
    2. SLAM
        * Navigation
            ```
            sudo apt-get install ros-melodic-navigation
            ```
    3. If you also need other functions, please try to such the tools by yourself.


## Implementation for simulation 

0. Download the project to your computer
```
git clone https://github.com/jingshenglyu/Trajectory-Estimation.git
cd catkin_ws
```

* Attention: If you don't set up the enviroment variable of ROS in your bash file, please don't forget do 'soure devel/setup.bash'

1. Launching the simulation environment
```
roslaunch mbot_gazebo c11_maze_gazebo.launch 
```

2. Collect the data with sensors by Rviz
```
roslaunch mbot_navigation exploring_slam_demo.launch
```

3. Run the simulation
```
roslaunch mbot_vision c11_find_target_pro.launch
```

