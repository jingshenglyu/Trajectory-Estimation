

<!--
 * @Author       : Jingsheng Lyu
 * @Date         : 2021-07-03 18:34:33
 * @LastEditors  : Jingsheng Lyu
 * @LastEditTime : 2021-09-25 18:35:06
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

    4. EKF 
            ```
            sudo apt-get install ros-melodic-robot-pose-ekf
            ```


## Implementation for simulation 

0. Download the project to your computer
```
git clone https://github.com/jingshenglyu/Trajectory-Estimation.git
cd catkin_ws
```

* Attention: If you don't set up the enviroment variable of ROS in your bash file, please don't forget do 'soure devel/setup.bash'

1. Launching the simulation environment
```
roslaunch mbot_gazebo TE_maze_gazebo.launch 
```

2. Collect the data with sensors by Rviz
```
roslaunch mbot_vision TE_find_target_pro.launch
```

3. Run the simulation
```
roslaunch mbot_navigation exploring_slam_demo.launch
```

4. If you want to control the robot by your self, please use the remote control on your keyboard

    1. Run the remote control
    ```
    roslauch mbot_teleop mbot_teleop.launch
    ```
    2. Remote Control
        * Control mbot!
        ---------------------------
        Moving around:
        u    i    o
        j    k    l
        m    ,    .

        q/z : increase/decrease max speeds by 10%
        w/x : increase/decrease only linear speed by 10%
        e/c : increase/decrease only angular speed by 10%
        space key, k : force stop
        anything else : stop smoothly

        CTRL-C to quit

## Camera Calibration

0. Preparation: Download the ROS stack for Camera Calibration
```
sudo apt-get install ros-melodic-camera-calibration
```

1. Run the camera
```
roslaunch robot_vision usb_cam.launch
```

2. Run the calibration program
```
rosrun camera_calibration cameracalibrator.py --ze 8x6 --square 0.024 image:=/usb_cam/image_raw camera:=/usb_cam
```

## Map Building

0. Preparation: Download the ROS stack for Grid Mapping
```
sudo apt-get install ros-melodic-gmapping
```

1. Launch the simulation environment
```
roslaunch mbot_gazebo TE_maze_gazebo.launch 
```

2. Map Building with Gmapping
```
roslaunch mbot_navigation gmapping_demo.launch 
```


4. Run the remote control - User build the map
```
roslauch mbot_teleop mbot_teleop.launch
```