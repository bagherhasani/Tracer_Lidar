# ROS Packages for Tracer Mobile Base with LIDAR Integration

## Packages

* tracer_base: a ROS wrapper around tracer SDK to monitor and control the robot
* tracer_bringup: launch and configuration files to start ROS nodes
* tracer_msgs: tracer related message definitions
* tracer_description: URDF model and visualization launch files for Tracer with LIDAR
* tracer_gazebo_sim: Gazebo simulation files and worlds for Tracer robot

## LIDAR Integration

This repository contains integration of LIDAR sensors with the Tracer mobile base, allowing for:
- 3D visualization in RViz
- Sensor data processing
- Potential use in mapping and navigation tasks

### Running the Tracer with LIDAR Visualization

To launch the Tracer robot with LIDAR in Gazebo and visualize it in RViz:

```bash
$ roslaunch tracer_description tracer_lidar_visualize.launch
```

This launch file will:
1. Load the URDF model with LIDAR
2. Start Gazebo with a custom world
3. Spawn the Tracer robot in the simulation
4. Launch RViz with a navigation configuration
5. Start all necessary controllers and publishers

![Tracer Robot with LIDAR](images/Screenshot%20from%202025-04-17%2015-04-35.png)

![Tracer Robot with LIDAR - Another View](images/Screenshot%20from%202025-04-17%2015-04-56.png)

## Communication interface setup

Please refer to the [README](https://github.com/agilexrobotics/ugv_sdk#hardware-interface) of "ugv_sdk" package for setup of communication interfaces.

#### Note on CAN interface on Nvidia Jetson Platforms

Nvidia Jeston TX2/Xavier/XavierNX have CAN controller(s) integrated in the main SOC. If you're using a dev kit, you need to add a CAN transceiver for proper CAN communication. 

## Basic usage of the ROS package

1. Install dependent packages

    ```
    $ sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard
    $ sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
    $ sudo apt install ros-$ROS_DISTRO-ros-controllers
    ```
    
2. Clone the packages into your catkin workspace and compile

    (the following instructions assume your catkin workspace is at: ~/catkin_ws/src)

    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/agilexrobotics/ugv_sdk.git
    $ git clone https://github.com/agilexrobotics/tracer_ros.git
    $ cd ..
    $ catkin_make
    ```

3. Setup CAN-To-USB adapter
* Enable gs_usb kernel module(If you have already added this module, you do not need to add it)
    ```
    $ sudo modprobe gs_usb
    ```
* first time use tracer-ros package
    ```
    $rosrun tracer_bringup setup_can2usb.bash
    ```
* If not the first time use tracer-ros package(Run this command every time you turn on the power)
    ```
    $rosrun tracer_bringup bringup_can2usb.bash
    ```
4. Launch ROS nodes

* Start the base node for the real robot whith can

    ```
    $ roslaunch tracer_bringup tracer_robot_base.launch
    ```
* Start the keyboard tele-op node

    ```
    $ roslaunch tracer_bringup tracer_teleop_keyboard.launch
    ```

**SAFETY PRECAUSION**: 

Always have your remote controller ready to take over the control whenever necessary. 

