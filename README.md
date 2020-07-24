# TortoiseBot

This repository contains a simple Gazebo robot with differential drive plugin which has a Ouster OS1-64 LiDAR mounted on it. 

## Dependencies & Setup

This was tested in Ubuntu 18.04, ROS Melodic and Gazebo 9.0.0.

* `hector_gazebo` (for simulating Ouster IMU)
* `ouster_example` (wilselby fork)
* `teleop_twist_keyboard` (apt install)

## Folder Structures

```
├── launch
│   └── tortoisebot.launch      (launch file & publish frequency)
├── rviz
│   └── tortoisebot.rviz        (tells rviz to point cloud)
├── urdf
│   └── tortoisebot.urdf.xacro  (defines robot & LiDAR hz, horizontal res)
├── CMakeLists.txt
├── package.xml
└── README.md
```


## References

* [Simulating an Ouster OS1 LiDAR Sensor in ROS Gazebo and RViz](https://www.wilselby.com/2019/05/simulating-an-ouster-os-1-lidar-sensor-in-ros-gazebo-and-rviz/) by Wil Selby
* Morgan Quigley, Brian Gerkey, William D. Smart - Programming Robots with ROS - A Practical Introduction to the Robot Operating System-O'Reilly Media (2015)

## Todo
- [ ] Create appropriate fork of `ouster_example`
- [ ] Add instructions for setting up the robot & LiDAR from scratch