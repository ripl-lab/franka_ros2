# ROS 2 integration for Franka Emika research robots

[![CI](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml)

This repository was forked from [franka_ros2](https://github.com/frankaemika/franka_ros2) at version **0.1.0** to ensure compatibility with the [Panda Powertool](https://uwaterloo.ca/robohub/profiles/panda-powertool) in RoboHub. The verified [libfranka](https://github.com/frankaemika/libfranka) version that works with this version of franka_ros2 is **0.9.2**.  

For more details about Franka robots in RoboHub, see the [uw_panda documentation](https://git.uwaterloo.ca/robohub/panda/uw_panda/-/wikis/home).  

## Modifications from the Original `franka_ros2`

Compared to the original `franka_ros2`, the file [`panda_arm.ros2_control.xacro`](franka_description/robots/panda_arm.ros2_control.xacro) has been **updated to support the position command interface** for simulation in Rviz2. Additionally, it has been modified to allow switching to the effort command interface for controlling the real robot.  

Currently, **position and effort command interfaces are mutually exclusive**, meaning the robot cannot be viewed in **both Rviz2 and the real world simultaneously**. A fix for this limitation may be implemented in a future version of the original `franka_ros2`.  

## Usage Instructions

### 1. Visualizing Motion in Rviz2

To visualize the robot's motion in Rviz2, use the following command:  

```bash
ros2 launch franka_bringup franka.launch.py robot_ip:=dont_care use_fake_hardware:=true fake_sensor_commands:=true use_rviz:=true
```

### 2. Sending Commands to the Real Robot
  
To send commands to the real robot, use the following command:  

```bash
ros2 launch franka_bringup franka.launch.py robot_ip:=franka2
```

See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.

## License

All packages of `franka_ros2` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html

[fci-docs]: https://frankaemika.github.io/docs
