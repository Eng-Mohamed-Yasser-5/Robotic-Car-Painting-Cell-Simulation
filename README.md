# Robotic Car Painting Cell Simulation

A simulation of an industrial **robotic car-painting cell**: a UR10 manipulator mounted next to a car body, planning and executing spray-painting paths over the car surface. Built with **ROS 2 Jazzy**, **Gazebo Harmonic**, **ros2_control** and **MoveIt 2**, with the motion logic written in **C++**.

## YUKTII AI LABS
Robotics Simulation Intern Practical Assessment for YUKTII AI LABS
 - Name : Mohamed Yasser
 - Software : Ros2 jazzy, Gazebo harmonic
 - Robot : UR10

## Overview

Automotive paint shops rely on robot arms to cover car bodies with a uniform coat of paint. This project reproduces a simplified version of that cell in simulation so that painting strategies can be developed and tested safely before touching real hardware.

The simulated cell contains:

- A **UR10** 6-DOF arm with a spray tool attached at `tool0`
- A **car body** (STL mesh) placed in the Gazebo world as the painting target
- **ros2_control** controllers that drive the arm joints in Gazebo
- A **MoveIt 2** configuration for motion planning and collision checking
- A **C++ motion node** that generates a painting path (zig-zag/raster pattern) and sends it to MoveIt

## Repository Structure

```
.
├── robot_description/     # URDF/Xacro, UR10 meshes, spray tool, world and display launch files
├── robot_controller/      # ros2_control configuration and controller setup
├── robot_moveit_config/   # MoveIt 2 config (SRDF, kinematics, planners, controllers)
├── robot_motion_cpp/      # C++ node that builds the painting path and executes it via MoveIt
└── fix.txt                # Quick environment fix for Gazebo resource paths
```

| Package | Purpose |
| --- | --- |
| `robot_description` | Robot model, meshes, Gazebo world with the car, RViz/Gazebo display launch |
| `robot_controller` | Joint trajectory controller and `ros2_control` hardware interface setup |
| `robot_moveit_config` | MoveIt planning group (`arm`), end effector (`tool0`), planners and controllers |
| `robot_motion_cpp` | Painting-path generation and execution through the MoveIt C++ API |

## Prerequisites

- Ubuntu 24.04
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install) with `ros_gz`
- MoveIt 2 and ros2_control

```bash
sudo apt update
sudo apt install ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
                 ros-jazzy-gz-ros2-control ros-jazzy-ros-gz \
                 ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro
```

## Build

```bash
mkdir -p ~/ws/src && cd ~/ws/src
git clone https://github.com/Eng-Mohamed-Yasser-5/Robotic-Car-Painting-Cell-Simulation.git
cd ~/ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Gazebo Resource Path Fix

If Gazebo cannot find the robot or car meshes, add the `robot_description` share directory to Gazebo's resource path:

```bash
echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix robot_description)/share' >> ~/.bashrc
source ~/.bashrc
```

## Usage

**1. View the robot model**

```bash
ros2 launch robot_description display.launch.xml
```

**2. Launch the simulation with controllers and MoveIt**

```bash
ros2 launch <package> <simulation_launch_file>
```

**3. Run the painting motion node**

```bash
ros2 run robot_motion_cpp <painting_node_executable>
```

> Replace the placeholders above with the launch file and executable names from your packages.

## How It Works

1. `robot_description` loads the UR10 with its spray tool and spawns it, together with the car, in Gazebo.
2. `robot_controller` starts `ros2_control` and the joint trajectory controller so the simulated arm accepts commands.
3. `robot_moveit_config` starts the MoveIt `move_group` for the `arm` planning group with `tool0` as the end effector.
4. `robot_motion_cpp` computes a zig-zag set of waypoints over the car surface, plans through them with MoveIt and executes the trajectory.

## Current Status and Limitations

- Painting a small rectangular patch on the car surface works with full Cartesian-path coverage.
- Planning over larger surfaces can fail with `INVALID_MOTION_PLAN`, usually because of reachability limits or collisions between the arm and the car.


## Demo

![Domo](media/vedios/demo.gif)

## Author

 - **Mohamed Yasser** — Mechatronics and Robotics Engineering student
 - email : eng.mohamed.yasser.5@gmail.com

## License

MIT
