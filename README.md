# ROS 2 Humble 3D SLAM Rover

This is a **ROS 2 Humble** project to run a **3D SLAM simulation** in Gazebo.

The project configures a **differential drive rover** with a virtual **Intel RealSense D435** depth camera and an **IMU**.  
It is set up to launch **rtabmap_ros** for 3D mapping and visualize the results in **RViz**.

---

⚠️ **Current Status: Not Working**

This project is in a **non-functional state**.  
The Gazebo launch file (`2_gazebo.launch.py`) fails to spawn the robot.  
It times out while waiting for the `/spawn_entity` service from Gazebo, which does not appear to be starting correctly.

Further debugging is required to resolve this **Gazebo/ROS 2 integration issue**.

---

## 🛠️ Installation

### 1. Install ROS 2 Dependencies
```bash
sudo apt update
sudo apt install ros-humble-rtabmap-ros ros-humble-teleop-twist-keyboard ros-humble-xacro
```

### 2. Build the Workspace

From the root of the workspace (`~/ros2_workspace2`):
```bash
rm -rf build install log
colcon build
```
---

## 🚀 How to Run (Current State)

You must run each command in a **separate, sourced terminal**.

---

### 🧩 Terminal 1: Launch Gazebo + RViz *(This will fail)*

This terminal attempts to start Gazebo, RViz, and the robot state publisher.  
It will fail with the `/spawn_entity` timeout error.
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_workspace2
source install/setup.bash
ros2 launch rover 2_gazebo.launch.py
```
---

### 🧭 Terminal 2: Launch SLAM Node

This terminal runs the **rtabmap SLAM** node in the background.
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_workspace2
source install/setup.bash
ros2 launch rover slam.launch.py
```
---

### 🎮 Terminal 3: Launch Teleop (Keyboard Control)

This terminal runs the **keyboard controller**.
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_workspace2
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 🧱 RViz Configuration (If Gazebo Worked)

If the simulation were running, you would configure RViz as follows:

- **Fixed Frame:** `map` (in *Global Options*)  
- **Add 3D Map:** Add a *PointCloud2* display with the topic `/rtabmap/cloud_map`  
- **Add Robot Path:** Add a *Path* display with the topic `/local_path`  
- **Add Robot Model:** Add a *RobotModel* display  

---

## 📦 Package Components

| File / Directory | Description |
|------------------|-------------|
| `urdf/rover.urdf` | The robot's description file, containing links, joints, and Gazebo plugins for the differential drive, IMU, and RealSense D435. |
| `launch/2_gazebo.launch.py` | The main simulation launch file. Starts Gazebo, robot_state_publisher, the robot spawner, and RViz. *(Currently failing)* |
| `launch/slam.launch.py` | The SLAM launch file. Includes the official `rtabmap.launch.py` and passes all necessary parameters and topic remappings to it. |

---

## ⚙️ Summary

This project demonstrates how to set up a **3D SLAM rover simulation** in **ROS 2 Humble** using **Gazebo**, **RTAB-Map**, and **RViz**.  
Currently, the Gazebo portion is **non-functional**, and further debugging is required to resolve the `/spawn_entity` timeout issue.

