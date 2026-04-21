# 🍷 WineBot —  Wine Glass Stacking using MTC

**Robotics | Motion Planning | ROS 2 | MoveIt 2 | NVIDIA Isaac Sim**

---

## 📌 Overview

WineBot is a robotics project that demonstrates ** pick-and-place with precision stacking** using a Franka Panda robot arm.

The system plans and executes the complete task of:
- Grasping a  wine glass  
- Transporting it safely  
- Stacking it accurately on another glass  

All motions are generated using **MoveIt Task Constructor (MTC)**, ensuring collision-aware and constraint-driven planning.

---

## 🎥 Demo

![WineBot Demo](https://github.com/logesh1516/Wine-glass-Manipulation/blob/9bd279c06396ce373b53717ac0c3dc1898bf3abe/src/isaacsim_assets/isaac_sim_1_1_1.gif)


---

## 🧠 Key Contributions

- Designed a **multi-stage task pipeline** for pick-and-place using MTC  
- Implemented **hybrid motion planning** combining joint-space and Cartesian control  
- Integrated **mesh-based collision objects** for realistic grasping  
- Applied **orientation constraints** to maintain object stability during transport  
- Improved grasp success using **pose sampling and multiple IK solutions**  

---

## ⚙️ System Architecture

The task is broken into structured stages:

1. **Pre-grasp**
   - Open gripper  
   - Move to object  

2. **Grasp**
   - Approach object  
   - Compute valid grasp poses  
   - Close gripper and attach object  

3. **Transport**
   - Lift object  
   - Move to target with orientation constraints  

4. **Place**
   - Lower object  
   - Release and detach  

5. **Post-task**
   - Retreat  
   - Return to home position  

---

## 🧩 Technical Highlights

### Hybrid Planning Strategy
- **Joint-space planning** for flexible movement  
- **Cartesian planning** for precision tasks  

### Realistic Simulation
- Uses **3D mesh models (DAE)** instead of simple primitives  

### Robust Grasping
- Multiple grasp orientations evaluated  
- Multiple inverse kinematics (IK) solutions tested  

### Constraint-Aware Motion
- Ensures the glass remains upright during transport  

---

## 🛠️ Tech Stack

- **ROS 2 (Humble / Iron)**  
- **MoveIt 2**  
- **MoveIt Task Constructor (MTC)**  
- **NVIDIA Isaac Sim**  
- **Eigen3, tf2, geometric_shapes**  

---

## 📁 Project Structure

isaacsim_winebot/  
├── src/                # MTC task implementation  
├── meshes/             # Wine glass model  
├── launch/             # Simulation & MoveIt launch files  
├── CMakeLists.txt  
└── package.xml  

---

## 🚀 Isaac Sim Setup 

- Load the wine_glass_manipulation.usd from the isaacsim_assets into isaac sim
- play the simulation


## 🚀 Setup & Execution

```bash
cd ~/ros2_ws/src
git clone git@github.com:logesh1516/Wine-glass-Manipulation.git

cd ~/ros2_ws
colcon build --packages-select isaacsim_winebot
source install/setup.bash

## for mock component(without isaac sim)
ros2 launch isaacsim_winebot mtc_mock.launch.py

## or with isaac sim
ros2 launch isaacsim_winebot mtc_isaac.launch.py

and then launch mtc 

ros2 launch isaacsim_winebot mtc_execute.launch.py
```

---

