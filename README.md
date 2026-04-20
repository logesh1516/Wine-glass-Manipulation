# 🍷 WineBot — Wine Glass Stacking Robot

> A robot that picks up a wine glass and carefully stacks it on another — all planned automatically.

---

## 📽️ Demo

> Add your Isaac Sim video or GIF here

---

## 🧠 What is this?

This project shows how a robot arm can:
- Pick up a wine glass  
- Move it safely  
- Place it exactly on top of another glass  

Everything is done using automatic motion planning, not hardcoded movements.

---

## ⚙️ How it Works

The robot uses two types of motion:

### 🔹 Smooth Movement (General Motion)
Used when the robot is just moving freely.

### 🔹 Straight-Line Precision Movement
Used when accuracy matters:
- Moving toward the glass  
- Lifting it  
- Placing it carefully  

---

## 🏗️ Task Flow

Open Gripper  
→ Move to Glass  
→ Approach  
→ Pick Glass  
→ Lift  
→ Move to Target  
→ Place on Second Glass  
→ Release  
→ Move Back  
→ Return Home  

---

## 🔑 Key Features

### 🧩 Realistic Glass Model
- Uses a 3D mesh (not a simple shape)
- Makes the simulation more realistic

---

### 🎯 Accurate Placement
- Robot places the glass precisely on top of another
- Maintains proper orientation (doesn’t tilt)

---

### 🔄 Smart Grasping
- Tries multiple angles to find a valid grasp
- Improves success rate

---

### 🤖 Stable Gripper Control
- Directly controls finger joints
- Avoids grasp failures

---

## 🌐 Scene Setup

Two glasses are placed in the simulation:
- One to pick  
- One to stack onto  

---

## 🚀 How to Run

cd ~/ros2_ws/src  
git clone <your-repo-url>  

cd ~/ros2_ws  
colcon build --packages-select isaacsim_winebot  
source install/setup.bash  

ros2 run isaacsim_winebot mtc_winebot_glass  

---

## 📁 Project Structure

isaacsim_winebot/  
├── src/  
├── meshes/  
├── launch/  
├── CMakeLists.txt  
└── package.xml  

---

## 🧩 Tech Used

- ROS 2  
- MoveIt 2  
- MoveIt Task Constructor  
- NVIDIA Isaac Sim  

---

## 🎯 Why this Project?

This is more than a basic pick-and-place.

It shows:
- Handling fragile objects  
- Precise stacking  
- Automatic planning  

---

## 🔮 Future Ideas

- Add camera-based detection  
- Stack multiple glasses  
- Run on a real robot  

---

## 📜 License

Add your license here
