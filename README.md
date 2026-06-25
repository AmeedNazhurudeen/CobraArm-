# 🦾 IntelliGrasp
> Say what to pick. The robot sees it and grabs it.

A ROS 2 simulation where a robot arm uses a camera to find objects on a table and pick them up based on a text command. Fully simulated — no real robot needed.

---

## 💡 The Idea

```
You type:  "pick red cube"
Robot:      sees it → finds it in 3D → moves → grabs it
```

---

## 📁 Structure

```
cobot_ws/
├── src/
│   ├── arm_bringup/        # Robot + Gazebo world + launch
│   ├── perception_pkg/     # Camera + object detection (OpenCV)
│   └── manipulation_pkg/   # Motion planning + pick + command
├── .gitignore
└── README.md
```

---

## 🛠️ Built With

- ROS 2 Humble
- Gazebo (simulation)
- RViz2 (visualisation)
- OpenCV (object detection)
- MoveIt2 (motion planning)
- Python 3

---

## 📅 5-Day Plan

| Day | What gets built |
|-----|----------------|
| 1 | Workspace setup + GitHub |
| 2 | UR5 arm running in Gazebo |
| 3 | Camera detecting objects |
| 4 | Arm moves to a target pose |
| 5 | Full pick-and-place from a text command |

---

## 🚀 Run It

```bash
# Build
cd cobot_ws
colcon build --symlink-install
source install/setup.bash

# Launch simulation
ros2 launch arm_bringup sim.launch.py

# Send a pick command
ros2 topic pub /pick_object std_msgs/String "data: 'red cube'"
```

---

## 👤 Author
Built as a 5-day ROS 2 beginner project.

## 📄 License
MIT
EOF
