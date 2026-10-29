# Lesson 01 — Installation Guide (ROS 2 Humble)

> **Branch:** `humble` | Ubuntu 22.04 LTS

---

## Prerequisites

| Requirement | Version |
|-------------|---------|
| Ubuntu | 22.04 LTS (Jammy Jellyfish) |
| ROS 2 | Humble Hawksbill |
| Python | 3.10 |
| CMake | ≥ 3.22 |
| RAM | ≥ 8 GB recommended |
| Disk Space | ≥ 10 GB free |

---

## Step 1: Install ROS 2 Humble

```bash
# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 APT repository
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
  | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Desktop (includes RViz)
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2 (add to ~/.bashrc for permanent effect)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 2: Install MoveIt 2

```bash
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-ros-planning-interface \
  ros-humble-moveit-ros-visualization \
  ros-humble-moveit-setup-assistant \
  ros-humble-moveit-resources-panda-moveit-config \
  ros-humble-moveit-resources-panda-description
```

---

## Step 3: Install Franka ROS 2 & Controllers

```bash
sudo apt install -y \
  ros-humble-franka-ros2 \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gripper-controllers \
  ros-humble-joint-trajectory-controller
```

---

## Step 4: Create Workspace & Clone This Repo

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# Clone the humble branch
git clone -b humble https://github.com/JosiahSK/Panda_Moveit2.git

# Install all ROS dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## Step 5: Build the Workspace

```bash
cd ~/ros2_ws

# Build with optimizations
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the overlay
source install/setup.bash

# Optionally add to ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Step 6: Verify Installation

```bash
# Test 1: MoveIt is available
ros2 pkg list | grep moveit

# Test 2: Launch the Panda MoveIt demo
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

If RViz opens with the Panda robot and the MotionPlanning panel, your installation is complete! 🎉

---

## Troubleshooting

### ❌ `colcon build` fails with CMake errors
```bash
# Make sure you're using the correct compiler
sudo apt install -y build-essential cmake
```

### ❌ RViz shows "No robot model loaded"
```bash
# Re-source your workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### ❌ `rosdep install` fails
```bash
# Update rosdep database
rosdep update --rosdistro humble
```

### ❌ ImportError with MoveIt Python bindings
```bash
sudo apt install -y python3-moveit-ros-planning-interface
```

---

## Next Steps

👉 Proceed to [Lesson 02 — RViz Basics](02_rviz_basics.md)
