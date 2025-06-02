---

# 🤖 Humanoid Robot System

## 📌 Project Overview

This project involves the development of a humanoid robot capable of:

* Moving its arms using servo motors
* Interacting with users through speech (ASR, chatbot, TTS)
* Navigating autonomously in a mapped environment

My primary contribution focused on **integrating and controlling the robotic arms**, developing the **chatbot system**, and ensuring **system-level integration**.

---

## 🔧 Responsibilities

* 🦾 Arm control via **ESP32** and **servo motors** using **ROS 2**
* 🖥️ GUI creation for **manual arm manipulation**
* 💬 Chatbot system using **Speech-to-Text**, **Mistral AI**, and **Text-to-Speech**
* 🧠 System integration: managing launch scripts, audio configuration, and component synchronization

The robot is powered by a **Jetson Orin Nano** and developed with **Python** and **ROS 2**.

---

## 🚀 How to Run the System

This system consists of three major components: **Arm Control**, **Chatbot**, and **Navigation**. Below are the instructions to run each part.

---

### 🦿 Arm Control (ESP32 + GUI)

#### 1. Launch the ESP32 Control Node

```bash
cd ros2_ws_arms
. install/setup.bash
ros2 run esp32_controller esp32_control_node
```

#### 2. Start the Arm Control GUI

In the **same terminal**, run:

```bash
python3 /home/ela2/ros2_ws_arms/src/GUI/GUI.py
```

This GUI allows you to manually control each joint using input fields or predefined gestures.

#### (Optional) Visualize in RViz

```bash
ros2 launch ela2_arms display.launch.py
```

This launches RViz with the robot model for visual debugging.

---

### 💬 Chatbot System (ASR + NLP + TTS)

#### 1. Configure Audio Output

Ensure the correct audio output is set:

```bash
./set-default-sink.sh
```

This script sets the USB speaker as the default, unmutes it, and sets volume to 100%.

#### 2. Launch the Chatbot System

```bash
./run_tmux_chatbot.sh
```

This script uses `tmux` to run all components in parallel:

* `ela2_ears.py` – Speech-to-Text (ASR)
* `Chatbot.py` – Mistral-based AI responses
* `ela2_mouth.py` – Text-to-Speech (TTS)
* `set-default-sink.sh` – Ensures audio settings are maintained

> Use `Ctrl+B` then `D` to detach from `tmux`, or `exit` to stop individual panes.

---

### 🧭 Navigation System (LiDAR + Map + RViz)

#### 1. Fix LiDAR Timestamp Issue

```bash
python3 /home/ela2/ELA2.0_NAV/src/ydlidar_ros2_driver/launch/fix_scan_timestamp.py
```

This script republishes the `/scan` topic with corrected timestamps to `/scan_fixed`.

#### 2. Launch Navigation Stack

```bash
ros2 launch ela2_nav ela2_nav.launch.xml bringup:=true display:=true pre_map:=true map:=apcore
```

This command launches:

* Robot visualization in RViz
* AMCL for localization
* Navigation2 stack using the preloaded `apcore` map

---

### 📡 ESP32 Firmware

The file `ESP32_Code.ino` contains the firmware for the ESP32 microcontroller. It receives joint angle commands from ROS and actuates the servos.

---

### 🔗 Additional Resources

The navigation system was developed by a teammate. More information is available at the following repository:

👉 [ELA2.0\_NAV GitHub Repository](https://github.com/LimJingXiang1226/ELA2.0_NAV?tab=readme-ov-file)

---
