# PX4 Autonomous Flight Control using MAVSDK-Python (Gazebo Simulation)

## Overview
A Python-based autonomous flight controller for PX4 drones in Gazebo simulation.

Instead of manual RC control, this system uses **MAVSDK-Python** to send continuous setpoints to the drone via Offboard mode, allowing programmatic flight path control.

**What the drone does:**
- Connects to PX4 flight controller
- Arms automatically
- Takes off to target altitude
- Changes direction in response to setpoints
- Lands autonomously

All tested in **Gazebo SITL (Software-in-the-Loop) simulation**.


## 🎯 Why I Built This

I wanted to understand how autonomous drones actually work beyond flying manually. 

This project taught me:
- How external code communicates with flight controllers
- The difference between manual RC control and programmed autonomy
- How Offboard mode works in PX4
- Basics of UAV flight dynamics and control

It's the foundation for building more complex autonomous systems like waypoint navigation and swarm robotics.



## ✨ Features
✅ PX4 + MAVSDK connection over MAVLink  
✅ Automatic health checks before arming  
✅ Programmatic arming and takeoff  
✅ Offboard mode activation  
✅ Continuous setpoint streaming (2 Hz minimum)  
✅ Position control in NED (North-East-Down) frame  
✅ Direction changes mid-flight  
✅ Autonomous landing  
✅ Works in Gazebo SITL simulation  


## 🛠 Tech Stack
- **Flight Controller:** PX4 Autopilot
- **Control Library:** MAVSDK-Python
- **Simulation:** Gazebo + PX4 SITL
- **Language:** Python 3.8+
- **Monitoring:** QGroundControl (optional)
- **Environment:** Ubuntu 20.04 / 22.04


## ⚙️ How Flight Control Works

The script executes this sequence:


    Connect to drone via MAVSDK
    ↓

    Wait for GPS lock & system health OK
    ↓

    Arm drone
    ↓

    Send initial setpoints (home position)
    ↓

    Activate Offboard mode
    ↓

    Stream setpoints continuously
    ↓

    Change direction (setpoint updates)
    ↓

    Hover at final position
    ↓

    Land safely
    ↓

    Disarm


