# WRO-2025 Future Engineers 🛠️ Durnibar_71 🛠️ 

<p align="center">
  <img src="media/Banner.png" alt="Durnibar_71 Banner" width="100%"/>
</p>

This repository contains our work for the **WRO Future Engineers** category.  
We are building an **Autonomous Car** using **Lidar, Computer Vision, Arduino nano, and Jetson Nano Dev Kit B01**.

---

## 👥 Team - Durnibar_71
- **Azmain Shak Rubayed** – Team Leader, Hardware Developer || [azmiansheikh.nub@gmail.com](mailto:azmiansheikh.nub@gmail.com)
- **Mohit Hasan** - Software Developer || [mohithasan.nub@gmail.com](mailto:azmiansheikh.nub@gmail.com)
---


## 🌟 The Meaning Behind "Durnibar_71"

> *"If the will to win is strong, then any goal can be achieved."*  

This is our **first participation in WRO**, and ever since we began working on this project, we have felt an **indomitable desire** to complete it. That is why we chose the name **"Durnibar"** *(Indomitable / Unstoppable)* — it reflects our relentless effort to give our absolute best, pushing beyond our current limits despite the challenges we faced.  

Every step of this journey has been difficult. We did not have adequate support, so we had to start from scratch and gather every resource on our own. This struggle itself embodies the true spirit of **Durnibar**, a determination to never give up.  

Now, why the **'71**?  
It represents the spirit of the **Bangladesh Liberation War of 1971**. Our independence was achieved not with abundant resources, but with an **unbreakable willpower and focus**. At that time, we were a nation decades behind the rest of the world, yet we shaped our own destiny.  

By carrying the name **Durnibar_71**, we honor that history and commemorate the indomitable willpower of our nation. Just as the people of ’71 stood unstoppable in their pursuit of freedom, we carry the same spirit forward in our journey — in robotics, innovation, and beyond.  

✨ Thank you, everyone, for standing with us in this mission. Together, we move forward with the spirit of **Durnibar_71** — unstoppable, unbreakable, indomitable.  

---

## 📌 Table of Contents
- [Overview](#overview)
- [Team Introduction & Team Information](#team-introduction--team-information)
- [Project Description](#project-description)
- [Hardware Used](#hardware-used)
- [Software & Libraries](#software--libraries)
- [Competition Challenges](#competition-challenges)
- [Spoilers](#spoilers)

---
## 🚙 Overview
Our self-driving vehicle prototype, developed for the **WRO Future Engineers 2025** competition, is a fully autonomous robotic system that merges **AI-based decision-making** with precise embedded hardware control.  

The platform is powered by a **Jetson Nano**, serving as the brain for vision and high-level processing, while an **Arduino Nano** handles real-time actuation and sensor management through custom firmware. Sensor data from the **MPU6050 (IMU)** and **QMC5883L (Compass)** is transmitted via serial communication, enabling accurate orientation tracking, drift correction, and smooth navigation.  

The system integrates:  
- **Raspberry Pi Camera v2.1** for image capture and object detection.  
- **MG996R servo motor** and **BTS7960 motor driver** for steering and propulsion control.  
- **1.3" OLED display**, **LED indicators**, **buzzer**, and **3 control buttons** for debugging and feedback.  

Structurally, the vehicle features a **custom PCB** for streamlined connections, alongside a **self-designed chassis**, built for optimal component placement and balance. Power is delivered through a **1000 mAh LiPo battery**, ensuring lightweight mobility and sufficient runtime for competition tasks.  

All functions are distributed between the **Jetson Nano** (core AI/vision algorithms) and the **Arduino Nano** (low-level hardware control), communicating through **PWM, UART, and I²C protocols**. This modular and efficient design ensures the vehicle can **adapt dynamically to track environments**, while maintaining precise actuation and reliable decision-making.  

Altogether, **Durnibar_71** represents an **indomitable and competition-ready platform**, combining embedded intelligence, compact hardware design, and robust autonomy.

---




## Team Introduction & Team Information
Introduce your team here...

## Project Description
Describe your project here...

## Hardware Used
List your hardware here...

## Software & Libraries
List your software/libraries here...

## Competition Challenges
Talk about your challenges here...

## Spoilers
Add spoilers or extras here...


## 📂 Repo Structure
- `docs/` → Reports, research, presentations
- `hardware/` → CAD designs, wiring diagrams
- `firmware/` → Arduino / ESP32 / STM32 codes
- `software/` → ROS, Python, AI/ML, Jetson Nano code
- `simulations/` → Gazebo, RViz, CARLA setups
- `datasets/` → Training & test datasets
- `media/` → Images, videos, project logo

---

## 🛠️ Features
- Lane detection with OpenCV
- Obstacle detection using Lidar/Ultrasonic
- PID speed & steering control
- Simulation with ROS + Gazebo
- Autonomous navigation with waypoints

---



## 📜 License
This project is licensed under the **MIT License** – see the [LICENSE](LICENSE) file for details.
