# 🚗 Line Following Robot

An autonomous line-following robot built with an **ATmega328P microcontroller**.  
It uses **IR sensors** to follow a path and an **ultrasonic sensor** to detect and stop for obstacles.  
Motor control and navigation logic are implemented in **C** using **Microchip Studio** for smooth, precise movement.

---

## ⚙️ Features
- Line detection using IR sensor array  
- Obstacle detection and stop using ultrasonic sensor  
- Dual DC gear motor control via L298N driver  
- Smooth movement control using custom logic  
- Fully written and compiled in Microchip Studio

---

## 🧰 Hardware
- ATmega328P Microcontroller  
- IR Sensors (3–5 units)  
- Ultrasonic Sensor (HC-SR04)  
- L298N Motor Driver  
- DC Gear Motors  
- Battery Power Supply (7.4V–9V)

---

## 📂 Repository Structure
code_9_4.ino
Schematic_micro_2025-11-03.png


---

## 🧠 How It Works
1. IR sensors read the line and guide motion (left, right, straight).  
2. Ultrasonic sensor checks for nearby obstacles.  
3. The robot stops when an obstacle is detected and resumes once clear.  

---

## 🔧 Setup
1. Open the project in **Microchip Studio**.  
2. Build and compile the `.atsln` solution.  
3. Upload the HEX file to your ATmega328P using an AVR programmer.  

---

## 📜 License
Released under the **MIT License** – free to use and modify with attribution.

---

**Author:** Dasun Anupiya  
🛠️ “Making robots follow the line so I don’t have to!”
