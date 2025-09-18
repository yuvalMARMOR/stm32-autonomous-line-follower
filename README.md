# STM32 Autonomous Line Follower 🚗

An autonomous line-following robot built with **STM32F103**.  
Features:
- PID control for precise line following  
- Obstacle detection  
- Autonomous parking  

---

## 📂 Project Structure


.
├── docs/ # Reports and documentation (Final_Report.pdf)
├── include/ # Header files (.h)
├── src/ # Source files (.c)
├── .gitignore # Git ignore rules
├── platformio.ini # PlatformIO project configuration


---

## ⚙️ Requirements
- **PlatformIO** (installed on VS Code or standalone)  
- **STM32F103 board** (tested on STM32F103RB)  
- **ST-LINK** debugger/programmer  

---

## 🚀 How to Build & Upload
1. Clone this repository:
   ```bash
   git clone https://github.com/yuvalMARMOR/stm32-autonomous-line-follower.git


Open the folder with VS Code + PlatformIO.

Connect your STM32 board via ST-LINK.

Build & upload the firmware:

pio run --target upload

🛠️ Language & Framework

This project is written in Embedded C for STM32,
using the Standard Peripheral Library (SPL).

📄 Documentation

Full project report:
👉 docs/Final_Report.pdf

👥 Authors

Yuval Marmur
Adi Ron