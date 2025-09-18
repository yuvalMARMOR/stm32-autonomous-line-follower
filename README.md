# STM32 Autonomous Line Follower 🚗

Autonomous line-following robot built with **STM32F103**.

## Features
- PID control for precise line following  
- Obstacle detection  
- Autonomous parking

---

## Project Structure
.
├─ docs/           # Reports & documentation (Final_Report.pdf)
├─ include/        # Header files (.h)
├─ src/            # Source files (.c)
├─ .gitignore      # Git ignore rules
└─ platformio.ini  # PlatformIO project configuration

---

## Requirements
- PlatformIO (in VS Code or standalone)  
- STM32F103 board (your config currently uses `disco_f100rb` in `platformio.ini`)  
- ST-LINK debugger/programmer  

---

## Quick Start (Build & Upload)
Clone the repo:
git clone https://github.com/yuvalMARMOR/stm32-autonomous-line-follower.git
cd stm32-autonomous-line-follower

Open the folder with **VS Code + PlatformIO**, connect the board via **ST-LINK**, then:
pio run --target upload

💡 If compile/upload fails, verify the `board` in `platformio.ini` matches your exact MCU/kit.

---

## Language & Framework
This project is written in **Embedded C** for STM32, using the **Standard Peripheral Library (SPL)**.

---

## Documentation
Full project report: docs/Final_Report.pdf

---

## Authors
- Yuval Marmur  
- Adi Ron
