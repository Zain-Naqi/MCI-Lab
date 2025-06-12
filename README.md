# Tehqiq Project: Revamping MCI Lab

This repository contains the lab work for our **Tehqiq Project: Revamping the MCI Lab**.

---

## 👨‍💻 Group Members
- **Maniya Nehal**  
- **Ghufran Alavi**  
- **Zain Naqi**

## 🧑‍🏫 Instructors
- **Waseem Hassan**  
- **Abeera Farooq**

---

## 🛠️ Pre-Requisites

Make sure you have the following hardware and software tools set up before starting:

- A laptop or PC running **Ubuntu**
- **Visual Studio Code** with **CubeMX** and **CMake** configured  
  *(Setup guide is included in the repository)*
- **STM32F303 Discovery Board**
- **Silicon Labs CP2102 USB to Serial Adapter**
- **Screen terminal** (install via terminal)

---

📘 Lab 01 Instructions

1. Install the Screen Terminal
In your terminal, run:

_sudo apt update && sudo apt install screen_
2. Before generating the code make sure to enable USART1 and Virtual Port Com
   - To enable USART-1 after clicking on the board you are redirected to a page with pins, in the side click the dropdown name connectivity -> USART 1 -> change to asynchronous.
   - To change to Virtual Port Com, under the Middleware and Software dropdown select USB and then change it Virtual Port Com.

3. The code is provided on Github, set the project as instructed above and update the main.
4. To run the code build-> Run task to open the terminal use _screen /dev/ttyACM0 115200_

