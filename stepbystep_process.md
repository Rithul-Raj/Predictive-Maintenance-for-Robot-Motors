# 🛠️ Setup & Getting Started Guide

A complete step-by-step guide to replicate this project on your own hardware — from scratch to fully running.

---

## 📋 Table of Contents

1. [What You Need](#1--what-you-need)
2. [Wiring & Hardware Assembly](#2--wiring--hardware-assembly)
3. [Software Prerequisites](#3--software-prerequisites)
4. [Setting Up Arduino IDE for ESP32](#4--setting-up-arduino-ide-for-esp32)
5. [Installing Required Libraries](#5--installing-required-libraries)
6. [Uploading the Code](#6--uploading-the-code)
7. [Powering Up the Robot](#7--powering-up-the-robot)
8. [Understanding the Warmup Phase](#8--understanding-the-warmup-phase)
9. [Accessing the Web Dashboard](#9--accessing-the-web-dashboard)
10. [Reading the Dashboard](#10--reading-the-dashboard)
11. [Testing Fault Detection](#11--testing-fault-detection)
12. [Troubleshooting](#12--troubleshooting)
13. [Important Notes & Tips](#13--important-notes--tips)

---

## 1. 🛒 What You Need

### Hardware

| Component | Quantity | Notes |
|---|---|---|
| ESP32 Development Board | 1 | Any 38-pin ESP32 dev board works |
| L298N Motor Driver | 1 | — |
| 2WD Robot Chassis with DC Motors | 1 | Comes with wheels and motor mounts |
| ACS712 Current Sensor | 1 | 5A or 20A version |
| MPU6050 Accelerometer/Gyroscope | 1 | GY-521 module recommended |
| DHT11 Temperature Sensor | 1 | With pull-up resistor on board preferred |
| Buck Converter | 1 | Set to output 5V |
| Li-Po Battery | 1 | 7.4V 2S recommended |
| Jumper Wires | Several | Male-to-male and male-to-female |
| Breadboard | 1 | For prototyping connections |
| USB Cable (Micro or USB-C) | 1 | Matching your ESP32 board |

### Software

- Arduino IDE (version 2.x recommended)
- Web browser (Chrome or Firefox) for the dashboard

---

## 2. 🔌 Wiring & Hardware Assembly

> ⚠️ **Do all wiring with the battery disconnected. Double-check every connection before powering on.**

Refer to the [Connection Diagram](images/connection_diagram.jpg) in this repository for the full visual reference. The table below describes each connection clearly.

### ESP32 → L298N Motor Driver

| ESP32 Pin | L298N Pin | Purpose |
|---|---|---|
| GPIO 27 | IN1 | Left motor direction |
| GPIO 26 | IN2 | Left motor direction |
| GPIO 25 | IN3 | Right motor direction |
| GPIO 33 | IN4 | Right motor direction |
| GPIO 32 | ENA (PWM) | Left motor speed |
| GPIO 14 | ENB (PWM) | Right motor speed |
| GND | GND | Common ground |

### ESP32 → ACS712 Current Sensor

| ESP32 Pin | ACS712 Pin | Purpose |
|---|---|---|
| 3.3V | VCC | Power |
| GND | GND | Ground |
| GPIO 34 (ADC) | OUT | Analog current reading |

### ESP32 → MPU6050

| ESP32 Pin | MPU6050 Pin | Purpose |
|---|---|---|
| 3.3V | VCC | Power |
| GND | GND | Ground |
| GPIO 21 | SDA | I2C Data |
| GPIO 22 | SCL | I2C Clock |

### ESP32 → DHT11

| ESP32 Pin | DHT11 Pin | Purpose |
|---|---|---|
| 3.3V | VCC | Power |
| GND | GND | Ground |
| GPIO 4 | DATA | Temperature/Humidity |

### Power Distribution

- Li-Po battery → Buck converter input
- Buck converter output (5V) → L298N 5V pin (logic power) and ESP32 VIN
- L298N 12V pin → Li-Po battery positive (motor power)
- All GND pins must share a **common ground**

> 💡 **Tip:** The ESP32 runs on 3.3V logic internally but the VIN pin accepts 5V. Always power sensors with 3.3V from the ESP32 unless the sensor specifically requires 5V.

---

## 3. 💻 Software Prerequisites

Before writing any code, make sure the following are installed on your computer:

- **Arduino IDE 2.x** — [Download here](https://www.arduino.cc/en/software)
- **CP2102 or CH340 USB Driver** — Required for the ESP32 to be recognized by your computer
  - CP2102: [Download](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)
  - CH340: [Download](https://sparks.gogo.co.nz/ch340.html)

> 💡 Check which USB chip your ESP32 board uses — it is usually printed near the USB port or mentioned in the product listing.

---

## 4. ⚙️ Setting Up Arduino IDE for ESP32

The Arduino IDE does not support ESP32 by default. Follow these steps to add it.

**Step 1** — Open Arduino IDE and go to:
`File` → `Preferences`

**Step 2** — In the **"Additional Boards Manager URLs"** field, paste this URL:
```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```

**Step 3** — Click **OK** to save.

**Step 4** — Go to:
`Tools` → `Board` → `Boards Manager`

**Step 5** — Search for `esp32` and install the package by **Espressif Systems**.

**Step 6** — Once installed, select your board:
`Tools` → `Board` → `ESP32 Arduino` → `ESP32 Dev Module`

**Step 7** — Set the upload settings:
| Setting | Value |
|---|---|
| Upload Speed | 115200 |
| CPU Frequency | 240MHz |
| Flash Size | 4MB |
| Partition Scheme | Default 4MB |
| Port | COMx (Windows) or /dev/ttyUSBx (Linux/Mac) |

> 💡 To find your COM port: On Windows, open Device Manager → Ports (COM & LPT). On Mac/Linux, run `ls /dev/tty.*` in the terminal.

---

## 5. 📦 Installing Required Libraries

Go to `Sketch` → `Include Library` → `Manage Libraries` and search for and install each of the following:

| Library | Author | Purpose |
|---|---|---|
| `MPU6050` | Electronic Cats or Jeff Rowberg | Accelerometer/Gyroscope |
| `DHT sensor library` | Adafruit | DHT11 temperature sensor |
| `Adafruit Unified Sensor` | Adafruit | Dependency for DHT library |
| `AsyncTCP` | dvarrel | Async TCP for web server |
| `ESPAsyncWebServer` | Me-No-Dev | Web dashboard server |
| `ArduinoJson` | Benoit Blanchon | JSON data for dashboard |
| `Wire` | Built-in | I2C communication (no install needed) |

> ⚠️ **Important:** `ESPAsyncWebServer` and `AsyncTCP` may not appear in the Library Manager. If that happens, download them manually from GitHub and install via `Sketch` → `Include Library` → `Add .ZIP Library`:
> - [AsyncTCP](https://github.com/dvarrel/AsyncTCP)
> - [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer)

---

## 6. 📤 Uploading the Code

**Step 1** — Clone or download this repository to your computer:
```bash
git clone https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git
```
Or click **Code → Download ZIP** on GitHub and extract it.

**Step 2** — Open the main `.ino` file in Arduino IDE.

**Step 3** — Find the Wi-Fi credentials section near the top of the code and update with your network details:
```cpp
const char* ssid     = "YOUR_WIFI_NAME";
const char* password = "YOUR_WIFI_PASSWORD";
```

**Step 4** — Connect the ESP32 to your computer via USB.

**Step 5** — Select the correct COM port under `Tools` → `Port`.

**Step 6** — Click the **Upload** button (→ arrow icon).

**Step 7** — Wait for the message:
```
Done uploading.
```

> ⚠️ If you get an upload error, hold the **BOOT button** on the ESP32 while the upload starts, then release it once you see "Connecting…" in the console.

---

## 7. 🔋 Powering Up the Robot

**Step 1** — Disconnect the USB cable after uploading.

**Step 2** — Double-check all wiring one more time before connecting the battery.

**Step 3** — Connect the Li-Po battery.

**Step 4** — The ESP32 onboard LED should light up and the robot will begin the **warmup phase** automatically.

**Step 5** — Open the **Serial Monitor** in Arduino IDE (`Tools` → `Serial Monitor`, baud rate: `115200`) to observe boot messages while the battery is not connected (USB monitoring only).

You should see output similar to:
```
[BOOT] System starting...
[WIFI] Connecting to network...
[WIFI] Connected! IP: 192.168.x.x
[WARMUP] Collecting baseline data...
[WARMUP] Baseline ready. System entering NORMAL state.
```

---

## 8. ⏱️ Understanding the Warmup Phase

The warmup phase is **critical** to the system working correctly.

- It runs automatically for the first **~30 seconds** after boot
- During this time, the robot moves under **normal load conditions**
- The system records sensor readings and computes the **mean and standard deviation** for current and vibration
- This becomes the **session baseline** — the reference point for all future fault detection

> ⚠️ **Do not block the wheels, apply extra load, or disturb the robot during warmup.** Any abnormal condition during this phase will skew the baseline and cause false detections or missed faults for the entire session.

> 💡 If you suspect the warmup was disrupted, simply **reset the ESP32** (press the EN/RST button) to start a fresh warmup.

---

## 9. 🌐 Accessing the Web Dashboard

Once the warmup phase is complete, the dashboard is live.

**Step 1** — Make sure your phone or computer is connected to **the same Wi-Fi network** as the ESP32.

**Step 2** — Find the ESP32's IP address from the Serial Monitor output:
```
[WIFI] Connected! IP: 192.168.x.x
```

**Step 3** — Open a browser and navigate to:
```
http://192.168.x.x
```

**Step 4** — The dashboard should load and begin displaying live data.

> 💡 **Tip:** Bookmark the IP address for quick access. Note that the IP address may change if the router assigns a new one after reboot. To get a fixed IP, configure a static IP in your router's DHCP settings for the ESP32's MAC address.

---

## 10. 📊 Reading the Dashboard

The dashboard displays the following in real time:

| Display Element | What It Means |
|---|---|
| Current (A) | Live motor current draw |
| Vibration Level | MPU6050 vibration intensity |
| Temperature (°C) | DHT11 ambient reading |
| Fault Score | Weighted anomaly score (0 = normal) |
| System State | WARMUP / NORMAL / WARNING / FAULT / CRITICAL |
| Motor Status | Running / Slowed Down / Stopped |

### System State Meanings

- 🟢 **NORMAL** — All readings within baseline range. Motors running at full speed.
- 🟡 **WARNING** — Minor deviation detected. System is monitoring closely.
- 🟠 **FAULT DETECTED** — Significant anomaly confirmed. Motors automatically slow down to reduce stress.
- 🔴 **CRITICAL FAULT** — Sudden spike detected across sensors. Motors halted immediately to prevent damage.
- ⛔ **MOTORS MANUALLY STOPPED** — User has stopped the motors via the dashboard during a fault condition.

---

## 11. 🧪 Testing Fault Detection

Once the robot is running normally after warmup, you can simulate faults to verify the system is working.

### Test 1 — Fault Detection (Motor Slowdown)
Gently press down on the robot chassis to **increase load on the wheels** while they are spinning. You should observe:
- Current reading rises
- Fault score increases
- System moves to **FAULT** state
- Motors slow down automatically

### Test 2 — Critical Fault (Sudden Spike)
Abruptly **block both wheels completely** for 1–2 seconds. You should observe:
- Sudden spike in current and vibration
- System immediately moves to **CRITICAL** state
- Motors stop immediately

### Test 3 — Manual Stop via Dashboard
During any fault condition, click the **Stop Motors** button on the dashboard. You should observe:
- Motors stop immediately
- Dashboard shows **MOTORS MANUALLY STOPPED** state

### Test 4 — Recovery
Release all load and allow the robot to run freely. After a few seconds of sustained normal readings, the system should automatically transition back to **NORMAL** state and resume full speed.

---

## 12. 🔧 Troubleshooting

| Problem | Likely Cause | Fix |
|---|---|---|
| ESP32 not detected by PC | Missing USB driver | Install CP2102 or CH340 driver |
| Upload fails | Wrong COM port or board selected | Recheck Tools → Board and Tools → Port |
| Upload fails with "Connecting…" stuck | ESP32 not entering flash mode | Hold BOOT button during upload start |
| Wi-Fi not connecting | Wrong credentials | Double-check ssid and password in code |
| Dashboard not loading | Different Wi-Fi network or wrong IP | Ensure same network; recheck IP from Serial Monitor |
| False faults immediately after warmup | Warmup was disturbed | Reset ESP32 and redo warmup cleanly |
| No current readings / always 0 | ACS712 wiring issue | Check VCC, GND, and OUT connections |
| MPU6050 not responding | I2C address conflict or wiring | Verify SDA/SCL pins; check I2C scanner sketch |
| DHT11 always shows -1 or NaN | Bad data pin or no pull-up | Check DATA pin connection; add 10kΩ pull-up resistor if needed |
| Motors not moving | L298N not powered or wrong pins | Check motor driver wiring and PWM pin assignments in code |

---

## 13. 📌 Important Notes & Tips

- 🔁 **Always reset the robot** before a new testing session. The baseline is session-specific and does not persist after power off.
- 🔋 **Monitor battery voltage.** A low battery will cause the current sensor to behave inconsistently, leading to false detections.
- 🌡️ **DHT11 has a 1–2 second response delay.** It is used for long-term trend monitoring, not instant temperature spikes.
- 📶 **Keep the robot within Wi-Fi range** of your router while using the dashboard.
- ⚡ **Never connect the Li-Po battery in reverse polarity.** Always verify positive and negative terminals before connecting.
- 🧰 **Use a multimeter** to verify buck converter output is exactly 5V before connecting it to the ESP32.
- 💾 **Do not modify the baseline computation** section of the code unless you fully understand the implications — it directly affects detection accuracy.

---

> 💬 If you run into any issue not covered here, open an **Issue** on this GitHub repository with a description of your problem, your wiring setup, and the Serial Monitor output. We will help you get it running.
