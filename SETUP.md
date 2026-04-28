# 🛠️ Setup Guide — Predictive Maintenance for Robot Motors

This guide explains how to set up the development environment, install all required dependencies, and upload the firmware to the ESP32 board.

---

## 📌 Important Note on `requirements.txt`

This project is written entirely in **C++ for the ESP32 microcontroller** using the Arduino framework.  
It does **not use Python**, so `requirements.txt` does **not contain pip packages**.

Instead, the file documents the **Arduino libraries** required for this project.  
All dependencies are managed through **Arduino CLI** (not pip), and the `setup.sh` script handles everything automatically.

---

## ⚡ Quick Setup (Recommended)

Run the provided shell script. It will automatically install Arduino CLI, the ESP32 board package, and all required libraries.

### On Linux or macOS

Open a terminal and run:

```bash
bash setup.sh
```

### On Windows

**Option A — `setup.bat` (Easiest for Windows — Recommended)**

A dedicated Windows batch file is provided so you don't need Git Bash or WSL at all.

1. Open **File Explorer** and navigate to the project folder
2. Double-click **`setup.bat`**  
   — OR —  
   Open **Command Prompt**, navigate to the project folder, and run:
   ```bat
   setup.bat
   ```
3. The script will automatically install Arduino CLI, the ESP32 board package, and all required libraries

> ✅ This is the simplest option for Windows users. No extra tools needed.

**Option B — Git Bash (if you already have Git installed)**

1. Open **Git Bash**
2. Navigate to the project folder:
   ```bash
   cd /c/path/to/Predictive-Maintenance-for-Robot-Motors
   ```
3. Run:
   ```bash
   bash setup.sh
   ```

**Option C — WSL (Windows Subsystem for Linux)**
1. Enable WSL and install Ubuntu from the Microsoft Store
2. Open Ubuntu terminal
3. Navigate to the project and run:
   ```bash
   bash setup.sh
   ```

---

## 🔧 Manual Setup (Step-by-Step)

If you prefer to set up manually or the script does not work on your system, follow these steps.

### Step 1 — Install Arduino IDE

Download and install Arduino IDE 2.x from:  
https://www.arduino.cc/en/software
- **CP2102 or CH340 USB Driver** — Required for the ESP32 to be recognized by your computer
  - CP2102: [Download](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)
  - CH340: [Download](https://sparks.gogo.co.nz/ch340.html)

---

### Step 2 — Add ESP32 Board Support

1. Open Arduino IDE
2. Go to **File → Preferences**
3. In the **"Additional boards manager URLs"** field, paste:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Click **OK**
5. Go to **Tools → Board → Boards Manager**
6. Search for `esp32`
7. Install **"esp32" by Espressif Systems** (version 2.x or later)

---

### Step 3 — Install Required Libraries

Go to **Tools → Manage Libraries** and install each of the following:

| Library Name | Publisher | Purpose |
|---|---|---|
| `Adafruit MPU6050` | Adafruit | MPU6050 accelerometer for vibration detection |
| `Adafruit Unified Sensor` | Adafruit | Dependency for MPU6050 and DHT |
| `DHT sensor library` | Adafruit | DHT11 temperature and humidity sensor |
| `Adafruit BusIO` | Adafruit | I2C/SPI communication layer |

> **Note:** `WiFi` and `Wire` are **built-in** with the ESP32 board package. No separate installation is needed.

---

### Step 4 — Open the Project

1. In Arduino IDE, go to **File → Open**
2. Navigate to the `codes/` folder in this repository
3. Open the [code file](working_code.ino).it is the final version of the prototype working code.

---

### Step 5 — Configure Board and Port

1. Go to **Tools → Board → esp32 → ESP32 Dev Module**
2. Go to **Tools → Port** and select the COM port where your ESP32 is connected
   - On Windows it will appear as `COM3`, `COM4`, etc.
   - On Linux/macOS it will appear as `/dev/ttyUSB0` or `/dev/cu.usbserial-...`

---


### Step 6 — Upload to ESP32

1. Click the **Upload** button (→) in Arduino IDE
2. Wait for the compilation and upload to complete
3. Open **Tools → Serial Monitor** at baud rate `115200` to see debug output

---

## 📦 Dependency Summary

| Dependency | Type | How to Install |
|---|---|---|
| Arduino IDE 2.x | Tool | Download from arduino.cc |
| ESP32 Board Package | Board Core | Via Boards Manager (see Step 2) |
| Adafruit MPU6050 | Library | Via Library Manager |
| Adafruit Unified Sensor | Library | Via Library Manager |
| DHT sensor library | Library | Via Library Manager |
| Adafruit BusIO | Library | Via Library Manager |
| WiFi | Built-in | Included with ESP32 core |
| WebServer | Built-in | Included with ESP32 core |
| Wire | Built-in | Included with ESP32 core |

---

## 🌐 Accessing the Web Dashboard

Once the ESP32 is powered and connected to Wi-Fi:

1. Open the Serial Monitor — the ESP32 will print its **IP address**
2. Open a browser on any device on the **same Wi-Fi network**
3. Enter the IP address in the browser (e.g., `http://192.168.1.105`)
4. The real-time monitoring dashboard will load

---

## ❓ Troubleshooting

| Problem | Solution |
|---|---|
| Port not visible | Install CP2102 or CH340 USB driver for ESP32 |
| Upload fails | Hold the BOOT button on ESP32 while uploading starts |
| Library not found | Ensure all 4 Adafruit libraries are installed |
| Wi-Fi not connecting | Double-check SSID and password in the code |
| Dashboard not loading | Make sure your device is on the same Wi-Fi network as the ESP32 |

---

## 📁 Repository Structure

```
Predictive-Maintenance-for-Robot-Motors/
│
├── codes/                  ← ESP32 Arduino source code (.ino)
├── images/                 ← Project images and dashboard screenshots
├── requirements.txt        ← Dependency reference (Arduino libraries)
├── setup.sh                ← Automated setup script
├── SETUP.md                ← This file — full setup documentation
├── stepbystep_process.md   ← Development process documentation
├── connection_diagram.pdf  ← Hardware wiring diagram
└── README.md               ← Project overview
```

---

*For any questions about the project, refer to the README.md or contact the development team.*
