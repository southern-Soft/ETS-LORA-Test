# RAK3112 ETS & Careflow — ESP32-S3 Firmware

A dual-mode embedded application for the **RAK3112** module (ESP32-S3), built with **ESP-IDF v5.5.3 + Arduino** component. At startup the user selects one of two operational modes via a touchscreen:

| Mode | Function |
|---|---|
| **ETS** | RFID card scanning → LoRaWAN Class C uplink (ChirpStack) |
| **Careflow** | MFM384 energy meter polling via RS-485 Modbus RTU |

---

## Table of Contents

- [Hardware](#hardware)
- [Software Stack](#software-stack)
- [Project Structure](#project-structure)
- [Pin Configuration](#pin-configuration)
- [Modes](#modes)
  - [ETS Mode](#ets-mode)
  - [Careflow Mode](#careflow-mode)
- [Display & Touch](#display--touch)
- [Build Instructions](#build-instructions)
- [Dependencies](#dependencies)
- [LoRaWAN Configuration](#lorawan-configuration)
- [Modbus Parameters](#modbus-parameters)
- [Troubleshooting](#troubleshooting)

---

## Hardware

| Component | Details |
|---|---|
| **MCU** | RAK3112 (ESP32-S3) |
| **LoRa Radio** | SX1262 |
| **Display** | 320×240 TFT (SPI, TFT_eSPI) |
| **Touch** | FT6336 capacitive touch controller (I2C) |
| **RFID Reader** | UART-based, 125 kHz EM4100 (Wiegand-like 12-byte frame) |
| **Energy Meter** | MFM384 — RS-485 Modbus RTU |

---

## Software Stack

- **Framework:** ESP-IDF v5.5.3 with `espressif/arduino-esp32 v3.3.8`
- **Language:** C++ (Arduino style)
- **Build System:** CMake + Ninja
- **Target:** `esp32s3`
- **Optimization:** `-O3`

---

## Project Structure

```
Rak3112_ETS_CAREFLOW/
├── main/
│   ├── main.cpp          # Application entry point (setup + loop)
│   ├── touch.h           # Touch API declarations
│   ├── touch.cpp         # FT6336 touch implementation
│   ├── FT6336.h          # FT6336 driver header
│   ├── FT6336.cpp        # FT6336 driver implementation
│   └── CMakeLists.txt    # Component build file
├── libraries/
│   ├── SX126x-Arduino/   # LoRa radio driver
│   ├── TFT_eSPI/         # Display driver
│   └── ModbusMaster/     # Modbus RTU master
├── managed_components/
│   └── espressif__esp-modbus/
├── sdkconfig             # ESP-IDF Kconfig
├── sdkconfig.defaults    # Default Kconfig values
├── idf_component.yml     # Managed component manifest
└── CMakeLists.txt        # Top-level build file
```

---

## Pin Configuration

### SX1262 LoRa Radio

| Signal | GPIO |
|---|---|
| NSS (CS) | 7 |
| SCLK | 5 |
| MISO | 3 |
| MOSI | 6 |
| RESET | 8 |
| DIO1 | 47 |
| BUSY | 48 |

### Display & Backlight

| Signal | GPIO |
|---|---|
| Backlight EN | 42 |
| SPI pins | Configured via `User_Setup.h` in TFT_eSPI |

### Touch Controller (FT6336, I2C)

| Signal | GPIO |
|---|---|
| SDA | 9 |
| SCL | 40 |
| RST | 41 |
| INT | *(not used, -1)* |

### RFID Reader (UART1)

| Signal | GPIO |
|---|---|
| RX | 18 |
| TX | 17 |
| Baud | 115200 |

### RS-485 / MFM384 (UART2)

| Signal | GPIO |
|---|---|
| RX | 44 |
| TX | 43 |
| DE/RE | 34 |
| Baud | 9600, 8N1 |

---

## Modes

### ETS Mode

**Purpose:** Scan EM4100 RFID cards and transmit the card UID over LoRaWAN Class C to a ChirpStack network server.

**Flow:**

```
Tap "ETS" on screen
        │
        ▼
  LoRa HW Init (SX1262)
        │
        ▼
  OTAA Join (Class C, AS923)
        │
        ▼
  Wait for RFID scan
        │
        ▼
  Parse 12-byte UART frame
  [0x02][8 HEX chars][0x0D][0x0A][0x03]
        │
        ▼
  Build 12-byte LoRa payload
  [4 bytes DEC UID][8 bytes HEX UID]
        │
        ▼
  lmh_send() → Unconfirmed uplink, port 1
        │
        ▼
  Display HEX + DEC UID on screen
        │
        ▼
  Handle downlink (displayed for 6s)
```

**Debounce:** Same card is ignored for 1500 ms after first scan.

---

### Careflow Mode

**Purpose:** Poll an MFM384 three-phase energy meter over RS-485 Modbus RTU and display live readings on a scrollable TFT list.

**Flow:**

```
Tap "Careflow" on screen
        │
        ▼
  Read all 23 Modbus parameters
        │
        ▼
  Display scrollable list (8 rows visible)
        │
        ▼
  Auto-refresh every 5 seconds
        │
        ▼
  Swipe up/down to scroll
```

**Modbus settings:** Slave ID `1`, Input Registers, float values (32-bit, little-endian word order).

---

## Display & Touch

- **Resolution:** 320×240, rotation 1 (landscape)
- **Library:** TFT_eSPI
- **Touch driver:** FT6336 capacitive, I2C
- **Touch API:**

```cpp
touch_init(320, 240, 1);   // width, height, rotation
touch_touched();            // returns true if screen is touched
touch_last_x;               // last touch X coordinate
touch_last_y;               // last touch Y coordinate
```

---

## Build Instructions

### Prerequisites

- [ESP-IDF v5.5.3](https://docs.espressif.com/projects/esp-idf/en/v5.5.3/)
- CMake ≥ 3.16
- Ninja
- Python 3.13+
- Xtensa ESP32-S3 toolchain (`xtensa-esp-elf`)

### Steps

```bash
# 1. Set up ESP-IDF environment
. $IDF_PATH/export.sh          # Linux/macOS
# or in Windows CMD:
# C:\esp\v5.5.3\esp-idf\export.bat

# 2. Navigate to project
cd Rak3112_ETS_CAREFLOW

# 3. Set target
idf.py set-target esp32s3

# 4. (Optional) Configure
idf.py menuconfig

# 5. Build
idf.py build

# 6. Flash
idf.py -p COM_PORT flash monitor
```

---

## Dependencies

### Managed (via `idf_component.yml`)

| Component | Version |
|---|---|
| espressif/arduino-esp32 | 3.3.8 |
| espressif/esp-modbus | 1.0.18 |
| espressif/esp-sr | 2.4.3 |
| espressif/mdns | 1.11.1 |
| espressif/esp_rainmaker | 1.5.2 |
| espressif/esp_insights | 1.2.2 |
| joltwallet/littlefs | 1.21.1 |
| chmorgan/esp-libhelix-mp3 | 1.0.3 |

### Local (`libraries/` folder)

| Library | Purpose |
|---|---|
| SX126x-Arduino | SX1262 LoRa radio driver + LoRaWAN stack |
| TFT_eSPI | TFT display driver |
| ModbusMaster | Modbus RTU master |
| FT6336 *(in `main/`)* | Capacitive touch controller driver |

---

## LoRaWAN Configuration

Edit the keys at the top of `main/main.cpp`:

```cpp
uint8_t nodeDeviceEUI[8] = { 0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x24, 0x02, 0x98 };
uint8_t nodeAppEUI[8]    = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t nodeAppKey[16]   = { /* 16 bytes */ };
```

| Parameter | Value |
|---|---|
| Activation | OTAA |
| Class | C |
| Region | AS923 |
| Data Rate | DR_3 |
| TX Power | TX_POWER_5 |
| Duty Cycle | OFF |
| App Port | 1 |

### Uplink Payload Format (12 bytes)

| Byte | Content |
|---|---|
| 0–3 | Card DEC UID (big-endian uint32) |
| 4–11 | Card HEX UID (ASCII string) |

---

## Modbus Parameters

All values read as 32-bit IEEE 754 floats from Input Registers (Function Code 04).

| Parameter | Register |
|---|---|
| V1N (V) | 0 |
| V2N (V) | 2 |
| V3N (V) | 4 |
| Avg VLN (V) | 6 |
| V12 (V) | 8 |
| V23 (V) | 10 |
| V31 (V) | 12 |
| Avg VLL (V) | 14 |
| I1 (A) | 16 |
| I3 (A) | 20 |
| Avg I (A) | 22 |
| kW Ph1 | 24 |
| kW Ph2 | 26 |
| kW Ph3 | 28 |
| kVAr Ph3 | 42 |
| Total kW | 44 |
| Total kVAr | 46 |
| PF1 | 48 |
| PF2 | 50 |
| PF3 | 52 |
| Avg PF | 54 |
| Freq (Hz) | 56 |
| Total kWh | 58 |

---

## Troubleshooting

| Problem | Cause | Fix |
|---|---|---|
| `Include directory ... FT6336/src is not a directory` | Missing FT6336 library folder | Place `FT6336.h` and `FT6336.cpp` in `main/` and update `CMakeLists.txt` |
| `fatal: bad config line` git errors | Corrupted ESP-IDF submodule git config | Safe to ignore — does not affect build |
| `git rev-parse` not a git repository | Project not in a git repo | Safe to ignore — version shows as `1` |
| LoRa HW init failed | Wrong SPI pins or hardware issue | Check GPIO assignments in `initLoRa()` |
| Touch not responding | Wrong I2C pins or rotation | Verify `TOUCH_FT6336_SDA/SCL` and rotation value in `touch_init()` |
| Modbus all ERROR | Wrong baud/pins or DE/RE wiring | Check `MFM_RX_PIN`, `MFM_TX_PIN`, `MAX485_DE_RE` and meter slave ID |
| RFID not detected | Wrong UART pins or frame format | Verify `RX_PIN`/`TX_PIN` and that reader outputs 12-byte EM4100 frames |

---

## License

This project is proprietary firmware for internal use. All rights reserved.
