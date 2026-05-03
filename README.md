# RAK3112_RFID_RS485_Schematic_

## 📌 Overview
This project is a **custom embedded hardware design** based on the **RAK3112 (LoRa + ESP32-S3 module)** integrating multiple communication interfaces and peripherals.

It is designed for **industrial / IoT applications** requiring:
- RS485 communication
- RFID interfacing
- Display output
- User interaction (buttons, buzzer, RGB LED)

---

## ⚙️ Features
- ✅ RAK3112 (ESP32-S3 + LoRa)
- ✅ RS485 Communication (TP8485E)
- ✅ RS232 Interface (MAX3232)
- ✅ RFID Module Interface
- ✅ 2.8" SPI LCD (ILI9341)
- ✅ WS2812 NeoPixel LED
- ✅ Buzzer Output (Transistor driven)
- ✅ Multiple Push Buttons
- ✅ Dual Power Input:
  - USB-C (5V)
  - DC Jack (5V)

---

## 🔌 Power Architecture
- Input: **5V (USB-C or DC Jack)**
- Regulator: **RT6160A Buck-Boost Converter**
- Output:
  - 3.3V for MCU and logic
  - 5V rail for peripherals (NeoPixel, RFID, etc.)

---

## 🔗 Communication Interfaces

### RS485
- Transceiver: **TP8485E**
- Direction Control: GPIO-controlled (DE/RE)
- Protection & filtering included

### RS232
- IC: **MAX3232**
- Full UART level shifting support

### UART
- Used for:
  - RS485
  - RS232
  - External modules

---

## 🧩 Peripherals

### 📟 Display
- 2.8" SPI TFT (ILI9341)
- SPI Interface (MOSI, MISO, SCK, CS, DC, RST)

### 🎫 RFID
- External module via connector
- Powered from 5V

### 🔘 Buttons
- 3x User input buttons
- Connected to GPIOs with pull configuration

### 🔊 Buzzer
- Driven via **BC547 transistor**
- Controlled by GPIO

### 🌈 RGB LED
- WS2812 (NeoPixel)
- Single-wire data control

---

## 📂 Project Structure