# 📡 LoRaWAN RFID Scanner

An ESP32-based RFID card scanner that transmits tag UIDs over LoRaWAN (ChirpStack) and displays scan results on a TFT screen in real time.

---

## 🧰 Hardware Requirements

| Component | Details |
|---|---|
| Microcontroller | ESP32 (RAK3212 or compatible) |
| LoRa Radio | SX1262 |
| RFID Reader | UART-based (125 kHz / compatible) |
| Display | TFT screen (via `TFT_eSPI`) |

---

## 📦 Dependencies

Install the following libraries via the Arduino Library Manager or PlatformIO:

- [`SX126x-Arduino`](https://github.com/beegee-tokyo/SX126x-Arduino) — SX1262 LoRa driver
- [`LoRaMacHelper`](https://github.com/beegee-tokyo/SX126x-Arduino) — LoRaWAN MAC layer
- [`TFT_eSPI`](https://github.com/Bodmer/TFT_eSPI) — TFT display driver

---

## 🔌 Pin Configuration

### SX1262 LoRa Module

| Signal | GPIO |
|---|---|
| RESET | 8 |
| NSS (CS) | 7 |
| SCLK | 5 |
| MISO | 3 |
| MOSI | 6 |
| DIO1 | 47 |
| BUSY | 48 |

### RFID Reader (UART)

| Signal | GPIO |
|---|---|
| RX | 18 |
| TX | 17 |
| Baud Rate | 115200 |

> **Note:** The TFT backlight is controlled via GPIO 42 (set HIGH to enable).

---

## 🔑 LoRaWAN Configuration

Update the following keys in the source file to match your ChirpStack device registration:

```cpp
uint8_t nodeDeviceEUI[8] = { ... };  // Device EUI
uint8_t nodeAppEUI[8]    = { ... };  // Application EUI
uint8_t nodeAppKey[16]   = { ... };  // Application Key
```

The device joins via **OTAA** and operates in **Class A** mode on the **AS923** frequency plan.

---

## 📤 Uplink Payload Format

Each uplink is sent on **port 1** and contains **12 bytes**:

| Bytes | Content |
|---|---|
| 0–3 | Decimal UID as a 32-bit big-endian integer |
| 4–11 | ASCII hex string of the UID (8 characters) |

---

## 🖥️ Display Screens

The TFT display cycles between two views:

**Waiting Screen** — shown when idle
- Scanner title header
- "Scan a card..." prompt
- LoRaWAN join status (Joining… / Connected)
- Total scan count

**Card Detected Screen** — shown after a successful scan
- HEX UID (large, centered)
- Decimal UID
- LoRa transmission status
- Current scan number

---

## ⚙️ Application Settings

| Setting | Value |
|---|---|
| LoRaWAN Port | 1 |
| Data Rate | DR_3 |
| TX Power | TX_POWER_5 |
| Duty Cycle | OFF |
| Debounce Time | 1500 ms |
| Card display duration | 4000 ms |

---

## 🔄 How It Works

1. On boot, the device initializes the display, RFID serial port, and SX1262 radio.
2. It attempts to join the LoRaWAN network via OTAA.
3. Once joined, it listens for RFID frames on the hardware UART.
4. When a valid 12-byte RFID frame is received (`0x02 ... 0x0D 0x0A 0x03`), the UID is extracted.
5. A debounce check prevents duplicate sends from the same card within 1.5 seconds.
6. The UID is transmitted as an unconfirmed uplink and displayed on screen.

---

## 🛠️ Building & Flashing

This project uses **PlatformIO** (recommended) or the Arduino IDE.

### PlatformIO

```bash
# Build
pio run

# Upload
pio run --target upload

# Monitor serial output
pio device monitor --baud 115200
```

### Arduino IDE

1. Select your ESP32 board from **Tools > Board**.
2. Install all required libraries listed above.
3. Configure `TFT_eSPI` for your specific display in its `User_Setup.h`.
4. Compile and upload.

---

## 📝 License

This project is released under the MIT License. See `LICENSE` for details.
