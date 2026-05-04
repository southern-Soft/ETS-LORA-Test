#include <Arduino.h>
#include <SX126x-Arduino.h>
#include <mac/LoRaMacHelper.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <ModbusMaster.h>
#include "touch.h"

// ============================================================
// LoRaWAN Keys (ETS mode)
// ============================================================
uint8_t nodeDeviceEUI[8] = {0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x24, 0x02, 0x98};
uint8_t nodeAppEUI[8]    = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t nodeAppKey[16]   = {0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x24, 0x02, 0x98,
                             0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x24, 0x02, 0x98};
hw_config hwConfig;

// ============================================================
// RFID (ETS mode)
// ============================================================
#define RX_PIN 18
#define TX_PIN 17
HardwareSerial rfidSerial(1);

// ============================================================
// RS-485 / MFM384 (Careflow mode)
// ============================================================
#define MFM_RX_PIN      44
#define MFM_TX_PIN      43
#define MFM_BAUD        9600
#define MAX485_DE_RE    34
#define MFM_SLAVE_ID    1
#define INTER_READ_DELAY 100
HardwareSerial MfmSerial(2);
ModbusMaster   node;

struct ModbusParameter {
  const char* name;
  uint16_t    reg;
  float       value;
};

ModbusParameter params[] = {
  {"V1N (V)",     0,  0.0},
  {"V2N (V)",     2,  0.0},
  {"V3N (V)",     4,  0.0},
  {"Avg VLN (V)", 6,  0.0},
  {"V12 (V)",     8,  0.0},
  {"V23 (V)",     10, 0.0},
  {"V31 (V)",     12, 0.0},
  {"Avg VLL (V)", 14, 0.0},
  {"I1 (A)",      16, 0.0},
  {"I3 (A)",      20, 0.0},
  {"Avg I (A)",   22, 0.0},
  {"kW Ph1",      24, 0.0},
  {"kW Ph2",      26, 0.0},
  {"kW Ph3",      28, 0.0},
  {"kVAr Ph3",    42, 0.0},
  {"Total kW",    44, 0.0},
  {"Total kVAr",  46, 0.0},
  {"PF1",         48, 0.0},
  {"PF2",         50, 0.0},
  {"PF3",         52, 0.0},
  {"Avg PF",      54, 0.0},
  {"Freq (Hz)",   56, 0.0},
  {"Total kWh",   58, 0.0}
};
const int PARAM_COUNT = sizeof(params) / sizeof(params[0]);

// ============================================================
// Display
// ============================================================
TFT_eSPI tft = TFT_eSPI();

// ============================================================
// App State
// ============================================================
enum AppMode { MODE_SELECT, MODE_ETS, MODE_CAREFLOW };
AppMode currentMode = MODE_SELECT;

#define LORAWAN_APP_PORT 1
static bool          joined           = false;
static String        lastCard         = "";
static unsigned long lastScanTime     = 0;
static const unsigned long DEBOUNCE_MS = 1500;
static int           scanCount        = 0;

static bool          showingDownlink  = false;
static unsigned long downlinkShowTime = 0;
static const unsigned long DOWNLINK_DISPLAY_MS = 6000;

static bool     pendingDownlink    = false;
static uint8_t  dlBuffer[64];
static uint16_t dlLen              = 0;
static uint8_t  dlPort             = 0;
static bool     pendingJoinDisplay = false;

// Careflow scroll
static int  scrollOffset     = 0;       // first visible row index
static const int ROWS_VISIBLE = 8;      // rows shown at once
static bool careflowNeedsRedraw = true;
static unsigned long lastMfmRead = 0;
static const unsigned long MFM_INTERVAL = 5000; // re-read every 5s

// ============================================================
// Forward declarations
// ============================================================
void showModeSelect();
void showWaiting();
void showCard(String hexUID, unsigned long decUID, bool loraSent);
void showDownlink(uint8_t port, uint8_t *data, uint16_t len);
void drawCareflowList();
void readAllMfmParams();
void initLoRa();
void preTransmission();
void postTransmission();

// ============================================================
// RS-485 direction
// ============================================================
void preTransmission()  { digitalWrite(MAX485_DE_RE, HIGH); }
void postTransmission() { digitalWrite(MAX485_DE_RE, LOW);  }

float readFloatInputRegister(uint16_t regIndex) {
  uint8_t result = node.readInputRegisters(regIndex, 2);
  if (result == node.ku8MBSuccess) {
    uint16_t lo = node.getResponseBuffer(0);
    uint16_t hi = node.getResponseBuffer(1);
    uint32_t u32 = ((uint32_t)hi << 16) | lo;
    float f;
    memcpy(&f, &u32, sizeof(f));
    node.clearResponseBuffer();
    return f;
  }
  node.clearResponseBuffer();
  return NAN;
}

void readAllMfmParams() {
  for (int i = 0; i < PARAM_COUNT; i++) {
    params[i].value = readFloatInputRegister(params[i].reg);
    delay(INTER_READ_DELAY);
  }
}

// ============================================================
// LoRaWAN Callbacks
// ============================================================
static void lorawan_has_joined_handler(void) {
  Serial.println("✅ Joined ChirpStack!");
  joined = true;
  pendingJoinDisplay = true;
}

static void lorawan_join_failed_handler(void) {
  Serial.println("❌ Join failed!");
}

static void lorawan_rx_handler(lmh_app_data_t *app_data) {
  Serial.printf("📥 Downlink port %d, %d bytes\n", app_data->port, app_data->buffsize);
  dlPort = app_data->port;
  dlLen  = app_data->buffsize < 64 ? app_data->buffsize : 64;
  memcpy(dlBuffer, app_data->buffer, dlLen);
  pendingDownlink = true;
}

static void lorawan_confirm_class_handler(DeviceClass_t Class) {
  Serial.printf("✅ Class %c\n", "ABC"[Class]);
}
static void lorawan_unconf_finished(void)       { Serial.println("📤 Uplink sent!"); }
static void lorawan_conf_finished(bool result)  { Serial.printf("📤 ACK: %s\n", result ? "yes" : "no"); }

static lmh_callback_t lora_callbacks = {
  BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
  lorawan_rx_handler, lorawan_has_joined_handler,
  lorawan_confirm_class_handler, lorawan_join_failed_handler,
  lorawan_unconf_finished, lorawan_conf_finished
};

static lmh_param_t lora_param_init = {
  false, DR_3, LORAWAN_PUBLIC_NETWORK, 8, TX_POWER_5, LORAWAN_DUTYCYCLE_OFF
};

void initLoRa() {
  hwConfig.CHIP_TYPE           = SX1262_CHIP;
  hwConfig.PIN_LORA_RESET      = 8;
  hwConfig.PIN_LORA_NSS        = 7;
  hwConfig.PIN_LORA_SCLK       = 5;
  hwConfig.PIN_LORA_MISO       = 3;
  hwConfig.PIN_LORA_MOSI       = 6;
  hwConfig.PIN_LORA_DIO_1      = 47;
  hwConfig.PIN_LORA_BUSY       = 48;
  hwConfig.RADIO_TXEN          = -1;
  hwConfig.RADIO_RXEN          = -1;
  hwConfig.USE_DIO2_ANT_SWITCH = true;
  hwConfig.USE_DIO3_TCXO       = true;
  hwConfig.USE_DIO3_ANT_SWITCH = false;
  hwConfig.USE_LDO             = false;
  hwConfig.USE_RXEN_ANT_PWR    = false;

  if (lora_hardware_init(hwConfig) != 0) {
    Serial.println("❌ LoRa HW init failed!");
    tft.fillScreen(TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(20, 100);
    tft.println("LoRa HW FAILED!");
    while (1);
  }

  lmh_setDevEui(nodeDeviceEUI);
  lmh_setAppEui(nodeAppEUI);
  lmh_setAppKey(nodeAppKey);

  if (lmh_init(&lora_callbacks, lora_param_init, true,
                CLASS_C, LORAMAC_REGION_AS923) != LMH_SUCCESS) {
    Serial.println("❌ LoRaWAN init failed!");
    while (1);
  }
  lmh_join();
}

void sendRFID(String hexUID, unsigned long decUID) {
  if (!joined) return;
  uint8_t payload[12];
  payload[0] = (decUID >> 24) & 0xFF;
  payload[1] = (decUID >> 16) & 0xFF;
  payload[2] = (decUID >>  8) & 0xFF;
  payload[3] = (decUID      ) & 0xFF;
  for (int i = 0; i < 8; i++) payload[4 + i] = hexUID[i];
  lmh_app_data_t d = {payload, sizeof(payload), LORAWAN_APP_PORT, 0, 0};
  lmh_send(&d, LMH_UNCONFIRMED_MSG);
}

// ============================================================
// Display: Mode Selection Screen
// ============================================================
void showModeSelect() {
  tft.fillScreen(TFT_BLACK);

  // Header
  tft.fillRect(0, 0, 320, 50, 0x1082);
  tft.setTextColor(TFT_CYAN, 0x1082);
  tft.setTextSize(2);
  tft.setCursor(70, 16);
  tft.println("SELECT MODE");

  // Subtitle
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(80, 58);
  tft.println("Tap a mode to begin");

  // ETS Button  (left half)
  tft.fillRoundRect(15, 80, 135, 110, 12, 0x0340);   // dark teal
  tft.drawRoundRect(15, 80, 135, 110, 12, TFT_CYAN);
  tft.setTextColor(TFT_CYAN, 0x0340);
  tft.setTextSize(3);
  tft.setCursor(40, 112);
  tft.println("ETS");
  tft.setTextColor(TFT_WHITE, 0x0340);
  tft.setTextSize(1);
  tft.setCursor(22, 155);
  tft.println("RFID + LoRaWAN");
  tft.setCursor(30, 168);
  tft.println("Class C Gateway");

  // Careflow Button (right half)
  tft.fillRoundRect(170, 80, 135, 110, 12, 0x3800);  // dark orange
  tft.drawRoundRect(170, 80, 135, 110, 12, TFT_ORANGE);
  tft.setTextColor(TFT_ORANGE, 0x3800);
  tft.setTextSize(2);
  tft.setCursor(178, 108);
  tft.println("CARE-");
  tft.setCursor(178, 130);
  tft.println("FLOW");
  tft.setTextColor(TFT_WHITE, 0x3800);
  tft.setTextSize(1);
  tft.setCursor(178, 158);
  tft.println("MFM384 RS-485");
  tft.setCursor(178, 170);
  tft.println("Energy Monitor");

  // Footer
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(60, 215);
  tft.println("Touch screen to select");
}

// ============================================================
// Display: ETS — Waiting screen
// ============================================================
void showWaiting() {
  showingDownlink = false;
  tft.fillScreen(TFT_BLACK);

  tft.fillRect(0, 0, 320, 45, 0x1082);
  tft.setTextColor(TFT_CYAN, 0x1082);
  tft.setTextSize(2);
  tft.setCursor(65, 12);
  tft.println("ETS - SCANNER");

  tft.drawRoundRect(110, 65, 100, 70, 10, TFT_DARKGREY);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setTextSize(4);
  tft.setCursor(133, 80);
  tft.println("[]");

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(55, 155);
  tft.println("Scan a card...");

  tft.setTextColor(joined ? TFT_GREEN : TFT_YELLOW, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(10, 210);
  tft.println(joined ? "LoRa: Class C Ready" : "LoRa: Joining...");

  if (scanCount > 0) {
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(10, 225);
    tft.print("Total scans: ");
    tft.println(scanCount);
  }
}

// ============================================================
// Display: ETS — Card detected
// ============================================================
void showCard(String hexUID, unsigned long decUID, bool loraSent) {
  showingDownlink = false;
  tft.fillScreen(TFT_BLACK);

  tft.fillRect(0, 0, 320, 45, TFT_DARKGREEN);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREEN);
  tft.setTextSize(2);
  tft.setCursor(65, 12);
  tft.println("CARD DETECTED");

  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(20, 60);
  tft.println("HEX UID:");
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextSize(3);
  int16_t x = (320 - (hexUID.length() * 18)) / 2;
  tft.setCursor(x > 0 ? x : 10, 88);
  tft.println(hexUID);

  tft.drawFastHLine(10, 130, 300, TFT_DARKGREY);

  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(20, 140);
  tft.println("DEC UID:");
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(20, 165);
  tft.println(String(decUID));

  tft.drawFastHLine(10, 195, 300, TFT_DARKGREY);

  tft.setTextColor(loraSent ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(10, 205);
  tft.println(loraSent ? "Sent via LoRa Class C" : "LoRa not joined");

  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(10, 220);
  tft.print("Scan #"); tft.println(scanCount);
}

// ============================================================
// Display: ETS — Downlink screen
// ============================================================
void showDownlink(uint8_t port, uint8_t *data, uint16_t len) {
  tft.fillScreen(TFT_BLACK);

  tft.fillRect(0, 0, 320, 45, TFT_MAGENTA);
  tft.setTextColor(TFT_WHITE, TFT_MAGENTA);
  tft.setTextSize(2);
  tft.setCursor(50, 12);
  tft.println("GATEWAY MESSAGE");

  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(10, 52);
  tft.printf("Port: %d    Bytes: %d", port, len);

  tft.setCursor(10, 67);
  tft.println("RAW HEX:");

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextSize(1);
  int yPos = 78;
  String hexLine = "";
  for (int i = 0; i < len && i < 16; i++) {
    char buf[4];
    sprintf(buf, "%02X ", data[i]);
    hexLine += buf;
    if ((i + 1) % 8 == 0 || i == (int)len - 1) {
      tft.setCursor(10, yPos);
      tft.println(hexLine);
      hexLine = "";
      yPos += 14;
    }
  }

  tft.drawFastHLine(10, yPos + 4, 300, TFT_DARKGREY);
  yPos += 10;

  bool allPrintable = true;
  for (int i = 0; i < len; i++) {
    if (data[i] < 0x20 || data[i] > 0x7E) { allPrintable = false; break; }
  }

  if (allPrintable && len > 0) {
    String msg = "";
    for (int i = 0; i < len && i < 20; i++) msg += (char)data[i];
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(10, yPos + 4);
    tft.println("TEXT:");
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(10, yPos + 16);
    tft.println(msg);
  } else {
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(10, yPos + 4);
    tft.println("(Binary data)");
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setCursor(10, yPos + 18);
    String decVals = "DEC: ";
    for (int i = 0; i < len && i < 8; i++) {
      decVals += String(data[i]);
      if (i < (int)len - 1 && i < 7) decVals += " ";
    }
    tft.println(decVals);
  }

  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(10, 228);
  tft.println("Returns to scan in 6s...");
}

// ============================================================
// Display: Careflow — Scrollable parameter list
// ============================================================
void drawCareflowList() {
  tft.fillScreen(TFT_BLACK);

  // Header
  tft.fillRect(0, 0, 320, 40, 0x3800);
  tft.setTextColor(TFT_ORANGE, 0x3800);
  tft.setTextSize(2);
  tft.setCursor(60, 10);
  tft.println("CAREFLOW");

  // Scroll indicator
  tft.setTextColor(TFT_DARKGREY, 0x3800);
  tft.setTextSize(1);
  tft.setCursor(230, 14);
  tft.printf("%d/%d", scrollOffset + 1, PARAM_COUNT);

  // Column headers
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(10, 44);
  tft.print("PARAMETER");
  tft.setCursor(210, 44);
  tft.print("VALUE");
  tft.drawFastHLine(0, 54, 320, 0x1082);

  // Rows
  int rowH = 22;
  int startY = 58;

  for (int i = 0; i < ROWS_VISIBLE; i++) {
    int idx = scrollOffset + i;
    if (idx >= PARAM_COUNT) break;

    int y = startY + i * rowH;

    // Alternating row background
    if (i % 2 == 0) {
      tft.fillRect(0, y - 2, 320, rowH, 0x0841);
    }

    // Parameter name
    tft.setTextColor(TFT_LIGHTGREY, i % 2 == 0 ? 0x0841 : TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(10, y + 5);
    tft.print(params[idx].name);

    // Value
    if (isnan(params[idx].value)) {
      tft.setTextColor(TFT_RED, i % 2 == 0 ? 0x0841 : TFT_BLACK);
      tft.setCursor(210, y + 5);
      tft.print("ERROR");
    } else {
      tft.setTextColor(TFT_YELLOW, i % 2 == 0 ? 0x0841 : TFT_BLACK);
      tft.setCursor(210, y + 5);
      tft.printf("%.2f", params[idx].value);
    }
  }

  // Scroll hint at bottom
  tft.drawFastHLine(0, startY + ROWS_VISIBLE * rowH, 320, 0x1082);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setTextSize(1);

  bool canUp   = scrollOffset > 0;
  bool canDown = scrollOffset + ROWS_VISIBLE < PARAM_COUNT;

  tft.setCursor(10, startY + ROWS_VISIBLE * rowH + 6);
  if (canUp && canDown) tft.println("Swipe up/down to scroll");
  else if (canDown)     tft.println("Swipe up for more");
  else if (canUp)       tft.println("Swipe down for previous");
  else                  tft.println("All parameters shown");

  // Last read time
  tft.setCursor(190, startY + ROWS_VISIBLE * rowH + 6);
  tft.printf("Next: %ds", (int)((MFM_INTERVAL - (millis() - lastMfmRead)) / 1000) + 1);
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Display + backlight
  pinMode(42, OUTPUT);
  digitalWrite(42, HIGH);
  tft.init();
  tft.setRotation(1);

  // Touch (320x240, rotation 1)
  touch_init(320, 240, 1);

  // RS-485 pin
  pinMode(MAX485_DE_RE, OUTPUT);
  digitalWrite(MAX485_DE_RE, LOW);

  // MFM Serial (always init, used in Careflow)
  MfmSerial.begin(MFM_BAUD, SERIAL_8N1, MFM_RX_PIN, MFM_TX_PIN);
  node.begin(MFM_SLAVE_ID, MfmSerial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  // RFID Serial (always init, used in ETS)
  rfidSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  showModeSelect();
}

// ============================================================
// Loop
// ============================================================
void loop() {

  // ── MODE SELECTION ─────────────────────────────────────────
  if (currentMode == MODE_SELECT) {
    if (touch_touched()) {
      int tx = touch_last_x;
      int ty = touch_last_y;
      delay(50); // debounce

      // ETS button:      x=15..150,  y=80..190
      if (tx >= 15 && tx <= 150 && ty >= 80 && ty <= 190) {
        currentMode = MODE_ETS;

        // Show "initializing" while LoRa starts
        tft.fillScreen(TFT_BLACK);
        tft.fillRect(0, 0, 320, 45, 0x1082);
        tft.setTextColor(TFT_CYAN, 0x1082);
        tft.setTextSize(2);
        tft.setCursor(65, 12);
        tft.println("ETS - SCANNER");
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.setTextSize(2);
        tft.setCursor(55, 120);
        tft.println("Joining LoRa...");

        initLoRa();
        showWaiting();
      }

      // Careflow button: x=170..305, y=80..190
      if (tx >= 170 && tx <= 305 && ty >= 80 && ty <= 190) {
        currentMode = MODE_CAREFLOW;

        // First read
        tft.fillScreen(TFT_BLACK);
        tft.fillRect(0, 0, 320, 40, 0x3800);
        tft.setTextColor(TFT_ORANGE, 0x3800);
        tft.setTextSize(2);
        tft.setCursor(60, 10);
        tft.println("CAREFLOW");
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setTextSize(2);
        tft.setCursor(50, 120);
        tft.println("Reading MFM384...");

        readAllMfmParams();
        lastMfmRead = millis();
        scrollOffset = 0;
        careflowNeedsRedraw = true;
      }
    }
    return;
  }

  // ── ETS MODE ───────────────────────────────────────────────
  if (currentMode == MODE_ETS) {
    Radio.IrqProcess();

    // Handle join display
    if (pendingJoinDisplay) {
      pendingJoinDisplay = false;
      tft.fillRect(0, 0, 320, 45, TFT_DARKGREEN);
      tft.setTextColor(TFT_WHITE, TFT_DARKGREEN);
      tft.setTextSize(2);
      tft.setCursor(60, 12);
      tft.println("LORAWAN JOINED!");
      delay(1500);
      showWaiting();
    }

    // Handle downlink display
    if (pendingDownlink) {
      pendingDownlink  = false;
      showingDownlink  = true;
      downlinkShowTime = millis();
      showDownlink(dlPort, dlBuffer, dlLen);
    }

    // Auto-return from downlink
    if (showingDownlink && (millis() - downlinkShowTime) >= DOWNLINK_DISPLAY_MS) {
      showWaiting();
    }

    // RFID state machine
    static byte buffer[12];
    static byte idx = 0;

    while (rfidSerial.available()) {
      byte b = rfidSerial.read();

      if (b == 0x02) {
        idx = 0;
        buffer[idx++] = b;
      } else if (idx > 0 && idx < 12) {
        buffer[idx++] = b;

        if (idx == 12) {
          if (buffer[0] == 0x02 && buffer[9] == 0x0D &&
              buffer[10] == 0x0A && buffer[11] == 0x03) {

            char hexUID[9];
            for (int i = 0; i < 8; i++) hexUID[i] = buffer[i + 1];
            hexUID[8] = '\0';

            unsigned long decUID = strtoul(hexUID, NULL, 16);
            String uidStr = String(hexUID);
            unsigned long now = millis();

            Serial.printf(">>> Tag: %s  DEC: %lu\n", hexUID, decUID);

            if (uidStr != lastCard || (now - lastScanTime) > DEBOUNCE_MS) {
              scanCount++;
              lastCard     = uidStr;
              lastScanTime = now;

              bool loraSent = joined;
              sendRFID(uidStr, decUID);
              showCard(uidStr, decUID, loraSent);

              delay(4000);
              while (rfidSerial.available()) rfidSerial.read();
              showWaiting();
            }
          }
          idx = 0;
        }
      }
    }
    return;
  }

  // ── CAREFLOW MODE ──────────────────────────────────────────
  if (currentMode == MODE_CAREFLOW) {

    // Periodic MFM re-read
    if (millis() - lastMfmRead >= MFM_INTERVAL) {
      readAllMfmParams();
      lastMfmRead = millis();
      careflowNeedsRedraw = true;
    }

    // Redraw if needed
    if (careflowNeedsRedraw) {
      careflowNeedsRedraw = false;
      drawCareflowList();
    }

    // Touch for scrolling
    static int      touchStartY   = -1;
    static bool     touchActive   = false;

    if (touch_touched()) {
      if (!touchActive) {
        touchStartY = touch_last_y;
        touchActive = true;
      }
    } else {
      if (touchActive) {
        // Swipe detected on release
        int dy = touch_last_y - touchStartY;

        if (dy < -30) {
          // Swipe up → scroll down (show lower params)
          if (scrollOffset + ROWS_VISIBLE < PARAM_COUNT) {
            scrollOffset++;
            careflowNeedsRedraw = true;
          }
        } else if (dy > 30) {
          // Swipe down → scroll up (show higher params)
          if (scrollOffset > 0) {
            scrollOffset--;
            careflowNeedsRedraw = true;
          }
        }
        touchActive = false;
      }
    }
  }
}