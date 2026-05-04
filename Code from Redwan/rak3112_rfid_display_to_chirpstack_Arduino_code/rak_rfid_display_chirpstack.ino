#include <Arduino.h>
#include <SX126x-Arduino.h>
#include <mac/LoRaMacHelper.h>
#include <SPI.h>
#include <TFT_eSPI.h>

// ============================================================
// LoRaWAN Keys
// ============================================================
uint8_t nodeDeviceEUI[8] = {0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x24, 0x02, 0x98};
uint8_t nodeAppEUI[8]    = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t nodeAppKey[16]   = {0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x24, 0x02, 0x98,
                             0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x24, 0x02, 0x98};

// ============================================================
// SX1262 Pins (RAK3212)
// ============================================================
hw_config hwConfig;

// ============================================================
// RFID & Display
// ============================================================
#define RX_PIN 18
#define TX_PIN 17
HardwareSerial rfidSerial(1);
TFT_eSPI tft = TFT_eSPI();

// ============================================================
// App state
// ============================================================
#define LORAWAN_APP_PORT  1

static bool   joined       = false;
static String lastCard     = "";
static unsigned long lastScanTime = 0;
static const unsigned long DEBOUNCE_MS = 1500;
static int    scanCount    = 0;

// ============================================================
// LoRaWAN Callbacks
// ============================================================
static void lorawan_has_joined_handler(void) {
  Serial.println("✅ Joined ChirpStack!");
  joined = true;

  tft.fillRect(0, 0, 320, 45, TFT_DARKGREEN);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREEN);
  tft.setTextSize(2);
  tft.setCursor(60, 12);
  tft.println("LORAWAN JOINED!");
  delay(1500);
  showWaiting();
}

static void lorawan_join_failed_handler(void) {
  Serial.println("❌ Join failed!");
  tft.fillRect(0, 0, 320, 45, TFT_RED);
  tft.setTextColor(TFT_WHITE, TFT_RED);
  tft.setTextSize(2);
  tft.setCursor(70, 12);
  tft.println("JOIN FAILED!");
}

static void lorawan_rx_handler(lmh_app_data_t *app_data) {
  Serial.printf("📥 Downlink port %d, %d bytes\n",
                app_data->port, app_data->buffsize);
}

static void lorawan_confirm_class_handler(DeviceClass_t Class) {
  Serial.printf("Class: %c\n", "ABC"[Class]);
}

static void lorawan_unconf_finished(void) {
  Serial.println("📤 Uplink sent!");
}

static void lorawan_conf_finished(bool result) {
  Serial.printf("📤 Confirmed uplink %s\n", result ? "ACKed ✅" : "No ACK ❌");
}

static lmh_callback_t lora_callbacks = {
  BoardGetBatteryLevel,
  BoardGetUniqueId,
  BoardGetRandomSeed,
  lorawan_rx_handler,
  lorawan_has_joined_handler,
  lorawan_confirm_class_handler,
  lorawan_join_failed_handler,
  lorawan_unconf_finished,
  lorawan_conf_finished
};

static lmh_param_t lora_param_init = {
  false, DR_3, LORAWAN_PUBLIC_NETWORK, 8, TX_POWER_5, LORAWAN_DUTYCYCLE_OFF
};

// ============================================================
// Send RFID UID over LoRaWAN
// ============================================================
void sendRFID(String hexUID, unsigned long decUID) {
  if (!joined) {
    Serial.println("⚠️ Not joined, skipping uplink.");
    return;
  }

  // Payload: 4 bytes decimal UID (big-endian) + 8 bytes ASCII hex UID
  uint8_t payload[12];
  payload[0] = (decUID >> 24) & 0xFF;
  payload[1] = (decUID >> 16) & 0xFF;
  payload[2] = (decUID >>  8) & 0xFF;
  payload[3] = (decUID      ) & 0xFF;
  for (int i = 0; i < 8; i++) payload[4 + i] = hexUID[i];

  lmh_app_data_t m_lora_app_data = {payload, sizeof(payload), LORAWAN_APP_PORT, 0, 0};

  Serial.printf("📡 Sending UID: %s  DEC: %lu\n", hexUID.c_str(), decUID);
  lmh_error_status result = lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
  if (result == LMH_SUCCESS) {
    Serial.println("Queued OK.");
  } else {
    Serial.printf("Send failed: %d\n", result);
  }
}

// ============================================================
// Display: Waiting screen
// ============================================================
void showWaiting() {
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0, 0, 320, 45, 0x1082);
  tft.setTextColor(TFT_CYAN, 0x1082);
  tft.setTextSize(2);
  tft.setCursor(75, 12);
  tft.println("RFID SCANNER");

  tft.drawRoundRect(110, 65, 100, 70, 10, TFT_DARKGREY);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setTextSize(4);
  tft.setCursor(133, 80);
  tft.println("[]");

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(55, 155);
  tft.println("Scan a card...");

  // Show LoRaWAN join status
  tft.setTextColor(joined ? TFT_GREEN : TFT_YELLOW, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(10, 210);
  tft.println(joined ? "LoRa: Connected" : "LoRa: Joining...");

  if (scanCount > 0) {
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(10, 225);
    tft.print("Total scans: ");
    tft.println(scanCount);
  }
}

// ============================================================
// Display: Card detected screen
// ============================================================
void showCard(String hexUID, unsigned long decUID, bool loraSent) {
  tft.fillScreen(TFT_BLACK);

  tft.fillRect(0, 0, 320, 45, TFT_DARKGREEN);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREEN);
  tft.setTextSize(2);
  tft.setCursor(75, 12);
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

  // LoRa send status
  tft.setTextColor(loraSent ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(10, 205);
  tft.println(loraSent ? "📡 Sent via LoRa" : "⚠️  LoRa not joined");

  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(10, 220);
  tft.print("Scan #");
  tft.println(scanCount);
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  // Display
  pinMode(42, OUTPUT);
  digitalWrite(42, HIGH);
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(60, 100);
  tft.println("Initializing...");

  // RFID
  rfidSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  // LoRaWAN hardware
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
    Serial.println("❌ LoRa hardware init failed!");
    tft.fillScreen(TFT_RED);
    tft.setCursor(20, 100);
    tft.println("LoRa HW FAILED!");
    while (1);
  }

  lmh_setDevEui(nodeDeviceEUI);
  lmh_setAppEui(nodeAppEUI);
  lmh_setAppKey(nodeAppKey);

  if (lmh_init(&lora_callbacks, lora_param_init, true,
                CLASS_A, LORAMAC_REGION_AS923) != LMH_SUCCESS) {
    Serial.println("❌ LoRaWAN init failed!");
    while (1);
  }

  Serial.println("🔗 Joining ChirpStack...");
  lmh_join();

  showWaiting();
}

// ============================================================
// Loop
// ============================================================
void loop() {
  Radio.IrqProcess();

  static byte buffer[12];
  static byte idx = 0;

  while (rfidSerial.available()) {
    byte b = rfidSerial.read();

    Serial.print("0x");
    if (b < 0x10) Serial.print("0");
    Serial.print(b, HEX);
    Serial.print(" ");

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

          Serial.printf("\n>>> Valid tag!  HEX: %s  DEC: %lu\n", hexUID, decUID);

          if (uidStr != lastCard || (now - lastScanTime) > DEBOUNCE_MS) {
            scanCount++;
            lastCard = uidStr;
            lastScanTime = now;

            bool loraSent = joined;
            sendRFID(uidStr, decUID);
            showCard(uidStr, decUID, loraSent);

            delay(4000);
            while (rfidSerial.available()) rfidSerial.read();
            showWaiting();
          }

        } else {
          Serial.print("\n>>> Bad frame: ");
          for (int i = 0; i < 12; i++) {
            Serial.print(buffer[i], HEX);
            Serial.print(" ");
          }
          Serial.println();
        }
        idx = 0;
      }
    }
  }
}