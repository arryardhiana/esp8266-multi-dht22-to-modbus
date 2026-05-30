// =============================================================================
// TEST-RUN: firmware minimal Wemos D1 Mini + modul RS485 saja.
// Tanpa DHT22 / OLED / EEPROM. Nilai suhu & kelembaban di-MOCK supaya kita bisa
// memastikan jalur Modbus RTU (hardware Serial -> RS485) benar-benar bisa dibaca
// master, lepas dari sensor.
//
// Modbus: slave RTU di hardware Serial (GPIO1 TX0 / GPIO3 RX0), 9600 8N1.
// Debug: Serial1 (GPIO2, TX-only) + ring buffer di dashboard http://192.168.4.1/
// =============================================================================
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// --- Konfigurasi Modbus ---
constexpr uint8_t MODBUS_SLAVE_ID = 1;  // GANTI agar cocok dengan master Anda
constexpr uint32_t MODBUS_BAUDRATE = 9600;
constexpr uint32_t MODBUS_FRAME_GAP_US = 4000;  // >3.5 char @ 9600 baud
constexpr size_t MODBUS_RX_BUFFER_SIZE = 64;
constexpr bool ENABLE_MODBUS_RX_LOG = true;
// Tes arah TX: kirim teks "HB:<detik>" ke bus RS485 tiap 2 dtk. Hubungkan ujung
// bus ke PC (USB-RS485) buka serial terminal 9600 8N1: kalau "HB:n" muncul ->
// jalur ESP TX -> modul -> bus sehat. Set false untuk operasi normal.
constexpr bool ENABLE_TX_HEARTBEAT = false;

// --- Debug / WiFi ---
constexpr bool ENABLE_SERIAL_LOG = true;
constexpr uint32_t SERIAL_LOG_BAUDRATE = 115200;
constexpr char WIFI_AP_SSID[] = "WemosModbusTest";
constexpr char WIFI_AP_PASSWORD[] = "modbus123";
constexpr size_t LOG_BUFFER_SIZE = 32;
constexpr size_t LOG_MESSAGE_MAX = 96;

// --- Register map (mock) ---
constexpr uint16_t REG_TEMPERATURE = 0x0000;  // signed 0.1 C
constexpr uint16_t REG_HUMIDITY = 0x0001;     // unsigned 0.1 %RH
constexpr uint16_t REG_UPTIME = 0x0002;       // detik sejak boot (liveness)
constexpr uint16_t HOLDING_REGISTER_COUNT = 8;
constexpr uint32_t MOCK_UPDATE_INTERVAL_MS = 1000;

ESP8266WebServer webServer(80);
uint16_t holdingRegisters[HOLDING_REGISTER_COUNT] = {0};
uint32_t modbusRxByteCount = 0;
uint32_t lastMockUpdate = 0;
float mockTemperature = NAN;
float mockHumidity = NAN;

char logBuffer[LOG_BUFFER_SIZE][LOG_MESSAGE_MAX];
size_t logNextIndex = 0;
size_t logEntryCount = 0;

void handleModbusInput();
void processModbusFrame(const uint8_t* frame, size_t length);
void sendHoldingRegisters(uint16_t startAddress, uint16_t count, uint8_t function);
void sendModbusException(uint8_t function, uint8_t exceptionCode);
uint16_t modbusCRC(const uint8_t* data, size_t length);
void updateMockRegisters();
void logMessage(const char* message);
void appendLog(const char* message);
void handleHttpRoot();
String formatFloat(float value, const char* unit);

uint16_t encodeSignedTenths(float value) {
  long scaled = lroundf(value * 10.0f);
  if (scaled > 999) scaled = 999;
  else if (scaled < -999) scaled = -999;
  return static_cast<uint16_t>(static_cast<int16_t>(scaled));
}

uint16_t encodeUnsignedTenths(float value) {
  long scaled = lroundf(value * 10.0f);
  if (scaled < 0) scaled = 0;
  if (scaled > 999) scaled = 999;
  return static_cast<uint16_t>(scaled);
}

void setup() {
  if (ENABLE_SERIAL_LOG) {
    Serial1.begin(SERIAL_LOG_BAUDRATE);  // GPIO2, TX-only
    logMessage("TEST-RUN logger started");
  }

  Serial.begin(MODBUS_BAUDRATE, SERIAL_8N1);  // hardware UART0 -> RS485 driver
  char msg[48];
  snprintf(msg, sizeof(msg), "Modbus slave ID=%u @%lu 8N1", MODBUS_SLAVE_ID,
           static_cast<unsigned long>(MODBUS_BAUDRATE));
  logMessage(msg);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
  char ipMessage[64];
  snprintf(ipMessage, sizeof(ipMessage), "AP up %s (%s)", WIFI_AP_SSID,
           WiFi.softAPIP().toString().c_str());
  logMessage(ipMessage);

  webServer.on("/", handleHttpRoot);
  webServer.begin();

  lastMockUpdate = millis() - MOCK_UPDATE_INTERVAL_MS;
}

void loop() {
  handleModbusInput();
  webServer.handleClient();

  const uint32_t now = millis();

  if (ENABLE_TX_HEARTBEAT) {
    static uint32_t lastHeartbeat = 0;
    if (now - lastHeartbeat >= 2000) {
      lastHeartbeat = now;
      char hb[24];
      int n = snprintf(hb, sizeof(hb), "HB:%lu\r\n",
                       static_cast<unsigned long>(now / 1000));
      Serial.write(reinterpret_cast<const uint8_t*>(hb), n);
      logMessage("TX heartbeat dikirim");
    }
  }

  if (now - lastMockUpdate >= MOCK_UPDATE_INTERVAL_MS) {
    lastMockUpdate = now;
    updateMockRegisters();
  }
}

// Suhu mock naik 20.0 -> 29.9 C lalu berulang (per detik) supaya master melihat
// nilai yang berubah = bukti pembacaan fresh. Kelembaban tetap 55.5 %RH.
void updateMockRegisters() {
  const uint32_t seconds = millis() / 1000;
  mockTemperature = 20.0f + static_cast<float>(seconds % 100) / 10.0f;
  mockHumidity = 55.5f;
  holdingRegisters[REG_TEMPERATURE] = encodeSignedTenths(mockTemperature);
  holdingRegisters[REG_HUMIDITY] = encodeUnsignedTenths(mockHumidity);
  holdingRegisters[REG_UPTIME] = static_cast<uint16_t>(seconds);
}

void handleModbusInput() {
  static uint8_t rxBuffer[MODBUS_RX_BUFFER_SIZE];
  static size_t rxLength = 0;
  static uint32_t lastByteMicros = 0;

  while (Serial.available()) {
    const int byteRead = Serial.read();
    if (byteRead < 0) break;
    ++modbusRxByteCount;
    if (rxLength < sizeof(rxBuffer)) {
      rxBuffer[rxLength++] = static_cast<uint8_t>(byteRead);
    }
    lastByteMicros = micros();
  }

  if (rxLength == 0) return;

  if (micros() - lastByteMicros > MODBUS_FRAME_GAP_US) {
    processModbusFrame(rxBuffer, rxLength);
    rxLength = 0;
  }
}

void processModbusFrame(const uint8_t* frame, size_t length) {
  if (length < 8) {
    if (ENABLE_MODBUS_RX_LOG) {
      char message[40];
      snprintf(message, sizeof(message), "MB rx frame pendek (len=%u)",
               static_cast<unsigned>(length));
      logMessage(message);
    }
    return;
  }

  const uint16_t crcReceived = static_cast<uint16_t>(frame[length - 2]) |
                               (static_cast<uint16_t>(frame[length - 1]) << 8);
  const uint16_t crcCalculated = modbusCRC(frame, length - 2);

  if (ENABLE_MODBUS_RX_LOG) {
    char message[64];
    snprintf(message, sizeof(message), "MB rx id=%u fn=0x%02X len=%u crc=%s",
             static_cast<unsigned>(frame[0]), static_cast<unsigned>(frame[1]),
             static_cast<unsigned>(length),
             crcReceived == crcCalculated ? "OK" : "BAD");
    logMessage(message);
  }

  if (crcReceived != crcCalculated) return;

  if (frame[0] != MODBUS_SLAVE_ID) {
    if (ENABLE_MODBUS_RX_LOG) {
      char message[48];
      snprintf(message, sizeof(message), "MB id %u != slave %u, diabaikan",
               static_cast<unsigned>(frame[0]),
               static_cast<unsigned>(MODBUS_SLAVE_ID));
      logMessage(message);
    }
    return;
  }

  const uint8_t function = frame[1];
  if (function == 0x03) {  // Read Holding Registers
    const uint16_t startAddress = (static_cast<uint16_t>(frame[2]) << 8) | frame[3];
    const uint16_t registerCount = (static_cast<uint16_t>(frame[4]) << 8) | frame[5];

    if (registerCount == 0 || registerCount > HOLDING_REGISTER_COUNT) {
      sendModbusException(function, 0x03);
      return;
    }
    if (startAddress + registerCount > HOLDING_REGISTER_COUNT) {
      sendModbusException(function, 0x02);
      return;
    }
    sendHoldingRegisters(startAddress, registerCount, function);
  } else {
    sendModbusException(function, 0x01);
  }
}

void sendHoldingRegisters(uint16_t startAddress, uint16_t count, uint8_t function) {
  const uint8_t byteCount = static_cast<uint8_t>(count * 2);
  uint8_t response[3 + 2 * HOLDING_REGISTER_COUNT + 2];
  response[0] = MODBUS_SLAVE_ID;
  response[1] = function;
  response[2] = byteCount;

  for (uint16_t i = 0; i < count; ++i) {
    const uint16_t value = holdingRegisters[startAddress + i];
    response[3 + i * 2] = static_cast<uint8_t>(value >> 8);
    response[4 + i * 2] = static_cast<uint8_t>(value & 0xFF);
  }

  const uint16_t crc = modbusCRC(response, 3 + byteCount);
  response[3 + byteCount] = static_cast<uint8_t>(crc & 0xFF);
  response[4 + byteCount] = static_cast<uint8_t>(crc >> 8);
  Serial.write(response, 5 + byteCount);

  if (ENABLE_MODBUS_RX_LOG) {
    char message[48];
    snprintf(message, sizeof(message), "MB tx balas %u register", count);
    logMessage(message);
  }
}

void sendModbusException(uint8_t function, uint8_t exceptionCode) {
  uint8_t response[5];
  response[0] = MODBUS_SLAVE_ID;
  response[1] = function | 0x80;
  response[2] = exceptionCode;
  const uint16_t crc = modbusCRC(response, 3);
  response[3] = static_cast<uint8_t>(crc & 0xFF);
  response[4] = static_cast<uint8_t>(crc >> 8);
  Serial.write(response, sizeof(response));
}

uint16_t modbusCRC(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void logMessage(const char* message) {
  appendLog(message);
  if (ENABLE_SERIAL_LOG) {
    Serial1.println(message);
  }
}

void appendLog(const char* message) {
  char line[LOG_MESSAGE_MAX];
  snprintf(line, sizeof(line), "[%lu] %s",
           static_cast<unsigned long>(millis()), message);
  strncpy(logBuffer[logNextIndex], line, LOG_MESSAGE_MAX - 1);
  logBuffer[logNextIndex][LOG_MESSAGE_MAX - 1] = '\0';
  logNextIndex = (logNextIndex + 1) % LOG_BUFFER_SIZE;
  if (logEntryCount < LOG_BUFFER_SIZE) ++logEntryCount;
}

String formatFloat(float value, const char* unit) {
  if (isnan(value)) return "N/A";
  char buffer[24];
  snprintf(buffer, sizeof(buffer), "%.1f %s", value, unit);
  return String(buffer);
}

void handleHttpRoot() {
  String page = F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
                  "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                  "<title>Wemos Modbus TEST-RUN</title><style>"
                  ":root{color-scheme:dark;--bg:#0f172a;--card:#1e293b;--accent:#38bdf8;"
                  "--text:#e2e8f0;--muted:#94a3b8;}"
                  "body{font-family:system-ui,sans-serif;margin:16px;background:var(--bg);color:var(--text);}"
                  ".card{background:var(--card);padding:20px;margin-bottom:16px;border-radius:14px;}"
                  ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:16px;}"
                  ".metric{font-size:2rem;font-weight:600;margin:6px 0;color:var(--accent);}"
                  ".label{font-size:0.8rem;color:var(--muted);text-transform:uppercase;letter-spacing:0.08em;}"
                  "h1,h2{margin-top:0;}ul{padding-left:20px;max-height:280px;overflow:auto;}"
                  "li{font-family:monospace;color:var(--muted);margin-bottom:6px;}"
                  "</style></head><body>");

  page += F("<div class='card'><h1>Modbus TEST-RUN (mock)</h1><div class='grid'>");
  page += "<div><div class='label'>Suhu (mock)</div><div class='metric'>" +
          formatFloat(mockTemperature, "°C") + "</div></div>";
  page += "<div><div class='label'>Kelembaban (mock)</div><div class='metric'>" +
          formatFloat(mockHumidity, "%RH") + "</div></div>";
  page += "<div><div class='label'>Slave ID</div><div class='metric'>" +
          String(MODBUS_SLAVE_ID) + "</div></div>";
  page += "<div><div class='label'>Baud</div><div class='metric'>" +
          String(MODBUS_BAUDRATE) + "</div></div>";
  page += "<div><div class='label'>RX Bytes (GPIO3)</div><div class='metric'>" +
          String(modbusRxByteCount) + "</div></div>";
  page += "<div><div class='label'>Uptime reg [2]</div><div class='metric'>" +
          String(holdingRegisters[REG_UPTIME]) + "</div></div>";
  page += F("</div></div>");

  page += F("<div class='card'><h2>Log</h2><ul>");
  if (logEntryCount == 0) {
    page += F("<li>Belum ada log</li>");
  } else {
    for (size_t i = 0; i < logEntryCount; ++i) {
      size_t index = (logNextIndex + LOG_BUFFER_SIZE - 1 - i) % LOG_BUFFER_SIZE;
      page += "<li>" + String(logBuffer[index]) + "</li>";
    }
  }
  page += F("</ul></div>"
            "<script>setInterval(function(){location.reload();},2000);</script>"
            "</body></html>");
  webServer.send(200, "text/html", page);
}
