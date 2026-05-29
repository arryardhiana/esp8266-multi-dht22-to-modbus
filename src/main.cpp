#include <Arduino.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------------------------------------------------------------------
// Pin map (Wemos D1 Mini). Lihat README untuk skematik & voltage divider.
// ---------------------------------------------------------------------------
constexpr uint8_t DHTPIN1 = D5;        // GPIO14 - DATA DHT22 #1
constexpr uint8_t DHTPIN2 = D6;        // GPIO12 - DATA DHT22 #2
constexpr uint8_t DHTTYPE = DHT22;
// Modbus RTU di hardware Serial: GPIO1 (TX0) -> RS485 driver DI, GPIO3 (RX0) <- RO.
// OLED dipasang di I2C default: SDA=D2 (GPIO4), SCL=D1 (GPIO5).
// Debug log di Serial1 (GPIO2, TX-only).

// ---------------------------------------------------------------------------
// Modbus RTU (di atas RS485 via SoftwareSerial).
// ---------------------------------------------------------------------------
constexpr uint8_t MODBUS_SLAVE_ID_DEFAULT = 10;
constexpr uint8_t MODBUS_SLAVE_ID_MIN = 1;
constexpr uint8_t MODBUS_SLAVE_ID_MAX = 247;
constexpr uint32_t MODBUS_BAUDRATE = 9600;
constexpr uint32_t MODBUS_FRAME_GAP_US = 4000;  // >3.5 char @ 9600 baud
constexpr size_t MODBUS_RX_BUFFER_SIZE = 64;

// ---------------------------------------------------------------------------
// Sensor poll.
// ---------------------------------------------------------------------------
constexpr uint32_t READ_INTERVAL = 5000;  // ms, > 2000ms minimum DHT22
constexpr uint8_t RETRY_MAX = 5;          // siklus gagal beruntun sebelum status error
constexpr float TEMP_MIN = -40.0f;
constexpr float TEMP_MAX = 80.0f;
constexpr float HUM_MIN = 0.0f;
constexpr float HUM_MAX = 100.0f;

// ---------------------------------------------------------------------------
// Diagnostik & WiFi.
// ---------------------------------------------------------------------------
constexpr bool ENABLE_SENSOR_SERIAL_LOG = true;
constexpr uint32_t SENSOR_LOG_BAUDRATE = 115200;
constexpr char WIFI_AP_SSID[] = "WemosModbusAP";
constexpr char WIFI_AP_PASSWORD[] = "modbus123";
constexpr size_t LOG_BUFFER_SIZE = 32;
constexpr size_t LOG_MESSAGE_MAX = 96;

// ---------------------------------------------------------------------------
// OLED SSD1306 128x64.
// ---------------------------------------------------------------------------
constexpr uint8_t OLED_I2C_ADDRESS = 0x3C;
constexpr uint8_t OLED_I2C_ADDRESS_ALT = 0x3D;  // sebagian modul pakai 0x3D
constexpr int16_t SCREEN_WIDTH = 128;
constexpr int16_t SCREEN_HEIGHT = 64;
constexpr int8_t OLED_RESET = -1;  // tanpa pin reset terpisah

// ---------------------------------------------------------------------------
// Layout EEPROM untuk slave ID persisten.
// ---------------------------------------------------------------------------
constexpr size_t EEPROM_SIZE = 16;
constexpr int EEPROM_ADDR_MAGIC = 0;     // 2 byte
constexpr int EEPROM_ADDR_SLAVE_ID = 2;  // 1 byte
constexpr uint16_t EEPROM_MAGIC = 0xCAFE;

// ---------------------------------------------------------------------------
// Holding register map (size 20, indeks 0..5 dipakai).
// ---------------------------------------------------------------------------
constexpr uint16_t REG_TEMP1 = 0x0000;    // suhu #1, signed 0.1 C
constexpr uint16_t REG_HUM1 = 0x0001;     // kelembaban #1, unsigned 0.1 %RH
constexpr uint16_t REG_TEMP2 = 0x0002;    // suhu #2, signed 0.1 C
constexpr uint16_t REG_HUM2 = 0x0003;     // kelembaban #2, unsigned 0.1 %RH
constexpr uint16_t REG_STATUS1 = 0x0004;  // status #1 (0=ok,1=init,2=read err)
constexpr uint16_t REG_STATUS2 = 0x0005;  // status #2
constexpr uint16_t HOLDING_REGISTER_COUNT = 20;

// ---------------------------------------------------------------------------
// State global.
// ---------------------------------------------------------------------------
DHT dht1(DHTPIN1, DHTTYPE);
DHT dht2(DHTPIN2, DHTTYPE);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
ESP8266WebServer webServer(80);

bool oledOnline = false;
uint32_t lastSensorSample = 0;
uint16_t holdingRegisters[HOLDING_REGISTER_COUNT] = {0};
float lastTemperature[2] = {NAN, NAN};
float lastHumidity[2] = {NAN, NAN};
uint8_t sensorStatus[2] = {1, 1};   // 0=ok, 1=init error, 2=read error
uint8_t sensorFailCount[2] = {0, 0};
uint8_t modbusSlaveId = MODBUS_SLAVE_ID_DEFAULT;

char logBuffer[LOG_BUFFER_SIZE][LOG_MESSAGE_MAX];
size_t logNextIndex = 0;
size_t logEntryCount = 0;

// Prototipe.
void pollSensor(uint8_t index, DHT& dht);
void updateHoldingRegisters();
void scanI2cBus();
bool initOled();
void updateOled();
void handleModbusInput();
void processModbusFrame(const uint8_t* frame, size_t length);
void sendHoldingRegisters(uint16_t startAddress, uint16_t count, uint8_t function);
void sendModbusException(uint8_t function, uint8_t exceptionCode);
uint16_t modbusCRC(const uint8_t* data, size_t length);
void logSensorMessage(const char* message);
void logSensorReading(uint8_t index, float temperatureC, float humidity);
void appendLog(const char* message);
void handleHttpRoot();
void handleHttpSaveSlaveId();
void loadSlaveIdFromEeprom();
bool saveSlaveIdToEeprom(uint8_t newId);
String formatFloat(float value, const char* unit);
String formatOledValue(float value, const char* unit);
const char* sensorStatusText(uint8_t status);

// Konversi float ke signed tenths di uint16_t register.
uint16_t encodeSignedTenths(float value) {
  if (isnan(value)) {
    return 0;
  }

  long scaled = lroundf(value * 10.0f);
  if (scaled > 999) {
    scaled = 999;
  } else if (scaled < -999) {
    scaled = -999;
  }

  return static_cast<uint16_t>(static_cast<int16_t>(scaled));
}

// Konversi float ke unsigned tenths di uint16_t register.
uint16_t encodeUnsignedTenths(float value) {
  if (isnan(value) || value < 0.0f) {
    return 0;
  }

  long scaled = lroundf(value * 10.0f);
  if (scaled > 999) {
    scaled = 999;
  }

  return static_cast<uint16_t>(scaled);
}

void setup() {
  Wire.begin(D2, D1);  // SDA, SCL untuk OLED

  if (ENABLE_SENSOR_SERIAL_LOG) {
    Serial1.begin(SENSOR_LOG_BAUDRATE);  // GPIO2, TX-only
    logSensorMessage("Sensor logger started");
  }

  EEPROM.begin(EEPROM_SIZE);
  loadSlaveIdFromEeprom();
  char slaveMsg[48];
  snprintf(slaveMsg, sizeof(slaveMsg), "Modbus slave ID = %u", modbusSlaveId);
  logSensorMessage(slaveMsg);

  dht1.begin();
  dht2.begin();
  logSensorMessage("DHT22 x2 initialized");

  Serial.begin(MODBUS_BAUDRATE, SERIAL_8N1);  // hardware UART0 -> RS485 driver
  logSensorMessage("Modbus on hardware Serial @9600");

  scanI2cBus();  // log alamat I2C yang terdeteksi untuk diagnosa OLED
  oledOnline = initOled();
  if (oledOnline) {
    logSensorMessage("OLED SSD1306 detected");
  } else {
    logSensorMessage("OLED not detected, continuing without display");
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
  IPAddress apIp = WiFi.softAPIP();
  char ipMessage[64];
  snprintf(ipMessage, sizeof(ipMessage), "AP up %s (%s)", WIFI_AP_SSID,
           apIp.toString().c_str());
  logSensorMessage(ipMessage);

  webServer.on("/", handleHttpRoot);
  webServer.on("/save-slave-id", HTTP_POST, handleHttpSaveSlaveId);
  webServer.begin();

  // Paksa pembacaan pertama segera di loop().
  lastSensorSample = millis() - READ_INTERVAL;
}

void loop() {
  handleModbusInput();
  webServer.handleClient();

  const uint32_t now = millis();
  if (now - lastSensorSample >= READ_INTERVAL) {
    lastSensorSample = now;

    pollSensor(0, dht1);
    pollSensor(1, dht2);
    updateHoldingRegisters();

    if (!oledOnline) {
      oledOnline = initOled();  // retry hot-plug
      if (oledOnline) {
        logSensorMessage("OLED back online");
      }
    }
    updateOled();
  }
}

// Baca satu sensor; validasi range. Tidak pakai delay() — retry dihitung sebagai
// jumlah siklus gagal beruntun, status jadi read-error (2) setelah RETRY_MAX.
void pollSensor(uint8_t index, DHT& dht) {
  const float temperatureC = dht.readTemperature();
  const float humidity = dht.readHumidity();

  const bool valid = !isnan(temperatureC) && !isnan(humidity) &&
                     temperatureC >= TEMP_MIN && temperatureC <= TEMP_MAX &&
                     humidity >= HUM_MIN && humidity <= HUM_MAX;

  if (valid) {
    lastTemperature[index] = temperatureC;
    lastHumidity[index] = humidity;
    sensorStatus[index] = 0;
    sensorFailCount[index] = 0;
    logSensorReading(index, temperatureC, humidity);
    return;
  }

  // Bacaan invalid: jangan update register dengan data buruk.
  if (sensorFailCount[index] < RETRY_MAX) {
    ++sensorFailCount[index];
  }
  if (sensorFailCount[index] >= RETRY_MAX) {
    sensorStatus[index] = 2;  // read error
  }

  char message[64];
  snprintf(message, sizeof(message), "DHT22 #%u read error (%u/%u)",
           static_cast<unsigned>(index + 1),
           static_cast<unsigned>(sensorFailCount[index]),
           static_cast<unsigned>(RETRY_MAX));
  logSensorMessage(message);
}

void updateHoldingRegisters() {
  holdingRegisters[REG_TEMP1] = encodeSignedTenths(lastTemperature[0]);
  holdingRegisters[REG_HUM1] = encodeUnsignedTenths(lastHumidity[0]);
  holdingRegisters[REG_TEMP2] = encodeSignedTenths(lastTemperature[1]);
  holdingRegisters[REG_HUM2] = encodeUnsignedTenths(lastHumidity[1]);
  holdingRegisters[REG_STATUS1] = sensorStatus[0];
  holdingRegisters[REG_STATUS2] = sensorStatus[1];
}

// Scan bus I2C dan log tiap alamat yang merespons (untuk diagnosa OLED).
void scanI2cBus() {
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      char message[24];
      snprintf(message, sizeof(message), "I2C device @0x%02X", addr);
      logSensorMessage(message);
      ++found;
    }
  }
  if (found == 0) {
    logSensorMessage("I2C: tidak ada device (cek wiring SDA D2/SCL D1)");
  }
}

// Coba init OLED di alamat primer lalu alternatif (0x3C / 0x3D).
bool initOled() {
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS) ||
      display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS_ALT)) {
    display.clearDisplay();
    display.display();
    return true;
  }
  return false;
}

void updateOled() {
  if (!oledOnline) {
    return;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println(F("Sensor 1"));
  display.print(F("Suhu : "));
  display.println(formatOledValue(lastTemperature[0], "C"));
  display.print(F("Humid: "));
  display.println(formatOledValue(lastHumidity[0], "%"));

  display.setCursor(0, 32);
  display.println(F("Sensor 2"));
  display.print(F("Suhu : "));
  display.println(formatOledValue(lastTemperature[1], "C"));
  display.print(F("Humid: "));
  display.println(formatOledValue(lastHumidity[1], "%"));

  display.setCursor(0, 56);
  char bottom[24];
  snprintf(bottom, sizeof(bottom), "ID:%u  RS485 OK", modbusSlaveId);
  display.print(bottom);

  display.display();
}

void handleModbusInput() {
  static uint8_t rxBuffer[MODBUS_RX_BUFFER_SIZE];
  static size_t rxLength = 0;
  static uint32_t lastByteMicros = 0;

  while (Serial.available()) {
    const int byteRead = Serial.read();
    if (byteRead < 0) {
      break;
    }

    if (rxLength < sizeof(rxBuffer)) {
      rxBuffer[rxLength++] = static_cast<uint8_t>(byteRead);
    }

    lastByteMicros = micros();
  }

  if (rxLength == 0) {
    return;
  }

  const uint32_t now = micros();
  if (now - lastByteMicros > MODBUS_FRAME_GAP_US) {
    processModbusFrame(rxBuffer, rxLength);
    rxLength = 0;
  }
}

void processModbusFrame(const uint8_t* frame, size_t length) {
  if (length < 8) {
    return;
  }

  const uint16_t crcReceived = static_cast<uint16_t>(frame[length - 2]) |
                               (static_cast<uint16_t>(frame[length - 1]) << 8);
  const uint16_t crcCalculated = modbusCRC(frame, length - 2);
  if (crcReceived != crcCalculated) {
    return;
  }

  if (frame[0] != modbusSlaveId) {
    return;
  }

  const uint8_t function = frame[1];
  if (function == 0x03) {  // Read Holding Registers
    const uint16_t startAddress = (static_cast<uint16_t>(frame[2]) << 8) | frame[3];
    const uint16_t registerCount = (static_cast<uint16_t>(frame[4]) << 8) | frame[5];

    if (registerCount == 0 || registerCount > HOLDING_REGISTER_COUNT) {
      sendModbusException(function, 0x03);  // Illegal data value
      return;
    }

    if (startAddress + registerCount > HOLDING_REGISTER_COUNT) {
      sendModbusException(function, 0x02);  // Illegal data address
      return;
    }

    sendHoldingRegisters(startAddress, registerCount, function);
  } else {
    sendModbusException(function, 0x01);  // Illegal function
  }
}

void sendHoldingRegisters(uint16_t startAddress, uint16_t count, uint8_t function) {
  const uint8_t byteCount = static_cast<uint8_t>(count * 2);
  uint8_t response[3 + 2 * HOLDING_REGISTER_COUNT + 2];
  response[0] = modbusSlaveId;
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
}

void sendModbusException(uint8_t function, uint8_t exceptionCode) {
  uint8_t response[5];
  response[0] = modbusSlaveId;
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

void logSensorMessage(const char* message) {
  appendLog(message);
  if (ENABLE_SENSOR_SERIAL_LOG) {
    Serial1.println(message);
  }
}

void logSensorReading(uint8_t index, float temperatureC, float humidity) {
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "DHT22 #%u -> %.2f C, %.2f %%RH",
           static_cast<unsigned>(index + 1), temperatureC, humidity);
  appendLog(buffer);
  if (ENABLE_SENSOR_SERIAL_LOG) {
    Serial1.println(buffer);
  }
}

void appendLog(const char* message) {
  const uint32_t timestamp = millis();
  char line[LOG_MESSAGE_MAX];
  snprintf(line, sizeof(line), "[%lu] %s", static_cast<unsigned long>(timestamp), message);
  strncpy(logBuffer[logNextIndex], line, LOG_MESSAGE_MAX - 1);
  logBuffer[logNextIndex][LOG_MESSAGE_MAX - 1] = '\0';
  logNextIndex = (logNextIndex + 1) % LOG_BUFFER_SIZE;
  if (logEntryCount < LOG_BUFFER_SIZE) {
    ++logEntryCount;
  }
}

void handleHttpRoot() {
  String page = F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
                  "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                  "<title>Wemos D1 Multi DHT22 Modbus</title>"
                  "<style>"
                  ":root{color-scheme:dark;--bg:#0f172a;--card:#1e293b;--accent:#38bdf8;"
                  "--text:#e2e8f0;--muted:#94a3b8;}"
                  "body{font-family:'Inter',system-ui,sans-serif;margin:16px;background:var(--bg);"
                  "color:var(--text);}"
                  ".card{background:var(--card);padding:20px;margin-bottom:16px;border-radius:14px;"
                  "box-shadow:0 12px 30px rgba(0,0,0,0.35);border:1px solid rgba(148,163,184,0.1);}"
                  ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:16px;}"
                  ".metric{font-size:2rem;font-weight:600;margin:6px 0;color:var(--accent);}"
                  ".label{font-size:0.85rem;color:var(--muted);text-transform:uppercase;letter-spacing:0.08em;}"
                  "h1,h2{margin-top:0;font-weight:600;}"
                  "ul{padding-left:20px;margin:0;max-height:280px;overflow:auto;}"
                  "li{margin-bottom:6px;font-family:\"SFMono-Regular\",Consolas,monospace;"
                  "color:var(--muted);}"
                  "a{color:var(--accent);}"
                  "</style></head><body>");

  page += F("<div class='card'><h1>Multi DHT22 Sensor Dashboard</h1>"
            "<div class='grid'>");
  page += "<div><div class='label'>Sensor 1 Temp</div><div class='metric'>" +
          formatFloat(lastTemperature[0], "°C") + "</div></div>";
  page += "<div><div class='label'>Sensor 1 Humidity</div><div class='metric'>" +
          formatFloat(lastHumidity[0], "%RH") + "</div></div>";
  page += "<div><div class='label'>Sensor 2 Temp</div><div class='metric'>" +
          formatFloat(lastTemperature[1], "°C") + "</div></div>";
  page += "<div><div class='label'>Sensor 2 Humidity</div><div class='metric'>" +
          formatFloat(lastHumidity[1], "%RH") + "</div></div>";
  page += "<div><div class='label'>Sensor 1 Status</div><div>" +
          String(sensorStatusText(sensorStatus[0])) + "</div></div>";
  page += "<div><div class='label'>Sensor 2 Status</div><div>" +
          String(sensorStatusText(sensorStatus[1])) + "</div></div>";
  page += "<div><div class='label'>Sample Interval</div><div>" +
          String(READ_INTERVAL / 1000.0f, 1) + " s</div></div>";
  page += "</div></div>";

  page += F("<div class='card'><h2>Modbus Configuration</h2>"
            "<div class='grid'>");
  page += "<div><div class='label'>Slave ID</div><div class='metric'>" + String(modbusSlaveId) + "</div></div>";
  page += "<div><div class='label'>Baud Rate</div><div class='metric'>" + String(MODBUS_BAUDRATE) + "</div></div>";
  page += "<div><div class='label'>Transport</div><div>Hardware Serial (GPIO1/3) → RS485</div></div>";
  page += "<div><div class='label'>Frame</div><div>8 data bits, 1 stop, no parity</div></div>";
  page += "<div><div class='label'>Register Scaling</div><div>Tenth units (289 = 28.9)</div></div>";
  page += "<div><div class='label'>Registers Used</div><div>0:T1 1:H1 2:T2 3:H2 4:St1 5:St2</div></div>";
  page += "</div>";

  page += F("<form method='POST' action='/save-slave-id' "
            "style='margin-top:16px;display:flex;gap:8px;flex-wrap:wrap;align-items:center;'>"
            "<label class='label' for='sid'>Set Slave ID (1-247)</label>"
            "<input id='sid' name='slave_id' type='number' min='1' max='247' required "
            "value='");
  page += String(modbusSlaveId);
  page += F("' style='padding:8px 10px;border-radius:8px;border:1px solid rgba(148,163,184,0.3);"
            "background:#0f172a;color:var(--text);width:100px;'>"
            "<button type='submit' style='padding:8px 16px;border-radius:8px;border:0;"
            "background:var(--accent);color:#0f172a;font-weight:600;cursor:pointer;'>"
            "Save to Flash</button></form></div>");

  page += F("<div class='card'><h2>Recent Logs</h2><ul>");
  if (logEntryCount == 0) {
    page += F("<li>No log entries yet</li>");
  } else {
    // Tampilkan dari terbaru ke terlama: entri terbaru ada di (logNextIndex - 1).
    for (size_t i = 0; i < logEntryCount; ++i) {
      size_t index = (logNextIndex + LOG_BUFFER_SIZE - 1 - i) % LOG_BUFFER_SIZE;
      page += "<li>" + String(logBuffer[index]) + "</li>";
    }
  }
  page += F("</ul></div>"
            "<footer style='text-align:center;color:var(--muted);font-size:0.8rem;"
            "margin:24px 0 8px;'>&copy; 2026 Arry &middot; ESP8266 Multi DHT22 Modbus RTU</footer>"
            "<script>"
            "(function(){"
            "var busy=false;"
            "document.addEventListener('focusin',function(e){"
            "if(e.target.matches('input,button,select,textarea'))busy=true;"
            "});"
            "document.addEventListener('focusout',function(){busy=false;});"
            "setInterval(function(){if(!busy)location.reload();},5000);"
            "})();"
            "</script>"
            "</body></html>");
  webServer.send(200, "text/html", page);
}

String formatFloat(float value, const char* unit) {
  if (isnan(value)) {
    return "N/A";
  }
  char buffer[24];
  snprintf(buffer, sizeof(buffer), "%.2f %s", value, unit);
  return String(buffer);
}

String formatOledValue(float value, const char* unit) {
  if (isnan(value)) {
    return String("N/A");
  }
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%.1f %s", value, unit);
  return String(buffer);
}

const char* sensorStatusText(uint8_t status) {
  switch (status) {
    case 0:
      return "OK";
    case 1:
      return "Sensor init error";
    case 2:
      return "Sensor read error";
    default:
      return "Unknown";
  }
}

void loadSlaveIdFromEeprom() {
  const uint16_t magic = (static_cast<uint16_t>(EEPROM.read(EEPROM_ADDR_MAGIC)) << 8) |
                         EEPROM.read(EEPROM_ADDR_MAGIC + 1);
  if (magic != EEPROM_MAGIC) {
    modbusSlaveId = MODBUS_SLAVE_ID_DEFAULT;
    logSensorMessage("EEPROM empty, using default slave ID");
    return;
  }

  const uint8_t stored = EEPROM.read(EEPROM_ADDR_SLAVE_ID);
  if (stored < MODBUS_SLAVE_ID_MIN || stored > MODBUS_SLAVE_ID_MAX) {
    modbusSlaveId = MODBUS_SLAVE_ID_DEFAULT;
    logSensorMessage("EEPROM slave ID out of range, using default");
    return;
  }

  modbusSlaveId = stored;
}

bool saveSlaveIdToEeprom(uint8_t newId) {
  if (newId < MODBUS_SLAVE_ID_MIN || newId > MODBUS_SLAVE_ID_MAX) {
    return false;
  }

  EEPROM.write(EEPROM_ADDR_MAGIC, static_cast<uint8_t>(EEPROM_MAGIC >> 8));
  EEPROM.write(EEPROM_ADDR_MAGIC + 1, static_cast<uint8_t>(EEPROM_MAGIC & 0xFF));
  EEPROM.write(EEPROM_ADDR_SLAVE_ID, newId);
  if (!EEPROM.commit()) {
    return false;
  }

  modbusSlaveId = newId;
  return true;
}

void handleHttpSaveSlaveId() {
  if (!webServer.hasArg("slave_id")) {
    webServer.send(400, "text/plain", "Missing slave_id");
    return;
  }

  const long parsed = webServer.arg("slave_id").toInt();
  if (parsed < MODBUS_SLAVE_ID_MIN || parsed > MODBUS_SLAVE_ID_MAX) {
    webServer.send(400, "text/plain", "Slave ID must be 1-247");
    return;
  }

  const uint8_t newId = static_cast<uint8_t>(parsed);
  if (!saveSlaveIdToEeprom(newId)) {
    webServer.send(500, "text/plain", "Failed to write EEPROM");
    return;
  }

  char msg[48];
  snprintf(msg, sizeof(msg), "Slave ID saved to flash: %u", newId);
  logSensorMessage(msg);

  webServer.sendHeader("Location", "/", true);
  webServer.send(303, "text/plain", "Saved");
}
