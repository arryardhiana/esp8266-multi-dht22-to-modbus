# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

PlatformIO/Arduino firmware for a **Wemos D1 Mini** (ESP8266, `d1_mini` board) that reads **two DHT22 temperature/humidity sensors** over single-wire protocol and exposes the readings as a **Modbus RTU slave** over the **hardware UART (`Serial`, GPIO1/GPIO3)** driving an RS485 transceiver. It also displays live readings on an **OLED SSD1306** display and brings up a WiFi access point with a small HTML dashboard for diagnostics.

All firmware lives in a single translation unit: [src/main.cpp](src/main.cpp). `include/`, `lib/`, and `test/` exist as PlatformIO scaffolding but contain only the default READMEs — there are no headers, libraries, or tests yet.

---

## Hardware

| Komponen       | Detail                                        |
|----------------|-----------------------------------------------|
| MCU            | Wemos D1 Mini (ESP8266, `d1_mini`)            |
| Sensor #1      | DHT22 module — D5 (GPIO14), kabel UTP 20m     |
| Sensor #2      | DHT22 module — D6 (GPIO12), kabel UTP 20m     |
| Display        | OLED SSD1306 128×64 — I2C (D1/D2)            |
| Komunikasi     | Modul RS485 ke hardware Serial (GPIO1/GPIO3)  |
| Kabel sensor   | UTP Cat6 20m dengan RJ45 keystone jack        |

### Pin Map

```
GPIO1 (TX0)  → RS485 driver DI  (Serial TX, Modbus)
GPIO3 (RX0)  → RS485 driver RO  (Serial RX, Modbus)
GPIO2 (TX1)  → debug log        (Serial1, 115200, TX-only)
D1  (GPIO5)  → OLED SCL  (I2C)
D2  (GPIO4)  → OLED SDA  (I2C)
D5  (GPIO14) → DHT22 #1 DATA  (via voltage divider)
D6  (GPIO12) → DHT22 #2 DATA  (via voltage divider)
3.3V         → VCC OLED
5V           → VCC DHT22, VCC RS485, pull-up 560Ω
GND          → semua komponen
```

### Voltage Divider DHT22 (per sensor)

Pull-up 5V ke DATA line + voltage divider agar GPIO 3.3V aman:

```
5V ── 560Ω ──┬── kabel UTP 20m ── DATA DHT22 module
             │
            1kΩ    →  Vgpio = 5 × 1k/(560+1k) = 3.2V ✓
             │
          GPIO Wemos (D5 / D6)
```

Pull-up 5.1kΩ bawaan module ter-parallel otomatis → efektif ~490Ω, cukup kuat untuk 20m.

---

## Build / flash / monitor

```bash
pio run                          # compile
pio run --target upload          # flash over USB
pio run --target clean           # clean build artifacts
pio device monitor -b 115200     # diagnostic log via Serial1 (GPIO2)
```

`lib_deps` yang dibutuhkan di `platformio.ini`:

```ini
lib_deps =
    adafruit/DHT sensor library
    adafruit/Adafruit SSD1306
    adafruit/Adafruit GFX Library
```

`ESP8266WiFi`, `ESP8266WebServer`, dan `Wire` ditarik otomatis dari platform espressif8266.

---

## Architecture notes that aren't obvious from the code

### Serial port usage — critical

ESP8266 punya dua UART, dibagi secara sengaja:

- **`Serial`** (GPIO1 TX / GPIO3 RX, 9600 8N1) — **jalur Modbus RTU** ke driver RS485. **Jangan** tambahkan `Serial.print` debug di mana pun — byte-nya akan diinterpretasi sebagai frame Modbus oleh master. USB-serial bridge Wemos juga tersambung ke `Serial`, jadi monitor USB tidak bisa dipakai bersamaan dengan Modbus.
- **`Serial1`** (GPIO2, TX-only, 115200) — **diagnostic channel**, gated oleh `ENABLE_SENSOR_SERIAL_LOG`. Semua logging lewat `logSensorMessage` / `logSensorReading`, yang menulis ke `Serial1` dan ke in-memory ring buffer untuk web dashboard.

### Modbus RTU — hardware Serial → RS485

Frame boundary dideteksi via inter-byte gap: `MODBUS_FRAME_GAP_US = 4000` (>3.5 character times at 9600 baud, sesuai spec Modbus RTU). `handleModbusInput` akumulasi byte ke static buffer, panggil `processModbusFrame` setelah gap berlalu.

Hanya function code `0x03` (Read Holding Registers) yang diimplementasi; lainnya return exception `0x01`.

**Slave ID — dinamis & persisten.** Tidak hard-coded:

- Default `MODBUS_SLAVE_ID_DEFAULT = 10`, range valid `MODBUS_SLAVE_ID_MIN..MAX` = `1..247`.
- Saat `setup()`, `loadSlaveIdFromEeprom()` membaca nilai dari EEPROM (divalidasi via magic number `0xCAFE`). Jika kosong/invalid/out-of-range → fallback ke default.
- Bisa diubah saat runtime lewat form di dashboard → `POST /save-slave-id` → `saveSlaveIdToEeprom()` menulis ke flash **dan** update variabel `modbusSlaveId` langsung (tanpa reboot).
- `processModbusFrame` membandingkan `frame[0]` terhadap `modbusSlaveId`.
- Layout EEPROM ada di konstanta `EEPROM_ADDR_*` di atas `main.cpp`.

**Register map** (`holdingRegisters[]`, size 20):

| Address | Isi                                              |
|---------|--------------------------------------------------|
| `0x0000` | Suhu sensor #1, signed tenths °C (`int16`, clamp ±999) |
| `0x0001` | Kelembaban sensor #1, unsigned tenths %RH (clamp 999) |
| `0x0002` | Suhu sensor #2, signed tenths °C (`int16`, clamp ±999) |
| `0x0003` | Kelembaban sensor #2, unsigned tenths %RH (clamp 999) |
| `0x0004` | Status sensor #1 (0=ok, 1=init error, 2=read error) |
| `0x0005` | Status sensor #2 (0=ok, 1=init error, 2=read error) |
| `0x0006`–`0x0013` | Reserved, selalu zero |

### DHT22 — sensor poll loop

- `loop()` membaca kedua sensor setiap `READ_INTERVAL` ms menggunakan `millis()` — **tidak boleh pakai `delay()`**.
- Setiap pembacaan menggunakan **retry logic maksimal `RETRY_MAX = 5`** sebelum dianggap gagal.
- Validasi range wajib: suhu -40°C s/d 80°C, kelembaban 0% s/d 100%.
- Jika NaN atau out-of-range → set `sensorStatus` = 2 (read error), **jangan update holding register** dengan data invalid.
- DHT22 butuh **minimum 2000ms** antar pembacaan per sensor.

```cpp
constexpr uint8_t  DHTPIN1        = 14;    // D5
constexpr uint8_t  DHTPIN2        = 12;    // D6
constexpr uint8_t  DHTTYPE        = DHT22;
constexpr uint32_t READ_INTERVAL  = 5000;  // ms
constexpr uint8_t  RETRY_MAX      = 5;
```

### OLED SSD1306 — layout display

I2C address default `0x3C`. Layout 128×64:

```
┌────────────────────────┐
│ Sensor 1               │
│ Suhu  : 28.5 °C        │
│ Humid : 65.2 %         │
│                        │
│ Sensor 2               │
│ Suhu  : 29.1 °C        │
│ Humid : 63.8 %         │
│ RS485: OK              │  ← status baris bawah
└────────────────────────┘
```

Jika OLED tidak terdeteksi saat `setup()` → lanjut tanpa crash, log error ke `Serial1`.

### WiFi / dashboard

Device berjalan sebagai SoftAP only (`WemosModbusAP` / `modbus123`, lihat konstanta `WIFI_AP_*`) — tidak ada STA mode. Dashboard di `http://192.168.4.1/` dibangun via string concatenation di `handleHttpRoot`, meta-refresh setiap 5 detik. Web server handling berbagi main loop dengan Modbus RX — handler yang panjang akan menunda Modbus response.

---

## Project structure

```
/
├── CLAUDE.md              ← file ini
├── README.md
├── platformio.ini         ← board: d1_mini, lib_deps
└── src/
    └── main.cpp           ← single translation unit, semua firmware
```

> **Catatan:** folder `docs/` (`wiring.md`, `protocol.md`) belum dibuat.

---

## Conventions for changes

- **Serial1 untuk debug** — semua output diagnostik lewat `logSensorMessage` (ring buffer + Serial1). Jangan tambah `Serial.print` langsung; `Serial` direservasi untuk Modbus RTU.
- **Modbus di hardware `Serial`** (GPIO1/GPIO3) → driver RS485. Jangan pindah balik ke SoftwareSerial kecuali ada perubahan arsitektur yang disengaja.
- **Holding registers** — jika menambah register, bump `HOLDING_REGISTER_COUNT` dan letakkan setelah reserved range agar address map tetap stabil.
- **Pin assignments dan protocol constants** — semua `constexpr` di bagian atas `main.cpp`. Edit di sana, bukan magic number di body.
- **Komentar dalam Bahasa Indonesia**.
- **camelCase** untuk variabel, **UPPER_SNAKE_CASE** untuk konstanta.
- **Tidak ada `delay()`** di loop utama — selalu gunakan `millis()`.
- Jika OLED atau salah satu sensor gagal init → lanjut tanpa crash, tampilkan status error.