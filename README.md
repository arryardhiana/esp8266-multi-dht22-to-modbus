# esp8266 Multi DHT22 to Modbus

Firmware untuk **Wemos D1 Mini** (ESP8266) yang membaca **dua sensor DHT22** suhu/kelembaban dan mempublikasikan nilainya sebagai **Modbus RTU slave** di atas **hardware UART (`Serial`)** yang men-drive transceiver RS485. Dilengkapi **OLED SSD1306** untuk tampilan live, serta WiFi access point dengan dashboard HTML kecil untuk diagnostik.

## Hardware

| Fungsi          | Pin            | Catatan                                       |
| --------------- | -------------- | --------------------------------------------- |
| Modbus RTU TX   | `GPIO1` (TX0)  | `Serial`, 9600 8N1 → RS485 driver DI          |
| Modbus RTU RX   | `GPIO3` (RX0)  | `Serial` → RS485 driver RO                    |
| Debug log TX    | `GPIO2` (TX1)  | `Serial1`, 115200, TX only                    |
| OLED SCL        | `D1` (GPIO5)   | I2C, address `0x3C`                           |
| OLED SDA        | `D2` (GPIO4)   | I2C                                           |
| DHT22 #1 DATA   | `D5` (GPIO14)  | via voltage divider, kabel UTP 20m            |
| DHT22 #2 DATA   | `D6` (GPIO12)  | via voltage divider, kabel UTP 20m            |

> Catu daya: 3.3V untuk OLED; 5V untuk DHT22 + RS485 (pull-up 560Ω). DATA DHT22 wajib lewat voltage divider agar level GPIO ~3.2V (aman untuk ESP8266). Detail skematik ada di [CLAUDE.md](CLAUDE.md).

## Build and flash

[PlatformIO](https://platformio.org/) project — buka di VS Code dengan extension PlatformIO, atau pakai CLI:

```bash
pio run                          # compile
pio run --target upload          # flash over USB
pio device monitor -b 115200     # baca log diagnostik Serial1
```

> USB-serial bridge Wemos tersambung ke `Serial` yang dipakai Modbus. Untuk membaca log
> diagnostik `Serial1` (GPIO2, TX-only) gunakan adapter USB-TTL eksternal tanpa mengganggu
> traffic Modbus.

`lib_deps` (lihat [platformio.ini](platformio.ini)):

```ini
lib_deps =
    adafruit/DHT sensor library
    adafruit/Adafruit SSD1306
    adafruit/Adafruit GFX Library
```

## Modbus interface

- **Slave ID:** default `10`, dapat diatur **1–247** lewat dashboard dan disimpan persisten di EEPROM (lihat di bawah). Dibaca dari EEPROM saat boot.
- **Frame:** 9600 baud, 8 data bits, 1 stop, no parity (hardware `Serial` GPIO1/GPIO3 → RS485 driver)
- **Function codes:** `0x03` Read Holding Registers (lainnya return exception `0x01`)
- **Frame detection:** inter-byte gap ≥3.5 character times (4 ms @ 9600 baud), sesuai spec Modbus RTU

### Holding register map

| Address  | Nama              | Tipe   | Scaling                                  |
| -------- | ----------------- | ------ | ---------------------------------------- |
| `0x0000` | Suhu sensor #1    | int16  | tenths °C, `289` = 28.9 °C, clamp ±99.9 °C |
| `0x0001` | Kelembaban #1     | uint16 | tenths %RH, `654` = 65.4 %RH, clamp 99.9 %RH |
| `0x0002` | Suhu sensor #2    | int16  | tenths °C, clamp ±99.9 °C                |
| `0x0003` | Kelembaban #2     | uint16 | tenths %RH, clamp 99.9 %RH               |
| `0x0004` | Status sensor #1  | uint16 | 0=ok, 1=init error, 2=read error         |
| `0x0005` | Status sensor #2  | uint16 | 0=ok, 1=init error, 2=read error         |
| `0x0006`–`0x0013` | Reserved | —      | selalu `0`                               |

Jika sensor offline atau membaca NaN/out-of-range, register suhu/kelembaban tidak di-update dengan data invalid dan status di-set ke error.

## WiFi dashboard

Saat boot perangkat menyalakan SoftAP:

- **SSID:** `WemosModbusAP`
- **Password:** `modbus123`
- **Dashboard:** `http://192.168.4.1/` (auto-reload tiap 5 s, jeda saat input difokuskan)

Halaman menampilkan suhu/kelembaban terbaru, status sensor, konfigurasi Modbus, log 32 event terakhir, dan **form untuk mengubah Slave ID** (1–247) yang langsung disimpan ke flash.

## Configuration

Konstanta tunable ada di bagian atas [src/main.cpp](src/main.cpp): pin, baud rate Modbus, interval baca sensor, kredensial AP, ukuran log buffer, serta `MODBUS_SLAVE_ID_DEFAULT` dan layout EEPROM. Slave ID juga bisa diubah runtime lewat dashboard tanpa re-flash.
