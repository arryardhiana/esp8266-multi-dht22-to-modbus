# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

PlatformIO/Arduino firmware for a Wemos D1 (ESP8266, `d1_wroom_02` board) that reads an Adafruit SHT3x temperature/humidity sensor over I2C and exposes the readings as a **Modbus RTU slave** over UART. It also brings up a WiFi access point with a small HTML dashboard for diagnostics.

All firmware lives in a single translation unit: [src/main.cpp](src/main.cpp). `include/`, `lib/`, and `test/` exist as PlatformIO scaffolding but contain only the default READMEs — there are no headers, libraries, or tests yet.

## Build / flash / monitor

```bash
pio run                          # compile
pio run --target upload          # flash over USB
pio run --target clean           # clean build artifacts
pio device monitor -b 115200     # read diagnostic logs (see "Serial port usage" below)
```

The sole `lib_deps` entry in [platformio.ini](platformio.ini) is `adafruit/Adafruit SHT31 Library`. PlatformIO will pull `ESP8266WiFi`, `ESP8266WebServer`, and `Wire` from the espressif8266 platform automatically.

## Architecture notes that aren't obvious from the code

### Serial port usage — critical
The ESP8266 has two UARTs, and they are deliberately split:

- **`Serial`** (GPIO1 TX / GPIO3 RX, 9600 8N1) is the **Modbus RTU wire**. Do **not** add `Serial.print` debug calls anywhere — bytes you inject will be interpreted as Modbus frames by any connected master.
- **`Serial1`** (GPIO2, TX-only, 115200) is the **diagnostic channel**, gated by `ENABLE_SENSOR_SERIAL_LOG`. All logging goes through `logSensorMessage` / `logSensorReading`, which writes both to `Serial1` and to the in-memory ring buffer surfaced on the web dashboard.

### Modbus framing
Frame boundaries are detected by inter-byte gap, not length: `MODBUS_FRAME_GAP_US = 4000` (>3.5 character times at 9600 baud per the Modbus RTU spec). `handleModbusInput` accumulates bytes into a static buffer and calls `processModbusFrame` once the gap elapses. Only function code `0x03` (Read Holding Registers) is implemented; everything else returns exception `0x01`.

Slave ID is hard-coded to `3`. Register map (in `holdingRegisters[]`, size 20):
- `0x0000` — temperature, signed tenths °C (`int16` clamped to ±999, i.e. ±99.9 °C)
- `0x0001` — humidity, unsigned tenths %RH (clamped to 999)
- `0x0002`–`0x0013` — reserved, always zero

### Sensor poll loop
`loop()` calls `sht31.begin(0x44)` again on every poll when the sensor is offline — this is intentional retry behavior for hot-plug recovery. `sensorStatus` (0=ok, 1=init error, 2=read error) is surfaced on the dashboard but is **not** exposed as a Modbus register.

### WiFi / dashboard
The device runs as a SoftAP only (`WemosModbusAP` / `modbus123`, see `WIFI_AP_*` constants) — no STA mode, no captive portal. The dashboard at `http://192.168.4.1/` is built by string concatenation in `handleHttpRoot` and meta-refreshes every 5 seconds. Web server handling shares the main loop with Modbus RX, so long client handlers will delay Modbus responses.

## Conventions for changes

- Keep `Serial` reserved for Modbus. New debug output goes through `logSensorMessage` (ring buffer + `Serial1`).
- When adding holding registers, bump `HOLDING_REGISTER_COUNT` and place new entries after the existing reserved range so the address map stays stable.
- Pin assignments (`I2C_SDA_PIN`, `I2C_SCL_PIN`) and protocol constants live at the top of `main.cpp` — prefer editing those `constexpr`s over magic numbers in the body.
