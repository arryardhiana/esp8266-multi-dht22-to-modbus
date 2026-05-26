# esp8266toModbus

Firmware for a **Wemos D1** (ESP8266) that reads an **Adafruit SHT3x** temperature/humidity sensor over I2C and publishes the values as a **Modbus RTU slave** on the hardware UART. A WiFi access point and small HTML dashboard are included for diagnostics.

## Hardware

| Function       | Pin            | Notes                                        |
| -------------- | -------------- | -------------------------------------------- |
| SHT3x SDA      | `D2` (GPIO4)   | I2C, default address `0x44`                  |
| SHT3x SCL      | `D1` (GPIO5)   |                                              |
| Modbus RTU TX  | `GPIO1` (TX0)  | `Serial`, 9600 8N1 — connect to RS-485 driver |
| Modbus RTU RX  | `GPIO3` (RX0)  | `Serial`                                     |
| Debug log TX   | `GPIO2` (TX1)  | `Serial1`, 115200, TX only                   |

> The USB-serial bridge on the Wemos D1 is wired to `Serial`, which this firmware reserves for Modbus. Use `Serial1` (GPIO2) with an external USB-TTL adapter to read logs without disturbing Modbus traffic.

## Build and flash

This is a [PlatformIO](https://platformio.org/) project — open it in VS Code with the PlatformIO extension, or use the CLI:

```bash
pio run                          # compile
pio run --target upload          # flash over USB
pio device monitor -b 115200     # read Serial1 diagnostic logs
```

The only declared dependency is `adafruit/Adafruit SHT31 Library` (see [platformio.ini](platformio.ini)).

## Modbus interface

- **Slave ID:** `3`
- **Frame:** 9600 baud, 8 data bits, 1 stop, no parity
- **Supported function codes:** `0x03` Read Holding Registers (all others return exception `0x01`)
- **Frame detection:** inter-byte gap of ≥3.5 character times (4 ms at 9600 baud), per Modbus RTU spec

### Holding register map

| Address  | Name        | Type   | Scaling                  |
| -------- | ----------- | ------ | ------------------------ |
| `0x0000` | Temperature | int16  | tenths °C, e.g. `289` = 28.9 °C, clamped to ±99.9 °C |
| `0x0001` | Humidity    | uint16 | tenths %RH, e.g. `654` = 65.4 %RH, clamped to 99.9 %RH |
| `0x0002`–`0x0013` | Reserved | — | always `0`               |

If the sensor is offline or returns NaN, the registers hold `0`. The sensor status is **not** exposed over Modbus — check the dashboard instead.

## WiFi dashboard

On boot the device starts a SoftAP:

- **SSID:** `WemosModbusAP`
- **Password:** `modbus123`
- **Dashboard:** `http://192.168.4.1/` (auto-refreshes every 5 s)

The page shows the latest temperature/humidity readings, sensor status, Modbus configuration, and a rolling log of the most recent 32 events.

## Configuration

All tunable constants live at the top of [src/main.cpp](src/main.cpp): pin assignments, Modbus slave ID and baud rate, sensor poll interval, AP credentials, and log buffer sizes. There is no runtime configuration interface — change the constants and re-flash.
