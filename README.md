# MyCaddie â€” ESP32-S3

Motorized two-wheel differential drive module for a push golf cart.
Supports manual BLE remote control via iPhone and an autonomous "follow me" mode.

## Hardware
- ESP32-S3 dev board
- Dual DC motor driver (e.g. DRV8833 or L298N)
- BNO08x 9-DoF IMU (I2C)
- Li-ion battery pack

## Prerequisites
- ESP-IDF v5.x installed and sourced
- USB connection to ESP32-S3

## Build & Flash
```bash
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
idf.py -p COM3 flash monitor
```

## Development Phases
1. Phase 1 - Motor control (LEDC PWM, differential steering)
2. Phase 2 - BLE GATT server + iPhone app integration
3. Phase 3 - BNO08x IMU for orientation
4. Phase 4 - Follow-me autonomous mode

## License
MIT
