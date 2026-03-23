# Functional Specification Document (FSD)
## ESP32-S3 Golf Cart Controller
**Version:** 1.1
**Last Updated:** March 2026
**Status:** In Development

---

## Revision History

| Version | Date | Changes |
|---|---|---|
| 1.0 | Initial | Original draft |
| 1.1 | March 2026 | Updated motor system to confirmed BLDC hardware: 36V system, 10" brushless hub motors, RioRand BLDC controllers, 3.3V→5V PWM level shifter added |

---

## 1. Introduction

This document provides the functional specifications for the ESP32-S3-based control unit for a two-wheel differential steering system intended for a modified golf pushcart. The system will allow manual remote control operation via iPhone and a "follow me" autonomous mode.

### 1.1 Project Overview

- **Hardware Platform:** ESP32-S3 microcontroller
- **Application:** Two-wheel motorized module for differential steering of a push golf cart
- **System Voltage:** 36V DC
- **Control Modes:**
  1. Manual remote control via iPhone application over BLE
  2. Autonomous "follow me" mode using proximity sensors

### 1.2 Objectives

- Provide reliable BLDC motor control for forward, reverse, and turning maneuvers
- Enable wireless communication with the iPhone app for manual control and telemetry
- Implement "follow me" functionality using onboard sensors (BLE RSSI, or UWB)
- Ensure safe operation including direction-change protection for BLDC controllers
- Maintain minimum 4-hour battery life under mixed usage

---

## 2. System Architecture

1. **ESP32-S3 MCU** — primary controller running FreeRTOS
2. **2× RioRand 350W BLDC Controllers** — one per drive wheel, handling 3-phase commutation
3. **2× 10" Brushless Hub Motors** — 36V, 350W, with Hall effect sensors
4. **3.3V → 5V PWM Level Shifter** — 2-channel, required between ESP32-S3 and RioRand PWM inputs
5. **Sensor Suite:**
   - BNO08x 9-DoF IMU (accelerometer / gyroscope / magnetometer) for orientation
   - BLE RSSI or UWB module for follow-me proximity tracking
6. **Wireless Communication:** Bluetooth LE for iPhone app integration
7. **Battery:** 36V Li-ion pack with ADC-based voltage monitoring on ESP32-S3

---

## 3. Hardware Specifications

### 3.1 Drive Motors

| Parameter | Value |
|---|---|
| Type | 3-phase brushless DC (BLDC) hub motor |
| Wheel diameter | 10 inches |
| Rated voltage | 36V DC |
| Rated power | 350W per motor |
| Hall sensors | Yes — 120° electrical angle |
| Hall signal lines | Ha, Hb, Hc |
| Hall power | 5V, GND |
| Phase wires | MA (U), MB (V), MC (W) |
| Quantity | 2 (left and right drive wheels) |

### 3.2 Motor Controllers

| Parameter | Value |
|---|---|
| Model | RioRand 350W 6-60V PWM DC Brushless Motor Speed Controller with Hall |
| Operating voltage | 6–60V (36V nominal in this system) |
| Continuous current | 16A rated, 20A peak |
| Speed control input | PWM signal, 2.5–5V amplitude |
| Direction control | Digital high/low signal |
| Hall sensor input | Ha, Hb, Hc + 5V + GND |
| Overcurrent protection | Built-in |
| Quantity | 2 (one per motor) |

> **Safety Note:** The RioRand uses non-delayed hard commutation for direction changes.
> Direction must never be reversed at full speed — the ESP32-S3 firmware must ramp
> motor speed below 50% before toggling the direction GPIO, or the power transistors
> and controller chip may be permanently damaged.

### 3.3 Level Shifter

| Parameter | Value |
|---|---|
| Type | BSS138-based bidirectional logic level shifter |
| Input voltage | 3.3V (from ESP32-S3) |
| Output voltage | 5V (to RioRand PWM input) |
| Channels required | 2 (one per motor controller) |
| Purpose | ESP32-S3 PWM output is 3.3V; RioRand requires 2.5–5V — level shifter ensures reliable operation |

### 3.4 Battery

| Parameter | Value |
|---|---|
| Chemistry | Li-ion |
| Nominal voltage | 36V |
| Recommended capacity | 10Ah minimum (for 4-hour runtime target) |
| Monitoring | ESP32-S3 ADC via resistor voltage divider |
| Low battery threshold | Configurable via menuconfig (default: 32V) |

---

## 4. Interfaces

### 4.1 iPhone App ↔ ESP32-S3
- **Protocol:** Bluetooth LE (BLE GATT)
- **Stack:** NimBLE (ESP-IDF)
- **Control characteristic:** Throttle + steering bytes at ~20Hz
- **Telemetry characteristic:** Battery voltage, motor status (notify)
- **Emergency stop:** Dedicated characteristic; also triggered on BLE disconnect

### 4.2 ESP32-S3 ↔ RioRand Motor Controllers

| Signal | ESP32-S3 GPIO | Direction | Notes |
|---|---|---|---|
| Left motor PWM speed | GPIO 18 | Output | Via 3.3V→5V level shifter |
| Left motor direction | GPIO 19 | Output | High = forward, Low = reverse |
| Right motor PWM speed | GPIO 20 | Output | Via 3.3V→5V level shifter |
| Right motor direction | GPIO 21 | Output | High = forward, Low = reverse |
| Common GND | GND | — | Shared between ESP32-S3 and both RioRand controllers |

- **PWM peripheral:** ESP32-S3 LEDC (LED Control) peripheral
- **PWM frequency:** 5 kHz
- **PWM resolution:** 10-bit (0–1023 duty cycle)
- **Speed range:** 0–100% (clamped by `GCC_MAX_SPEED_PCT` in menuconfig, default 80%)

### 4.3 ESP32-S3 ↔ BNO08x IMU
- **Protocol:** I²C
- **I²C address:** 0x4A (SA0 low) or 0x4B (SA0 high)
- **Report type:** ARVR Stabilised Rotation Vector
- **Poll rate:** ~100 Hz
- **Purpose:** Heading hold and straight-line correction during follow-me mode

### 4.4 ESP32-S3 ↔ Battery Monitor
- **Method:** Resistor voltage divider on ADC input
- **ADC pin:** TBD based on devkit layout
- **Sample rate:** 1 Hz
- **Action on low battery:** Ramp motors to stop, notify iPhone app, enter safe state

---

## 5. Functional Requirements

### 5.1 Manual Remote Control Mode

- **Command reception:** Throttle and steering values received from iPhone over BLE GATT at ~20Hz
- **Motor control:** Differential steering — `left_speed = throttle + steering`, `right_speed = throttle - steering`
- **Direction safety:** Firmware ramps speed below 50% before any direction change
- **Watchdog:** If no BLE command received within `GCC_BLE_WATCHDOG_MS` (default 500ms), motors stop immediately
- **Status feedback:** Battery voltage and connection status sent to iPhone every 1 second

### 5.2 "Follow Me" Autonomous Mode

- **Proximity sensing:** BLE RSSI (Phase 4 MVP) or UWB ranging (upgrade path)
- **Distance estimate:** Kalman-filtered RSSI → log-distance path loss model
- **Control loop:** 10 Hz — if distance > target + deadband → drive forward; if distance < target − deadband → stop
- **Steering:** IMU heading used to maintain straight-line tracking toward signal source
- **Target distance:** Configurable via menuconfig (`GCC_FOLLOW_ME_DISTANCE_CM`, default 150cm)
- **Safe stop range:** 1–15 meters operational range; stops outside this range

### 5.3 Safety & Fail-Safe Features

| Condition | Action |
|---|---|
| BLE signal lost | Stop both motors within 500ms |
| Battery voltage low | Ramp motors to stop, notify app |
| Direction change at speed | Firmware ramp-down before direction GPIO toggle |
| Overcurrent | RioRand hardware protection activates |
| Follow-me range exceeded (>15m) | Stop and wait |
| Emergency stop command | Immediate motor stop |

---

## 6. FreeRTOS Task Architecture

| Task | Core | Priority | Stack | Rate |
|---|---|---|---|---|
| `ble_task` | Core 0 | High | 4KB | Event-driven |
| `motor_ctrl_task` | Core 1 | High | 2KB | 50 Hz |
| `imu_task` | Core 1 | Medium | 2KB | 100 Hz |
| `follow_me_task` | Core 1 | Medium | 3KB | 10 Hz |
| `telemetry_task` | Core 0 | Low | 2KB | 1 Hz |

---

## 7. Performance Requirements

- **BLE command latency:** < 100ms end-to-end
- **Follow-me operating range:** 1–15 meters
- **Battery life:** Minimum 4 hours under mixed usage at 36V / 10Ah
- **Top speed:** 5 km/h (adjustable via `GCC_MAX_SPEED_PCT`)
- **PWM update rate:** 50 Hz motor control loop

---

## 8. Environmental & Physical Requirements

- **Operating temperature:** 0°C to 40°C
- **Weather resistance:** Light splash resistance for outdoor golf course use
- **Vibration tolerance:** Must function reliably on uneven turf terrain
- **Motor mounting:** Single-shaft hub motors mounted as rear drive wheels on modified push cart frame

---

## 9. Development Phases

| Phase | Scope | Status |
|---|---|---|
| Phase 1 | Motor driver — LEDC PWM, differential steering, direction safety ramp | Up next |
| Phase 2 | BLE GATT server, iPhone app, watchdog | Planned |
| Phase 3 | BNO08x IMU integration, heading hold | Planned |
| Phase 4 | Follow-me mode — BLE RSSI tracking, Kalman filter | Planned |

---

## 10. Future Enhancements (Optional)

- UWB ranging module to replace BLE RSSI for improved follow-me accuracy
- GPS integration for golf course mapping and hole navigation
- Obstacle avoidance using ultrasonic or IR sensors (Phase 2 stretch goal)
- Voice command recognition via iPhone
- Automatic bag weight load compensation via motor current sensing

---

## 11. Acceptance Criteria

- Both hub motors spin correctly under PWM control with direction safety enforced
- Stable BLE remote manual control from iPhone with < 100ms latency
- Emergency stop triggers reliably on BLE disconnect
- Accurate and stable "follow me" operation within 1–15 meter range
- Battery monitoring triggers safe stop below low-voltage threshold
- System operates for minimum 4 hours on a single 36V / 10Ah charge
