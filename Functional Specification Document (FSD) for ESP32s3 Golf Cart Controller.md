**Functional Specification Document (FSD) for ESP32s3 Golf Cart Controller**



**1. Introduction**

This document provides the functional specifications for the ESP32s3-based control unit for a two-wheel differential steering system intended for a modified golf pushcart. The system will allow manual remote control operation and a “follow me” autonomous mode.



**1.1 Project Overview**

- **Hardware Platform:** ESP32s3 microcontroller
- **Application:** Two-wheel motorized module for differential steering of a push golf cart
- **Control Modes:**
  1. Manual remote control via iPhone application
  2. Autonomous “follow me” mode using sensors



**1.2 Objectives**

- Provide reliable motor control for forward, reverse, and turning maneuvers
- Enable wireless communication with the iPhone app for manual control and telemetry
- Implement “follow me” functionality using onboard sensors (e.g., Bluetooth RSSI, camera, or UWB)
- Ensure low power consumption and safe operation



\---



**2. System Architecture**

1. **ESP32s3 MCU** as the primary controller
2. **Motor Driver Module** for two DC motors (left and right)
3. **Sensor Suite** (tentative):
   - UWB/Bluetooth for proximity and localization
   - IMU (accelerometer/gyroscope) for orientation
4. **Wireless Communication:**
   - Wi-Fi/Bluetooth LE for iPhone integration
5. **Battery Management:**
   - Li-ion battery pack with onboard voltage and current monitoring



\---



**3. Functional Requirements**



**3.1 Manual Remote Control Mode**

- **Command Reception:**
  - Receive directional and speed commands from iPhone app via BLE
- **Motor Control Logic:**
  - Differential steering control for forward, reverse, left, and right
- **Status Feedback:**
  - Send battery level and motor status to iPhone app



**3.2 “Follow Me” Autonomous Mode**

- **Tracking:**
  - Follow the user based on proximity signal (BLE RSSI/UWB)
- **Navigation:**
  - Adjust speed and direction to maintain safe distance
- **Obstacle Awareness (Optional Phase 2):**
  - Detect and stop for obstacles using IR/ultrasonic sensors



**3.3 Safety & Fail-Safe Features**

- **Emergency Stop:**
  - Immediate motor stop on signal loss or low battery
- **Overcurrent/Overheat Shutdown:**
  - Integrated with motor driver feedback



\---



**4. Interfaces**

1. **iPhone App ↔ ESP32s3:**
   - BLE for command/control and status feedback
2. **ESP32s3 ↔ Motor Drivers:**
   - 0 to 3.3 volt input for speed  and digital 0/1 input for direction
3. **ESP32s3 ↔ Sensors:**
   - I²C/SPI for IMU (BNO08x with 9-DoF)
   - UART/I²C for UWB/BLE modules



\---



**5. Performance Requirements**

- **Response Latency:** < 100 ms for control commands
- **Follow Me Range:** 1–15 meters
- **Battery Life:** Minimum 4 hours under mixed usage
- **Top Speed:** 5 km/h (adjustable)



\---



**6. Environmental & Physical Requirements**

- **Operating Temperature:** 0°C to 40°C
- **Weather Resistance:** Light splash resistance for outdoor use
- **Vibration Tolerance:** Must function on uneven terrain



\---



**7. Development & Testing**

- Prototype testing with manual mode first
- Incremental development of “follow me” mode
- Field tests on golf course terrain



\---



**8. Future Enhancements (Optional)**

- GPS integration for course mapping
- Obstacle avoidance using vision sensors
- Voice command recognition via iPhone



\---



**9. Acceptance Criteria**

- Stable remote manual control over BLE
- Accurate and stable “follow me” operation within defined range
- Compliance with safety and battery requirements