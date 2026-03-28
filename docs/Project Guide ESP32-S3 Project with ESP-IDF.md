# Project Guide: ESP32-S3 Project with ESP-IDF

This file provides context for the Claude AI when working with this repository.

## Overview
This project is an embedded C/C++ application for the Espressif ESP32-S3 microcontroller, utilizing the [ESP-IDF (Espressif IoT Development Framework)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/index.html) as the primary framework. The project structure follows standard ESP-IDF conventions.

## Technology Stack
*   **Microcontroller:** ESP32-S3
*   **Framework:** ESP-IDF (version 5.x recommended)
*   **Language:** C/C++
*   **Build System:** CMake, Ninja
*   **Configuration:** Kconfig (accessed via `idf.py menuconfig`)

## Key Directories and Files
*   `/main`: Contains the core application logic (`app_main.c` or similar).
*   `/components`: Contains custom components specific to this project.
*   `CMakeLists.txt`: Main project build file.
*   `sdkconfig`: Generated configuration file (do not modify manually, use `menuconfig`).
*   `README.md`: Human-readable documentation and build instructions.

## Coding Standards
*   Follow the [Espressif IoT Development Framework Style Guide](https://docs.espressif.com).
*   Use standard C types and avoid non-standard libraries unless necessary.
*   Prioritize thread-safe functions when working within FreeRTOS tasks.

## Common Workflows & Commands (Linux/macOS)
*   **Set Target (once per project):** `idf.py set-target esp32s3`
*   **Configure Project:** `idf.py menuconfig`
*   **Build Project:** `idf.py build`
*   **Flash and Monitor:** `idf.py -p PORT flash monitor` (replace `PORT` with your device's actual port, e.g., `/dev/ttyUSB0` or `COM3`)
*   **Erase Flash:** `idf.py erase_flash`

## Instructions for Claude
*   When proposing code changes, ensure they are compatible with the ESP-IDF API and adhere to C/C++ best practices for embedded systems.
*   Focus on efficient memory usage and non-blocking operations.
*   Do not modify the `sdkconfig` file directly in your output. Instead, suggest specific Kconfig options and the values to set using `menuconfig`.