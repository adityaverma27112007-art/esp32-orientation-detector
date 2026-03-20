# ESP32 Orientation Detector

An MPU6050 + OLED orientation detector built with ESP32.

## Hardware
- ESP32
- MPU6050 (GY-521)
- SSD1306 OLED (128x64)

## Wiring
| MPU6050 | ESP32 |
|---------|-------|
| SDA     | GPIO 21 |
| SCL     | GPIO 22 |
| VCC     | 3.3V |
| GND     | GND |

## Libraries
- Adafruit SSD1306
- Adafruit GFX

## Features
- 7 display pages (orientation, accel, gyro, angles, temp, g-force, events)
- Press BOOT button to cycle pages
- Spirit level bubble display

## Usage
1. Wire components as above
2. Upload `orientation_detector.ino`
3. Press BOOT button to change pages
