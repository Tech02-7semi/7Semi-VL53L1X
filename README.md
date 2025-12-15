# 7Semi-VL53L1X
Arduino library for the VL53L1X Time-of-Flight distance sensor.

Minimal register-level Arduino library for the ST VL53L1X
Time-of-Flight distance sensor.

This library focuses on:
- Simple APIs
- Clear behavior
- No hidden background tasks
- Polling-based data ready (no interrupt pin required)
  
![platform](https://img.shields.io/badge/platform-arduino-blue.svg)
![license](https://img.shields.io/badge/license-MIT-green.svg)
![status](https://img.shields.io/badge/status-active-brightgreen.svg)


Features
--------

- Soft reset and boot wait
- ST reference default configuration block
- Distance presets (Short / Medium / Long)
- Timing budget control
- Blocking and non-blocking read APIs
- Runtime I2C address change


Supported boards
----------------

- Any Arduino-compatible board with I2C (Wire)
- Tested with:
  - AVR
  - ESP32


How to add this library to Arduino
---------------------------------

Method 1: Arduino Library Manager (recommended)

1. Open Arduino IDE
2. Go to Sketch -> Include Library -> Manage Libraries…
3. Search for 7Semi VL53L1X
4. Click Install


Method 2: Install from ZIP

1. Download the library ZIP from GitHub
2. Open Arduino IDE
3. Go to Sketch -> Include Library -> Add .ZIP Library…
4. Select the downloaded ZIP file
5. Restart Arduino IDE if needed


Method 3: Manual installation

1. Download or clone the repository
2. Rename the folder to:

   7Semi_VL53L1X

3. Copy it into your Arduino libraries folder:

   Windows:
   Documents/Arduino/libraries/

   Linux / macOS:
   ~/Arduino/libraries/

4. Restart Arduino IDE


Hardware connection
-------------------


Basic wiring

VL53L1X pin -> Arduino pin
- VIN        -> 3.3V or 5V
- GND        -> GND
- SDA        -> SDA
- SCL        -> SCL


Notes
-----

- SDA and SCL pins depend on your board
  - Arduino Uno / Nano:
    SDA = A4, SCL = A5
  - ESP32:
    SDA = GPIO21, SCL = GPIO22

- XSHUT can be used to reset or manage multiple sensors,
  but it is not required
