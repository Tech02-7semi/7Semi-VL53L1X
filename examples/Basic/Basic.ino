/**
 * File: examples/CustomI2C_Pins_and_Wiring/CustomI2C_Pins_and_Wiring.ino
 * - Custom I2C pins + VL53L1X basic ranging
 * - Includes hardware connection notes at the top for quick wiring
 *
 * Hardware connection (I2C)
 * - VL53L1X VIN/VCC  -> 3.3V or 5V (check your module)
 * - VL53L1X GND      -> GND
 * - VL53L1X SDA      -> SDA pin of your board (or custom SDA below)
 * - VL53L1X SCL      -> SCL pin of your board (or custom SCL below)
 * - VL53L1X XSHUT    -> Optional (not required for single sensor)
 */

#include <7Semi_VL53L1X.h>

VL53L1X_7Semi tof;

void setup() {
  Serial.begin(115200);
  delay(200);

  uint8_t sensorAddr = 0x29;
  uint32_t i2cClockHz = 400000;
  uint32_t timeoutMs = 500;
  uint8_t sdaPin = 255;
  uint8_t sclPin = 255;
  /**
   * Custom I2C pins
   * - ESP32: common default is SDA=21, SCL=22 (can be changed)
   * - ESP8266: common default is SDA=4 (D2), SCL=5 (D1)
   * - Use 255 to use default pins on supported boards
   */


#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
  sdaPin = 21;
  sclPin = 22;
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ESP8266)
  sdaPin = 4;
  sclPin = 5;
#endif

  if (!tof.begin(sensorAddr, sdaPin, sclPin, i2cClockHz, timeoutMs)) {
    Serial.println("VL53L1X begin failed");
    while (1) {}
  }

  /**
   * Optional tuning
   * - Range preset changes internal timing and VCSEL periods
   * - Timing budget affects noise vs speed
   */
  tof.setMeasurementRange(LONG);
  tof.setMeasurementTime(50);
}

void loop() {
  /**
   * Blocking read
   * - Waits until sensor has a new sample or timeout
   */
  if (tof.readBlocking() == STATUS_OK) {
    Serial.print("Distance(mm): ");
    Serial.print(tof.readDistance());
    Serial.print("  Ambient: ");
    Serial.print(tof.getAmbient());
    Serial.print("  Status: ");
    Serial.println(tof.getStatus());
  } else {
    Serial.println("Read timeout/error");
  }

  delay(100);
}
