#pragma once

#include <Arduino.h>
#include <Wire.h>

/**
 * Register aliases
 * - Map friendly names to 16-bit register addresses
 * - Values from ST VL53L1X register map / ULD
 */
#define REG_CHIP_ID                     0x010F  /** IDENTIFICATION__MODEL_ID (hi byte at 0x010F) */
#define REG_CHIP_ID_EXPECTED            0xEACC  /** Expected model ID (0xEACC) */

#define REG_SYSTEM_START                0x0087  /** SYSTEM__MODE_START */
#define REG_GPIO_TIO_HV_STATUS          0x0031  /** GPIO__TIO_HV_STATUS */

#define REG_RESULT_DISTANCE             0x0096  /** RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 */
#define REG_RESULT_AMBIENT              0x0090  /** RESULT__AMBIENT_COUNT_RATE_MCPS_SD */
#define REG_RESULT_STATUS               0x0089  /** RESULT__RANGE_STATUS */
#define RESULT_RANGE_STATUS             0x0089  /** RESULT__RANGE_STATUS (interrupt/data-ready bits) */

#define REG_GPIO_CLEAR                  0x0086  /** SYSTEM__INTERRUPT_CLEAR */
#define REG_I2C_SLAVE_DEVICE_ADDR       0x0001  /** I2C_SLAVE__DEVICE_ADDRESS */

#define RANGE_CONFIG__TIMEOUT_MACROP_A_HI  0x005E
#define RANGE_CONFIG__TIMEOUT_MACROP_B_HI  0x0061

#define PHASECAL_CONFIG_TIMEOUT_MACROP       0x004B
#define RANGE_CONFIG_VCSEL_PERIOD_A          0x0060
#define RANGE_CONFIG_VCSEL_PERIOD_B          0x0063
#define RANGE_CONFIG_VALID_PHASE_HIGH        0x0069
#define SD_CONFIG_WOI_SD0                    0x0078
#define SD_CONFIG_INITIAL_PHASE_SD0          0x007A

#define SOFT_RESET                      0x001E  /** SOFT_RESET */
  /**
   * Distance mode selection
   * - SHORT: better close range performance
   * - MEDIUM: balanced
   * - LONG: best max range
   */
  enum Mode : uint8_t {
    SHORT = 0,
    MEDIUM = 1,
    LONG = 2,
  };

  /**S
   * Read status returned by read functions
   * - STATUS_OK: sample read and cache updated
   * - STATUS_TIMEOUT: sample not ready within timeout
   * - STATUS_ERROR: I2C/register access error
   */
  enum Status : uint8_t {
    STATUS_OK = 0,
    STATUS_TIMEOUT,
    STATUS_ERROR,
  };

/**
 * 7Semi VL53L1X driver
 * - Minimal register-level driver for VL53L1X
 * - Default config block write + basic continuous ranging
 * - 7Semi style: simple APIs, clear comments, no @param tags
 */
class VL53L1X_7Semi {
public:


/**
 * Constructor
 * - wirePort: I2C bus instance
 * - Sets default address to 0x29
 */
VL53L1X_7Semi(TwoWire &wirePort = Wire);

/**
 * Sensor begin (custom I2C pins + optional address)
 * - Sets I2C address used by this instance
 * - Starts I2C bus (custom pins where supported)
 * - Sets I2C clock if supported by core
 * - Performs soft reset and waits for boot
 * - Applies default configuration
 *
 * Notes
 * - For fixed-pin boards (Uno/Nano), SDA/SCL parameters are ignored
 * - Use 255 for sdaPin/sclPin to use default pins on supported boards
 */
bool begin(uint8_t i2cAddress = 0x29,
           uint8_t sdaPin = 255,
           uint8_t sclPin = 255,
           uint32_t i2cClockHz = 400000,
           uint32_t timeoutMs = 500);


  /**
   * Timeout (ms) used by blocking reads
   */
  void setTimeout(uint32_t timeoutMs);
  uint32_t getTimeout() const;

  /**
   * Set measurement range mode
   * - Writes a small set of registers for Short/Medium/Long presets
   * - Stops measurement before applying changes
   */
  bool setMeasurementRange(Mode mode);

  /**
   * Set timing budget (ms)
   * - Clamps between 15..500 ms
   * - Writes RANGE_CONFIG__TIMEOUT_MACROP_* registers
   */
  bool setMeasurementTime(uint16_t timingBudgetMs);

  /**
   * Get timing budget (ms)
   * - Reads RANGE_CONFIG__TIMEOUT_MACROP_* registers
   * - Converts back to approximate ms
   */
  uint16_t getMeasurementTime();

  /**
   * Start/stop continuous measurement
   */
  bool startMeasurement();
  bool stopMeasurement();

  /**
   * Blocking read
   * - Ensures ranging is running
   * - Waits until data ready or timeout
   * - Reads one sample then clears interrupt
   */
  Status readBlocking();

  /**
   * Continuous-read convenience
   * - Compatibility alias for older sketches
   * - Same behavior as readDistance()
   */
  uint16_t readContinous() { return readDistance(); }

  /**
   * Non-blocking read
   * - If ready reads distance + ambient + range status
   */
  Status readOnce();

  /**
   * Last read values
   */
  uint16_t readDistance() ;
  uint16_t getAmbient() const;
  uint8_t getStatus() const;

  /**
   * Change I2C address
   * - newAddress is 7-bit
   * - Do not left-shift (Wire uses 7-bit addressing)
   */
  bool changeI2CAddress(uint8_t newAddress);

  /**
   * Read the 16-bit chip ID
   * - Reads IDENTIFICATION__MODEL_ID high/low bytes
   */
  bool readChipID(uint16_t &chipId);

private:
  /**
   * Internal state
   */
  TwoWire *i2c;
  uint8_t i2c_addr;
  uint32_t timeout_ms;

  uint16_t distance_mm;
  uint16_t ambient;
  uint8_t range_status;

  bool is_ranging;
  uint8_t distance_range;

  /**
   * Low-level I2C helpers
   */
  bool writeReg(uint16_t reg, uint8_t value);
  bool write2Reg(uint16_t reg, uint16_t value);
  bool writeMulti(uint16_t reg, const uint8_t *data, uint8_t len);

  bool readReg(uint16_t reg, uint8_t &value);
  bool read2Reg(uint16_t reg, uint16_t &value);

  /**
   * Data-ready / interrupt helpers
   */
  bool waitDataReady(uint32_t timeoutMs);

  /**
   * Data-ready status check
   * - Reads RESULT__RANGE_STATUS interrupt bits [2:0]
   * - Non-zero indicates a new measurement is ready to read
   */
  bool isDataReady(bool &ready);

  bool clearInterrupt();

  /**
   * Default configuration
   * - Writes ST reference configuration block to 0x2D..0x87
   * - Does NOT start ranging automatically
   */
  bool applyDefaultConfiguration();
};
