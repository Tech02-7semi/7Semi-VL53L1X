#include "7Semi_VL53L1X.h"

/**
 * Constructor
 * - Stores I2C port and resets runtime state
 */
VL53L1X_7Semi::VL53L1X_7Semi(TwoWire &wirePort)
    : i2c(&wirePort),
      i2c_addr(0x29),
      timeout_ms(500),
      distance_mm(0),
      ambient(0),
      range_status(0),
      is_ranging(false),
      distance_range(0)
{
}

/**
 * Sensor begin (custom I2C pins + optional address)
 * - Sets I2C address used by this instance
 * - Starts I2C bus (custom pins where supported)
 * - Performs sensor init sequence and loads default config
 */
bool VL53L1X_7Semi::begin(uint8_t i2cAddress,
                          uint8_t sdaPin,
                          uint8_t sclPin,
                          uint32_t i2cClockHz,
                          uint32_t timeoutMs)
{
  i2c_addr = (uint8_t)(i2cAddress & 0x7F); /* Force 7-bit */
  timeout_ms = timeoutMs;

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32) || defined(ARDUINO_ARCH_ESP8266) || defined(ESP8266)
  if (sdaPin != 255 && sclPin != 255)
  {
    i2c->begin(sdaPin, sclPin); /* Custom pins */
  }
  else
  {
    i2c->begin(); /* Default pins */
  }
#elif defined(ARDUINO_ARCH_RP2040)
  if (sdaPin != 255 && sclPin != 255)
  {
    i2c->setSDA(sdaPin); /* Pin select */
    i2c->setSCL(sclPin);
  }
  i2c->begin();
#else
  i2c->begin(); /* Fixed-pin boards */
#endif

#if defined(ARDUINO)
  i2c->setClock(i2cClockHz); /* Core-supported on most platforms */
#endif

  writeReg(SOFT_RESET, 0x00); /* Soft reset sequence */
  delay(2);
  writeReg(SOFT_RESET, 0x01); /* Release reset */
  delay(2);

  uint8_t status = 0;
  uint32_t start = millis();
  do
  {
    if (!readReg(REG_GPIO_TIO_HV_STATUS, status))
    {
      return false;
    }
    if ((millis() - start) > 1000UL)
    {
      break;
    }
  } while (status & 0x02);
  uint16_t chipId = 0;
  if (!readChipID(chipId))
  {
    return false;
  }

  if (chipId != REG_CHIP_ID_EXPECTED)
  {
    /**
     * Optional strict silicon check
     * - Enable return false if you want begin() to fail on unexpected ID
     */
    /* return false; */
  }

  if (!applyDefaultConfiguration())
  {
    return false;
  }

  return true;
}

/**
 * Set timeout (ms)
 * - Used by blocking waits for data-ready polling
 */
void VL53L1X_7Semi::setTimeout(uint32_t timeoutMs)
{
  timeout_ms = timeoutMs;
}

/**
 * Get timeout (ms)
 */
uint32_t VL53L1X_7Semi::getTimeout() const
{
  return timeout_ms;
}

/**
 * Set measurement range presets
 * - Stops measurement before applying changes
 */
bool VL53L1X_7Semi::setMeasurementRange(Mode mode)
{
  stopMeasurement();

  switch (mode)
  {
  case SHORT:
    writeReg(PHASECAL_CONFIG_TIMEOUT_MACROP, 0x14); /* ST tuning preset */
    writeReg(RANGE_CONFIG_VCSEL_PERIOD_A, 0x07);    /* VCSEL period A for short */
    writeReg(RANGE_CONFIG_VCSEL_PERIOD_B, 0x05);    /* VCSEL period B for short */
    writeReg(RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
    write2Reg(SD_CONFIG_WOI_SD0, 0x0705);
    write2Reg(SD_CONFIG_INITIAL_PHASE_SD0, 0x0606);
    distance_range = 0;
    break;

  case MEDIUM:
    writeReg(PHASECAL_CONFIG_TIMEOUT_MACROP, 0x0B); /* ST tuning preset */
    writeReg(RANGE_CONFIG_VCSEL_PERIOD_A, 0x0F);    /* VCSEL period A for medium/long */
    writeReg(RANGE_CONFIG_VCSEL_PERIOD_B, 0x0D);    /* VCSEL period B for medium/long */
    writeReg(RANGE_CONFIG_VALID_PHASE_HIGH, 0x68);
    write2Reg(SD_CONFIG_WOI_SD0, 0x0F0D);
    write2Reg(SD_CONFIG_INITIAL_PHASE_SD0, 0x0D0D);
    distance_range = 1;
    break;

  case LONG:
    writeReg(PHASECAL_CONFIG_TIMEOUT_MACROP, 0x0A); /* ST tuning preset */
    writeReg(RANGE_CONFIG_VCSEL_PERIOD_A, 0x0F);
    writeReg(RANGE_CONFIG_VCSEL_PERIOD_B, 0x0D);
    writeReg(RANGE_CONFIG_VALID_PHASE_HIGH, 0xB8);
    write2Reg(SD_CONFIG_WOI_SD0, 0x0F0D);
    write2Reg(SD_CONFIG_INITIAL_PHASE_SD0, 0x0E0E);
    distance_range = 2;
    break;
  }

  return true;
}

/**
 * Set timing budget (ms)
 * - Clamps between 15..500 ms
 * - Programs RANGE_CONFIG__TIMEOUT_MACROP_* registers
 */
bool VL53L1X_7Semi::setMeasurementTime(uint16_t timingBudgetMs)
{
  if (timingBudgetMs < 15)
    timingBudgetMs = 15;
  if (timingBudgetMs > 500)
    timingBudgetMs = 500;

  uint8_t vcselPeriodA = 0;
  uint8_t vcselPeriodB = 0;

  if (distance_range)
  {
    vcselPeriodA = 0x0F;
    vcselPeriodB = 0x0D;
  }
  else
  {
    vcselPeriodA = 0x07;
    vcselPeriodB = 0x05;
  }

  auto calcMacroPeriod = [](uint8_t vcsel_period_pclks) -> uint32_t
  {
    return ((uint32_t)2304 * (uint32_t)vcsel_period_pclks * 1655 + 500) * 2.17 / 1000;
  };

  uint32_t macroA_ns = calcMacroPeriod(vcselPeriodA);
  uint32_t macroB_ns = calcMacroPeriod(vcselPeriodB);

  const uint32_t overheadA_ns = 1320;
  const uint32_t overheadB_ns = 960;

  uint32_t timeoutMacroA = ((uint32_t)timingBudgetMs * 1000000UL - overheadA_ns) / macroA_ns;
  uint32_t timeoutMacroB = ((uint32_t)timingBudgetMs * 1000000UL - overheadB_ns) / macroB_ns;

  auto encodeTimeout = [](uint32_t timeoutMclks) -> uint16_t
  {
    if (timeoutMclks == 0)
      return 0;
    uint32_t lsByte = timeoutMclks - 1; /* Stored as (timeout - 1) */
    uint16_t msByte = 0;
    while (lsByte > 0xFF)
    {
      lsByte >>= 1; /* Normalize mantissa */
      msByte++;     /* Exponent increments */
    }
    return (uint16_t)((msByte << 8) | (lsByte & 0xFF)); /* {exponent, mantissa} */
  };

  if (!write2Reg(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, encodeTimeout(timeoutMacroA)))
    return false;
  if (!write2Reg(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, encodeTimeout(timeoutMacroB)))
    return false;

  return true;
}

/**
 * Get timing budget (ms)
 * - Reads RANGE_CONFIG__TIMEOUT_MACROP_* registers
 * - Converts back to approximate ms
 */
uint16_t VL53L1X_7Semi::getMeasurementTime()
{
  uint16_t regA = 0;
  uint16_t regB = 0;

  if (!read2Reg(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, regA))
    return 0;
  if (!read2Reg(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, regB))
    return 0;

  auto decodeTimeout = [](uint16_t regValue) -> uint32_t
  {
    uint16_t msByte = (regValue >> 8) & 0xFF;
    uint16_t lsByte = regValue & 0xFF;
    return ((uint32_t)(lsByte) + 1) << msByte; /* Inverse of {exponent, mantissa} */
  };

  uint32_t timeoutMacroA = decodeTimeout(regA);
  uint32_t timeoutMacroB = decodeTimeout(regB);

  uint8_t vcselPeriodA = 0;
  uint8_t vcselPeriodB = 0;

  if (distance_range)
  {
    vcselPeriodA = 0x0F;
    vcselPeriodB = 0x0D;
  }
  else
  {
    vcselPeriodA = 0x07;
    vcselPeriodB = 0x05;
  }

  auto calcMacroPeriod = [](uint8_t vcsel_period_pclks) -> uint32_t
  {
    return ((uint32_t)2304 * (uint32_t)vcsel_period_pclks * 1655 + 500) * 2.17 / 1000;
  };

  uint32_t macroA_ns = calcMacroPeriod(vcselPeriodA);
  uint32_t macroB_ns = calcMacroPeriod(vcselPeriodB);

  const uint32_t overheadA_ns = 1320;
  const uint32_t overheadB_ns = 960;

  uint32_t timingA_ms = (timeoutMacroA * macroA_ns + overheadA_ns) / 1000000UL;
  uint32_t timingB_ms = (timeoutMacroB * macroB_ns + overheadB_ns) / 1000000UL;

  return (uint16_t)((timingA_ms > timingB_ms) ? timingA_ms : timingB_ms);
}

/**
 * Start continuous measurement
 * - Sets SYSTEM__MODE_START to ranging mode
 */
bool VL53L1X_7Semi::startMeasurement()
{
  if (!writeReg(REG_SYSTEM_START, 0x40))
  {
    return false;
  }
  is_ranging = true;
  return true;
}

/**
 * Stop continuous measurement
 * - Clears SYSTEM__MODE_START and updates internal state
 */
bool VL53L1X_7Semi::stopMeasurement()
{
  if (!writeReg(REG_SYSTEM_START, 0x00))
  {
    return false;
  }
  is_ranging = false;
  return true;
}

/**
 * Blocking read
 * - Starts ranging if not running
 * - Waits until sample is ready or timeout
 * - Reads one sample then clears interrupt
 */
Status VL53L1X_7Semi::readBlocking()
{
  if (!is_ranging)
  {
    if (!startMeasurement())
    {
      return STATUS_ERROR;
    }
  }

  if (!waitDataReady(timeout_ms))
  {
    return STATUS_TIMEOUT;
  }

  Status s = readOnce();
  if (s != STATUS_OK)
  {
    return s;
  }

  clearInterrupt();
  return STATUS_OK;
}

/**
 * Non-blocking read
 * - Reads distance + ambient + range status if a sample is ready
 */
Status VL53L1X_7Semi::readOnce()
{
  bool ready = false;
  if (!isDataReady(ready))
  {
    return STATUS_ERROR;
  }

  if (!ready)
  {
    return STATUS_TIMEOUT;
  }

  uint16_t distance = 0;
  uint16_t current_ambient = 0;
  uint8_t status = 0;

  if (!read2Reg(REG_RESULT_DISTANCE, distance))
    return STATUS_ERROR;
  if (!read2Reg(REG_RESULT_AMBIENT, current_ambient))
    return STATUS_ERROR;
  if (!readReg(REG_RESULT_STATUS, status))
    return STATUS_ERROR;

  distance_mm = distance;
  ambient = current_ambient;
  range_status = (status & 0x1F); /* Status is in lower bits */

  return STATUS_OK;
}

/**
 * Read distance (mm)
 * - Performs a blocking read and returns cached distance
 */
uint16_t VL53L1X_7Semi::readDistance()
{
  (void)readBlocking();
  if (distance_mm < 60)
    distance_mm = 0;
  else
    distance_mm -= 60; // calibration offset

  return distance_mm;
}

/**
 * Get last ambient value
 */
uint16_t VL53L1X_7Semi::getAmbient() const
{
  return ambient;
}

/**
 * Get last range status
 */
uint8_t VL53L1X_7Semi::getStatus() const
{
  return range_status;
}

/**
 * Change I2C address
 * - Writes 7-bit address into device address register
 */
bool VL53L1X_7Semi::changeI2CAddress(uint8_t newAddress)
{
  uint8_t raw = (uint8_t)(newAddress & 0x7F); /* Force 7-bit */

  if (!writeReg(REG_I2C_SLAVE_DEVICE_ADDR, raw))
  {
    return false;
  }

  i2c_addr = newAddress;
  return true;
}

/**
 * Read chip ID
 * - Reads 16-bit IDENTIFICATION__MODEL_ID
 */
bool VL53L1X_7Semi::readChipID(uint16_t &chipId)
{
  uint8_t d = 0;
  chipId = 0;

  if (!readReg(REG_CHIP_ID, d))
    return false;
  chipId = ((uint16_t)d << 8);

  if (!readReg(REG_CHIP_ID + 1, d))
    return false;
  chipId |= d;

  return true;
}

/**
 * I2C write (8-bit)
 * - Uses 16-bit register addressing (MSB first)
 */
bool VL53L1X_7Semi::writeReg(uint16_t reg, uint8_t value)
{
  i2c->beginTransmission(i2c_addr);
  i2c->write((uint8_t)(reg >> 8));
  i2c->write((uint8_t)(reg & 0xFF));
  i2c->write(value);
  return (i2c->endTransmission() == 0);
}

/**
 * I2C write (16-bit)
 * - Uses 16-bit register addressing (MSB first)
 */
bool VL53L1X_7Semi::write2Reg(uint16_t reg, uint16_t value)
{
  i2c->beginTransmission(i2c_addr);
  i2c->write((uint8_t)(reg >> 8));
  i2c->write((uint8_t)(reg & 0xFF));
  i2c->write((uint8_t)(value >> 8));
  i2c->write((uint8_t)(value & 0xFF));
  return (i2c->endTransmission() == 0);
}

/**
 * I2C read (8-bit)
 * - Uses repeated start for register read sequence
 */
bool VL53L1X_7Semi::readReg(uint16_t reg, uint8_t &value)
{
  if (!i2c)
    return false;

  i2c->beginTransmission(i2c_addr);
  i2c->write((uint8_t)(reg >> 8));
  i2c->write((uint8_t)(reg & 0xFF));
  if (i2c->endTransmission(false) != 0)
    return false; /* Repeated start required */

  uint8_t count = i2c->requestFrom((int)i2c_addr, 1);
  if (count != 1)
    return false;

  value = i2c->read();
  return true;
}

/**
 * I2C read (16-bit)
 * - Uses repeated start for register read sequence
 */
bool VL53L1X_7Semi::read2Reg(uint16_t reg, uint16_t &value)
{
  if (!i2c)
    return false;

  i2c->beginTransmission(i2c_addr);
  i2c->write((uint8_t)(reg >> 8));
  i2c->write((uint8_t)(reg & 0xFF));
  if (i2c->endTransmission(false) != 0)
    return false; /* Repeated start required */

  uint8_t count = i2c->requestFrom((int)i2c_addr, (int)2);
  if (count != 2)
    return false;

  uint8_t msb = i2c->read();
  uint8_t lsb = i2c->read();
  value = ((uint16_t)msb << 8) | lsb;

  return true;
}

/**
 * Wait for data ready
 * - Polls data-ready state until timeout
 */
bool VL53L1X_7Semi::waitDataReady(uint32_t timeoutMs)
{
  uint32_t start = millis();
  bool ready = false;

  while ((millis() - start) < timeoutMs)
  {
    if (!isDataReady(ready))
    {
      return false;
    }
    if (ready)
    {
      return true;
    }
    delay(2);
  }

  return false;
}

/**
 * Data-ready status check
 * - Reads RESULT__RANGE_STATUS interrupt bits [2:0]
 * - Non-zero indicates a new measurement is ready to read
 */
bool VL53L1X_7Semi::isDataReady(bool &ready)
{
  uint8_t intStatus = 0;
  if (!readReg(RESULT_RANGE_STATUS, intStatus))
  {
    return false;
  }

  ready = ((intStatus & 0x07) != 0);
  return true;
}

/**
 * Clear interrupt
 * - Required to arm next measurement ready event
 */
bool VL53L1X_7Semi::clearInterrupt()
{
  return writeReg(REG_GPIO_CLEAR, 0x01);
}

/**
 * Apply default configuration
 * - Writes ST reference configuration block to 0x2D..0x87
 * - Does NOT start ranging automatically
 */
bool VL53L1X_7Semi::applyDefaultConfiguration()
{
  static const uint8_t DEFAULT_CFG[] = {
      0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01,
      0x01, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x20, 0x0b, 0x00, 0x00, 0x02, 0x0a, 0x21, 0x00, 0x00, 0x05, 0x00,
      0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00,
      0x00, 0x01, 0xcc, 0x0f, 0x01, 0xf1, 0x0d, 0x01, 0x68, 0x00, 0x80, 0x08,
      0xb8, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00, 0x00, 0x02, 0xc7, 0xff,
      0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};

  for (uint16_t addr = 0x2D; addr <= 0x87; addr++)
  {
    uint8_t value = DEFAULT_CFG[addr - 0x2D];
    if (!writeReg(addr, value))
    {
      return false;
    }
  }

  return true;
}

