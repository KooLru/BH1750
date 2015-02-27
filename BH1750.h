/*

This is a library for the BH1750FVI Digital Light Sensor
breakout board.

The board uses I2C for communication. 2 pins are required to
interface to the device.

Datasheet:
http://rohmfs.rohm.com/en/products/databook/datasheet/ic/sensor/light/bh1750fvi-e.pdf

Written by Christopher Laws, March, 2013.

Enhanced by Mark Pruden (February 2015) to add the following Features:

- Support for setting the i2C Address of the Device
- Supports Changing the Measurement Time for the Device
- Supports reading the RAW value from the sensor
- Supports powering down, power up, and reset of the device
- Also:
- Renamed configure() to startMeasurement(), more aligned to its function
- Deprecated readLightLevel() use readLuxLevel() instead.

*/

#ifndef BH1750_h
#define BH1750_h

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include "Wire.h"

#define BH1750_DEBUG 0

// The Two Possible i2C Addresses
#define BH1750_I2CADDR_LOW  0x23
#define BH1750_I2CADDR_HIGH 0x5C

// Measurement Time that are commonly defined
#define BH1750_MEASURE_TIME_MIN     31
#define BH1750_MEASURE_TIME_0_5X    35
#define BH1750_MEASURE_TIME_1_0X    69
#define BH1750_MEASURE_TIME_DEFAULT 69
#define BH1750_MEASURE_TIME_1_5X    104
#define BH1750_MEASURE_TIME_2_0X    138
#define BH1750_MEASURE_TIME_2_5X    173
#define BH1750_MEASURE_TIME_3_0X    207
#define BH1750_MEASURE_TIME_3_5X    242
#define BH1750_MEASURE_TIME_MAX     254

// No active state
#define BH1750_POWER_DOWN 0x00

// Wating for measurment command
#define BH1750_POWER_ON 0x01

// Reset data register value - not accepted in POWER_DOWN mode
#define BH1750_RESET 0x07

// Start measurement at 1lx resolution. Measurement time is approx 120ms.
#define BH1750_CONTINUOUS_HIGH_RES_MODE  0x10

// Start measurement at 0.5lx resolution. Measurement time is approx 120ms.
#define BH1750_CONTINUOUS_HIGH_RES_MODE_2  0x11

// Start measurement at 4lx resolution. Measurement time is approx 16ms.
#define BH1750_CONTINUOUS_LOW_RES_MODE  0x13

// Start measurement at 1lx resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONE_TIME_HIGH_RES_MODE  0x20

// Start measurement at 0.5lx resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONE_TIME_HIGH_RES_MODE_2  0x21

// Start measurement at 1lx resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONE_TIME_LOW_RES_MODE  0x23

class BH1750 {
  public:

  // Constructs a New Instance, allows the Bus address to be specified
  BH1750(int16_t busAddress = BH1750_I2CADDR_LOW);

  void begin(uint8_t mode = BH1750_CONTINUOUS_HIGH_RES_MODE);
  
  // Deprecated (Renamed) - Use startMeasurement() instead.
  void configure(uint8_t mode);
  
  // Deprecated - Use readLuxLevel() instead.
  uint16_t readLightLevel(void);
  
  // allows the Measure ment time to be set, see BH1750FVI documentation
  void changeMeasurementTime(uint8_t measurementtime);
		
  // starts the measurement with the specified Mode.
  void startMeasurement(uint8_t mode);
		
  // Native read of the raw sensor data, NOT a LUX value
  uint16_t readSensor(void);
		
  // Return the Lux Level from the sensor.
  float readLuxLevel(void);
		
  // issues the power down command
  void powerDown(void);
		
  // issues the power on Command
  void powerOn(void);
		
  // issues the reset command.
  void reset(void);

 private:
  void write8(uint8_t data);
  
  int16_t i2cAddress;
  uint8_t currentmode;
  float highresmodefactor;
  float mtregfactor;
  uint8_t currentmtreg;

};

#endif