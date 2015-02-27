/*

This is a library for the BH1750FVI Digital Light Sensor
breakout board.

The board uses I2C for communication. 2 pins are required to
interface to the device.


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

#include "BH1750.h"

BH1750::BH1750(int16_t busAddress) {
	i2cAddress = busAddress;
	highresmodefactor = 1.2;
	mtregfactor = 1.0;
	currentmode = BH1750_POWER_DOWN;
	currentmtreg = BH1750_MEASURE_TIME_DEFAULT;
}

void BH1750::begin(uint8_t mode) {

  Wire.begin();
  //write8(mode);
  startMeasurement(mode);
}


void BH1750::configure(uint8_t mode) {

	// apply a valid mode change
	startMeasurement(mode);

}

void BH1750::startMeasurement(uint8_t mode) {

    switch (mode) {
        case BH1750_CONTINUOUS_HIGH_RES_MODE:
        case BH1750_CONTINUOUS_HIGH_RES_MODE_2:
        case BH1750_CONTINUOUS_LOW_RES_MODE:
        case BH1750_ONE_TIME_HIGH_RES_MODE:
        case BH1750_ONE_TIME_HIGH_RES_MODE_2:
        case BH1750_ONE_TIME_LOW_RES_MODE:
            // apply a valid mode change
            write8(mode);
            break;
        default:
            // Invalid measurement mode
            #if BH1750_DEBUG == 1
            Serial.println("Invalid measurement mode");
            #endif
            break;
    }
    
    // set the Hi res mode factor 1.2 or 2.4
	if ( mode == BH1750_CONTINUOUS_HIGH_RES_MODE_2 || 
		mode == BH1750_ONE_TIME_HIGH_RES_MODE_2 ) {
		highresmodefactor = 2.4;
	} else {
		highresmodefactor = 1.2;
	}
    
	if ( mode == BH1750_ONE_TIME_HIGH_RES_MODE || 
		mode == BH1750_ONE_TIME_HIGH_RES_MODE_2 ||
		mode == BH1750_ONE_TIME_LOW_RES_MODE ) {
		// after reading will go back to power down
		currentmode = BH1750_POWER_DOWN;
	} else {
    	currentmode = mode;
    }

	if ( mode == BH1750_ONE_TIME_LOW_RES_MODE ||
    	mode == BH1750_CONTINUOUS_LOW_RES_MODE ) {
    	// delay enough time for first reading
    	delay(30/mtregfactor);
	} else {
		delay(120/mtregfactor);
    }
}

/**
 * Read the Raw Sensor Value 0 - 65535
 */
uint16_t BH1750::readSensor(void) {

  uint16_t level;
  
  Wire.beginTransmission(i2cAddress);
  Wire.requestFrom(i2cAddress, 2);
#if (ARDUINO >= 100)
  level = Wire.read();
  level <<= 8;
  level |= Wire.read();
#else
  level = Wire.receive();
  level <<= 8;
  level |= Wire.receive();
#endif
  Wire.endTransmission();
  return level;
}

/**
 * DEPRECATED - Kept for Backward Compatibility
 * Use readLuxLevel instead of this method.
 */
uint16_t BH1750::readLightLevel(void) {
  uint16_t level = readSensor();
#if BH1750_DEBUG == 1
  Serial.print("Raw light level: ");
  Serial.println(level);
#endif

  level = level/1.2; // convert to lux

#if BH1750_DEBUG == 1
  Serial.print("Light level: ");
  Serial.println(level);
#endif
  return level;
}

/**
 * Read LUX Level.
 */
float BH1750::readLuxLevel(void) {

  // read sensor, adjust for Measurement Time and HiRes Adjustment Factor
  return mtregfactor * (float)readSensor() / highresmodefactor;
}

void BH1750::changeMeasurementTime(uint8_t measurementtime) {

	if ( measurementtime == currentmtreg ) return;
	if ( measurementtime<BH1750_MEASURE_TIME_MIN || measurementtime>BH1750_MEASURE_TIME_MAX ) return;
	
	// Change Measurement time ( High bit ) 01000_MT[7,6,5]
	write8( 0x40 | ( measurementtime >> 5 ) );

	// Change Masurement time ( Low bit ) 011_MT[4,3,2,1,0]
	write8( 0x60 | ( measurementtime & 0x1F) );
	
	currentmtreg = measurementtime;
	mtregfactor = (float)BH1750_MEASURE_TIME_DEFAULT / (float)measurementtime;
}

void BH1750::powerDown(void) {
	write8(BH1750_POWER_DOWN);
	currentmode = BH1750_POWER_DOWN;
}
		
void BH1750::powerOn(void) {
	write8(BH1750_POWER_ON);
	currentmode = BH1750_POWER_ON;
}
		
void BH1750::reset(void) {
	write8(BH1750_RESET);
	currentmode = BH1750_RESET;
}

/*********************************************************************/

void BH1750::write8(uint8_t d) {
  Wire.beginTransmission(i2cAddress);
#if (ARDUINO >= 100)
  Wire.write(d);
#else
  Wire.send(d);
#endif
  Wire.endTransmission();
}

