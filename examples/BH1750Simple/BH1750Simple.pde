/*

Example of BH1750 library usage.

This example initalises the BH1750 object using the default
high resolution mode and then makes a light level reading every second.

The measurement time can be changed, to improve accuracy, or extend range.

Connection:
 VCC-5v
 GND-GND
 SCL-SCL(analog pin 5)
 SDA-SDA(analog pin 4)
 ADD-NC or GND

*/

#include <Wire.h>
#include <BH1750.h>

BH1750 lightMeter;

void setup(){
  Serial.begin(9600);
  Wire.begin();
  lightMeter.changeMeasurementTime( BH1750_MEASURE_TIME_DEFAULT );
  lightMeter.startMeasurement(BH1750_CONTINUOUS_HIGH_RES_MODE);
  Serial.println("Running...");
}


void loop() {
  uint16_t lux = lightMeter.readLuxLevel();
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
  delay(1000);
}