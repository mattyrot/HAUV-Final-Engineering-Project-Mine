#include <Wire.h>
#include "MS5837.h"

MS5837 sensor;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BAR test...");
  Wire.begin();
  while (!sensor.init()) {
    Serial.println("Init failed! Check SDA/SCL wiring.");
    delay(2000);
  }
  sensor.setFluidDensity(997);
  Serial.println("Sensor initialized OK.");
}

void loop() {
  sensor.read();
  Serial.print("Pressure: "); Serial.print(sensor.pressure()); Serial.println(" mbar");
  Serial.print("Temperature: "); Serial.print(sensor.temperature()); Serial.println(" C");
  Serial.print("Depth: "); Serial.print(sensor.depth()); Serial.println(" m");
  delay(1000);
}
