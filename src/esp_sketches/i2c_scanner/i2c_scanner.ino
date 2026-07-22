#include <Wire.h>

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);
    Serial.println("I2C Scanner starting...");

    int found = 0;
    for (byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        byte err = Wire.endTransmission();
        if (err == 0) {
            Serial.print("Device found at 0x");
            if (addr < 16) Serial.print("0");
            Serial.println(addr, HEX);
            found++;
        }
    }
    if (found == 0)
        Serial.println("No I2C devices found.");
    else
        Serial.print(found); Serial.println(" device(s) found.");
}

void loop() {
    delay(5000);
    Serial.println("--- I2C Scan ---");
    int found = 0;
    for (byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        byte err = Wire.endTransmission();
        if (err == 0) {
            Serial.print("Device found at 0x");
            if (addr < 16) Serial.print("0");
            Serial.println(addr, HEX);
            found++;
        }
    }
    if (found == 0) Serial.println("No devices found.");
}
