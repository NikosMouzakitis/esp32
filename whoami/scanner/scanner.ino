#include <Wire.h>

void setup() {
  Wire.begin(21, 22);  // Use GPIO 21 for SDA, GPIO 22 for SCL (default for many ESP32 boards)
  Serial.begin(115200);
  delay(2000);
  Serial.println("Scanning...");

  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (i < 16) {
        Serial.print("0");
      }
      Serial.println(i, HEX);
    }
    delay(50);
  }
  Serial.println("Scanning done.");
}

void loop() {}
