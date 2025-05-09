#include <Wire.h>

uint8_t findMPU6050() {
  Wire.begin();
  for (uint8_t addr = 0x0; addr <= 255; addr++) {
    Serial.println("test for: ");
    Serial.println(addr);
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      return addr;  // Return the address if the device is found
    }
  }
  return 0;  // Return 0 if no device is found
}

void setup() {
  Serial.begin(115200);

  uint8_t mpuAddr = findMPU6050();
  if (mpuAddr == 0) {
    Serial.println("MPU6050 not found!");
    while (1);
  }

  Serial.print("MPU6050 found at address 0x");
  Serial.println(mpuAddr, HEX);

  // Use the detected address for further communication
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up (clear sleep bit)
  Wire.endTransmission(true);

  // Check WHO_AM_I register
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x75);  // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(mpuAddr, 1, true);
  uint8_t whoAmI = Wire.read();

  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoAmI, HEX);

  if (whoAmI != 0x68) {
    Serial.println("Unexpected WHO_AM_I value!");
    while (1);
  }

  Serial.println("MPU6050 initialized!");
}
void loop()
{

}
