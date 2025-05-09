#include <SPI.h>

// Define SPI pins
#define MPU6050_SS_PIN 5  // Chip Select (SS) pin

// MPU6050 registers
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43

// Function to read a byte from a register
uint8_t readRegister(uint8_t reg) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(MPU6050_SS_PIN, LOW);  // Select MPU6050
  SPI.transfer(reg | 0x80);           // Read command (set MSB to 1)
  uint8_t value = SPI.transfer(0x00); // Read the value
  digitalWrite(MPU6050_SS_PIN, HIGH); // Deselect MPU6050
  SPI.endTransaction();
  return value;
}

// Function to write a byte to a register
void writeRegister(uint8_t reg, uint8_t value) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(MPU6050_SS_PIN, LOW);  // Select MPU6050
  SPI.transfer(reg & 0x7F);           // Write command (clear MSB to 0)
  SPI.transfer(value);                // Write the value
  digitalWrite(MPU6050_SS_PIN, HIGH); // Deselect MPU6050
  SPI.endTransaction();
}

// Function to read 2 bytes from a register (high and low)
int16_t readRegister16(uint8_t reg) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(MPU6050_SS_PIN, LOW);  // Select MPU6050
  SPI.transfer(reg | 0x80);           // Read command (set MSB to 1)
  int16_t value = SPI.transfer(0x00) << 8; // Read high byte
  value |= SPI.transfer(0x00);             // Read low byte
  digitalWrite(MPU6050_SS_PIN, HIGH); // Deselect MPU6050
  SPI.endTransaction();
  return value;
}

void setup() {
  Serial.begin(115200);

  // Initialize SPI
  SPI.begin();
  pinMode(MPU6050_SS_PIN, OUTPUT);
  digitalWrite(MPU6050_SS_PIN, HIGH); // Deselect MPU6050

  // Wake up MPU6050
  writeRegister(MPU6050_PWR_MGMT_1, 0x00);

  // Check WHO_AM_I register
  uint8_t whoAmI = readRegister(MPU6050_WHO_AM_I);
  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoAmI, HEX);

  if (whoAmI != 0x68) {
    Serial.println("MPU6050 not found!");
    while (1);
  }

  Serial.println("MPU6050 initialized!");
}

void loop() {
  // Read accelerometer data
  int16_t ax = readRegister16(MPU6050_ACCEL_XOUT_H);
  int16_t ay = readRegister16(MPU6050_ACCEL_XOUT_H + 2);
  int16_t az = readRegister16(MPU6050_ACCEL_XOUT_H + 4);

  // Read gyroscope data
  int16_t gx = readRegister16(MPU6050_GYRO_XOUT_H);
  int16_t gy = readRegister16(MPU6050_GYRO_XOUT_H + 2);
  int16_t gz = readRegister16(MPU6050_GYRO_XOUT_H + 4);

  // Print data
  Serial.print("Accel: ");
  Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", ");
  Serial.print(az); Serial.println();

  Serial.print("Gyro: ");
  Serial.print(gx); Serial.print(", ");
  Serial.print(gy); Serial.print(", ");
  Serial.print(gz); Serial.println();

  delay(500);
}
