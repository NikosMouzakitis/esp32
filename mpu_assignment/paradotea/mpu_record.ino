#include "mpu9250.h"
#include <Wire.h>

bfs::Mpu9250 imu;

const int led1Pin = 15;
const int led2Pin = 18;

bool led1State = false;
bool led2State = false;

int pwmLevel = 128;
const int pwmMax = 255;
const int pwmMin = 0;

bool isSampling = false;
int sampleIndex = 0;
const int sampleSize = 10;

float aySamples[sampleSize];
float gySamples[sampleSize];

void setup() {
  Serial.begin(115200);
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);

  Wire.begin();
  Wire.setClock(400000);
  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);

  if (!imu.Begin()) {
    Serial.println("IMU init failed");
    while (1) {}
  }
  imu.ConfigSrd(19);

  Serial.println("Type 'record' to begin capturing a gesture...");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "record") {
      isSampling = true;
      sampleIndex = 0;
      Serial.println("Recording started...");
    }
  }

  if (imu.Read()) {
    float ay = imu.accel_y_mps2();
    float gy = imu.gyro_y_radps();
    float mx = imu.mag_x_ut();

    // Threshold-based gesture control (keep real-time behavior)
    bool toggle1 = (ay > 0.5);
    bool toggle2 = (ay < -10);

    static bool prevToggle1 = false;
    static bool prevToggle2 = false;

    if (toggle1 && !prevToggle1) led1State = !led1State;
    if (toggle2 && !prevToggle2) led2State = !led2State;

    prevToggle1 = toggle1;
    prevToggle2 = toggle2;

    digitalWrite(led1Pin, led1State);
    digitalWrite(led2Pin, led2State);

    if (mx > 60) pwmLevel = min(pwmMax, pwmLevel + 10);
    else if (mx < 30) pwmLevel = max(pwmMin, pwmLevel - 10);

    // Sample for gesture classification
    if (isSampling) {
      aySamples[sampleIndex] = ay;
      gySamples[sampleIndex] = gy;
      sampleIndex++;

      if (sampleIndex >= sampleSize) {
        // Print as CSV row: ay0,gy0,ay1,gy1,...,LABEL
//        Serial.print("DATA:");
        for (int i = 0; i < sampleSize; ++i) {
          Serial.print(aySamples[i]); Serial.print(",");
          Serial.print(gySamples[i]);
          if (i < sampleSize - 1) Serial.print(",");
        }
    //    Serial.println(",LABEL_HERE");  // Replace LABEL_HERE manually later

        isSampling = false;
        //Serial.println("Recording done. Type 'record' to start again.");
      }
    }

    delay(100);  // 10Hz sampling
  }
}

