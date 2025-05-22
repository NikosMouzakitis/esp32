#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "model_data.h"  // Your model header

// TensorFlow Lite includes
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>

// Constants
constexpr int NUM_SAMPLES = 10;
constexpr int FEATURES_PER_SAMPLE = 2;
constexpr int NUM_FEATURES = NUM_SAMPLES * FEATURES_PER_SAMPLE;

// Global variables
float feature_buffer[NUM_FEATURES];
int feature_index = 0;

// TensorFlow Lite globals
namespace {
tflite::ErrorReporter* error_reporter = nullptr;
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;

constexpr int kTensorArenaSize = 16 * 1024;
uint8_t tensor_arena[kTensorArenaSize];
}  // namespace

// Initialize TensorFlow Lite
void setupTFLite() {
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;

  model = tflite::GetModel(g_model);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema mismatch!");
    while(1);
  }

  static tflite::AllOpsResolver resolver;
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("AllocateTensors() failed");
    while(1);
  }

  input = interpreter->input(0);
  output = interpreter->output(0);
}

// Classify movement
int classifyMovement() {
  for (int i = 0; i < NUM_FEATURES; i++) {
    input->data.f[i] = feature_buffer[i];
  }

  if (interpreter->Invoke() != kTfLiteOk) {
    Serial.println("Invoke failed!");
    return -1;
  }

  float led1_prob = output->data.f[0];
  float led2_prob = output->data.f[1];

  if (led1_prob > 0.7) return 0;
  if (led2_prob > 0.7) return 1;
  return -1;
}

void setup() {
  Serial.begin(115200);
  setupTFLite();
  // Add your other setup code here
}

void loop() {
  // Add your IMU reading and classification logic here
  // Example:
  
  if (imu.Read()) {
    float ay = imu.accel_y_mps2();
    float gy = imu.gyro_y_radps();
    
    feature_buffer[feature_index++] = ay;
    feature_buffer[feature_index++] = gy;
    
    if (feature_index >= NUM_FEATURES) {
      feature_index = 0;
      
      int classification = classifyMovement();
      if (classification == 0) {
        Serial.println("classification 0");

      } else if (classification == 1) {

        Serial.println("classification 1");
      }

    }
  }
  
  delay(100);
}
