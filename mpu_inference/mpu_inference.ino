#include "mpu9250.h"
#include <WiFi.h>
#include <Arduino.h>
#include <PubSubClient.h>
#include <driver/ledc.h>
#include <TensorFlowLite.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

// Include your model data (generated from your Python script)
#include "model_data.cc"

// Model and classification configuration
constexpr int NUM_SAMPLES = 10;       // 10 samples in sliding window
constexpr int FEATURES_PER_SAMPLE = 2; // ay and gy per sample
constexpr int NUM_FEATURES = NUM_SAMPLES * FEATURES_PER_SAMPLE;
float feature_buffer[NUM_FEATURES];
int feature_index = 0;

// WiFi credentials
const char* ssid = "Test_net";
const char* password = "123456789";

// MQTT server
const char* mqtt_server = "192.168.88.247";
int mqtt_port = 1883;

// LED pins
const int led1Pin = 15;
const int led2Pin = 18;

// PWM settings
const int freq = 5000;
ledc_channel_t pwmChannel1 = LEDC_CHANNEL_0;
ledc_channel_t pwmChannel2 = LEDC_CHANNEL_1;
const int resolution = 8;

int pwmLevel = 128;  // Start at medium brightness

WiFiClient espClient;
PubSubClient client(espClient);
bfs::Mpu9250 imu;

// LED toggle states
bool led1State = false;
bool led2State = false;
bool receivedInitialState = false;

// TFLite globals
namespace {
tflite::ErrorReporter* error_reporter = nullptr;
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;

// Create an area of memory to use for input, output, and intermediate arrays.
constexpr int kTensorArenaSize = 8 * 1024;
uint8_t tensor_arena[kTensorArenaSize];
}  // namespace

void setupPWM() {
  // LEDC timer config
  ledc_timer_config_t timer_conf = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = freq,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer_conf);

  // LED1 config
  ledc_channel_config_t channel_conf1 = {
    .gpio_num = led1Pin,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&channel_conf1);

  // LED2 config
  ledc_channel_config_t channel_conf2 = {
    .gpio_num = led2Pin,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_1,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&channel_conf2);
}

void setupIMU() {
  Serial.println("Connecting to MPU...");
  Wire.begin();
  Wire.setClock(400000);
  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);

  if (!imu.Begin()) {
    Serial.println("IMU init failed");
    while (1) {}
  }
  if (!imu.ConfigSrd(19)) {
    Serial.println("SRD config failed");
    while (1) {}
  }
}

void setupTFLite() {
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;

  // Map the model into a usable data structure
  model = tflite::GetModel(g_model);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema mismatch!");
    while(1);
  }

  // This pulls in all the operation implementations we need
  static tflite::AllOpsResolver resolver;

  // Build an interpreter to run the model
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    Serial.println("AllocateTensors() failed");
    while(1);
  }

  // Obtain pointers to the model's input and output tensors
  input = interpreter->input(0);
  output = interpreter->output(0);

  Serial.println("TensorFlow Lite initialized");
}

void setupWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to wifi.");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.print("\nWiFi connected - IP address: ");
  Serial.println(WiFi.localIP());
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String incoming = "";
  for (unsigned int i = 0; i < length; i++) {
    incoming += (char)payload[i];
  }
  Serial.println("Received message on topic: " + String(topic));
  Serial.println("Payload: " + incoming);

  if (String(topic) == "esp32/restore_state") {
    Serial.println("RECEIVED RESTORE MESSAGE");
    int pwm;
    char led1[4], led2[4];
    if (sscanf(incoming.c_str(), "LED1:%3s LED2:%3s PWM:%d", led1, led2, &pwm) == 3) {
      led1State = (String(led1) == "ON");
      led2State = (String(led2) == "ON");
      pwmLevel = constrain(pwm, 0, 255);

      Serial.println("Restored state from InfluxDB");
      receivedInitialState = true;
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (client.connect("ESP32 client")) {
      Serial.println("Connected");
      client.subscribe("esp32/restore_state");
      client.publish("esp32/restore_request", "get_latest");
    } else {
      Serial.print(client.state());
      Serial.println(" Failed - retrying in 5 seconds");
      delay(5000);
    }
  }
}

int classifyMovement() {
  // Copy feature buffer to model input
  for (int i = 0; i < NUM_FEATURES; i++) {
    input->data.f[i] = feature_buffer[i];
  }

  // Run inference
  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    Serial.println("Invoke failed!");
    return -1;
  }

  // Get the output (probability for each class)
  float led1_prob = output->data.f[0];
  float led2_prob = output->data.f[1];

  // Simple threshold-based classification
  if (led1_prob > 0.7) return 0;  // LED1 movement
  if (led2_prob > 0.7) return 1;  // LED2 movement
  return -1;  // No clear classification
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting up...");

  setupPWM();
  setupIMU();
  setupTFLite();
  setupWiFi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);

  // Connect to MQTT and wait for initial state
  while (!client.connected()) {
    reconnect();
    delay(100);
  }

  Serial.println("Setup complete");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (imu.Read()) {
    float ay = imu.accel_y_mps2();
    float gy = imu.gyro_y_radps();

    // Add new data to feature buffer (sliding window)
    feature_buffer[feature_index++] = ay;
    feature_buffer[feature_index++] = gy;
    
    // Wrap around the circular buffer
    if (feature_index >= NUM_FEATURES) {
      feature_index = 0;
    }

    // Classify every 200ms (adjust as needed)
    static unsigned long lastClassificationTime = 0;
    if (millis() - lastClassificationTime > 200) {
      lastClassificationTime = millis();
      
      int classification = classifyMovement();
      if (classification == 0) {  // LED1 movement detected
        led1State = !led1State;
        Serial.println("LED1 toggled by movement");
      } 
      else if (classification == 1) {  // LED2 movement detected
        led2State = !led2State;
        Serial.println("LED2 toggled by movement");
      }
    }

    // PWM control via mx (optional - keep your existing logic)
    float mx = imu.mag_x_ut();
    if (mx > 60) {
      pwmLevel = min(255, pwmLevel + 10);
    } else if (mx < 30) {
      pwmLevel = max(0, pwmLevel - 10);
    }

    // LED control
    ledc_set_duty(LEDC_LOW_SPEED_MODE, pwmChannel1, led1State ? pwmLevel : 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, pwmChannel1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, pwmChannel2, led2State ? pwmLevel : 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, pwmChannel2);

    // Publish status (optional)
    String statusMessage = "LED1:" + String(led1State ? "ON" : "OFF") +
                         " LED2:" + String(led2State ? "ON" : "OFF") +
                         " PWM:" + String(pwmLevel);
    client.publish("esp32/status", statusMessage.c_str());
  }

  delay(10);  // Small delay to maintain stability
}
