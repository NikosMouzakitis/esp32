#include "mpu9250.h"
#include <WiFi.h>
#include <PubSubClient.h>

// Replace with your credentials
const char* ssid = "Test_net";
const char* password = "123456789";

// Replace with your IP
//const char* mqtt_server = "192.168.252.247"; 
IPAddress mqtt_server(192, 168, 252, 247);
int mqtt_port = 1883;

String message;

bool state = false;

WiFiClient espClient;
PubSubClient client(espClient);

/* Mpu9250 object */
bfs::Mpu9250 imu;

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  Serial.println("operating");

  // Connect to WiFi
  WiFi.begin(ssid, password);   
  Serial.print("Connecting to wifi.");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.print("\nWiFi connected - IP address: ");
  Serial.println(WiFi.localIP());
  delay(500);
  
  client.setServer(mqtt_server, 1883);
//  client.setCallback(callback);

  Serial.println("connection to MPU..");
  delay(1000);
  while(!Serial) {}
  /* Start the I2C bus */
  Wire.begin();
  Wire.setClock(400000);
  /* I2C bus,  0x68 address */
  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }
  /* Set the sample rate divider */
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configured SRD");
    while(1) {}
  }
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  // boolean loop ()
  // This should be called regularly to allow the client to process 
  // incoming messages and maintain its connection to the server.
  client.loop();
  
  /* Check if data read */
  if (imu.Read()) {

    Serial.print("AccX: ");
    float ax=imu.accel_x_mps2();
    Serial.print(ax);
    Serial.println(" ");
    Serial.print("AccY: ");
    float ay=imu.accel_y_mps2();
    Serial.print(ay);
    Serial.println(" ");
    Serial.print("AccZ: ");
    float az = imu.accel_z_mps2();
    Serial.print(az);
    Serial.println(" ");
    String accData = "ax:" + String(ax) + " ay:" + String(ay) + " az:" + String(az);
    Serial.println(accData);

    Serial.print("GyroX: ");
    float gx = imu.gyro_x_radps();
    Serial.print(gx);
    Serial.println(" ");
    Serial.print("GyroY: ");
    float gy = imu.gyro_y_radps();
    Serial.print(gy);
    Serial.println(" ");
    Serial.print("GyroZ: ");
    float gz=imu.gyro_z_radps();
    Serial.print(gz);
    Serial.println(" ");
    String gyroData = "gx:" + String(gx) + " gy:" + String(gy) + " gz:" + String(gz);

    Serial.println(gyroData);
    /*
    Serial.print(imu.mag_x_ut());
    Serial.println(" ");
    Serial.print(imu.mag_y_ut());
    Serial.println(" ");
    Serial.print(imu.mag_z_ut());
    Serial.println(" ");
    */

    client.publish("esp32/gyro", gyroData.c_str());
    client.publish("esp32/acc", accData.c_str());

  }
  delay(200);
}

void reconnect(){
  // Loop until we're reconnected
  while (!client.connected()){
    Serial.println("Attempting MQTT connection...");
    if (client.connect("ESP32 client")){
      Serial.println("Connected");
  //    client.subscribe("esp32/out");
  //   client.subscribe("esp32/pwm");
    }
    else{
      Serial.print(client.state());
      Serial.println("Failed - Try again in 5 seconds");
      delay(5000);
    }
  }
}
