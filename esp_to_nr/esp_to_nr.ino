#include <WiFi.h>
#include <PubSubClient.h>

#define Sensor1Pin 34  // Analog pin for Sensor 1
#define Sensor2Pin 35  // Analog pin for Sensor 2
#define Sensor3Pin 32  // Analog pin for Sensor 3

const char* ssid = "Test_net";
const char* password = "123456789";
const char* mqtt_server = "192.168.177.247";

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("Connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Read sensor values
  int sensor3Value =random(0,101);
  int sensor2Value =random(0,101);
  int sensor1Value =random(0,101);

  // Publish sensor values to MQTT topics
  client.publish("/sensors/sensor1", String(sensor1Value).c_str());
  client.publish("/sensors/sensor2", String(sensor2Value).c_str());
  client.publish("/sensors/sensor3", String(sensor3Value).c_str());
  Serial.println("pubed");
  delay(1000);  // Publish every second
}
