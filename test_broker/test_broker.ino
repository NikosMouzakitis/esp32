#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "Test_net";
const char* password = "123456789";
IPAddress mqtt_server(192,168,252,247);

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  
  client.setServer(mqtt_server, 1883);

  Serial.println("Connecting to MQTT...");
  if (client.connect("esp32_test_client")) {
    Serial.println("MQTT connected!");
  } else {
    Serial.print("MQTT connect failed, rc=");
    Serial.println(client.state());
  }
}

void loop() {}

