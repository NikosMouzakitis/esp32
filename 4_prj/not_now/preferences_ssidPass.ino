#include <Preferences.h>

Preferences preferences;

const char* ssid = "Test_net";
const char* password = "123456789";
const char* state = "AP";

void setup() {
  Serial.begin(115200);
  Serial.println();

  preferences.begin("credentials", false);
  preferences.putString("ssid", ssid); 
  preferences.putString("password", password);
  preferences.putString("state", state);

  Serial.println("Save using Preferences(credentiantls and initial AP state");

  preferences.end();
}

void loop() {
	Serial.println("wrote creds");
	delay(2000);
}

