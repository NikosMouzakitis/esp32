#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <Preferences.h>
#include <ElegantOTA.h>
#include <AsyncTCP.h>


//not needed with new version of ElegantOTA.
//#include <nvs_flash.h>

#define debug 1

AsyncWebServer server(80);

Preferences preferences;
String ota_ssid;
String ota_pass;
String state;
const char *ssid = "esp32net";
const char *password = "1234";
int ota_mode = 0;
unsigned long ota_progress_millis = 0;

////////////////////////////////////////
/// OVER THE AIR related functions /////
////////////////////////////////////////
void onOTAStart() {
  // Log when OTA has started
  Serial.println("OTA update started!");
  // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final) {
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    Serial.println("OTA update finished successfully!");
  } else {
    Serial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
}

//////////////////////////////////////////
////     SPIFF related			//
//////////////////////////////////////////
/*
void listFiles() {

	Serial.println("Listing SPIFFS files...");
	File root = SPIFFS.open("/");
	if (!root || !root.isDirectory()) {
		Serial.println("SPIFFS root is not a directory!");
		return;
	}

	File file = root.openNextFile();
	while (file) {
		Serial.print("FILE: ");
		Serial.print(file.name());
		Serial.print(" - SIZE: ");
		Serial.println(file.size());
		file = root.openNextFile();
	}

}

void readFile(const char *path) {

	Serial.printf("Reading file: %s\n", path);
	File file = SPIFFS.open(path, "r");
	if (!file) {
		Serial.println("Failed to open file for reading");
		return;
	}

	Serial.println("File content:");
	while (file.available()) {
		Serial.write(file.read());  // Use Serial.write() instead of Serial.println()
	}
	Serial.println();

	file.close();

}
*/
//switch to STA station mode.
void startSTA(void)
{
	WiFi.mode(WIFI_STA);
	WiFi.begin(ota_ssid, ota_pass);
	
	while(WiFi.status() != WL_CONNECTED) {
		Serial.println("connecting to WiFi.");
		delay(500);
	}
	Serial.println("connected to WiFi for OTA.");
}

void setup()
{
	Serial.begin(115200);

	//load ssid and password from preferences
	// Open the "credentials" namespace
	preferences.begin("credentials", true); // true for read-only mode
	// Retrieve the saved SSID and password
	ota_ssid = preferences.getString("ssid", ""); // Default to empty string if not found
	ota_pass = preferences.getString("password", ""); // Default to empty string if not found
	state = preferences.getString("state","");
	if(debug) {
		if (ota_ssid == "" || ota_pass == "" || state == "") {
			Serial.println("No values saved for ssid or password");
		} else {

			Serial.println("Credentials for OTA network read success.");
			Serial.println("pass");
			Serial.println(ota_pass);
			Serial.println("ssid");
			Serial.println(ota_ssid);
		}
	}
	
	// Close the preferences
	preferences.end();

	Serial.print("System in: "); Serial.print(state); Serial.println(" mode");

	// Initialize SPIFFS
	if (!SPIFFS.begin(true)) {
		Serial.println("SPIFFS Mount Failed!");
		return;
	} else {
		Serial.println("SPIFFS Mount success!");
	}

	WiFi.disconnect(true,true); //forget previous stored credentials
	delay(2000);
	
	if(state=="AP")
	{

		// Connect to Wi-Fi
		WiFi.softAP(ssid, password);
		Serial.println("AP started");
		Serial.println(WiFi.softAPIP());


		// Serve a simple HTML page
		server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
			request->send(SPIFFS, "/index.html", "text/html");
			Serial.println("index.html accessed");
		});

		// Handle button presses logic (move requests)
		server.on("/move", HTTP_GET, [](AsyncWebServerRequest *request) {
			String direction = request->getParam("dir")->value();
			String state = request->getParam("state")->value();
			//fetch state to be (high on pressed, low on non-pressed)
			int pinState = (state == "1") ? HIGH : LOW;
			// printout to inform developer on monitoring.
			Serial.printf("Direction: %s, State: %s\n", direction.c_str(), state.c_str());
			request->send(200, "text/plain", "OK");
		});
	
		server.on("/ota", HTTP_GET, [](AsyncWebServerRequest *request) {
			Serial.println("OTA button pressed");
			request->send(200, "text/plain", "Switching to OTA mode.");

			//modify state on preferences and reboot.
			preferences.begin("credentials",false);
			preferences.putString("state","STA");
			Serial.println("modifying AP-->STA");
			preferences.end();

			//reboot system.
			Serial.println("Rebooting system.");
			ESP.restart();
		});

	} else if(state=="STA") {

		startSTA();
		ota_mode++;
		ElegantOTA.begin(&server);    // Start ElegantOTA
		// ElegantOTA callbacks
		ElegantOTA.onStart(onOTAStart);
		ElegantOTA.onProgress(onOTAProgress);
		ElegantOTA.onEnd(onOTAEnd);
	} else {
		Serial.println("you should never end here. Halting");
		while(1);
	}


	server.begin();
//	Serial.println("worked the OTA");
}

void loop()
{
	if(ota_mode) {
		ElegantOTA.loop();
	} else {
		Serial.println("old");
		delay(1000);
	}
}
