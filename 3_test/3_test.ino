#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <Preferences.h>
#include <ElegantOTA.h>
#include <Arduino_JSON.h>

//not needed with new version of ElegantOTA.
//#include <nvs_flash.h>
//#include <AsyncTCP.h>

#define debug 1

AsyncWebServer server(80);

// our websocket.
AsyncWebSocket ws("/ws");
String message="";
String sliderValue="50";
String txtValue = "0";
JSONVar sliderValues;
JSONVar speedTxtValue;

Preferences preferences;
String ota_ssid;
String ota_pass;
String state;
const char *ssid = "esp32net";
const char *password = "1234";
int ota_mode = 0;
unsigned long ota_progress_millis = 0;


String getSliderValues(int arg)
{
	if(arg == 1)
	{
		sliderValues["sliderValue"] = String(sliderValue);
		String jsonString = JSON.stringify(sliderValues);
		Serial.println(jsonString);
		return jsonString;
	}
	if(arg == 2)
	{
		speedTxtValue["speedTxtValue"] = String(txtValue);
		String jsonString = JSON.stringify(speedTxtValue);
		Serial.println(jsonString);
		return jsonString;
	}
  return "";
}

void notifyClients(String sliderValues)
{
	ws.textAll(sliderValues);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
	AwsFrameInfo *info = (AwsFrameInfo*)arg;
	Serial.println("Message: ");
	Serial.println((char )*data);
	if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
		data[len] = 0;
		message = (char*)data;

		if (message.indexOf("1s") >= 0) {
			Serial.println("Updating sliderValue");
			sliderValue = message.substring(2);
			Serial.print(getSliderValues(1));
			notifyClients(getSliderValues(1));
		}
		if(message.indexOf("2s") >= 0) {
			Serial.println("update speedTextValue");
			sliderValue = message.substring(2);

			Serial.print(getSliderValues(2));
			notifyClients(getSliderValues(2));
		}

		if (strcmp((char*)data, "getValues") == 0) {
		//	notifyClients(getSliderValues());
			notifyClients(getSliderValues(1));
			notifyClients(getSliderValues(2));
		}
	}
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  Serial.println("new event");
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.println("conn event");
      break;
    case WS_EVT_DISCONNECT:
      Serial.println("discon event");
      Serial.printf("WebSocket client #%lu disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      Serial.println("data event");
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_PING:
    case WS_EVT_ERROR:
      Serial.println("err event");
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

////////////////////////////////////////
/// OVER THE AIR related functions /////
////////////////////////////////////////
void onOTAStart() {
	Serial.println("OTA update started!");
}

void onOTAProgress(size_t current, size_t final) {
}

void onOTAEnd(bool success) {
	//modification of the state into the preferences,
	//then reboot on new state WIFI_STA
	
	preferences.begin("credentials",false);
	preferences.putString("state","AP");
	Serial.println("modifying STA-->AP");
	preferences.end();

	if (success) {
		Serial.println("OTA update finished successfully!");
	} else {
		Serial.println("There was an error during OTA update!");
	}
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

	Serial.print("esp in: ");
	Serial.print(state);
	Serial.println(" mode");

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
			//int pinState = (state == "1") ? HIGH : LOW;
			// printout to inform developer on monitoring.
			Serial.printf("Direction: %s, State: %s\n", direction.c_str(), state.c_str());
			request->send(200, "text/plain", "OK");
		});

		server.on("/ota", HTTP_GET, [](AsyncWebServerRequest *request) {
			Serial.println("OTA button pressed");
			request->send(200, "text/plain", "Switching to OTA mode.");

			//modification of the state into the preferences,
			//then reboot on new state WIFI_STA
			preferences.begin("credentials",false);
			preferences.putString("state","STA");
			Serial.println("modifying AP-->STA");
			preferences.end();

			//reboot system.
			Serial.println("Rebooting system.");
			ESP.restart();
		});

		initWebSocket();

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
}

void loop()
{
	if(ota_mode) {
		ElegantOTA.loop();
	} else {
		Serial.println("exec");
		delay(1000);
	}
}
