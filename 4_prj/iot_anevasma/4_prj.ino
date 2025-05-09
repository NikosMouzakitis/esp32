#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <Preferences.h>
#include <ElegantOTA.h>
#include <Arduino_JSON.h>
#include <DHT.h>
#include <DHT_U.h>


//not needed with new version of ElegantOTA.
//#include <nvs_flash.h>
//#include <AsyncTCP.h>

#define debug 1

AsyncWebServer server(80);
String mySliderValue = "128"; //initial Led value
// our websocket.
AsyncWebSocket ws("/ws");
String message="";

JSONVar readings;
int ledPin = 5;
#define DHTPIN 4
#define DHTTYPE DHT11
float mock_temp = 28.5; //mock start values.
float mock_hum = 67.5;
float temperature;
float humidity;
DHT dht(DHTPIN, DHTTYPE);

Preferences preferences;
//for Over The Air firmware Update.
String ota_ssid;
String ota_pass;
String state;

const char *ssid = "esp32net";
const char *password = "1234";
int ota_mode = 0;

unsigned long ota_progress_millis = 0;

void notifyClients(String readings)
{
	ws.textAll(readings.c_str());
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
	AwsFrameInfo *info = (AwsFrameInfo*)arg;
	Serial.println("Message: ");
	Serial.println((char )*data);
	if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
		data[len] = 0;
		message = (char*)data;
		if(message.startsWith("brightness")) {
			mySliderValue = message.substring(10);
			analogWrite(ledPin, mySliderValue.toInt());
			notifyClients(getSensorReadings());
		}
	
	}
}

String getSensorReadings() {
	readings["temperature"] = String(mock_temp);
	readings["humidity"] = String(mock_hum);
	readings["brightness"] = mySliderValue;
	return JSON.stringify(readings);

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
	pinMode(ledPin, OUTPUT);
	//load ssid and password from preferences
	// Open the "credentials" namespace
	preferences.begin("credentials", true); // true for read-only mode
	// Retrieve the saved SSID and password
	//print for our validation only when debug is ON.
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
	
	dht.begin(); //start dht sensor.

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
		// Starting the Access Point with our ssid pass.
		WiFi.softAP(ssid, password);
		Serial.println("AP started");
		Serial.println(WiFi.softAPIP());

		// Serve a simple HTML page
		server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
			request->send(SPIFFS, "/index.html", "text/html");
			if(debug)
				Serial.println("index.html accessed");
		});

		// Handle button presses logic (move requests)
		server.on("/move", HTTP_GET, [](AsyncWebServerRequest *request) {
			String direction = request->getParam("dir")->value();
			String state = request->getParam("state")->value();
			//fetch state to be (high on pressed, low on non-pressed)
			//int pinState = (state == "1") ? HIGH : LOW;
				// printout to inform developer on monitoring.
			if(debug)
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
			preferences.end();

			if(debug) {
				Serial.println("modifying AP-->STA");
				Serial.println("Rebooting system.");
			}
			//reboot system.
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
void readDHT11() {
  // Mock data since failed to work the sensor took from the lab.
  mock_temp += 0.1;
  if (mock_temp > 30.0) mock_temp = 20.0;
  
  mock_hum += 0.5;
  if (mock_hum > 70.0) mock_hum = 30.0;
  
  // Code when using a real DHT sensor
 /* 
  float newTemp = dht.readTemperature();
  float newHum = dht.readHumidity();
  Serial.println("Reading real!!!!!!!!!!!!!!!!!!!!!!"); 
  if (!isnan(newTemp)) temperature = newTemp;
  if (!isnan(newHum)) humidity = newHum;
  
  Serial.println("DHT hw");
  Serial.println("tmp: ");
  Serial.println(temperature); 
  Serial.println("hum: ");
  Serial.println(humidity);
  // in our not working case, always both values were printed as 0.0, 0.0. 
  */
}

void loop()
{
	if(ota_mode) {
		ElegantOTA.loop();
	} else {
		if(debug)
		{
			int lala = 0;
		//	Serial.println("run");
		}
		static unsigned long lastSensorRead = 0;
		if(millis() - lastSensorRead > 2000) { //every 2 seconds update.
			Serial.println("run ota done");
			readDHT11();	
			notifyClients(getSensorReadings());
			lastSensorRead = millis();
		}
		ws.cleanupClients();
	}
}
