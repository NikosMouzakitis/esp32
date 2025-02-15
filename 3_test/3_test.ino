#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
const char *ssid = "espnet";
const char *pass = "1234";

AsyncWebServer server(80);

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


void setup()
{
	Serial.begin(115200);
	// Initialize SPIFFS
	if (!SPIFFS.begin(true)) {
		Serial.println("SPIFFS Mount Failed!");
		return;
	} else {
		Serial.println("SPIFFS Mount success!");
	}
	WiFi.disconnect(true,true); //forget previous stored credentials
	delay(2000);
	// Connect to Wi-Fi
	WiFi.softAP(ssid, pass);
	Serial.println("AP started");
	Serial.println(WiFi.softAPIP());

//	listFiles();
//	readFile("/index.html");
//	Serial.println("SPIFFS read out completes");



	// Serve a simple HTML page
	server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
		request->send(SPIFFS, "/index.html", "text/html");
		Serial.println("fired");
	});

	// Handle button presses logic (move request)
	server.on("/move", HTTP_GET, [](AsyncWebServerRequest *request) {
		String direction = request->getParam("dir")->value();
		String state = request->getParam("state")->value();
		int pinState = (state == "1") ? HIGH : LOW;
	
		Serial.printf("Direction: %s, State: %s\n", direction.c_str(), state.c_str());
		request->send(200, "text/plain", "OK");
	});

	server.begin();
}

void loop()
{
	Serial.println("3test: hi");
	delay(1000);
}
