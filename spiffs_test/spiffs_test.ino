
#include <SPIFFS.h>

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

	listFiles();
  
	readFile("/test.txt");

	Serial.println("SPIFFS read out completes");
}

void loop()
{
	Serial.println("hi");
	delay(1000);
}
