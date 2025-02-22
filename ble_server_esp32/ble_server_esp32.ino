#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// Create BLE Server
BLEServer* pServer = NULL;
// Create BLE Characteristic
BLECharacteristic* pCharacteristic = NULL;
int connected = 0;
// Characteristic UUID
#define CHARACTERISTIC_UUID "4fa464b9-8b10-4f9d-b2e6-7d20a1ffbc78"
// Service UUID
#define SERVICE_UUID "5f5e87f2-d0a8-40b5-bc39-92915c186478"

// Value to send
int value = 0;

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        Serial.println("Client connected");
	connected++;
    }

    void onDisconnect(BLEServer* pServer) {
        Serial.println("Client disconnected");
	connected = 0;
    }
};

void setup() {
    Serial.begin(115200);
    
    // Initialize BLE
    BLEDevice::init("ESP32_BLE_Server");
    
    // Create BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create BLE Characteristic
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ
    );

    // Start the service
    pService->start();
    // Start advertising
    pServer->getAdvertising()->start();
    Serial.println("MAC");
    Serial.println(BLEDevice::getAddress().toString().c_str());
    Serial.println("Waiting for a client connection...");
}

void loop() {
	
    if(!connected)
	pServer->getAdvertising()->start();

    // Update value and notify if needed
    value++;
    pCharacteristic->setValue(value);
    delay(1000); // Delay for demonstration
    Serial.println(BLEDevice::getAddress().toString().c_str());
}
