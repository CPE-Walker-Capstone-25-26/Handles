#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic *pLeftHandleChar = NULL;
BLECharacteristic *pRightHandleChar = NULL;
BLECharacteristic *pThresholdChar = NULL;
BLECharacteristic *pVibrationChar = NULL;
BLECharacteristic *pBatteryChar = NULL;
BLECharacteristic *pTimeSyncChar = NULL;
BLECharacteristic *pLogDataChar = NULL;

bool deviceConnected = false;

// Service UUID (you can keep yours or use this standard one)
#define SERVICE_UUID "FFE0"

// Characteristic UUIDs (MUST match iOS app)
#define LEFT_HANDLE_UUID    "FFE1"
#define RIGHT_HANDLE_UUID   "FFE2"
#define THRESHOLD_UUID      "FFE3"
#define VIBRATION_UUID      "FFE4"
#define BATTERY_UUID        "FFE5"
#define TIME_SYNC_UUID      "FFE6"
#define LOG_DATA_UUID       "FFE7"

#define LED_PIN 8

// Simulated sensor data
uint16_t leftWeight = 0;
uint16_t rightWeight = 0;
uint16_t leftBatteryMv = 3700;   // 3.7V
uint16_t rightBatteryMv = 3650;  // 3.65V

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("✅ Device connected!");
      digitalWrite(LED_PIN, HIGH);  // Turn on LED when connected
    };
    
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("🔌 Device disconnected!");
      digitalWrite(LED_PIN, LOW);   // Turn off LED when disconnected
      BLEDevice::startAdvertising();
      Serial.println("📡 Advertising restarted...");
    }
};

// Handle threshold updates from iOS
class ThresholdCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
      String value = pChar->getValue();
      if (value.length() == 2) {
        uint8_t leftThreshold = value[0];
        uint8_t rightThreshold = value[1];
        Serial.printf("🎚️ Threshold set - Left: %d, Right: %d\n", leftThreshold, rightThreshold);
      }
    }
};

// Handle vibration settings from iOS
class VibrationCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
      String value = pChar->getValue();
      if (value.length() == 2) {
        uint8_t leftVibration = value[0];
        uint8_t rightVibration = value[1];
        Serial.printf("📳 Vibration set - Left: %d, Right: %d\n", leftVibration, rightVibration);
        
        // Blink LED to confirm
        for(int i = 0; i < leftVibration/2; i++) {
          digitalWrite(LED_PIN, HIGH);
          delay(50);
          digitalWrite(LED_PIN, LOW);
          delay(50);
        }
      }
    }
};

// Handle time sync from iOS
class TimeSyncCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
      String value = pChar->getValue();
      if (value.length() == 4) {
        uint32_t timestamp = (uint8_t)value[0] | 
                           ((uint8_t)value[1] << 8) | 
                           ((uint8_t)value[2] << 16) | 
                           ((uint8_t)value[3] << 24);
        Serial.printf("🕐 Time synced: %u (Unix timestamp)\n", timestamp);
      }
    }
};

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  Serial.println("\n\n***** ESP32 Walker Handle BLE Starting *****");
  
  // Initialize BLE with a clear name
  BLEDevice::init("SMARTHANDLE");
  
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // FFE1 - Left Handle Weight (Notify + Read)
  pLeftHandleChar = pService->createCharacteristic(
    LEFT_HANDLE_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pLeftHandleChar->addDescriptor(new BLE2902());
  
  // FFE2 - Right Handle Weight (Notify + Read)
  pRightHandleChar = pService->createCharacteristic(
    RIGHT_HANDLE_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pRightHandleChar->addDescriptor(new BLE2902());
  
  // FFE3 - Threshold Settings (Write)
  pThresholdChar = pService->createCharacteristic(
    THRESHOLD_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  pThresholdChar->setCallbacks(new ThresholdCallbacks());
  
  // FFE4 - Vibration Settings (Write)
  pVibrationChar = pService->createCharacteristic(
    VIBRATION_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  pVibrationChar->setCallbacks(new VibrationCallbacks());
  
  // FFE5 - Battery Status (Notify + Read)
  pBatteryChar = pService->createCharacteristic(
    BATTERY_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pBatteryChar->addDescriptor(new BLE2902());
  
  // FFE6 - Time Sync (Write)
  pTimeSyncChar = pService->createCharacteristic(
    TIME_SYNC_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  pTimeSyncChar->setCallbacks(new TimeSyncCallbacks());
  
  // FFE7 - Log Data (Notify + Read)
  pLogDataChar = pService->createCharacteristic(
    LOG_DATA_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pLogDataChar->addDescriptor(new BLE2902());
  
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.println("✅ BLE Ready!");
  Serial.println("📱 Device name: SMARTHANDLE");
  Serial.println("🔍 Waiting for iOS app to connect...");
  Serial.println("======================================\n");
}

void loop() {
  if (deviceConnected) {
    // Simulate changing weight values (you'll replace this with real sensor data)
    leftWeight = random(0, 100);   // 0-100 lbs
    rightWeight = random(0, 100);
    
    // Send left handle weight (2 bytes, little-endian)
    uint8_t leftData[2];
    leftData[0] = leftWeight & 0xFF;
    leftData[1] = (leftWeight >> 8) & 0xFF;
    pLeftHandleChar->setValue(leftData, 2);
    pLeftHandleChar->notify();
    
    // Send right handle weight (2 bytes, little-endian)
    uint8_t rightData[2];
    rightData[0] = rightWeight & 0xFF;
    rightData[1] = (rightWeight >> 8) & 0xFF;
    pRightHandleChar->setValue(rightData, 2);
    pRightHandleChar->notify();
    
    // Send battery status every 5 seconds (4 bytes: left voltage + right voltage)
    static unsigned long lastBattery = 0;
    if (millis() - lastBattery > 5000) {
      uint8_t batteryData[4];
      batteryData[0] = leftBatteryMv & 0xFF;
      batteryData[1] = (leftBatteryMv >> 8) & 0xFF;
      batteryData[2] = rightBatteryMv & 0xFF;
      batteryData[3] = (rightBatteryMv >> 8) & 0xFF;
      pBatteryChar->setValue(batteryData, 4);
      pBatteryChar->notify();
      
      lastBattery = millis();
      Serial.printf("🔋 Battery: L=%.2fV R=%.2fV\n", 
                    leftBatteryMv/1000.0, rightBatteryMv/1000.0);
    }
    
    // Print weight data
    Serial.printf("⚖️ Weight: L=%d lbs, R=%d lbs\n", leftWeight, rightWeight);
    
    delay(500);  // Send updates twice per second
  }
  
  delay(100);
}