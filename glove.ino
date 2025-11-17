#include <Wire.h>
#include <MPU6050.h>         // MPU6050 library

#if __has_include("sdkconfig.h")
#include "sdkconfig.h"
#endif

#if defined(CONFIG_BT_BLUEDROID_ENABLED) && defined(CONFIG_BT_SPP_ENABLED)
#define USE_BT_CLASSIC 1
#else
#define USE_BT_CLASSIC 0
#endif

#if USE_BT_CLASSIC
#include <BluetoothSerial.h> // For ESP32 Bluetooth Classic
#else
#include <NimBLEDevice.h>    // BLE fallback for ESP32 variants without BT Classic
#endif

MPU6050 imu;
const int LED_PIN = 2; // Use GPIO 2 for LED on ESP32 Mini

#if USE_BT_CLASSIC

BluetoothSerial BTSerial;
bool btConnected = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize Bluetooth as client (connects to wheelchair)
  if (!BTSerial.begin("GestureGlove", true)) { // true = client mode
    Serial.println("Bluetooth initialization failed!");
    while (1); // Halt if Bluetooth fails
  }
  
  pinMode(LED_PIN, OUTPUT);

  // Initialize MPU6050
  imu.initialize();
  if (!imu.testConnection()) {
    Serial.println("IMU connection failed!");
    while (1); // Halt if sensor fails
  } else {
    Serial.println("IMU connected.");
  }

  // Connect to wheelchair
  Serial.println("Connecting to WheelchairBase...");
  BTSerial.connect("WheelchairBase");
  
  // Wait for connection with timeout
  unsigned long startTime = millis();
  while (!BTSerial.connected() && (millis() - startTime < 10000)) {
    delay(100);
    Serial.print(".");
  }
  
  if (BTSerial.connected()) {
    btConnected = true;
    Serial.println("\nConnected to wheelchair!");
    digitalWrite(LED_PIN, HIGH); // LED on when connected
  } else {
    Serial.println("\nFailed to connect to wheelchair!");
    digitalWrite(LED_PIN, LOW);
  }
}

void loop() {
  // Check Bluetooth connection status
  if (!BTSerial.connected()) {
    btConnected = false;
    // Try to reconnect
    Serial.println("Reconnecting to wheelchair...");
    BTSerial.connect("WheelchairBase");
    delay(2000);
    if (BTSerial.connected()) {
      btConnected = true;
      Serial.println("Reconnected!");
      digitalWrite(LED_PIN, HIGH);
    } else {
      Serial.println("Connection lost. Retrying...");
      digitalWrite(LED_PIN, LOW);
      delay(1000);
      return; // Skip sending command if not connected
    }
  } else if (!btConnected) {
    btConnected = true;
    Serial.println("Connected!");
    digitalWrite(LED_PIN, HIGH);
  }

  int16_t ax, ay, az;
  imu.getAcceleration(&ax, &ay, &az);

  // Normalize acceleration values
  float x = ax / 16384.0;
  float y = ay / 16384.0;

  String command = "";

  // Gesture logic based on tilt
  if (y > 0.8) {
    command = "FORWARD";
  } else if (y < -0.8) {
    command = "REVERSE";
  } else if (x > 0.8) {
    command = "RIGHT";
  } else if (x < -0.8) {
    command = "LEFT";
  } else {
    command = "STOP";
  }

  // Send command via Bluetooth only if connected
  if (btConnected && BTSerial.connected()) {
    BTSerial.println(command);
    Serial.println("Sent: " + command);
    
    // LED feedback: blink once per gesture
    digitalWrite(LED_PIN, LOW);
    delay(50);
    digitalWrite(LED_PIN, HIGH);
  } else {
    Serial.println("Not connected - " + command);
  }

  delay(300); // Delay between readings
}

#else  // USE_BT_CLASSIC == 0 -> BLE fallback

// BLE UUIDs (Nordic UART compatible)
const char *SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
const char *CHAR_UUID_TX = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"; // Glove -> Wheelchair (write)
const char *WHEELCHAIR_NAME = "WheelchairBase";

#if defined(ESP_PWR_LVL_P7)
#define BLE_TX_POWER ESP_PWR_LVL_P7
#elif defined(ESP_PWR_LVL_P9)
#define BLE_TX_POWER ESP_PWR_LVL_P9
#elif defined(ESP_PWR_LVL_P5)
#define BLE_TX_POWER ESP_PWR_LVL_P5
#else
#define BLE_TX_POWER ESP_PWR_LVL_P3
#endif

NimBLEClient *bleClient = nullptr;
NimBLERemoteCharacteristic *commandCharacteristic = nullptr;

bool isConnected = false;
unsigned long lastConnectAttempt = 0;
const unsigned long RECONNECT_INTERVAL = 5000;

bool connectToWheelchair();
void disconnectWheelchair();

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(LED_PIN, OUTPUT);

  // Initialize MPU6050
  imu.initialize();
  if (!imu.testConnection()) {
    Serial.println("IMU connection failed!");
    while (1); // Halt if sensor fails
  } else {
    Serial.println("IMU connected.");
  }

  // Initialize BLE
  NimBLEDevice::init("GestureGlove");
  NimBLEDevice::setPower(BLE_TX_POWER);
  NimBLEDevice::setSecurityAuth(false, false, false);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

  Serial.println("Scanning for WheelchairBase...");
  connectToWheelchair();
}

void loop() {
  if (!bleClient || !bleClient->isConnected()) {
    if (isConnected) {
      Serial.println("Connection lost.");
      disconnectWheelchair();
    }
    if (millis() - lastConnectAttempt > RECONNECT_INTERVAL) {
      Serial.println("Attempting to reconnect...");
      connectToWheelchair();
    }
    delay(200);
    return;
  } else if (!isConnected) {
    isConnected = true;
    Serial.println("Connected to wheelchair!");
    digitalWrite(LED_PIN, HIGH);
  }

  int16_t ax, ay, az;
  imu.getAcceleration(&ax, &ay, &az);

  // Normalize acceleration values
  float x = ax / 16384.0;
  float y = ay / 16384.0;

  String command = "";

  // Gesture logic based on tilt
  if (y > 0.8) {
    command = "FORWARD";
  } else if (y < -0.8) {
    command = "REVERSE";
  } else if (x > 0.8) {
    command = "RIGHT";
  } else if (x < -0.8) {
    command = "LEFT";
  } else {
    command = "STOP";
  }

  // Send command via BLE only if connected
  if (isConnected && commandCharacteristic) {
    commandCharacteristic->writeValue(command.c_str(), command.length(), false);
    Serial.println("Sent: " + command);
    
    // LED feedback: blink once per gesture
    digitalWrite(LED_PIN, LOW);
    delay(50);
    digitalWrite(LED_PIN, HIGH);
  } else {
    Serial.println("Not connected - " + command);
  }

  delay(300); // Delay between readings
}

bool connectToWheelchair() {
  lastConnectAttempt = millis();

  NimBLEScan *pScan = NimBLEDevice::getScan();
  pScan->setActiveScan(true);
  pScan->start(5, false);
  NimBLEScanResults results = pScan->getResults();

  for (int i = 0; i < results.getCount(); ++i) {
    const NimBLEAdvertisedDevice *device = results.getDevice(i);
    if (!device) {
      continue;
    }
    if (device->getName() == std::string(WHEELCHAIR_NAME) ||
        device->isAdvertisingService(NimBLEUUID(SERVICE_UUID))) {
      Serial.print("Found ");
      Serial.println(device->getName().c_str());

      bleClient = NimBLEDevice::createClient();
      if (!bleClient->connect(device->getAddress())) {
        Serial.println("Failed to connect.");
        NimBLEDevice::deleteClient(bleClient);
        bleClient = nullptr;
        continue;
      }

      NimBLERemoteService *service = bleClient->getService(SERVICE_UUID);
      if (!service) {
        Serial.println("Service not found on device.");
        bleClient->disconnect();
        NimBLEDevice::deleteClient(bleClient);
        bleClient = nullptr;
        continue;
      }

      commandCharacteristic = service->getCharacteristic(CHAR_UUID_TX);
      if (!commandCharacteristic) {
        Serial.println("Command characteristic missing.");
        bleClient->disconnect();
        NimBLEDevice::deleteClient(bleClient);
        bleClient = nullptr;
        continue;
      }

      isConnected = true;
      digitalWrite(LED_PIN, HIGH);
      Serial.println("BLE link ready.");
      return true;
    }
  }

  Serial.println("WheelchairBase not found.");
  return false;
}

void disconnectWheelchair() {
  isConnected = false;
  commandCharacteristic = nullptr;
  digitalWrite(LED_PIN, LOW);

  if (bleClient) {
    if (bleClient->isConnected()) {
      bleClient->disconnect();
    }
    NimBLEDevice::deleteClient(bleClient);
    bleClient = nullptr;
  }
}

#endif  // USE_BT_CLASSIC