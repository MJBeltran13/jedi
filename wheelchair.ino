#if __has_include("sdkconfig.h")
#include "sdkconfig.h"
#endif

#if defined(CONFIG_BT_BLUEDROID_ENABLED) && CONFIG_BT_BLUEDROID_ENABLED && \
    defined(CONFIG_BT_SPP_ENABLED) && CONFIG_BT_SPP_ENABLED
#define USE_BT_CLASSIC 1
#else
#define USE_BT_CLASSIC 0
#endif

#if USE_BT_CLASSIC
#include <BluetoothSerial.h>
#else
#include <NimBLEDevice.h>
#endif

// Pin definitions
#define IR_SENSOR_PIN 34
#define BUZZER_PIN 25
#define L_EN 26
#define L_IN1 27
#define L_IN2 14
#define R_EN 12
#define R_IN1 13
#define R_IN2 15
#define LED_OVERRIDE 23
#define LED_OK 22
#define LED_LOWBAT 21
#define BATTERY_PIN 35

#if USE_BT_CLASSIC
BluetoothSerial BTSerial;
#else
const char *SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
const char *CHAR_UUID_RX = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"; // Glove writes to this characteristic

#if defined(ESP_PWR_LVL_P7)
#define BLE_TX_POWER ESP_PWR_LVL_P7
#elif defined(ESP_PWR_LVL_P9)
#define BLE_TX_POWER ESP_PWR_LVL_P9
#elif defined(ESP_PWR_LVL_P5)
#define BLE_TX_POWER ESP_PWR_LVL_P5
#else
#define BLE_TX_POWER ESP_PWR_LVL_P3
#endif

NimBLEServer *bleServer = nullptr;
NimBLECharacteristic *commandCharacteristic = nullptr;
bool bleClientConnected = false;
#endif

bool overrideActive = false;
unsigned long overrideStart = 0;
unsigned long lastCommandTime = 0;
const unsigned long overrideDuration = 60000; // 60 seconds
const unsigned long COMMAND_TIMEOUT = 2000; // Stop motors if no command for 2 seconds
const int WARNING_THRESHOLD = 2000;
const int STOP_THRESHOLD = 1500;
const float BATTERY_THRESHOLD = 11.5; // volts

void handleCommand(const String &command);

#if !USE_BT_CLASSIC
class CommandCharacteristicCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *characteristic) override {
    std::string value = characteristic->getValue();
    if (value.empty()) {
      return;
    }
    String command(value.c_str());
    handleCommand(command);
  }
};

class ServerConnectionCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *) override {
    bleClientConnected = true;
  }

  void onDisconnect(NimBLEServer *) override {
    bleClientConnected = false;
    NimBLEDevice::startAdvertising();
  }
};
#endif

void setup() {
  Serial.begin(115200);
#if USE_BT_CLASSIC
  BTSerial.begin("WheelchairBase");
#else
  NimBLEDevice::init("WheelchairBase");
  NimBLEDevice::setPower(BLE_TX_POWER);
  NimBLEDevice::setSecurityAuth(false, false, false);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new ServerConnectionCallbacks());

  NimBLEService *service = bleServer->createService(SERVICE_UUID);
  commandCharacteristic = service->createCharacteristic(
    CHAR_UUID_RX,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  commandCharacteristic->setCallbacks(new CommandCharacteristicCallbacks());

  service->start();

  NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();

  Serial.println("BLE ready: advertising as WheelchairBase");
#endif

  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(L_EN, OUTPUT); pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_EN, OUTPUT); pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);

  pinMode(LED_OVERRIDE, OUTPUT);
  pinMode(LED_OK, OUTPUT);
  pinMode(LED_LOWBAT, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);

  stopMotors();
}

void loop() {
  int irValue = analogRead(IR_SENSOR_PIN);
  float batteryVoltage = analogRead(BATTERY_PIN) * (12.0 / 4095.0); // adjust if using voltage divider

  // Buzzer warning
  digitalWrite(BUZZER_PIN, irValue < WARNING_THRESHOLD ? HIGH : LOW);

  // Override logic
  if (irValue < STOP_THRESHOLD) {
    overrideActive = true;
    overrideStart = millis();
    stopMotors();
    Serial.println("Override: Obstacle too close.");
  }

  if (overrideActive && millis() - overrideStart >= overrideDuration) {
    overrideActive = false;
    Serial.println("Override cleared.");
  }

  // LED status
  digitalWrite(LED_OVERRIDE, overrideActive ? HIGH : LOW);
  digitalWrite(LED_OK, overrideActive ? LOW : HIGH);
  digitalWrite(LED_LOWBAT, batteryVoltage < BATTERY_THRESHOLD ? HIGH : LOW);

  // Safety: Stop motors if no command received for timeout period
  if (lastCommandTime > 0 && (millis() - lastCommandTime > COMMAND_TIMEOUT)) {
    stopMotors();
    lastCommandTime = 0; // Reset to prevent repeated stops
  }

#if USE_BT_CLASSIC
  // Bluetooth command handling
  if (BTSerial.hasClient() && BTSerial.available()) {
    String command = BTSerial.readStringUntil('\n');
    handleCommand(command);
  } else if (!BTSerial.hasClient()) {
    // No client connected, stop motors for safety
    stopMotors();
    lastCommandTime = 0;
  }
#else
  if (!bleClientConnected) {
    stopMotors();
    lastCommandTime = 0;
  }
#endif
}

void handleCommand(const String &incoming) {
  String command = incoming;
  command.trim();
  if (command.length() == 0) {
    return;
  }

  Serial.println("Received: " + command);
  lastCommandTime = millis(); // Update last command time

  if (overrideActive) {
    if (command == "LEFT") {
      turnLeft();
    } else if (command == "RIGHT") {
      turnRight();
    } else {
      stopMotors(); // ignore FORWARD/REVERSE
    }
  } else {
    if (command == "FORWARD") {
      moveForward();
    } else if (command == "REVERSE") {
      moveReverse();
    } else if (command == "LEFT") {
      turnLeft();
    } else if (command == "RIGHT") {
      turnRight();
    } else {
      stopMotors();
    }
  }
}

// Motor control functions
void moveForward() {
  analogWrite(L_EN, 255); digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW);
  analogWrite(R_EN, 255); digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW);
}

void moveReverse() {
  analogWrite(L_EN, 255); digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, HIGH);
  analogWrite(R_EN, 255); digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, HIGH);
}

void turnLeft() {
  analogWrite(L_EN, 255); digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, HIGH);
  analogWrite(R_EN, 255); digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW);
}

void turnRight() {
  analogWrite(L_EN, 255); digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW);
  analogWrite(R_EN, 255); digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, HIGH);
}

void stopMotors() {
  analogWrite(L_EN, 0); digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, LOW);
  analogWrite(R_EN, 0); digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW);
}