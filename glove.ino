#include <Wire.h>
#include <MPU6050.h>         // MPU6050 library
#include <BluetoothSerial.h> // For ESP32 Bluetooth Classic

MPU6050 imu;
BluetoothSerial BTSerial;

const int LED_PIN = 2; // Use GPIO 2 for LED on ESP32 Mini
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