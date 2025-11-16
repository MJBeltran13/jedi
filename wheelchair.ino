#include <BluetoothSerial.h>

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

BluetoothSerial BTSerial;

bool overrideActive = false;
unsigned long overrideStart = 0;
unsigned long lastCommandTime = 0;
const unsigned long overrideDuration = 60000; // 60 seconds
const unsigned long COMMAND_TIMEOUT = 2000; // Stop motors if no command for 2 seconds
const int WARNING_THRESHOLD = 2000;
const int STOP_THRESHOLD = 1500;
const float BATTERY_THRESHOLD = 11.5; // volts

void setup() {
  Serial.begin(115200);
  BTSerial.begin("WheelchairBase");

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

  // Bluetooth command handling
  if (BTSerial.hasClient() && BTSerial.available()) {
    String command = BTSerial.readStringUntil('\n');
    command.trim();
    Serial.println("Received: " + command);
    lastCommandTime = millis(); // Update last command time

    if (overrideActive) {
      if (command == "LEFT") turnLeft();
      else if (command == "RIGHT") turnRight();
      else stopMotors(); // ignore FORWARD/REVERSE
    } else {
      if (command == "FORWARD") moveForward();
      else if (command == "REVERSE") moveReverse();
      else if (command == "LEFT") turnLeft();
      else if (command == "RIGHT") turnRight();
      else stopMotors();
    }
  } else if (!BTSerial.hasClient()) {
    // No client connected, stop motors for safety
    stopMotors();
    lastCommandTime = 0;
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