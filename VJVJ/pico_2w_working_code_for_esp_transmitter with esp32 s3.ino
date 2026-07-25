/*
 * ==========================================================
 * RASPBERRY PI PICO 2 W - UDP MOTOR RECEIVER (C++)
 * Board: Raspberry Pi Pico 2 W (Earle Philhower Core)
 * ==========================================================
 */

#include <WiFi.h>
#include <WiFiUdp.h>

// ---------------- PIN DEFINITIONS ----------------
#define PIN_IN1 3
#define PIN_IN2 4
#define PIN_ENA 2

#define PIN_IN3 6
#define PIN_IN4 7
#define PIN_ENB 5

// ---------------- WI-FI & UDP CONFIGURATION ----------------
const char* ssid     = "TAS-TCM1";
const char* password = "Password123";
const int   udpPort  = 4210;

WiFiUDP udp;

// Packed Struct matching the ESP32 transmitter
struct __attribute__((packed)) ControlPacket {
  int16_t joyX;       // Raw 0 - 4095
  int16_t joyY;       // Raw 0 - 4095
  uint8_t joyBtn;     // 1 pressed, 0 released
  int16_t encoderPos; // 0 to 36
};

ControlPacket rxData;
unsigned long lastPacketTime = 0;

void setMotors(float leftSpeed, float rightSpeed) {
  // Left Motor
  if (leftSpeed > 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  } else if (leftSpeed < 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
  }
  analogWrite(PIN_ENA, (int)(fabs(leftSpeed) * 255.0f));

  // Right Motor
  if (rightSpeed > 0) {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
  } else if (rightSpeed < 0) {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
  } else {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
  }
  analogWrite(PIN_ENB, (int)(fabs(rightSpeed) * 255.0f));
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);

  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);

  // Stop motors initially
  setMotors(0.0f, 0.0f);

  // Connect to ESP32 AP
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
  }

  udp.begin(udpPort);
}

void loop() {
  int packetSize = udp.parsePacket();

  if (packetSize == sizeof(ControlPacket)) {
    udp.read((char*)&rxData, sizeof(ControlPacket));
    lastPacketTime = millis();

    // Normalize Joystick (-1.0 to 1.0)
    float normY = (rxData.joyY - 2048) / 2048.0f;
    float normX = (rxData.joyX - 2048) / 2048.0f;

    // Deadzone filter
    if (fabs(normY) < 0.08f) normY = 0.0f;
    if (fabs(normX) < 0.08f) normX = 0.0f;

    // Encoder Speed Governor (0-36 -> 0.0 to 1.0)
    float maxThrottle = (float)rxData.encoderPos / 36.0f;

    // Differential Steering Calculation
    float leftMotor  = (normY + normX) * maxThrottle;
    float rightMotor = (normY - normX) * maxThrottle;

    // Clamp values between -1.0 and 1.0
    leftMotor  = constrain(leftMotor, -1.0f, 1.0f);
    rightMotor = constrain(rightMotor, -1.0f, 1.0f);

    setMotors(leftMotor, rightMotor);
  }

  // Safety Timeout: Stop motors if signal lost for > 500ms
  if (millis() - lastPacketTime > 500) {
    setMotors(0.0f, 0.0f);
  }
}