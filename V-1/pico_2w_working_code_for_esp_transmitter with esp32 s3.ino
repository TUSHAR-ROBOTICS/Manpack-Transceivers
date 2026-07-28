/*
 * ==========================================================
 * TAS INDUSTRIES - TACTICAL CONTROL MODULE (PICO RX)
 * Hardware: Raspberry Pi Pico 2 W + TB6612FNG Motor Driver
 * Architecture: Wi-Fi Station (Client) + UDP Receiver
 * Features:
 *   - Corrected Joystick Axis Polarities (Inverted Mappings)
 *   - Active Braking & Low-Power Standby Management
 *   - Master Speed Cap via Encoder Telemetry (0 - 100%)
 *   - LED Connection Status Indicator
 *   - Failsafe Watchdog (Auto-Brake on Signal Loss)
 * ==========================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ---------------- PIN DEFINITIONS (TB6612FNG) ----------------
#define PIN_MOTOR_AIN1   18
#define PIN_MOTOR_AIN2   17
#define PIN_MOTOR_PWMA   16   // PWM Output Channel A

#define PIN_MOTOR_BIN1   5
#define PIN_MOTOR_BIN2   6
#define PIN_MOTOR_PWMB   7   // PWM Output Channel B

#define PIN_MOTOR_STBY   19   // Standby Control Pin

// ---------------- WI-FI & UDP CONFIGURATION ----------------
const char* ssid     = "TAS-TCM1";
const char* password = "Password123";
const int   udpPort  = 4210;

WiFiUDP udp;

// Packed Struct matching ESP32-S3 Transmitter Binary Layout
struct __attribute__((packed)) ControlPacket {
  int16_t joyX;       // Raw 0 - 4095
  int16_t joyY;       // Raw 0 - 4095
  uint8_t joyBtn;     // 1 if pressed, 0 if released
  int16_t encoderPos; // 0 to 36 steps
};

ControlPacket rxData;

// ---------------- STATE & FAILSAFE VARIABLES ----------------
unsigned long lastPacketTime = 0;
unsigned long lastBlinkTime  = 0;
bool ledState                = LOW;
bool isConnected             = false;

const unsigned long PACKET_TIMEOUT_MS = 1500; // Signal loss threshold
const unsigned long BLINK_INTERVAL_MS = 300;  // Searching indicator speed

// Joystick Deadzone Constants (ADC Range 0-4095)
const int JOY_CENTER   = 2047;
const int JOY_DEADZONE = 200;

// ---------------- MOTOR DRIVER CONTROL FUNCTIONS ----------------

// Completely disable driver output and place module in low-power standby
void stopMotors() {
  digitalWrite(PIN_MOTOR_STBY, LOW);

  digitalWrite(PIN_MOTOR_AIN1, LOW);
  digitalWrite(PIN_MOTOR_AIN2, LOW);
  analogWrite(PIN_MOTOR_PWMA, 0);

  digitalWrite(PIN_MOTOR_BIN1, LOW);
  digitalWrite(PIN_MOTOR_BIN2, LOW);
  analogWrite(PIN_MOTOR_PWMB, 0);
}

// Drive Motor Channels with Active Short-Braking support
void driveMotors(int speedA, int speedB) {
  // Wake TB6612FNG out of Standby
  digitalWrite(PIN_MOTOR_STBY, HIGH);

  // --- Motor A (Left Channel) ---
  if (speedA > 0) {
    digitalWrite(PIN_MOTOR_AIN1, HIGH);
    digitalWrite(PIN_MOTOR_AIN2, LOW);
  } else if (speedA < 0) {
    digitalWrite(PIN_MOTOR_AIN1, LOW);
    digitalWrite(PIN_MOTOR_AIN2, HIGH);
    speedA = -speedA; // Convert to absolute PWM magnitude
  } else {
    // Active Short-Brake mode on zero speed
    digitalWrite(PIN_MOTOR_AIN1, HIGH);
    digitalWrite(PIN_MOTOR_AIN2, HIGH);
  }

  // --- Motor B (Right Channel) ---
  if (speedB > 0) {
    digitalWrite(PIN_MOTOR_BIN1, HIGH);
    digitalWrite(PIN_MOTOR_BIN2, LOW);
  } else if (speedB < 0) {
    digitalWrite(PIN_MOTOR_BIN1, LOW);
    digitalWrite(PIN_MOTOR_BIN2, HIGH);
    speedB = -speedB; // Convert to absolute PWM magnitude
  } else {
    // Active Short-Brake mode on zero speed
    digitalWrite(PIN_MOTOR_BIN1, HIGH);
    digitalWrite(PIN_MOTOR_BIN2, HIGH);
  }

  // Output 8-bit PWM Values (0 - 255)
  analogWrite(PIN_MOTOR_PWMA, constrain(speedA, 0, 255));
  analogWrite(PIN_MOTOR_PWMB, constrain(speedB, 0, 255));
}

// Process telemetry and generate mixed differential movement values
void processControlData() {
  // 1. Calculate Master Speed Multiplier from Encoder Steps (0 to 36 -> 0.0 to 1.0)
  float speedLimitCap = (float)rxData.encoderPos / 36.0f;

  // 2. Map Raw Coordinates with Inverted Endpoints to Fix Motion Direction
  // Mapping (0 -> 4095) to (+255 -> -255) flips forward/reverse and left/right
  int throttle = map(rxData.joyY, 0, 4095, 255, -255);
  int steering = map(rxData.joyX, 0, 4095, 255, -255);

  // 3. Center Deadzone Filtering
  if (abs(rxData.joyY - JOY_CENTER) < JOY_DEADZONE) throttle = 0;
  if (abs(rxData.joyX - JOY_CENTER) < JOY_DEADZONE) steering = 0;

  // 4. Differential Drive Steering Mixing Formula
  int motorA_Speed = (throttle + steering); // Left Side
  int motorB_Speed = (throttle - steering); // Right Side

  // 5. Apply Dynamic Master Speed Governor
  motorA_Speed = (int)(motorA_Speed * speedLimitCap);
  motorB_Speed = (int)(motorB_Speed * speedLimitCap);

  // 6. Actuate Motor Driver
  driveMotors(motorA_Speed, motorB_Speed);
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);

  // Initialize Motor Control Pins
  pinMode(PIN_MOTOR_AIN1, OUTPUT);
  pinMode(PIN_MOTOR_AIN2, OUTPUT);
  pinMode(PIN_MOTOR_PWMA, OUTPUT);

  pinMode(PIN_MOTOR_BIN1, OUTPUT);
  pinMode(PIN_MOTOR_BIN2, OUTPUT);
  pinMode(PIN_MOTOR_PWMB, OUTPUT);

  pinMode(PIN_MOTOR_STBY, OUTPUT);

  // Ensure motors start completely powered off
  stopMotors();

  // Initialize Status LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Connect to ESP32-S3 Access Point
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Start UDP Listener
  udp.begin(udpPort);
}

// ---------------- MAIN LOOP ----------------
void loop() {
  // Parse incoming UDP telemetry packet
  int packetSize = udp.parsePacket();
  if (packetSize >= sizeof(ControlPacket)) {
    udp.read((char*)&rxData, sizeof(ControlPacket));
    lastPacketTime = millis();
    isConnected    = true;

    // Apply motor parameters
    processControlData();

    // Serial Debug Monitor Output
    Serial.printf("Pico RX -> Raw Y: %4d | Raw X: %4d | Speed Cap: %2d%%\n", 
                  rxData.joyY, rxData.joyX, (int)((rxData.encoderPos / 36.0f) * 100));
  }

  // Watchdog Timer: Safely stop motors if telemetry cuts out
  if (millis() - lastPacketTime > PACKET_TIMEOUT_MS) {
    if (isConnected) {
      isConnected = false;
      stopMotors(); // Force immediate driver standby mode
    }
  }

  // LED Link Status Visualizer
  if (isConnected) {
    digitalWrite(LED_BUILTIN, HIGH); // Solid ON when receiving packets
  } else {
    // Flash LED while looking for ESP32 transmitter
    if (millis() - lastBlinkTime >= BLINK_INTERVAL_MS) {
      lastBlinkTime = millis();
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState);
    }
  }
}
