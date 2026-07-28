/*
 * ==========================================================
 * TAS INDUSTRIES - TACTICAL CONTROL MODULE (TCM-1 TX)
 * Hardware: ESP32-S3-N16R8
 * Architecture: Local Wi-Fi Access Point + UDP Broadcast
 * Display: OLED (128x64 SH1106, Fast 400kHz Hardware I2C)
 * ==========================================================
 */

#include <Arduino.h> 
#include <Wire.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>

// ---------------- PIN DEFINITIONS ----------------
#define PIN_OLED_SDA     8
#define PIN_OLED_SCL     9

#define PIN_JOY_X        4
#define PIN_JOY_Y        5
#define PIN_JOY_SW       6   // VRz

#define PIN_ENC_CLK      7
#define PIN_ENC_DT       15
#define PIN_ENC_SW       16

#define PIN_BUZZER       13

#define OLED_I2C_ADDR    0x3C

// ---------------- WI-FI & UDP CONFIGURATION ----------------
const char* ssid     = "TAS-TCM1";
const char* password = "Password123";
const int   udpPort  = 4210;

WiFiUDP udp;
IPAddress broadcastIP(192, 168, 4, 255);

// Packed Struct for Cross-Platform Byte Alignment
struct __attribute__((packed)) ControlPacket {
  int16_t joyX;       // Raw 0 - 4095
  int16_t joyY;       // Raw 0 - 4095
  uint8_t joyBtn;     // 1 if pressed, 0 if released
  int16_t encoderPos; // 0 to 36 steps
};

ControlPacket txData;

// ---------------- DISPLAY OBJECT ----------------
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// ---------------- GLOBAL STATE ----------------
volatile int32_t encoderPos = 0;
volatile uint8_t lastClkState = HIGH;

bool encBtnState = false;
unsigned long lastEncBtnTime = 0;

int joyXRaw = 0;
int joyYRaw = 0;
bool joyBtnState = false;

bool buzzerActive = false;
const int ENCODER_MAX_STEPS  = 36;
const int ENCODER_50_PERCENT = 18;

// Timers
unsigned long lastOledUpdate  = 0;
unsigned long lastUdpSend     = 0;

// Dynamic link status tracking
bool picoIsConnected = false;

// ---------------- ENCODER ISR ----------------
void IRAM_ATTR readEncoderISR() {
  uint8_t currentClk = digitalRead(PIN_ENC_CLK);
  if (currentClk != lastClkState) {
    bool isClockwise = (digitalRead(PIN_ENC_DT) != currentClk);

    // FIXED: Consistent direction calculation regardless of connection state
    if (isClockwise) {
      encoderPos++;
    } else {
      encoderPos--;
    }

    if (encoderPos < 0) encoderPos = 0;
    if (encoderPos > ENCODER_MAX_STEPS) encoderPos = ENCODER_MAX_STEPS;
    
    lastClkState = currentClk;
  }
}

// ---------------- CORNER BRACKET HELPER ----------------
void drawCornerBrackets(int x, int y, int w, int h, int len) {
  u8g2.drawLine(x, y, x + len, y);
  u8g2.drawLine(x, y, x, y + len);
  u8g2.drawLine(x + w - len, y, x + w, y);
  u8g2.drawLine(x + w, y, x + w, y + len);
  u8g2.drawLine(x, y + h - len, x, y + h);
  u8g2.drawLine(x, y + h, x + len, y + h);
  u8g2.drawLine(x + w - len, y + h, x + w, y + h);
  u8g2.drawLine(x + w, y + h - len, x + w, y + h);
}

// ---------------- BOOT SPLASH ----------------
void showBootSplash() {
  u8g2.clearBuffer();
  drawCornerBrackets(2, 19, 124, 43, 6);
  
  u8g2.setFont(u8g2_font_6x12_tf);
  const char* l1 = "TAS INDUSTRIES";
  u8g2.drawStr((128 - u8g2.getStrWidth(l1)) / 2, 36, l1);

  u8g2.setFont(u8g2_font_5x7_tf);
  const char* l2 = "TCM-1 AP ACTIVE";
  u8g2.drawStr((128 - u8g2.getStrWidth(l2)) / 2, 50, l2);

  u8g2.sendBuffer();
  delay(1200);
}

// ---------------- INPUT READING & ALARM ----------------
void readInputs() {
  // Check station connection count
  picoIsConnected = (WiFi.softAPgetStationNum() > 0);

  // FIXED: Straight raw ADC readings without dynamic inversion
  joyXRaw = analogRead(PIN_JOY_X);
  joyYRaw = analogRead(PIN_JOY_Y);

  joyBtnState = (digitalRead(PIN_JOY_SW) == LOW);

  if (digitalRead(PIN_ENC_SW) == LOW) {
    if (millis() - lastEncBtnTime > 150) {
      encBtnState = true;
      lastEncBtnTime = millis();
    }
  } else {
    encBtnState = false;
  }

  // Speed >= 50% Continuous Alarm Logic
  if (encoderPos >= ENCODER_50_PERCENT) {
    buzzerActive = true;
    tone(PIN_BUZZER, 2500);
  } else {
    buzzerActive = false;
    noTone(PIN_BUZZER);
    digitalWrite(PIN_BUZZER, LOW);
  }
}

// ---------------- BROADCAST UDP PACKET ----------------
void sendUdpPacket() {
  txData.joyX       = (int16_t)joyXRaw;
  txData.joyY       = (int16_t)joyYRaw;
  txData.joyBtn     = joyBtnState ? 1 : 0;
  txData.encoderPos = (int16_t)encoderPos;

  udp.beginPacket(broadcastIP, udpPort);
  udp.write((uint8_t*)&txData, sizeof(ControlPacket));
  udp.endPacket();
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_ENC_CLK, INPUT_PULLUP);
  pinMode(PIN_ENC_DT, INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);
  pinMode(PIN_JOY_SW, INPUT_PULLUP);

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  // Initialize I2C Bus at 400kHz
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  Wire.setClock(400000);
  u8g2.begin();

  lastClkState = digitalRead(PIN_ENC_CLK);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), readEncoderISR, CHANGE);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  udp.begin(udpPort);

  showBootSplash();
}

// ---------------- DASHBOARD RENDER ----------------
void renderDashboard() {
  u8g2.clearBuffer();

  u8g2.drawHLine(0, 19, 128);

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(2, 27, "TAS-IND");
  
  int encPct = (encoderPos * 100) / ENCODER_MAX_STEPS;
  u8g2.setCursor(48, 27);
  u8g2.print("SPD:");
  u8g2.print(encPct);
  u8g2.print("%");

  // Visual status indicators
  if (picoIsConnected) {
    u8g2.drawStr(96, 27, "LINK");
    u8g2.drawDisc(122, 24, 2);   // Solid dot = Connected
  } else {
    u8g2.drawStr(96, 27, "WAIT");
    u8g2.drawCircle(122, 24, 2); // Hollow dot = Waiting
  }

  u8g2.drawHLine(0, 29, 128);

  // Panel 1: Reticle Target
  int boxX = 6, boxY = 31, boxSize = 31; 
  drawCornerBrackets(boxX, boxY, boxSize, boxSize, 4);

  int dotX = map(joyYRaw, 0, 4095, boxX + boxSize - 3, boxX + 3);
  int dotY = map(joyXRaw, 0, 4095, boxY + 3, boxY + boxSize - 3);

  u8g2.drawCircle(dotX, dotY, 2);
  u8g2.drawLine(dotX - 4, dotY, dotX - 2, dotY);
  u8g2.drawLine(dotX + 2, dotY, dotX + 4, dotY);
  u8g2.drawLine(dotX, dotY - 4, dotX, dotY - 2);
  u8g2.drawLine(dotX, dotY + 2, dotX, dotY + 4);

  if (joyBtnState) u8g2.drawDisc(dotX, dotY, 2);

  // Panel 2: Numeric Telemetry
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setCursor(44, 38);
  u8g2.print("X:");
  u8g2.print(joyXRaw);

  u8g2.setCursor(44, 46);
  u8g2.print("Y:");
  u8g2.print(joyYRaw);

  u8g2.setCursor(44, 54);
  u8g2.print("AZ:");
  u8g2.print((encoderPos * 10) % 360);
  u8g2.print((char)176);

  if (buzzerActive) {
    u8g2.drawBox(43, 56, 34, 7);
    u8g2.setDrawColor(0);
    u8g2.drawStr(44, 62, "!ALARM!");
    u8g2.setDrawColor(1);
  } else {
    u8g2.drawFrame(43, 56, 34, 7);
    u8g2.drawStr(45, 62, "NORMAL");
  }

  // Panel 3: Radar Dial
  int dialCx = 104, dialCy = 46, dialR = 15;
  u8g2.drawCircle(dialCx, dialCy, dialR);

  for (int a = 0; a < 360; a += 90) {
    float rad = a * PI / 180.0;
    int x1 = dialCx + (int)((dialR - 2) * sin(rad));
    int y1 = dialCy - (int)((dialR - 2) * cos(rad));
    int x2 = dialCx + (int)(dialR * sin(rad));
    int y2 = dialCy - (int)(dialR * cos(rad));
    u8g2.drawLine(x1, y1, x2, y2);
  }

  float needleAngle = (encoderPos % 36) * (2 * PI / 36.0);
  int needleX = dialCx + (int)((dialR - 2) * sin(needleAngle));
  int needleY = dialCy - (int)((dialR - 2) * cos(needleAngle));
  u8g2.drawLine(dialCx, dialCy, needleX, needleY);

  u8g2.drawDisc(dialCx, dialCy, encBtnState ? 3 : 1);

  u8g2.sendBuffer();
}

// ---------------- MAIN LOOP ----------------
void loop() {
  readInputs();

  if (millis() - lastUdpSend >= 20) {
    lastUdpSend = millis();
    sendUdpPacket();
  }

  if (millis() - lastOledUpdate >= 33) {
    lastOledUpdate = millis();
    renderDashboard();
  }
}
