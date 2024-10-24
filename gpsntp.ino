#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_timer.h"

volatile int64_t lastPPSTime = 0;

// GPS Setup
TinyGPSPlus gps;
HardwareSerial ss(1); // UART1

volatile unsigned long lastPPSMicros = 0;
volatile bool ppsFlag = false;
time_t currentTime = 0;
time_t lastValidGPSTime = 0;
const int RXPin = 16, TXPin = 17;

// PPS Setup
#define PPS_PIN 23 // Replace with the GPIO pin connected to the PPS signal

// Wi-Fi Setup
const char* ssid = "TP-Link_E854";
const char* password = "13772857";

// NTP Setup
WiFiUDP ntpUDP;
const int NTP_PORT = 123;

// OLED Setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

void IRAM_ATTR ppsISR() {
  lastPPSTime = esp_timer_get_time();  // Returns time in microseconds
  ppsFlag = true;
}

void setup() {
  ss.begin(9600, SERIAL_8N1, RXPin, TXPin);

  connectToWiFi();
  ntpUDP.begin(NTP_PORT);

  Wire.begin(21, 22);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    ESP.restart();
  }
  display.clearDisplay();

  pinMode(PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), ppsISR, RISING);

  xTaskCreatePinnedToCore(
    handlePPSTask,
    "Handle PPS",
    8192,
    NULL,
    configMAX_PRIORITIES - 1,
    NULL,
    1
  );
}

void loop() {
  while (ss.available() > 0) {
    gps.encode(ss.read());
  }

  handleNTPRequest();
  updateDisplay();
}

void handlePPSTask(void * parameter) {
  for (;;) {
    if (ppsFlag) {
      ppsFlag = false;
      if (gps.time.isValid() && gps.date.isValid()) {
        tmElements_t tm;
        tm.Year = gps.date.year() - 1970;
        tm.Month = gps.date.month();
        tm.Day = gps.date.day();
        tm.Hour = gps.time.hour();
        tm.Minute = gps.time.minute();
        tm.Second = gps.time.second();
        currentTime = makeTime(tm);
        lastValidGPSTime = currentTime;
      } else if (lastValidGPSTime != 0) {
        currentTime = lastValidGPSTime + ((esp_timer_get_time() - lastPPSTime) / 1000000);
      } else {
        currentTime++;
      }
    }
    if (WiFi.status() != WL_CONNECTED) {
      connectToWiFi();
    }
    vTaskDelay(1);
  }
}
void connectToWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(100);
  }
}

void handleNTPRequest() {
  if (ntpUDP.parsePacket()) {
    uint32_t refTimestampSec, refTimestampFrac;
    getNTPTimestamp(&refTimestampSec, &refTimestampFrac);

    uint8_t packetBuffer[48];
    ntpUDP.read(packetBuffer, 48);

    // Extract Originate Timestamp (T1)
    uint32_t originateTimestampSec = ntohl(*(uint32_t*)&packetBuffer[40]);
    uint32_t originateTimestampFrac = ntohl(*(uint32_t*)&packetBuffer[44]);

    // Prepare NTP response packet
    memset(packetBuffer, 0, 48);
    packetBuffer[0] = 0b00100100; // LI, Version, Mode (4 for server)
    packetBuffer[1] = 1;          // Stratum, 1 for primary reference (GPS)
    packetBuffer[2] = 0;          // Poll interval
    packetBuffer[3] = 0xFA;       // Precision (set appropriate precision value)

    // Reference ID can be set to "GPS\0"
    packetBuffer[12] = 'G';
    packetBuffer[13] = 'P';
    packetBuffer[14] = 'S';
    packetBuffer[15] = '\0';

    // Originate Timestamp (Copied from client's Transmit Timestamp) T1
    *(uint32_t*)&packetBuffer[24] = htonl(originateTimestampSec);
    *(uint32_t*)&packetBuffer[28] = htonl(originateTimestampFrac);

    // Receive Timestamp (Time at which the request arrived at the server) T2
    *(uint32_t*)&packetBuffer[32] = htonl(refTimestampSec);
    *(uint32_t*)&packetBuffer[36] = htonl(refTimestampFrac);

    // Reference Timestamp (time when system clock was last set or corrected)
    *(uint32_t*)&packetBuffer[16] = htonl(getLastGPSNTPTimestamp());
    *(uint32_t*)&packetBuffer[20] = htonl(getLastGPSNTPTimestamp());

    // Transmit Timestamp (Time at which the reply is sent) T3
    uint32_t transmitTimestampSec, transmitTimestampFrac;
    getNTPTimestamp(&transmitTimestampSec, &transmitTimestampFrac);
    *(uint32_t*)&packetBuffer[40] = htonl(transmitTimestampSec);
    *(uint32_t*)&packetBuffer[44] = htonl(transmitTimestampFrac);

    ntpUDP.beginPacket(ntpUDP.remoteIP(), ntpUDP.remotePort());
    ntpUDP.write(packetBuffer, 48);
    ntpUDP.endPacket();
  }
}

void updateDisplay() {
  tmElements_t tm;
  breakTime(currentTime, tm);

  int displayYear = tm.Year + 1970;
  int displayMonth = tm.Month;
  int displayDay = tm.Day;
  int displayHour = tm.Hour;
  int displayMinute = tm.Minute;
  int displaySecond = tm.Second;

  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.printf("Date: %02d/%02d/%04d", displayDay, displayMonth, displayYear);

  display.setCursor(0, 16);
  display.setTextSize(2);
  display.printf("%02d:%02d:%02d", displayHour, displayMinute, displaySecond);

  display.display();
}

void getNTPTimestamp(uint32_t* seconds, uint32_t* fraction) {
  noInterrupts();  // Ensure atomic access to shared variables
  int64_t currentMicros = esp_timer_get_time() + 800000;
  int64_t microsSincePPS = currentMicros - lastPPSTime;

  // Calculate the NTP timestamp seconds part
  uint32_t epochSeconds = (uint32_t)(currentTime + (microsSincePPS / 1000000LL)) + 2208988800UL;

  // Calculate the NTP timestamp fractional part (convert microseconds to fractions of a second in NTP format)
  uint32_t epochFraction = (uint32_t)(((microsSincePPS % 1000000LL) * 4294.967296) + 0.5);

  *seconds = epochSeconds;
  *fraction = epochFraction;
  interrupts();  // Re-enable interrupts
}

uint32_t getLastGPSNTPTimestamp() {
  // Calculate the NTP timestamp seconds part
  return (uint32_t)lastValidGPSTime+ 2208988800UL;
}
