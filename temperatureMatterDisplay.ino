#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <Matter.h>
#include <MatterTemperature.h>

#include <U8g2lib.h>
#include <Wire.h>

#define SENSOR_CONFIG 2 

const float TEMP_MIN = -30.0;
const float TEMP_MAX = 99.9;
const int MAX_RETRIES = 3;
const int RETRY_DELAY_MS = 300;
const int READOUT_DELAY_MS = 5000;
const int LOOP_DELAY_MS = 100;

#define ONE_WIRE_BUS 2
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R1, /* reset=*/U8X8_PIN_NONE);  // display
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
unsigned long lastReadout = 0; 

class TempSensor {
private:
  DeviceAddress address;
  MatterTemperature matterSensor;

  bool isValid(float temp) {
    return temp > TEMP_MIN && temp < TEMP_MAX && temp != 85;  // 85.0 is sometimes returned as faulty value
  }

public:
  TempSensor(uint8_t addr[8]) {
    for (int i = 0; i < 8; i++) {
      address[i] = addr[i];
    }
  }

  void begin() {
    matterSensor.begin();
  }

  float readTemperature() {
    float temp;
    for (int attempt = 0; attempt <= MAX_RETRIES; attempt++) {
      temp = sensors.getTempC(address);
      if (isValid(temp)) {
        return temp;
      }
      delay(RETRY_DELAY_MS);
    }
    return DEVICE_DISCONNECTED_C;
  }

  void printTemperature(int index, float temp) {
    Serial.print("Sensor ");
    Serial.print(index + 1);
    Serial.print(": ");
    Serial.print(temp);
    Serial.println(" °C");
  }

  void update(int index) {
    float temp = readTemperature();

    // send to serial and Matter
    if (isValid(temp)) {
      printTemperature(index, temp);
      matterSensor.set_measured_value_celsius(temp);
    } else {
      Serial.print("Sensor ");
      Serial.print(index + 1);
      Serial.print(": invalid reading: ");
      Serial.print(temp);
      Serial.println("°C (skipped)");
    }

    // ---- draw on display ----
      const uint8_t* indexFont = u8g2_font_helvB10_tf;
      const uint8_t* valueFont = u8g2_font_helvR12_tf;

      int y = 24 * (index + 1);

      u8g2.setFont(indexFont);
      uint8_t charW = u8g2.getMaxCharWidth();
      uint8_t charH = u8g2.getMaxCharHeight();

      u8g2.setDrawColor(1);
      u8g2.drawBox(0, y - charH + 3, charW - 1, charH - 1);

      u8g2.setDrawColor(0);
      char sensorChar[2];
      sprintf(sensorChar, "%d", index + 1);
      u8g2.drawStr(3, y, sensorChar);

      u8g2.setFont(valueFont);
      u8g2.setDrawColor(1);
      char tempStr[16];
      if (isValid(temp)) {
        sprintf(tempStr, "%.1f\xB0", temp);
      } else {
        sprintf(tempStr, "--.-\xB0");
      }

      int textWidth = u8g2.getStrWidth(tempStr);
      int x = 64 - textWidth;  // rotated 90°, width is 64
      u8g2.drawStr(x, y, tempStr);
  }

  bool isOnline() {
    return matterSensor.is_online();
  }
};

// sensor addresses
  #if SENSOR_CONFIG == 1
  TempSensor tempSensors[] = {
    TempSensor((uint8_t[8]){ 0x28, 0x60, 0xC1, 0x4D, 0x40, 0x24, 0x0B, 0xA4 }),
    TempSensor((uint8_t[8]){ 0x28, 0xCC, 0x61, 0xD8, 0x40, 0x24, 0x0B, 0xD6 }),
    TempSensor((uint8_t[8]){ 0x28, 0xB2, 0xEC, 0x1A, 0x40, 0x24, 0x0B, 0x44 }),
    TempSensor((uint8_t[8]){ 0x28, 0xB6, 0xEA, 0xE7, 0x40, 0x24, 0x0B, 0xE8 }),
    TempSensor((uint8_t[8]){ 0x28, 0xED, 0x45, 0xA2, 0x40, 0x24, 0x0B, 0x03 })
  };
  const int numSensors = 5;

  #elif SENSOR_CONFIG == 2
  TempSensor tempSensors[] = {
    TempSensor((uint8_t[8]){ 0x28, 0x94, 0x1B, 0x51, 0x00, 0x00, 0x00, 0x28 }),
    TempSensor((uint8_t[8]){ 0x28, 0xC2, 0x5A, 0x52, 0x00, 0x00, 0x00, 0x18 })
  };
  const int numSensors = 2;

  #else
  TempSensor* tempSensors = nullptr;
  const int numSensors = 0;
  #endif

void setLEDColor(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(LEDR, r);
  analogWrite(LEDG, g);
  analogWrite(LEDB, b);
}

void printUptime() {
  unsigned long totalSeconds = millis() / 1000;
  int days = totalSeconds / 86400;
  int hours = (totalSeconds % 86400) / 3600;
  int minutes = (totalSeconds % 3600) / 60;
  int seconds = totalSeconds % 60;

  Serial.print("Uptime: ");
  Serial.print(days);
  Serial.print(" days, ");
  Serial.print(hours);
  Serial.print(" hours, ");
  Serial.print(minutes);
  Serial.print(" minutes, ");
  Serial.print(seconds);
  Serial.println(" seconds");
}

void setup() {
  printUptime();

  u8g2.begin();

  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  setLEDColor(230, 255, 255);

  Serial.begin(9600);
  sensors.begin();
  sensors.setResolution(11); // ~375 ms instead of 750 ms
  Matter.begin();

  // print sensor addresses if numSensors == 0 -----
  if (SENSOR_CONFIG == 0) {
    Serial.println("No sensors defined in configuration. Scanning for connected sensors...");
    int count = sensors.getDeviceCount();
    Serial.print("Found ");
    Serial.print(count);
    Serial.println(" sensor(s):");

    DeviceAddress addr;
    for (int i = 0; i < count; i++) {
      if (sensors.getAddress(addr, i)) {
        Serial.print("{ ");
        for (uint8_t j = 0; j < 8; j++) {
          Serial.print("0x");
          if (addr[j] < 16) Serial.print("0");  // leading zero for values < 0x10
          Serial.print(addr[j], HEX);
          if (j < 7) Serial.print(", ");
        }
        Serial.println(" }");
      } else {
        Serial.print("Unable to read address for sensor index ");
        Serial.println(i);
      }
    }

    // Stop execution if no configured sensors
    while (true) delay(1000);
  }

  for (int i = 0; i < numSensors; i++) {
    tempSensors[i].begin();
  }

  Serial.println("");
  Serial.println("Matter temperature sensors initialized");

  if (!Matter.isDeviceCommissioned()) {
    Serial.println("Matter device is not commissioned");
    Serial.printf("Manual pairing code: %s\n", Matter.getManualPairingCode().c_str());
    Serial.printf("QR code URL: %s\n", Matter.getOnboardingQRCodeUrl().c_str());
  }
  while (!Matter.isDeviceCommissioned()) delay(200);

  printUptime();
  Serial.println("Waiting for Thread network...");
  while (!Matter.isDeviceThreadConnected()) delay(200);

  printUptime();
  Serial.println("Connected to Thread network");
  setLEDColor(255, 255, 230);
}


void loop() {
  unsigned long now = millis();

  // Only perform sensor readout + display update if interval elapsed
  if (now - lastReadout >= READOUT_DELAY_MS) {
    lastReadout = now;

    printUptime();
    setLEDColor(230, 230, 230);

    sensors.requestTemperatures();

    int onlineCount = 0;
    bool allOnline = true;
    for (int i = 0; i < numSensors; i++) {
      if (tempSensors[i].isOnline()) {
        onlineCount++;
      } else {
        allOnline = false;
      }
    }

    Serial.print("Matter devices online: ");
    Serial.print(onlineCount);
    Serial.print(" / ");
    Serial.println(numSensors);

    if (allOnline) {
      setLEDColor(255, 230, 255);
    } else {
      setLEDColor(255, 255, 230);
    }

    // clear display buffer before drawing new frame
    u8g2.clearBuffer();

    // draw all sensor rows
    for (int i = 0; i < numSensors; i++) {
      tempSensors[i].update(i);
    }

    // send buffer to display
    u8g2.sendBuffer();

    Serial.println();
    delay(LOOP_DELAY_MS);
  }
  
}