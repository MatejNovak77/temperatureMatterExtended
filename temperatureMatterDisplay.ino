#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <Matter.h>
#include <MatterTemperature.h>
#include "MatterSwitches.h"

#include <U8g2lib.h>
#include <Wire.h>

#define SENSOR_CONFIG 2

// Sensor configuration - addresses only
#if SENSOR_CONFIG == 1
const uint8_t SENSOR_ADDRESSES[][8] = {
  { 0x28, 0x60, 0xC1, 0x4D, 0x40, 0x24, 0x0B, 0xA4 },
  { 0x28, 0xCC, 0x61, 0xD8, 0x40, 0x24, 0x0B, 0xD6 },
  { 0x28, 0xB2, 0xEC, 0x1A, 0x40, 0x24, 0x0B, 0x44 },
  { 0x28, 0xB6, 0xEA, 0xE7, 0x40, 0x24, 0x0B, 0xE8 },
  { 0x28, 0xED, 0x45, 0xA2, 0x40, 0x24, 0x0B, 0x03 }
};

#elif SENSOR_CONFIG == 2
const uint8_t SENSOR_ADDRESSES[][8] = {
  { 0x28, 0x94, 0x1B, 0x51, 0x00, 0x00, 0x00, 0x28 },
  { 0x28, 0xC2, 0x5A, 0x52, 0x00, 0x00, 0x00, 0x18 }
};

#else
const uint8_t* SENSOR_ADDRESSES = nullptr;
#endif

// Calculate number of sensors automatically
#if SENSOR_CONFIG > 0
const int numSensors = sizeof(SENSOR_ADDRESSES) / sizeof(SENSOR_ADDRESSES[0]);
#else
const int numSensors = 0;
#endif

const float TEMP_MIN = -30.0;
const float TEMP_MAX = 99.9;
const int MAX_RETRIES = 3;            // kept for reference; no blocking delays anymore
const int READOUT_DELAY_MS = 5000;
const int MATTER_DELAY_MS = 10000;

#define ONE_WIRE_BUS 2
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R1, /* reset=*/U8X8_PIN_NONE);  // display
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// ---------- Async conversion scheduling ----------
unsigned long lastReadout = 0;               // when we STARTED the last conversion
bool          conversionInProgress = false;
unsigned long conversionStart = 0;
const uint16_t CONVERSION_TIMEOUT_MS = 1000; // > 12-bit (750 ms) for safety

class TempSensor {
private:
  DeviceAddress address;
  MatterTemperature matterSensor;

  bool isValid(float temp) {
    return temp > TEMP_MIN && temp < TEMP_MAX && temp != 85;  // 85.0 often means "bogus"
  }

public:
  TempSensor() {
    // Default constructor - initialize with zeros
    for (int i = 0; i < 8; i++) address[i] = 0;
  }
  
  TempSensor(uint8_t addr[8]) {
    for (int i = 0; i < 8; i++) address[i] = addr[i];
  }

  void begin() {
    matterSensor.begin();
  }

  // After an async conversion completes, a single immediate read is enough.
  // No blocking retries; if invalid, we report it and try again next cycle.
  float readTemperature() {
    float temp = sensors.getTempC(address);
    if (isValid(temp)) return temp;
    return DEVICE_DISCONNECTED_C; // or invalid reading
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

// TempSensor instances - initialized later in setup()
TempSensor* tempSensors = nullptr;

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
  Serial.print(days);    Serial.print(" days, ");
  Serial.print(hours);   Serial.print(" hours, ");
  Serial.print(minutes); Serial.print(" minutes, ");
  Serial.print(seconds); Serial.println(" seconds");
}

// ---------------- Heating modes (only for SENSOR_CONFIG 2) ----------------
#if SENSOR_CONFIG == 2
static const uint8_t FURNACE_MODES[]  = { 1 };
static const uint8_t ELECTRIC_MODES[] = { 2 };

// EEPROM map suggestion for heating switches (bytes):
//   0: Furnace ON/OFF
//   1: Furnace sub-mode index
//   2: Electric ON/OFF
//   3: Electric sub-mode index
ModeSwitch furnaceSwitch("Heating: Furnace", /*state*/0, /*idx*/1, FURNACE_MODES, sizeof(FURNACE_MODES));
ModeSwitch electricSwitch("Heating: Electric", /*state*/2, /*idx*/3, ELECTRIC_MODES, sizeof(ELECTRIC_MODES));
MultiModeSelector heatingSelector(/*totalModes=*/2);
#endif

void setup() {
  printUptime();

  u8g2.begin();

  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  setLEDColor(230, 255, 255);

  Serial.begin(9600);
  sensors.begin();

  // ---- ASYNC conversions ----
  sensors.setWaitForConversion(false);   // requestTemperatures() won't block
  // Optional: only some lib versions have this; ok to omit if not available.
  #if defined(DALLASTEMPERATUREVERSION)
  sensors.setCheckForConversion(true);   // allow isConversionComplete() polling
  #endif
  // You can also speed up conversions:
  // sensors.setResolution(11); // ~375 ms instead of 750 ms

  Matter.begin();

  // print sensor addresses if numSensors == 0 -----
  if (SENSOR_CONFIG == 0) {
    Serial.println("No sensors defined in configuration. Scanning for connected sensors...");
    int count = sensors.getDeviceCount();
    Serial.print("Found "); Serial.print(count); Serial.println(" sensor(s):");

    DeviceAddress addr;
    for (int i = 0; i < count; i++) {
      if (sensors.getAddress(addr, i)) {
        Serial.print("{ ");
        for (uint8_t j = 0; j < 8; j++) {
          Serial.print("0x");
          if (addr[j] < 16) Serial.print("0");
          Serial.print(addr[j], HEX);
          if (j < 7) Serial.print(", ");
        }
        Serial.println(" }");
      } else {
        Serial.print("Unable to read address for sensor index ");
        Serial.println(i);
      }
    }

    while (true) delay(1000); // stop; user copies addresses
  }

  // Initialize TempSensor instances
  if (numSensors > 0) {
    tempSensors = new TempSensor[numSensors];
    for (int i = 0; i < numSensors; i++) {
      // Use placement new to construct with address parameter
      new (&tempSensors[i]) TempSensor((uint8_t*)SENSOR_ADDRESSES[i]);
      tempSensors[i].begin();
    }
  }

  Serial.println("");
  Serial.println("Matter temperature sensors initialized");

#if SENSOR_CONFIG == 2
  // Initialize heating mode endpoints and selector
  furnaceSwitch.begin();
  electricSwitch.begin();
  heatingSelector.attach(&furnaceSwitch);
  heatingSelector.attach(&electricSwitch);
  heatingSelector.onGroupChange([](uint8_t mode){
    const char* name = (mode==0) ? "OFF" : (mode==1) ? "FURNACE" : (mode==2) ? "ELECTRIC" : "UNKNOWN";
    Serial.printf("[Heating] Group mode -> %s (%u)\n", name, mode);
  });
#endif

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

  // Start a new conversion every READOUT_DELAY_MS (non-blocking)
  if (!conversionInProgress && (now - lastReadout >= READOUT_DELAY_MS)) {
    lastReadout = now;               // anchor schedule to conversion start
    sensors.requestTemperatures();   // kick off async conversion for ALL sensors
    conversionInProgress = true;
    conversionStart = now;
  }

  // If a conversion is running, check whether it's finished (or timed out)
  if (conversionInProgress) {
    bool ready = false;

    // Preferred: library readiness check (if supported)
    ready = sensors.isConversionComplete();
    // Fallback: time-based guard (covers libs without ready checks)
    if (!ready && (now - conversionStart >= CONVERSION_TIMEOUT_MS)) {
      ready = true;
    }

    if (ready) {
      printUptime();
      setLEDColor(230, 230, 230);

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

      // Draw new frame
      u8g2.clearBuffer();
      for (int i = 0; i < numSensors; i++) {
        tempSensors[i].update(i);  // immediate read after conversion
      }
      u8g2.sendBuffer();

      Serial.println();

      // Mark cycle complete
      conversionInProgress = false;
    }
  }

  // Update heating group state (mutual exclusivity + mode reporting)
#if SENSOR_CONFIG == 2
  heatingSelector.updateGroup();
#endif

  // No delays anywhere — loop stays responsive, Matter can breathe.
}
