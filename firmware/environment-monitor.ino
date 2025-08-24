// c++
#include <Wire.h>
#include <Adafruit_SGP30.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Preferences.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WebServer.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define BUZZER_PIN 19

// I2C pins for ESP32
#define SDA_PIN 21
#define SCL_PIN 22

// WiFi credentials
const char* ssid = "";
const char* password = "";

// Web server on port 80
WebServer server(80);

// buzzer control
bool buzzerOn = false;
unsigned long buzzerStartTime = 0;
bool alreadyBeeped = false;

// Preferences for storing baseline
Preferences preferences;
unsigned long lastBaselineSave = 0;

// define sensor objects
Adafruit_BME280 bme;  // BME280 object
Adafruit_SGP30 sgp;   // SGP30 object

// create display object
LiquidCrystal_I2C lcd(0x27, 16, 2);

unsigned long lastSwitch = 0;
int displayPage = 0;

// absolute humidity function
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
  const float absHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature));
  return static_cast<uint32_t>(absHumidity * 1000.0f); // mg/m³
}

// ESP32 web server text
void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<title>Environment Monitor</title>";
  html += "<meta http-equiv='refresh' content='5'>";  // Auto-refresh
  html += "</head><body>";
  html += "<h1>Environment Monitor</h1>";
  html += "<p>Temperature: " + String(bme.readTemperature(), 1) + " °C</p>";
  html += "<p>Humidity: " + String(bme.readHumidity(), 1) + " %</p>";
  html += "<p>Pressure: " + String(bme.readPressure()/100.0F, 0) + " hPa</p>";
  html += "<p>Weather Forecast: " + String(forecast)
  html += "<p>eCO2: " + String(sgp.eCO2) + " ppm</p>";
  html += "<p>TVOC: " + String(sgp.TVOC) + " ppb</p>";
  html += "<p>Air Quality: " + String(air_quality);
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  unsigned long serialTimeout = millis();
  while (!Serial && (millis() - serialTimeout < 1000)) { delay(10); }

  // setup buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // setup lcd
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  Wire.begin(SDA_PIN, SCL_PIN);

  // Connect to WiFi
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // Setup web server
  server.on("/", handleRoot);
  server.begin();

  if (!bme.begin(0x77)) {
    Serial.println("BME280 not found!");
    while (1) delay(10);
  }

  if (!sgp.begin()) {
    Serial.println("SGP30 not found!");
    while (1) delay(10);
  }

  // Open preferences (namespace "sgp30")
  preferences.begin("sgp30", false);

  // Try to restore baseline
  uint32_t eco2_base = preferences.getUInt("eco2_base", 0);
  uint32_t tvoc_base = preferences.getUInt("tvoc_base", 0);
  if (eco2_base != 0 && tvoc_base != 0) {
    sgp.setIAQBaseline((uint16_t)eco2_base, (uint16_t)tvoc_base);
    Serial.print("Restored baseline -> eCO2: 0x");
    Serial.print(eco2_base, HEX);
    Serial.print(" TVOC: 0x");
    Serial.println(tvoc_base, HEX);
  } else {
    Serial.println("No baseline stored yet, running fresh calibration...");
  }

  Serial.println("Sensors initialized!");
}

void loop() {
  server.handleClient();  // Handle web requests

  // BME280 measurements
  float temperature = bme.readTemperature(); // °C
  float humidity = bme.readHumidity();       // %
  float pressure = bme.readPressure() / 100.0F; // hPa
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA); // m

  // feed absolute humidity to SGP30
  sgp.setHumidity(getAbsoluteHumidity(temperature, humidity));

  // SGP30 measurements
  if (!sgp.IAQmeasure()) {
    Serial.println("SGP30 measurement failed");
    return;
  }

  // print to serial monitor
  Serial.print("Temperature: "); Serial.print(temperature); Serial.print(" °C, ");
  Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");

  Serial.print("Pressure: "); Serial.print(pressure); Serial.print(" hPa, ");
  Serial.print("Altitude: "); Serial.print(altitude); Serial.println(" m");

  Serial.print("eCO2: "); Serial.print(sgp.eCO2); Serial.print(" ppm, ");
  Serial.print("TVOC: "); Serial.println(sgp.TVOC);

  // weather forecast
  String forecast;
  static float lastPressure = 0;

  if (lastPressure == 0) {
    lastPressure = pressure; // only on first run
  }

  if (pressure > 1020) {
    forecast = "Sunny";
  } else if (pressure < 1000) {
    forecast = "Rainy";
  } else {
    forecast = "Cloudy";
  }

  // trend detection
  if (pressure < lastPressure - 1) {
    forecast += " falling";   // falling pressure
  } else if (pressure > lastPressure + 1) {
    forecast += " rising";   // rising pressure
  }

  lastPressure = pressure;  // store for next loop

  // air quality
  String air_quality;
  if (sgp.eCO2 < 800) {
    air_quality = "Good";
    alreadyBeeped = false;
    digitalWrite(BUZZER_PIN, LOW);
  } else if (sgp.eCO2 < 1200) {
    air_quality = "Moderate";
    alreadyBeeped = false;
    digitalWrite(BUZZER_PIN, LOW);
  } else {
    air_quality = "Poor";

    if (!alreadyBeeped) {
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerStartTime = millis();
      buzzerOn = true;
      alreadyBeeped = true;
    }
  }

  // turn buzzer off after 500 ms
  if (buzzerOn && millis() - buzzerStartTime >= 500) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerOn = false;
  }


  // print to lcd screen
  if (millis() - lastSwitch > 5000) {
    displayPage = (displayPage + 1) % 5; // 5 pages rotating
    lastSwitch = millis();

    lcd.clear();

    if (displayPage == 0) {
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(temperature, 1);
      lcd.print("C");

      lcd.setCursor(0, 1);
      lcd.print("Hum: ");
      lcd.print(humidity, 1);
      lcd.print("%");
    }
    else if (displayPage == 1) {
      lcd.setCursor(0, 0);
      lcd.print("Press: ");
      lcd.print(pressure, 0);
      lcd.print("hPa");

      lcd.setCursor(0, 1);
      lcd.print("Alt: ");
      lcd.print(altitude, 0);
      lcd.print("m");
    }
    else if (displayPage == 2) {
      lcd.setCursor(0, 0);
      lcd.print("Forecast:");

      lcd.setCursor(0, 1);
      lcd.print(forecast);
    }
    else if (displayPage == 3) {
      lcd.setCursor(0, 0);
      lcd.print("eCO2: ");
      lcd.print(sgp.eCO2);
      lcd.print("ppm");

      lcd.setCursor(0, 1);
      lcd.print("TVOC: ");
      lcd.print(sgp.TVOC);
      lcd.print("ppb");
    }
    else if (displayPage == 4) {
      lcd.setCursor(0, 0);
      lcd.print("Air Quality: ");
      lcd.setCursor(0, 1);
      lcd.print(air_quality);
    }
  }

  // Save baseline every 5 hours
  if (millis() - lastBaselineSave > 5UL * 3600000UL) { // 5 hours
    uint16_t eco2_base_val, tvoc_base_val;
    if (sgp.getIAQBaseline(&eco2_base_val, &tvoc_base_val)) {
      preferences.putUInt("eco2_base", eco2_base_val);
      preferences.putUInt("tvoc_base", tvoc_base_val);
      Serial.print("Saved baseline -> eCO2: 0x");
      Serial.print(eco2_base_val, HEX);
      Serial.print(" TVOC: 0x");
      Serial.println(tvoc_base_val, HEX);
    }
    lastBaselineSave = millis();
  }

  delay(1000);
}
