// c++
#include <Wire.h>
#include <Adafruit_SGP30.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Preferences.h>

#define SEALEVELPRESSURE_HPA (1013.25)

// I2C pins for ESP32
#define SDA_PIN 21
#define SCL_PIN 22

// Preferences for storing baseline
Preferences preferences;
unsigned long lastBaselineSave = 0;

// define sensor objects
Adafruit_BME280 bme;  // BME280 object
Adafruit_SGP30 sgp;   // SGP30 object

// absolute humidity function
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    const float absHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature));
    return static_cast<uint32_t>(absHumidity * 1000.0f); // mg/m³
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    Wire.begin(SDA_PIN, SCL_PIN);

    if (!bme.begin(0x76)) {
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
