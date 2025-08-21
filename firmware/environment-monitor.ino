// c++
#include <Wire.h>
#include <Adafruit_SGP30.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)

// I2C pins for ESP32
#define SDA_PIN 21
#define SCL_PIN 22

// define sensor objects
Adafruit_BME280 bme;  // BME280 object
Adafruit_SGP30 sgp;    // SGP30 object

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

  Serial.println("Sensors initialized!");
}

void loop() {
  // BME280 measurements
  float temperature = bme.readTemperature(); // °C
  float humidity = bme.readHumidity(); // %
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
  Serial.print("Altitude: "); Serial.print(altitude); Serial.print(" m");

  Serial.print("eCO2: "); Serial.print(sgp.eCO2); Serial.print(" ppm, ");
  Serial.print("TVOC: "); Serial.println(sgp.TVOC);

  delay(1000);
}
