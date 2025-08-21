// c++
#include <Wire.h>
#include <Adafruit_SGP30.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)

// absolute humidity function
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
  const float absHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature));
  return static_cast<uint32_t>(absHumidity * 1000.0f); // mg/m³
}

void loop() {
  float temperature = bme.readTemperature(); // °C
  float humidity = bme.readHumidity();       // %

  // feed absolute humidity to SGP30
  sgp.setHumidity(getAbsoluteHumidity(temperature, humidity));

  if (!sgp.IAQmeasure()) {
    Serial.println("SGP30 measurement failed");
    return;
  }

  Serial.print("Temperature: "); Serial.print(temperature); Serial.print(" °C, ");
  Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");

  Serial.print("eCO2: "); Serial.print(sgp.eCO2); Serial.print(" ppm, ");
  Serial.print("TVOC: "); Serial.println(sgp.TVOC);

  delay(1000);
}
