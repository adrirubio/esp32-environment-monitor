# esp32-environment-monitor
ESP32-based environment and air quality monitor

![Environment Monitor](https://raw.githubusercontent.com/adrirubio/esp32-environment-monitor/refs/heads/main/my-build/temperature-humidity.jpg)

## Demo & Full Video
Watch a quick preview below (GIF), or download the full demo video

### Preview (GIF)
![ESP32 Environment Monitor](https://github.com/adrirubio/esp32-environment-monitor/raw/main/my-build/demos/esp32-environment-demo.gif)

### Full Video
[Click here to download the full demo video (MP4)](https://github.com/adrirubio/esp32-environment-monitor/raw/main/my-build/demos/esp32-environment-demo.mp4)

## How It Works

The monitor uses BME280 and SGP30 sensors to track environmental data and air quality. Data shows on an LCD screen with temperature, humidity, pressure, CO2, and TVOC readings. The system includes a web server for remote monitoring and alerts when air quality drops below safe levels.

## Components Used

- ESP32 DevKit V1
- BME280 sensor (temperature, humidity, pressure)
- SGP30 sensor (CO2, TVOC)
- 16x2 I2C LCD display
- Buzzer for alerts
- Resistors and wiring

## Repo Structure

- `firmware/environment-monitor/environment-monitor.ino` - Main ESP32 code
- `docs/` - Documentation including schematics and component list
- `my-build/` - Build photos and demos

## Features

- Real time temperature, humidity, and pressure monitoring
- Air quality tracking with CO2 and TVOC levels
- LCD display with multiple data screens
- Weather forecast based on pressure trends
- Web interface for remote monitoring
- Alert system for poor air quality
- Automatic sensor calibration

## Quick Start

1. **Clone this repository**
   ```bash
   git clone https://github.com/adrirubio/esp32-environment-monitor.git
   ```

2. **Open the Arduino sketch**
   - Open `firmware/environment-monitor/environment-monitor.ino`

3. **Install required libraries**
   - Adafruit BME280
   - Adafruit SGP30
   - LiquidCrystal I2C
   - WiFi (ESP32)

4. **Wire your components**
   - Connect I2C devices to pins 21 (SDA) and 22 (SCL)
   - Connect buzzer to pin 19
   - See `docs/` for full schematic

5. **Configure WiFi**
   - Add your network credentials in the sketch
   - Upload to your ESP32

6. **Upload and Monitor**
   - Upload the sketch to your ESP32
   - Access web interface at the board's IP address
   - Watch environmental data in real time

## Screenshots

![top view](https://raw.githubusercontent.com/adrirubio/esp32-environment-monitor/refs/heads/main/my-build/top-view.jpg)

![pressure and altitude](https://raw.githubusercontent.com/adrirubio/esp32-environment-monitor/refs/heads/main/my-build/pressure-altitude.jpg)

![forecast](https://raw.githubusercontent.com/adrirubio/esp32-environment-monitor/refs/heads/main/my-build/forecast.jpg)

![air quality](https://raw.githubusercontent.com/adrirubio/esp32-environment-monitor/refs/heads/main/my-build/air-quality.jpg)

![air quality good](https://raw.githubusercontent.com/adrirubio/esp32-environment-monitor/refs/heads/main/my-build/air-quality-good.jpg)

![web page](https://raw.githubusercontent.com/adrirubio/esp32-environment-monitor/refs/heads/main/my-build/web-page.jpg)

