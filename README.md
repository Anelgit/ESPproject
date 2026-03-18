# ESP32 Voice-Controlled Smart Home System

A locally controlled ESP32-based smart home system combining **voice command recognition**, **environmental sensing**, and **MQTT-based automation** for lighting and cooling control.

Voice commands are detected entirely on-device using an I2S MEMS microphone and a lightweight DSP pipeline (MFCC + Dynamic Time Warping), then forwarded via MQTT to a Node-RED backend for coordinated automation.

---

## System Architecture

```
┌─────────────┐      I2S       ┌──────────────────────────────┐
│  MEMS Mic   │───────────────▶│         ESP32                │
└─────────────┘                │                              │
                               │  Audio → FFT → MFCC → DTW   │
┌─────────────┐      SPI      │  Voice Command Classification│
│   BME280    │───────────────▶│                              │
│ Temp/Hum/P  │                │  Environmental Monitoring    │
└─────────────┘                └──────┬──────────┬────────────┘
                                      │ MQTT     │ GPIO
                                      ▼          ▼
                               ┌────────────┐  ┌──────────┐
                               │  Node-RED  │  │ MOSFET   │
                               │  Backend   │  │ Drivers  │
                               │            │  │          │
                               │ Dashboard  │  │ LED / Fan│
                               │ Automation │  └──────────┘
                               └────────────┘
```

---

## Features

| Feature | Description |
|---|---|
| **Voice Commands** | On-device keyword spotting — "wake", "lights on", "lights off" — using MFCC feature extraction and DTW template matching |
| **Lighting Control** | MOSFET-driven LED output controllable via voice or MQTT |
| **Automatic Cooling** | Thermostat mode with configurable setpoint (default 20°C) and 1°C hysteresis |
| **Environmental Sensing** | BME280 reads temperature, humidity, and pressure; publishes to MQTT with delta-based rate limiting |
| **MQTT Integration** | Retained status messages, LWT for online/offline detection, and full Node-RED dashboard support |
| **Voice Activity Detection** | RMS-based VAD with hysteresis and per-class debounce to prevent false triggers |

---

## Hardware

| Component | ESP32 Pin(s) | Notes |
|---|---|---|
| I2S MEMS Microphone | GPIO 26 (BCLK), 25 (LRCLK), 33 (DOUT) | 16 kHz mono audio input |
| BME280 Sensor (SPI) | GPIO 18 (SCK), 19 (MISO), 23 (MOSI), 5 (CS) | Temperature, humidity, pressure |
| LED Light Output | GPIO 4 | Via N-channel MOSFET |
| Fan Output | GPIO 27 | Via N-channel MOSFET |
| Auto-Mode Indicator | GPIO 2 | Built-in LED |

---

## Software Stack

- **Firmware:** C/C++ on Arduino framework via PlatformIO
- **Communication:** MQTT (PubSubClient) over Wi-Fi
- **Backend:** Node-RED for dashboards, automation logic, and control
- **Signal Processing:** arduinoFFT for spectral analysis, custom MFCC and DTW implementation
- **Sensor Library:** Adafruit BME280 (SPI)
- **Circuit Design:** LTSpice for simulation

---

## Getting Started

### Prerequisites

- [PlatformIO](https://platformio.org/) (CLI or VS Code extension)
- ESP32 DevKit (NodeMCU-32S)
- MQTT broker (e.g. Mosquitto) running on your network
- Node-RED instance (optional, for dashboard and automation)

### Configuration

Edit the following in `src/main.cpp` before building:

```cpp
#define WIFI_SSID     "your_ssid"
#define WIFI_PASS     "your_password"
#define MQTT_HOST     "192.168.1.2"    // your broker IP
```

Fan thermostat settings:

```cpp
static const float FAN_TARGET_C = 20.0f;   // target temperature
static const float FAN_HYST_C   = 1.0f;    // hysteresis band
```

### Build & Upload

```bash
# Build
pio run

# Upload to ESP32
pio run -t upload

# Monitor serial output
pio device monitor --baud 115200
```

---

## MQTT Topics

| Topic | Direction | Description |
|---|---|---|
| `ahodza/home/lights/set` | Subscribe | Set light state (`ON`/`OFF`) |
| `ahodza/home/lights/status` | Publish | Current light state (retained) |
| `cooling/set` | Subscribe | Set fan mode (`ON`/`OFF`/`AUTO`/`SLEEP`) |
| `cooling/status` | Publish | Current fan state (retained) |
| `ahodza/home/env/state` | Publish | JSON: `{temp_c, humidity, pressure_hpa, rssi}` |
| `ahodza/home/env/temp_c` | Publish | Temperature in °C |
| `ahodza/home/env/humidity` | Publish | Relative humidity % |
| `ahodza/home/env/pressure_hpa` | Publish | Atmospheric pressure in hPa |
| `ahodza/home/env/status` | Publish | LWT — `online` / `offline` |

---

## Voice Commands

| Command | Action |
|---|---|
| **"wake"** | Toggle auto-fan mode (auto ↔ sleep) |
| **"lights on"** | Turn lights on |
| **"lights off"** | Turn lights off |

The recognition pipeline processes 25 ms frames at a 10 ms hop rate, extracts 13 MFCCs per frame, and classifies utterances against 60 pre-trained DTW templates (20 per class). A confidence margin check prevents ambiguous matches from triggering actions.

---

## Project Structure

```
├── src/
│   ├── main.cpp                 # Main firmware — MQTT, sensors, audio, control logic
│   ├── voicedemo.cpp            # Standalone voice recognition demo
│   └── test.cpp                 # I2S microphone sanity test
├── include/
│   ├── KWSmedoids20.h           # Pre-trained KWS templates (60 medoids)
│   └── wakeword_templates.h     # MFCC template metadata
├── scripts/
│   ├── ESP32ProjectPythonNotebook.ipynb      # MFCC extraction & medoid selection
│   └── FFT_Spectrum_Melfilterbank_MFCC.ipynb # DSP reference notebook
├── platformio.ini               # PlatformIO build configuration
└── README.md
```

---

## License

This project was developed by **Anel Hodza** (September 2025).
