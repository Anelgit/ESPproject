Overview

This project is a locally controlled, ESP32-based smart lighting and cooling system that integrates voice command detection, environmental sensing, 
and MQTT-based automation.
The system is designed with embedded robustness, modularity, and real-world deployment constraints in mind.

Voice commands are detected locally on the ESP32 using an I2S MEMS microphone, processed with a lightweight DSP pipeline, and forwarded via MQTT to 
a Node-RED automation backend, which coordinates lighting and cooling behavior based on both voice intent and sensor data.

System Architecture:

ESP32 samples audio via I2S MEMS microphone

Audio is processed locally (framing, filtering, template-based KWS)

Recognized commands are published over MQTT

Node-RED evaluates logic and environmental data

Commands are sent back to ESP32

ESP32 drives MOSFET-controlled LED lighting and fan cooling

Environmental feedback is provided by a BME280 temperature/humidity sensor to enable closed-loop control.


Hardware Components:
ESP32 (Wi-Fi MCU)

I2S MEMS Microphone (audio capture for voice commands)

BME280 (temperature, humidity, pressure)

N-channel MOSFETs

LED lighting control

Fan speed/control

External power regulation and decoupling


Software Stack:

Firmware

Language: C/C++

Framework: ESP32

Audio processing: framing, energy detection, template-based KWS

Communication

MQTT (publish/subscribe architecture)

Backend

Node-RED (logic, dashboards, automation)

Design Tools

LTSpice

PlatformIO

GitHub for version control
