
/*******************************************************
 * Project: ESP32 Voice-Controlled Lighting & Cooling
 * Author : Anel Hodza
 * Date   : September 2025
 *
 * Description:
 *   This project implements an ESP32-based IoT system 
 *   for controlling lights and cooling through MQTT.
 *   The ESP32 connects to Wi-Fi and communicates with 
 *   a Mosquitto broker running locally on the LAN.
 *   Node-RED is used as the visualization and control 
 *   dashboard, with switches and gauges for monitoring 
 *   system state.
 *
 * Features:
 *   - MQTT subscribe: listens for ON/OFF commands 
 *     on topic: "ahodza/home/lights/set"
 *   - MQTT publish : reports LED state 
 *     on topic: "ahodza/home/lights/status"
 *   - Retained messages ensure UI and device stay synced
 *   - Designed for easy extension (cooling fan, sensors)
 *
 * Hardware:
 *   - ESP32 DevKit
 *   - Onboard LED (GPIO 2) or external LED/fan on GPIO 14
 *   - Optional: DHT sensor for temperature & humidity
 *
 * Software Stack:
 *   - Arduino Core for ESP32
 *   - PubSubClient (MQTT)
 *   - Node-RED Dashboard
 *   - Mosquitto Broker (local)
 * TODO:
 *   [ ] Replace onboard LED with 12V LED strip + MOSFET driver
 *   [ ] Add DC fan control via transistor/MOSFET
 *   [ ] Integrate DHT22 for temperature & humidity
 *   [ ] Push sensor data to MQTT (e.g. ahodza/home/temperature)
 *   [ ] Expand Node-RED dashboard with real-time charts
 *   [ ] Add voice command interface (ESP-SR / Google Assistant)
 *   [ ] Improve error handling & auto-reconnect logic
 *
 *
 *******************************************************/
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <Adafruit_BME280.h>
// Keep for later voice work (not used here yet)
#include "wakeword_templates.h"

// ---------- Wi-Fi ----------

// Stores Wi-Fi network name (SSID) as a read-only C string in flash.
const char* WIFI_SSID = "FBI Surveillance van 4";
// Stores Wi-Fi password as a read-only C string in flash.
const char* WIFI_PASS = "anelshjem";

// ---------- MQTT ----------

// IP/hostname of the MQTT broker the ESP connects to.
const char* MQTT_HOST = "192.168.1.9";
// TCP port for the MQTT broker.
const uint16_t MQTT_PORT = 1883;

// Lights topics 

// Topic to which we subscribe to receive light control commands (setpoint).
static const char* TOPIC_LIGHT_SET    = "ahodza/home/lights/set";
// Topic to which we publish the current light state so other clients can reflect it.
static const char* TOPIC_LIGHT_STATUS = "ahodza/home/lights/status";

// Env topics (new/used)

// LWT/online status topic telling others if this device is online/offline.
static const char* TOPIC_ENV_STATUS = "ahodza/home/env/status";       // online/offline (LWT)
// JSON “snapshot” of environment values; retained so dashboards load with last known.
static const char* TOPIC_ENV_STATE  = "ahodza/home/env/state";        // JSON (retained)
// Simple scalar temp topic; retained for easy binding to dashboard widgets.
static const char* TOPIC_ENV_TEMP   = "ahodza/home/env/temp_c";       // retained
// Simple scalar humidity topic; retained.
static const char* TOPIC_ENV_HUM    = "ahodza/home/env/humidity";     // retained
// Simple scalar pressure topic; retained.
static const char* TOPIC_ENV_PRES   = "ahodza/home/env/pressure_hpa"; // retained

// ---------- Hardware ----------

// Logical pin number for the on-board LED on ESP32 DevKit (GPIO2).
constexpr int LED_PIN = 2;   // Built-in LED

// BME280 SPI pins
// BME SCK  -> GPIO18
// BME SDO  -> GPIO19 (MISO)
// BME SDA  -> GPIO23 (MOSI)
// BME CS   -> GPIO5

// Pin constant for SPI SCK clock line to the BME280.
#define BME_SCK   18
// Pin constant for SPI MISO (sensor → ESP32 data).
#define BME_MISO  19
// Pin constant for SPI MOSI (ESP32 → sensor data).
#define BME_MOSI  23
// Pin constant for the BME280’s chip-select line (active low).
#define BME_CS     5

// Instantiates a BME280 driver object configured to use “software SPI” on the pins above.
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

// ---------- Globals ----------

// Creates a TCP socket-like object used by PubSubClient to move bytes over Wi-Fi.
WiFiClient esp;
// Creates the MQTT client, binding it to the WiFiClient so it can connect to the broker.
PubSubClient mqtt(esp);

// Lights state

// Tracks whether the light is currently on; mirrors hardware and status topic.
bool lightOn = false;

// Env publish cadence / hysteresis

// Minimum time between environment publishes; prevents spamming broker/UI.
const uint32_t PUBLISH_MS     = 5000;  // 5 s
// Minimal temperature change needed to trigger a publish (noise filtering).
const float    MIN_DELTA_TEMP = 0.05f; // °C
// Minimal humidity change needed to trigger a publish.
const float    MIN_DELTA_HUM  = 0.50f; // %RH
// Minimal pressure change needed to trigger a publish.
const float    MIN_DELTA_PRES = 0.50f; // hPa

// Stores the last time (millis) we published env data to rate-limit updates.
unsigned long t_last_pub = 0;
// Caches last published temperature/humidity/pressure; starts as NaN so the first read publishes.
float last_t = NAN, last_h = NAN, last_p = NAN;

// ---------- Helpers ----------

// Turns the light on/off, updates hardware pin, and publishes retained status.
void applyLight(bool on) {
  // Quick exit if requested state equals the current state (avoids redundant writes/publishes).
  if (on == lightOn) return;
  // Remember the new state so logic/UI stay in sync.
  lightOn = on;
  // Drive the LED pin HIGH for on and LOW for off (board LED is active-high).
  digitalWrite(LED_PIN, on ? HIGH : LOW);
  // Publish the new light status as a retained message so subscribers/Dashboard get it immediately.
  mqtt.publish(TOPIC_LIGHT_STATUS, on ? "ON" : "OFF", true); // retained
  // Print to serial for human-readable debugging.
  Serial.printf("[STATE] lightOn=%s\n", on ? "ON" : "OFF");
}

// Handles every MQTT message we receive; routes by topic and parses payload.
void onMsg(char* topic, byte* payload, unsigned int len) {
  // Local buffer to copy the incoming payload into (adds a null terminator for safe string ops).
  char buf[16];
  // Clamp copy count to buffer size minus 1 so we never overflow.
  size_t n = min(len, (unsigned int)sizeof(buf) - 1);
  // Copy raw bytes from MQTT payload to our local buffer.
  memcpy(buf, payload, n);
  // Manually terminate with '\0' so we can treat it as a C string.
  buf[n] = '\0';
  // Log the topic and text payload so we can see what arrived.
  Serial.printf("[MQTT] %s <- '%s'\n", topic, buf);

  // If the message is for the lights/set topic, interpret it as a command.
  if (strcmp(topic, TOPIC_LIGHT_SET) == 0) {
    // Uppercase the payload in-place to make comparisons case-insensitive.
    for (size_t i = 0; i < n; ++i) buf[i] = (char)toupper((unsigned char)buf[i]);
    // If payload equals any “true/on” variants, turn the light on.
    if (!strcmp(buf, "ON") || !strcmp(buf, "1") || !strcmp(buf, "TRUE")) {
      applyLight(true);
    // If payload equals any “false/off” variants, turn the light off.
    } else if (!strcmp(buf, "OFF") || !strcmp(buf, "0") || !strcmp(buf, "FALSE")) {
      applyLight(false);
    // Otherwise, it’s unrecognized input; warn but do nothing.
    } else {
      Serial.printf("[WARN] Unknown payload on lights/set: '%s'\n", buf);
    }
  }
}

// Ensures we’re connected to Wi-Fi; if not, it blocks until association succeeds.
void ensureWiFi() {
  // If already connected, there’s nothing to do.
  if (WiFi.status() == WL_CONNECTED) return;
  // Put the radio in “station” (client) mode so we can join an AP.
  WiFi.mode(WIFI_STA);
  // Start connecting with the provided SSID/password.
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  // Print a progress indicator while waiting for DHCP/association.
  Serial.print("[WiFi] Connecting");
  // Loop until Wi-Fi says we’re connected.
  while (WiFi.status() != WL_CONNECTED) {
    // Small delay to avoid busy-waiting; also gives the radio time to work.
    delay(250);
    // Emit a dot so we can see connection progress in the serial monitor.
    Serial.print(".");
  }
  // Once connected, print the IP address and the signal strength (RSSI).
  Serial.printf("\n[WiFi] Connected. IP=%s, RSSI=%d dBm\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());
}

// Ensures we’re connected to the MQTT broker; sets LWT, subscribes, and announces presence.
void ensureMqtt() {
  // If the MQTT client is already connected, bail out early.
  if (mqtt.connected()) return;
  // Configure which broker/port to talk to (does not actually connect yet).
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  // Build a semi-unique client ID using the ESP32’s MAC so multiple devices don’t collide.
  String clientId = "esp32-env-" + String((uint32_t)ESP.getEfuseMac(), HEX);

  // Keep trying to connect until it succeeds (simple blocking retry loop).
  while (!mqtt.connected()) {
    // Visual progress in serial monitor for debugging.
    Serial.print("[MQTT] Connecting");
    // Attempt to connect; also sets a Last Will and Testament on env/status with “offline”.
    bool ok = mqtt.connect(clientId.c_str(),
                           nullptr, nullptr,
                           TOPIC_ENV_STATUS, 0, true, "offline");
    // If connection worked, finalize session setup.
    if (ok) {
      // Newline to end the “Connecting...” line.
      Serial.println("\n[MQTT] Connected");
      // Subscribe to the light control topic so we can receive commands.
      mqtt.subscribe(TOPIC_LIGHT_SET);
      // Immediately publish current light state so dashboard widgets sync (retained).
      mqtt.publish(TOPIC_LIGHT_STATUS, lightOn ? "ON" : "OFF", true);
      // Announce we’re online so watchers know device is alive (also retained).
      mqtt.publish(TOPIC_ENV_STATUS, "online", true);
    // If connection failed, wait a moment and try again (keeps outputting dots).
    } else {
      Serial.print(".");
      delay(500);
    }
  }
}

// Publishes environmental readings as both a JSON blob and individual retained topics.
void publish_env(float t, float h, float p) {
  // Build a compact JSON string including Wi-Fi RSSI for quick diagnostics.
  char json[160];
  snprintf(json, sizeof(json),
           "{\"temp_c\":%.2f,\"humidity\":%.2f,\"pressure_hpa\":%.2f,\"rssi\":%d}",
           t, h, p, WiFi.RSSI());
  // Publish the JSON snapshot and retain it so dashboards can fetch the last known on load.
  mqtt.publish(TOPIC_ENV_STATE, json, true);

  // Prepare a generic buffer for printing scalar floats as strings.
  char buf[32];
  // Convert temperature to text and publish/retain under a dedicated topic (good for simple gauges).
  dtostrf(t, 0, 2, buf); mqtt.publish(TOPIC_ENV_TEMP, buf, true);
  // Convert humidity to text and publish/retain.
  dtostrf(h, 0, 2, buf); mqtt.publish(TOPIC_ENV_HUM,  buf, true);
  // Convert pressure to text and publish/retain.
  dtostrf(p, 0, 2, buf); mqtt.publish(TOPIC_ENV_PRES, buf, true);

  // Log the values to serial for human verification during testing.
  Serial.printf("[ENV] T=%.2f°C  RH=%.2f%%  P=%.2f hPa\n", t, h, p);
}

// Initializes the BME280 over SPI, verifies the sensor type, and sets oversampling/filtering.
bool init_bme_spi() {
  // Attempt to start the sensor; returns false if not found or not responding on these pins.
  if (!bme.begin()) return false;  // SPI: begin() has no args
  // Read the sensor ID register to distinguish BME280 (has humidity) from BMP280 (no humidity).
  uint8_t id = bme.sensorID();     // 0x60 = BME280, 0x58 = BMP280
  // Print which sensor we detected for transparency and to aid wiring/debugging.
  Serial.printf("[BME] sensorID=0x%02X (%s)\n",
                id, (id==0x60?"BME280":(id==0x58?"BMP280":"unknown")));

  // Configure measurement mode, oversampling, IIR filter, and standby time for stable readings.
  bme.setSampling(
    Adafruit_BME280::MODE_NORMAL,
    Adafruit_BME280::SAMPLING_X2,     // temp
    Adafruit_BME280::SAMPLING_X16,    // pressure
    Adafruit_BME280::SAMPLING_X1,     // humidity (ignored on BMP)
    Adafruit_BME280::FILTER_X16,
    Adafruit_BME280::STANDBY_MS_500
  );
  // Signal to caller that initialization worked.
  return true;
}

// ---------- Arduino ----------

// Arduino lifecycle hook; runs once at boot to set everything up.
void setup() {
  // Start the serial port so we can print logs at 115200 baud.
  Serial.begin(115200);
  // Small delay to let the USB serial terminal attach before we print the first lines.
  delay(150);
  // Intro banner so we know the firmware variant and sensor bus.
  Serial.println("\n[BOOT] ESP32 + MQTT + BME280 (SPI)");

  // Make the LED pin an output so we can drive it HIGH/LOW.
  pinMode(LED_PIN, OUTPUT);
  // Ensure LED starts off (LOW) at boot for a known safe state.
  digitalWrite(LED_PIN, LOW);
  // Initialize the logical light state so code/UI agree at startup.
  lightOn = false;

  // Join the Wi-Fi network (blocks until connected).
  ensureWiFi();
  // Assign the MQTT message callback handler so incoming messages get routed to onMsg.
  mqtt.setCallback(onMsg);
  // Establish/re-establish the MQTT session; subscribes and publishes LWT/online.
  ensureMqtt();

  // Try to bring up the BME/BMP over SPI; if it fails, we continue with just MQTT lights.
  if (!init_bme_spi()) {
    // Helpful wiring hint if sensor isn’t detected; prints once at boot.
    Serial.println("[ERROR] BME/BMP not found over SPI. Check SCK=18, MISO=19, MOSI=23, CS=5, power & GND.");
    // continue running so MQTT lights still work
  } else {
    // Confirmation that environmental sensing is ready to go.
    Serial.println("[BME] Initialized.");
  }

  // Publish initial retained state so dashboards immediately pick up known values.
  mqtt.publish(TOPIC_LIGHT_STATUS, "OFF", true);
  // Mark device online after everything is initialized to reduce false “online” states.
  mqtt.publish(TOPIC_ENV_STATUS, "online", true);
}

// Arduino main loop; runs repeatedly and should return quickly each iteration.
void loop() {
  // Keep Wi-Fi alive; reconnect if needed (handles AP drops).
  ensureWiFi();
  // Keep MQTT session alive; reconnect if needed (handles broker restarts).
  ensureMqtt();
  // Let PubSubClient process incoming/outgoing packets; must be called often.
  mqtt.loop();

  // Tracks time of last BME read to throttle sensor access independently of publish rate.
  static unsigned long lastRead = 0;
  // Snapshot current uptime in milliseconds for non-blocking timing.
  const unsigned long now = millis();

  // Poll the sensor frequently (every 500 ms) so we have fresh data; we’ll rate-limit publishes later.
  if (now - lastRead >= 500) { // read often (0.5s); publish throttled below
    // Record the moment we performed this read to schedule the next one.
    lastRead = now;

    // Read temperature in Celsius from the BME/BMP; may be NaN if sensor missing.
    float t = bme.readTemperature();
    // Read relative humidity; will be NaN on BMP280 (it has no humidity sensor).
    float h = bme.readHumidity();             // NaN on BMP280
    // Read pressure in Pascals and convert to hPa (divide by 100.0f).
    float p = bme.readPressure() / 100.0f;    // hPa

    // If at least one value is a number, proceed (protects against all-NaN cases).
    if (!isnan(t) || !isnan(h) || !isnan(p)) {
      // Flag to decide whether we should publish this sample based on change thresholds/timing.
      bool changed = false;
      // Trigger if temp changed more than the configured epsilon or if it’s the first valid reading.
      if (isnan(last_t) || (!isnan(t) && fabsf(t - last_t) > MIN_DELTA_TEMP)) changed = true;
      // Trigger if humidity changed sufficiently (and humidity is valid on BME280).
      if (isnan(last_h) || (!isnan(h) && fabsf(h - last_h) > MIN_DELTA_HUM))  changed = true;
      // Trigger if pressure changed sufficiently.
      if (isnan(last_p) || (!isnan(p) && fabsf(p - last_p) > MIN_DELTA_PRES)) changed = true;

      // Publish if any field changed beyond hysteresis OR if the publish interval has elapsed.
      if (changed || (now - t_last_pub) >= PUBLISH_MS) {
        // Send JSON + scalar topics (retained) so dashboards stay in sync.
        publish_env(t, h, p);
        // Update our last-known values so hysteresis works next time.
        last_t = t; last_h = h; last_p = p;
        // Remember when we last published to enforce PUBLISH_MS.
        t_last_pub = now;
      }
    } else {
      // If all values are NaN, stay silent. This guard only triggers when everything is NaN.
    }
  }

  // Tiny delay to yield CPU time to Wi-Fi/MQTT stacks and avoid a 100% busy loop.
  delay(10);
}