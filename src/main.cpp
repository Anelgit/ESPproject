
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
 
 // ---------- Wi-Fi ----------
 const char* WIFI_SSID = "FBI Surveillance van 4";
 const char* WIFI_PASS = "anelshjem";
 
 // ---------- MQTT ----------
 const char* MQTT_HOST = "192.168.1.9";   // your PC/broker IP
 const uint16_t MQTT_PORT = 1883;
 
 static const char* TOPIC_LIGHT_SET    = "ahodza/home/lights/set";
 static const char* TOPIC_LIGHT_STATUS = "ahodza/home/lights/status";
 
 // ---------- Hardware ----------
 constexpr int LED_PIN = 2;   // Built-in LED 
 // later move to an external LED on GPIO14:
 // constexpr int LED_PIN = 14;
 
 WiFiClient esp;
 PubSubClient mqtt(esp);
 
 // Keep track of the current state to avoid redundant publishes
 bool lightOn = false;
 
 // --- applyLight: drive GPIO and publish retained status (only on change) ---
 void applyLight(bool on) {
   if (on == lightOn) return;             // no change => nothing to do
   lightOn = on;
 
   digitalWrite(LED_PIN, on ? HIGH : LOW);
 
   // Retained status so dashboards know the last state immediately
   mqtt.publish(TOPIC_LIGHT_STATUS, on ? "ON" : "OFF", true);
 
   Serial.printf("[STATE] lightOn=%s\n", on ? "ON" : "OFF");
 }
 
 // --- MQTT message handler ---
 void onMsg(char* topic, byte* payload, unsigned int len) {
   // Copy payload to a small, null-terminated buffer
   char buf[16];
   size_t n = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;
   memcpy(buf, payload, n);
   buf[n] = '\0';
 
   Serial.printf("[MQTT] %s <- '%s'\n", topic, buf);
 
   if (strcmp(topic, TOPIC_LIGHT_SET) == 0) {
     // normalize to upper-case
     for (size_t i = 0; i < n; ++i) buf[i] = (char)toupper((unsigned char)buf[i]);
 
     if (!strcmp(buf, "ON") || !strcmp(buf, "1") || !strcmp(buf, "TRUE")) {
       applyLight(true);
     } else if (!strcmp(buf, "OFF") || !strcmp(buf, "0") || !strcmp(buf, "FALSE")) {
       applyLight(false);
     } else {
       Serial.printf("[WARN] Unknown lights/set payload: '%s'\n", buf);
     }
   }
 }
 
 // --- Wi-Fi connect (simple blocking, but reliable for now) ---
 void ensureWiFi() {
   if (WiFi.status() == WL_CONNECTED) return;
 
   WiFi.mode(WIFI_STA);
   WiFi.begin(WIFI_SSID, WIFI_PASS);
 
   Serial.print("[WiFi] Connecting");
   while (WiFi.status() != WL_CONNECTED) {
     delay(250);
     Serial.print(".");
   }
   Serial.printf("\n[WiFi] Connected. IP=%s\n", WiFi.localIP().toString().c_str());
 }
 
 // --- MQTT connect / reconnect ---
 void ensureMqtt() {
   if (mqtt.connected()) return;
 
   Serial.print("[MQTT] Connecting");
   while (!mqtt.connected()) {
     if (mqtt.connect("esp32-voice-demo")) {
       Serial.println("\n[MQTT] Connected");
       // Always (re)subscribe after a successful connect
       mqtt.subscribe(TOPIC_LIGHT_SET);
 
       // Publish current state (retained) so UIs sync
       mqtt.publish(TOPIC_LIGHT_STATUS, lightOn ? "ON" : "OFF", true);
     } else {
       Serial.print(".");
       delay(500);
     }
   }
 }
 
 void setup() {
   Serial.begin(115200);
   pinMode(LED_PIN, OUTPUT);
   digitalWrite(LED_PIN, LOW);                 // default OFF
   lightOn = false;
 
   ensureWiFi();
 
   mqtt.setServer(MQTT_HOST, MQTT_PORT);
   mqtt.setCallback(onMsg);
 
   ensureMqtt();
 
   // Publish initial retained status once at boot
   mqtt.publish(TOPIC_LIGHT_STATUS, "OFF", true);
   Serial.println("[BOOT] Ready");
 }
 
 void loop() {
   ensureWiFi();
   ensureMqtt();
   mqtt.loop();                                // MUST be called frequently
 }
 