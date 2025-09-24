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
 *   - BME sensor for temperature & humidity                   
 *
 * Software Stack:                                                       
 *   - Arduino Core for ESP32                                            
 *   - PubSubClient (MQTT)                                               
 *   - Node-RED Dashboard                                                
 *   - Mosquitto Broker (local)                                          
 *
 * TODO (updated):                                                       
 *   [ ] Persist settings in NVS (Preferences): autoFanEnabled & FAN_TARGET_C      
 *   [ ] Add MQTT config topics:                                           
 *       - ahodza/home/fan/target_c/set (Number)                           
 *       - ahodza/home/fan/mode/set ("auto"/"sleep")                       
 *   [ ] Node-RED: add fan tile (mode/target/status) + charts for T/RH/P    
 *   [ ] Hardware: drive fan via MOSFET or relay + flyback, add series resistor on LEDs 
 *   [ ] Security: optional MQTT auth/TLS; move creds to secrets header               
 *   [ ] KWS: capture more templates (esp. wake), optional distinct "off" phrase      
 *   [ ] KWS: disable debug in prod, add lightweight telemetry event (optional)       
 *   [ ] Networking: exponential backoff and error LED pattern on Wi-Fi/MQTT failure  
 *   [ ] Power: consider light sleep when idle, tune VAD thresholds to lower CPU      
 *
 *******************************************************/
#include <Arduino.h>                     
#include <WiFi.h>                        
#include <PubSubClient.h>                
#include <SPI.h>                         
#include <Adafruit_BME280.h>             
#include "wakeword_templates.h"          

#include <math.h>                        
#include <driver/i2s.h>                  
#include <ArduinoFFT.h>                  

// ---------- Wi-Fi ----------

const char* WIFI_SSID = "FBI Surveillance van 4";   // SSID string stored in flash 
const char* WIFI_PASS = "anelshjem";                // Wi-Fi password string 

// ---------- MQTT ----------

const char* MQTT_HOST = "192.168.1.2";             // MQTT broker host/IP used by WiFiClient to connect
//const char* MQTT_HOST = "192.168.1.10";          // Alternate broker host/IP (disabled here)

const uint16_t MQTT_PORT = 1883;                   // Broker TCP port (default Mosquitto is 1883, no TLS)

// Lights topics 

static const char* TOPIC_LIGHT_SET    = "ahodza/home/lights/set";    // Subscribed command topic for light
static const char* TOPIC_LIGHT_STATUS = "ahodza/home/lights/status"; // Published status topic (retained)

// Env topics (new/used)

static const char* TOPIC_ENV_STATUS = "ahodza/home/env/status";       // LWT/online status retained topic
static const char* TOPIC_ENV_STATE  = "ahodza/home/env/state";        // JSON snapshot retained topic
static const char* TOPIC_ENV_TEMP   = "ahodza/home/env/temp_c";       // Retained numeric temperature
static const char* TOPIC_ENV_HUM    = "ahodza/home/env/humidity";     // Retained numeric humidity
static const char* TOPIC_ENV_PRES   = "ahodza/home/env/pressure_hpa"; // Retained numeric pressure

// ---------- Hardware ----------

constexpr int LED_PIN = 2;              // Built-in LED (GPIO2) used as "auto fan enabled" indicator
constexpr int LED_LIGHT_PIN = 4;        // External LED output for "lights on/off" indication (GPIO4)
constexpr int FAN_PIN = 27;             // Fan driver output pin (to MOSFET/relay input), HIGH = fan ON

// BME280 SPI pins (software SPI wiring for Adafruit_BME280)
#define BME_SCK   18                    // SPI clock pin wired to BME SCK
#define BME_MISO  19                    // SPI MISO pin wired to BME SDO (sensor→ESP32)
#define BME_MOSI  23                    // SPI MOSI pin wired to BME SDA (ESP32→sensor data)
#define BME_CS     5                    // Chip Select pin wired to BME CS (active low)

Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // Construct sensor driver in software SPI mode

// ---------- Globals ----------

WiFiClient esp;                         // TCP client used by PubSubClient to transport MQTT bytes
PubSubClient mqtt(esp);                 // MQTT client bound to the WiFiClient socket

// Lights state
bool lightOn = false;                   // Shadow state for external LED (true → LED on)

// Env publish cadence / hysteresis
const uint32_t PUBLISH_MS     = 5000;   // Minimum ms between publishes to avoid UI spam
const float    MIN_DELTA_TEMP = 0.05f;  // Epsilon change to trigger temp publish (°C)
const float    MIN_DELTA_HUM  = 0.50f;  // Epsilon change to trigger humidity publish (%RH)
const float    MIN_DELTA_PRES = 0.50f;  // Epsilon change to trigger pressure publish (hPa)

unsigned long t_last_pub = 0;           // Last publish timestamp (millis) for rate limiting
float last_t = NAN, last_h = NAN, last_p = NAN; // Last published readings (NaN forces first publish)

// ---------- Fan auto-control (thermostat) ----------

const float FAN_TARGET_C = 20.0f;       // Desired room temperature setpoint (°C)
const float FAN_HYST_C   = 1.0f;        // Total hysteresis band (°C) to prevent fan chatter

bool  autoFanEnabled = false;           // Flag: true when listening to temp sensor (auto mode)
bool  fanOn = false;                    // Fan output shadow state (HIGH when true)
float current_temp_c = NAN;             // Latest temperature sample for control logic

void setAutoFan(bool enabled) {         // Enable/disable auto fan mode and update indicator/fan
  autoFanEnabled = enabled;             // Remember desired auto mode
  digitalWrite(LED_PIN, enabled ? HIGH : LOW); // Built-in LED shows auto fan listening state
  if (!enabled) {                       // If disabling auto mode ("sleep")
    if (fanOn) {                        // And fan currently on, force it off
      digitalWrite(FAN_PIN, LOW);       // Drive output LOW to stop fan (assumes active-HIGH)
      fanOn = false;                    // Update shadow state
      Serial.println("[FAN] OFF (sleep)"); // Log action for debugging
    }
  } else {
    Serial.println("[FAN] Auto mode ON");  // Log we entered auto mode
  }
}

void controlFanByTemp() {               // Apply simple thermostat control using hysteresis
  if (!autoFanEnabled || isnan(current_temp_c)) return; // Exit if not in auto or temp invalid
  const float hi = FAN_TARGET_C + (FAN_HYST_C * 0.5f);  // Upper threshold to turn ON
  const float lo = FAN_TARGET_C - (FAN_HYST_C * 0.5f);  // Lower threshold to turn OFF

  if (!fanOn && current_temp_c > hi) {  // If fan is off and temp exceeded high
    digitalWrite(FAN_PIN, HIGH);        // Turn fan ON (energize output)
    fanOn = true;                       // Update shadow state
    Serial.printf("[FAN] ON (T=%.2f > %.2f)\n", current_temp_c, hi); // Log with numbers
  } else if (fanOn && current_temp_c < lo) { // If fan is on and temp dropped below low
    digitalWrite(FAN_PIN, LOW);         // Turn fan OFF
    fanOn = false;                      // Update shadow state
    Serial.printf("[FAN] OFF (T=%.2f < %.2f)\n", current_temp_c, lo); // Log with numbers
  }
}

void applyLight(bool on);               // Forward declaration so KWS logic can call before defined
void ensureWiFi();                      // Forward declaration for Wi-Fi helper
void ensureMqtt();                      // Forward declaration for MQTT helper

// ---------- Audio / KWS ----------
//
// Lightweight I²S mic + MFCC + template matching (approximate).         // Overview of audio pipeline
// - Uses arduinoFFT to compute spectra                                  // FFT basis
// - 16 kHz mono, 25 ms window, 10 ms hop                                // Framing parameters
// - 26 mel filters → 13 MFCCs (matches KWS_N_MFCC from wakeword_templates.h) // Feature dimensions
// - Classes: 0 = wake, 1 = "lights on", 2 = "lights off"                // Class mapping
//
#ifndef I2S_BCLK_PIN
#define I2S_BCLK_PIN 26                 // I²S bit-clock (SCK/BCLK) pin to mic breakout
#endif
#ifndef I2S_WS_PIN
#define I2S_WS_PIN   25                 // I²S word-select (WS/LRCLK) pin to mic breakout
#endif
#ifndef I2S_DOUT_PIN
#define I2S_DOUT_PIN 33                 // I²S data-out from mic to ESP32 (SD/DO)
#endif

// Audio / feature params
static const int   KWS_SAMPLE_RATE   = 16000; // Sample rate in Hz; chosen to match templates
static const int   KWS_FFT_N         = 512;   // FFT size (power of two) for spectral analysis
static const int   KWS_FRAME_LEN     = 400;   // Samples per 25 ms frame at 16 kHz (0.025*16000)
static const int   KWS_HOP_LEN       = 160;   // Hop length 10 ms between consecutive frames
static const int   KWS_N_FILT        = 26;    // Count of mel filters before DCT to MFCC
static const int   KWS_MAX_FRAMES    = 300;   // Max MFCC frames kept (~3 s window)

// === KWS debug/toggles ===
#define KWS_DEBUG 1                         // 1 enables serial debug prints for KWS internals
#define MIC_RIGHT_CHANNEL 0                 // 0 read LEFT (mono), 1 read RIGHT (from stereo)

// Utterance-level VAD & decisioning
#define VAD_ON            0.0070f           // RMS threshold to consider speech starting
#define VAD_OFF           0.0035f           // RMS threshold to consider speech stopped
#define SILENCE_HANG_MS   700               // Silence duration to declare end of utterance
#define CLASS_COOLDOWN_MS 1500              // Per-class cooldown to avoid repeated triggers
#define MARGIN_RATIO      0.92f             // Require best/second ratio below this
#define GAP_MIN           50.0f             // Or require absolute gap (second-best − best)
#define ABS_DIST_CAP      1e9f              // Optional absolute cap (disabled here)
#define MIN_SEG_FRAMES    30                // Ignore segments shorter than ~0.3 s
#define MAX_SEG_FRAMES    280               // Clamp over-long segments
#define ON_MIN_FRAMES     60                // Short “ON” → treat as wake toggle

// Class mapping (keep aligned with wakeword_templates.h)
enum { KWS_CLS_WAKE = 0, KWS_CLS_LIGHTS_ON = 1, KWS_CLS_LIGHTS_OFF = 2 }; // Indices into class scores

// DSP state
static float hamming[KWS_FRAME_LEN];                    // Precomputed Hamming window coefficients
static float dct_m[KWS_N_MFCC * KWS_N_FILT];            // DCT-II matrix (row-major) for 26→13 MFCC
static uint16_t mel_bin_edges[KWS_N_FILT + 2];          // Mel filter bank FFT bin edges

// MFCC ring buffer (time axis)
static float mfcc_seq[KWS_MAX_FRAMES][KWS_N_MFCC];      // Rolling window of MFCC frames
static int   mfcc_T = 0;                                // Count of valid frames in ring buffer

// Work buffers for FFT
static double vReal[KWS_FFT_N];                         // Real part buffer for FFT/amps
static double vImag[KWS_FFT_N];                         // Imag part buffer for FFT (zero for magnitude)
static ArduinoFFT<double> FFT(vReal, vImag, KWS_FFT_N, KWS_SAMPLE_RATE); // FFT instance bound to buffers

// Utility: Hz<->Mel
static inline float hz2mel(float hz){ return 2595.0f * log10f(1.0f + hz / 700.0f); } // Convert Hz→Mel
static inline float mel2hz(float m){ return 700.0f * (powf(10.0f, m/2595.0f) - 1.0f); } // Convert Mel→Hz

void kws_prepare_dsp() {                               // Precompute window, mel bins, and DCT rows
  for (int n = 0; n < KWS_FRAME_LEN; ++n) {            // Iterate each time-domain sample index
    hamming[n] = 0.54f - 0.46f * cosf(2.0f * PI * n / (KWS_FRAME_LEN - 1)); // Hamming window formula
  }
  float mel_lo = hz2mel(0.0f);                         // Mel at 0 Hz (low edge)
  float mel_hi = hz2mel(KWS_SAMPLE_RATE / 2.0f);       // Mel at Nyquist (high edge)
  for (int m = 0; m < KWS_N_FILT + 2; ++m) {           // Compute mel bin edges for triangular filters
    float mel = mel_lo + (mel_hi - mel_lo) * m / (KWS_N_FILT + 1); // Linearly spaced in mel domain
    float hz  = mel2hz(mel);                           // Convert each mel point back to Hz
    int bin   = (int)floor((KWS_FFT_N + 1) * hz / KWS_SAMPLE_RATE); // Map Hz to FFT bin index
    if (bin < 0) bin = 0;                              // Clip lower bound
    if (bin > KWS_FFT_N/2) bin = KWS_FFT_N/2;          // Clip to Nyquist
    mel_bin_edges[m] = (uint16_t)bin;                  // Store as unsigned short
  }
  for (int i = 0; i < KWS_N_MFCC; ++i) {               // Build DCT-II transform rows
    for (int j = 0; j < KWS_N_FILT; ++j) {             // Iterate mel filter index
      dct_m[i*KWS_N_FILT + j] = cosf(PI * i * (2*j + 1) / (2.0f * KWS_N_FILT)); // DCT-II basis
    }
  }
}

void kws_mfcc_frame(const float *frame, float *out13) { // Compute 13 MFCCs from a windowed frame
  for (int i = 0; i < KWS_FFT_N; ++i) {                 // Prepare FFT input buffers
    if (i < KWS_FRAME_LEN) {                            // Inside frame length
      vReal[i] = (double)(frame[i] * hamming[i]);       // Apply Hamming window to sample
    } else {
      vReal[i] = 0.0;                                   // Zero-pad beyond frame
    }
    vImag[i] = 0.0;                                     // Imag part zeroed for real FFT
  }

  FFT.windowing(vReal, KWS_FFT_N, FFT_WIN_TYP_RECTANGLE, FFT_FORWARD); // Keep rectangle since window applied
  FFT.compute(vReal, vImag, KWS_FFT_N, FFT_FORWARD);    // Perform forward FFT to spectrum
  FFT.complexToMagnitude(vReal, vImag, KWS_FFT_N);      // Convert complex bins to magnitudes in vReal

  float e[KWS_N_FILT];                                  // Accumulate mel filter energies
  for (int m = 0; m < KWS_N_FILT; ++m) {                // For each triangular mel filter
    int b0 = mel_bin_edges[m];                          // Left edge bin
    int b1 = mel_bin_edges[m+1];                        // Center bin
    int b2 = mel_bin_edges[m+2];                        // Right edge bin
    float sum = 0.0f;                                   // Energy accumulator
    if (b0 == b1) b0 = (b0>0?b0-1:b0);                  // Ensure nonzero width on left slope
    if (b1 == b2) b2 = (b2<KWS_FFT_N/2?b2+1:b2);        // Ensure nonzero width on right slope

    for (int k = b0; k < b1; ++k) {                     // Rising slope region
      float w = (k - b0) / (float)(b1 - b0);            // Linear weight from 0→1
      float p = (float)vReal[k];                        // Magnitude at bin k
      sum += (p * p) * w;                               // Add weighted power to sum
    }
    for (int k = b1; k < b2; ++k) {                     // Falling slope region
      float w = (b2 - k) / (float)(b2 - b1);            // Linear weight from 1→0
      float p = (float)vReal[k];                        // Magnitude at bin k
      sum += (p * p) * w;                               // Add weighted power to sum
    }
    e[m] = logf(sum + 1e-8f);                           // Log-compress energy to approximate loudness
  }

  for (int i = 0; i < KWS_N_MFCC; ++i) {                // DCT-II of log-mels → MFCC coefficients
    float acc = 0.0f;                                   // Accumulator for dot-product
    const float *row = &dct_m[i*KWS_N_FILT];            // Pointer to precomputed DCT row i
    for (int j = 0; j < KWS_N_FILT; ++j) acc += row[j] * e[j]; // Multiply-add across 26 filters
    out13[i] = acc;                                     // Store MFCC i
  }
}

float kws_template_distance_align_end(const float *seq, int T, int K,
                                      const int16_t *templ, int L, float scale) { // Distance between last Lp frames of utterance & template (CMN)
  int Lp = (T < L ? T : L);                             // Number of frames to compare (shorter of both)
  if (Lp <= 0) return 1e30f;                            // If no overlap, return huge distance

  const int startA = (T - Lp) * K;                      // Start offset into utterance flattened MFCCs
  const int startB = (L - Lp) * K;                      // Start offset into template flattened MFCCs
  const float invS = 1.0f / scale;                      // Inverse of template quantization scale

  float meanA[32]; float meanB[32];                     // Storage for mean vectors (max 32 MFCCs)
  for (int k = 0; k < K; ++k) { meanA[k] = 0.0f; meanB[k] = 0.0f; } // Initialize means
  for (int t = 0; t < Lp; ++t) {                        // Accumulate sums to compute means (CMN)
    const float   *a = &seq[startA + t*K];              // Pointer to utterance frame t
    const int16_t *b = &templ[startB + t*K];            // Pointer to template frame t (int16 quantized)
    for (int k = 0; k < K; ++k) {
      meanA[k] += a[k];                                 // Sum utterance MFCC k
      meanB[k] += (float)b[k] * invS;                   // Sum de-quantized template MFCC k
    }
  }
  for (int k = 0; k < K; ++k) { meanA[k] /= (float)Lp; meanB[k] /= (float)Lp; } // Average to get means

  float acc = 0.0f;                                     // Distance accumulator
  for (int t = 0; t < Lp; ++t) {                        // Iterate frames to compute CMN distance
    const float   *a = &seq[startA + t*K];              // Utterance frame pointer
    const int16_t *b = &templ[startB + t*K];            // Template frame pointer
    for (int k = 0; k < K; ++k) {
      float av = a[k] - meanA[k];                       // CMN: subtract utterance mean
      float bv = (float)b[k] * invS - meanB[k];         // CMN: subtract template mean
      float d  = av - bv;                               // Difference for coefficient k
      acc += d * d;                                     // Accumulate squared distance
    }
  }
  return acc / (float)(Lp * K);                         // Average per coefficient per frame
}

int kws_classify_segment(int startT, int endT, float *bestOut, float *secondOut) { // Classify one utterance slice
  int T = endT - startT;                                // Segment length in frames
  if (T < MIN_SEG_FRAMES) return -1;                    // Ignore too-short segments
  if (T > MAX_SEG_FRAMES) T = MAX_SEG_FRAMES;           // Clamp too-long segments

  static float flat[KWS_MAX_FRAMES * KWS_N_MFCC];       // Flattened buffer (T×K)
  for (int t = 0; t < T; ++t) {                         // Copy slice into contiguous array
    for (int k = 0; k < KWS_N_MFCC; ++k) {
      flat[t*KWS_N_MFCC + k] = mfcc_seq[startT + t][k]; // Flatten frame by frame
    }
  }

  float bestByClass[3] = {1e30f, 1e30f, 1e30f};         // Per-class best distances initialized high
  for (int ti = 0; ti < KWS_NUM_TEMPLATES; ++ti) {      // Iterate all stored templates
    int cls   = KWS_TEMPL_CLASS_IDX[ti];                // Template's class index
    int L     = KWS_TEMPL_FRAMES[ti];                   // Template length (frames)
    int K     = KWS_TEMPL_COEFFS[ti];                   // Coefficients per frame (MFCC count)
    int off   = (int)KWS_TEMPL_OFFSET[ti];              // Offset into packed template data array
    const int16_t *td = &KWS_TEMPL_DATA[off];           // Pointer to template MFCC data (int16)
    float sc   = KWS_TEMPL_SCALE[ti];                   // Scale factor to de-quantize back to float

    float d = kws_template_distance_align_end(flat, T, K, td, L, sc); // Compute end-aligned CMN distance
    if (d < bestByClass[cls]) bestByClass[cls] = d;     // Keep the best distance for this class
  }

  int winner = -1;                                      // Class with smallest distance
  float best = 1e30f, second = 1e30f;                   // Track best and second-best distances
  for (int c = 0; c < 3; ++c) {                         // Evaluate classes
    float d = bestByClass[c];                           // Distance for class c
    if (d < best) { second = best; best = d; winner = c; } // Update winners
    else if (d < second) { second = d; }                // Update runner-up
  }
  if (bestOut) *bestOut = best;                         // Return best distance if requested
  if (secondOut) *secondOut = second;                   // Return second-best if requested

  float ratio = (second > 0.0f) ? (best / second) : 0.0f; // Relative margin (lower better)
  float gap   = (second > 0.0f) ? (second - best) : 0.0f; // Absolute gap

  int seg_len = T;                                      // Cache segment length
  bool len_ok = true;                                   // Assume length ok
  if (winner == KWS_CLS_LIGHTS_ON)  len_ok = (seg_len >= 40 && seg_len <= 120);  // Heuristic for ON
  if (winner == KWS_CLS_LIGHTS_OFF) len_ok = (seg_len >= 45 && seg_len <= 140);  // Heuristic for OFF

  if (winner >= 0
      && len_ok
      && (ratio < MARGIN_RATIO || gap > GAP_MIN)
      && best < ABS_DIST_CAP) {                         // Accept if passes all heuristics
    return winner;                                      // Return predicted class index
  }
  return -1;                                            // Otherwise reject segment
}

// I²S mic init + non-blocking sample → frame → MFCC pipeline
static int   hop_accum = 0;                             // Position in current frame buffer
static float frame_buf[KWS_FRAME_LEN];                  // Sliding time-domain window buffer
static float vad_rms = 0.0f;                            // Exponential RMS VAD tracker

// Utterance state (for endpointing)
static bool     in_speech = false;                      // True while inside a speech segment
static uint32_t silence_since = 0;                      // Timestamp when we first saw silence
static int      seg_start_T = 0;                        // Start index (in MFCC frames) of utterance
static uint32_t last_trigger_ms[3] = {0,0,0};           // Cooldown timestamps per class

bool kws_init_i2s() {                                   // Bring up I²S in RX mode for the mic
  i2s_config_t cfg = {                                  // Configure I²S driver parameters
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),// ESP32 provides clock, receives data
    .sample_rate = KWS_SAMPLE_RATE,                     // 16 kHz sample rate
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,       // 32-bit container (mic is 24-bit left-aligned)
    // .channel_format = ...                             // Set later with i2s_set_clk to pick mono/stereo
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,  // Standard I²S framing
    .intr_alloc_flags = 0,                              // Default interrupt allocation
    .dma_buf_count = 6,                                 // DMA buffers for smooth streaming
    .dma_buf_len = 256,                                 // Samples per DMA buffer
    .use_apll = false,                                  // No APLL (use default clocking)
    .tx_desc_auto_clear = false,                        // TX not used (recording only)
    .fixed_mclk = 0                                     // Let driver select MCLK if needed
  };

  i2s_pin_config_t pins = {                             // Map I²S pins to GPIO
    .bck_io_num   = I2S_BCLK_PIN,                       // BCLK/SCK pin
    .ws_io_num    = I2S_WS_PIN,                         // LRCLK/WS pin
    .data_out_num = -1,                                 // No TX (not transmitting audio)
    .data_in_num  = I2S_DOUT_PIN                        // Data-in connected to mic DO/SD
  };

  if (i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr) != ESP_OK) { // Install I²S driver on port 0
    Serial.println("[KWS] i2s_driver_install failed");  // Log failure
    return false;                                       // Abort mic
  }
  if (i2s_set_pin(I2S_NUM_0, &pins) != ESP_OK) {        // Apply pin mapping to driver
    Serial.println("[KWS] i2s_set_pin failed");         // Log failure
    return false;                                       // Abort mic
  }

  if (MIC_RIGHT_CHANNEL) {                              // If using RIGHT channel from stereo
    i2s_set_clk(I2S_NUM_0, KWS_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO); // Stereo mode
    Serial.printf("[KWS] I2S stereo @%d Hz (picking RIGHT)\n", KWS_SAMPLE_RATE); // Announce mode
  } else {
    i2s_set_clk(I2S_NUM_0, KWS_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);   // Mono LEFT
    Serial.printf("[KWS] I2S mono-left @%d Hz\n", KWS_SAMPLE_RATE); // Announce mode
  }

  i2s_zero_dma_buffer(I2S_NUM_0);                       // Clear DMA buffers to known state
  return true;                                          // Mic ready
}

void kws_process_audio() {                              // Main audio ISR-like pump (called in loop)
  int32_t raw[128];                                     // Buffer for raw 32-bit I²S words
  size_t nbytes = 0;                                    // Byte count actually read
  if (i2s_read(I2S_NUM_0, (void*)raw, sizeof(raw), &nbytes, 0) != ESP_OK) return; // Non-blocking read
  int n = nbytes / sizeof(raw[0]);                      // Convert bytes to sample count
  if (n <= 0) return;                                   // Nothing to process

  const int start  = MIC_RIGHT_CHANNEL ? 1 : 0;         // Starting index (RIGHT channel if stereo)
  const int stride = MIC_RIGHT_CHANNEL ? 2 : 1;         // Step over interleaved stereo if needed

  for (int i = start; i < n; i += stride) {             // Iterate samples of the chosen channel
    int32_t s24 = (raw[i] >> 8);                        // Shift to sign-extend 24-bit sample
    float   s   = (float)s24 / 8388607.0f;              // Normalize 24-bit to float in [-1,1]

    if (s > 1.5f) s = 1.5f;                             // Clamp extreme spikes (basic protection)
    if (s < -1.5f) s = -1.5f;                           // Clamp negative spikes

    if (hop_accum < KWS_FRAME_LEN) {                    // Fill the current 25 ms frame buffer
      frame_buf[hop_accum] = s;                         // Store sample into frame
      hop_accum++;                                      // Advance frame cursor
    }

    if (hop_accum == KWS_FRAME_LEN) {                   // When we complete one frame
      float rms = 0.0f;                                 // Local RMS accumulator
      for (int k = 0; k < KWS_FRAME_LEN; ++k) rms += frame_buf[k]*frame_buf[k]; // Sum squares
      rms = sqrtf(rms / KWS_FRAME_LEN);                 // RMS = sqrt(mean of squares)
      vad_rms = 0.95f*vad_rms + 0.05f*rms;              // Exponential smoothing for VAD

      float coeffs[KWS_N_MFCC];                         // Output buffer for MFCCs
      kws_mfcc_frame(frame_buf, coeffs);                // Compute MFCCs for this frame
      if (mfcc_T < KWS_MAX_FRAMES) {                    // If ring buffer not full
        for (int k = 0; k < KWS_N_MFCC; ++k) mfcc_seq[mfcc_T][k] = coeffs[k]; // Append frame
        mfcc_T++;                                       // Increase valid frame count
      } else {                                          // If ring buffer full, shift left by one
        for (int t = 0; t < KWS_MAX_FRAMES-1; ++t) {
          memcpy(mfcc_seq[t], mfcc_seq[t+1], sizeof(mfcc_seq[t])); // Move older frames
        }
        memcpy(mfcc_seq[KWS_MAX_FRAMES-1], coeffs, sizeof(coeffs)); // Put newest at end
        if (in_speech && seg_start_T > 0) seg_start_T--; // Keep start index aligned after shift
      }

      uint32_t now = millis();                          // Current time for hang/cooldown logic
      if (!in_speech) {                                 // If currently not speaking
        if (vad_rms >= VAD_ON) {                        // Threshold crossed upward → start speech
          in_speech = true;                              // Enter speech state
          seg_start_T = max(0, mfcc_T - 1);             // Mark start near current frame
          silence_since = 0;                             // Reset silence timer
        }
      } else {                                          // Currently in speech
        if (vad_rms < VAD_OFF) {                        // Fell below off threshold (silence)
          if (silence_since == 0) silence_since = now;  // Start silence hang timer on first entry
          if (now - silence_since >= SILENCE_HANG_MS) { // If long enough silence → end utterance
            int seg_end_T = mfcc_T;                     // End index (one past last frame)
            int seg_len   = seg_end_T - seg_start_T;    // Utterance length in frames

            float d1=0, d2=0;                           // Distances for debug
            int cls = (seg_len >= MIN_SEG_FRAMES)
                      ? kws_classify_segment(seg_start_T, seg_end_T, &d1, &d2) // Classify the segment
                      : -1;                              // Too short → reject

            #if KWS_DEBUG
              Serial.printf("[KWS] SEG end: cls=%d best=%.2f second=%.2f frames=%d\n",
                            cls, d1, d2, seg_len);      // Print classification result
            #endif

            if (cls >= 0 && (now - last_trigger_ms[cls] >= CLASS_COOLDOWN_MS)) { // Debounce per class
              last_trigger_ms[cls] = now;               // Arm cooldown

              if (cls == KWS_CLS_WAKE) {                // Wake toggles auto-fan mode
                bool newState = !autoFanEnabled;        // Flip mode
                setAutoFan(newState);                   // Apply indicator + fan/sleep behavior
                Serial.println(newState ? "ready" : "sleep"); // Print acknowledgement

              } else if (cls == KWS_CLS_LIGHTS_ON) {    // Lights ON detected
                if (seg_len < ON_MIN_FRAMES) {          // If very short, treat as wake fallback
                  bool newState = !autoFanEnabled;      // Toggle auto mode instead of lights
                  setAutoFan(newState);                 // Apply
                  Serial.println(newState ? "ready" : "sleep"); // Report
                  #if KWS_DEBUG
                    Serial.printf("[KWS] ON->WAKE fallback (seg_len=%d)\n", seg_len); // Explain fallback
                  #endif
                } else {
                  applyLight(true);                     // Drive external LED ON and publish status
                }

              } else if (cls == KWS_CLS_LIGHTS_OFF) {   // Lights OFF detected
                applyLight(false);                      // Drive external LED OFF and publish status
              }
            }
            in_speech = false;                          // Reset speech state after classification
            silence_since = 0;                          // Clear silence timer
          }
        } else {
          silence_since = 0;                            // If voice continues, keep resetting timer
        }
      }

      memmove(frame_buf,                                // Slide window by hop (overlap-add style)
              frame_buf + KWS_HOP_LEN,
              sizeof(float) * (KWS_FRAME_LEN - KWS_HOP_LEN)); // Shift retained samples
      hop_accum = KWS_FRAME_LEN - KWS_HOP_LEN;          // Set new cursor position after shift
    }
  }
}

// ---------- Helpers ----------

void applyLight(bool on) {                              // Toggle external LED and publish status
  if (on == lightOn) return;                            // No change → no work
  lightOn = on;                                         // Update shadow state
  digitalWrite(LED_LIGHT_PIN, on ? HIGH : LOW);         // Write GPIO to drive LED on/off
  mqtt.publish(TOPIC_LIGHT_STATUS, on ? "ON" : "OFF", true); // Publish retained status to broker
  Serial.printf("[STATE] lightOn=%s\n", on ? "ON" : "OFF"); // Log state
}

void onMsg(char* topic, byte* payload, unsigned int len) { // MQTT callback for all subscribed topics
  char buf[16];                                         // Temporary string buffer for payload
  size_t n = min(len, (unsigned int)sizeof(buf) - 1);   // Clamp copy to buffer capacity
  memcpy(buf, payload, n);                              // Copy raw bytes into local buffer
  buf[n] = '\0';                                        // Null-terminate so it's a C-string
  Serial.printf("[MQTT] %s <- '%s'\n", topic, buf);     // Log incoming message for debugging

  if (strcmp(topic, TOPIC_LIGHT_SET) == 0) {            // If command is for lights/set
    for (size_t i = 0; i < n; ++i) buf[i] = (char)toupper((unsigned char)buf[i]); // Uppercase for case-insensitive compare
    if (!strcmp(buf, "ON") || !strcmp(buf, "1") || !strcmp(buf, "TRUE")) { // On tokens
      applyLight(true);                                 // Turn lights on
    } else if (!strcmp(buf, "OFF") || !strcmp(buf, "0") || !strcmp(buf, "FALSE")) { // Off tokens
      applyLight(false);                                // Turn lights off
    } else {
      Serial.printf("[WARN] Unknown payload on lights/set: '%s'\n", buf); // Warn on unknown command
    }
  }
}

// ---------- Connectivity ----------

void ensureWiFi() {                                     // Connect to Wi-Fi if not already connected
  if (WiFi.status() == WL_CONNECTED) return;            // Early exit if already connected
  WiFi.mode(WIFI_STA);                                  // Station mode to join AP
  WiFi.begin(WIFI_SSID, WIFI_PASS);                     // Start association with credentials
  Serial.print("[WiFi] Connecting");                    // Progress log
  while (WiFi.status() != WL_CONNECTED) {               // Loop until connected
    delay(250);                                         // Small delay to yield CPU/radio
    Serial.print(".");                                  // Visual progress dots
  }
  Serial.printf("\n[WiFi] Connected. IP=%s, RSSI=%d dBm\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI()); // Print DHCP IP and signal strength
}

// Optional creds (compile-time overrides or defaults)
#ifndef MQTT_USER
  #define MQTT_USER nullptr                              // Default to anonymous if not defined
  #define MQTT_PASS nullptr                              // Default to anonymous if not defined
#endif

void ensureMqtt() {                                     // Connect/reconnect to MQTT broker
  if (mqtt.connected()) return;                         // Early exit if already connected

  IPAddress hostIP;                                     // Will hold resolved numeric IP
  if (!WiFi.hostByName(MQTT_HOST, hostIP)) {            // DNS or literal IP resolution
    Serial.printf("[MQTT] RESOLVE FAIL for '%s'\n", MQTT_HOST); // Log failure
    delay(1000);                                        // Backoff
    return;                                             // Try again next loop
  }
  Serial.printf("[MQTT] HOST '%s' -> %s:%u\n",
                MQTT_HOST, hostIP.toString().c_str(), MQTT_PORT); // Show resolved address

  WiFiClient probe;                                     // Short-lived TCP probe client
  Serial.print("[MQTT] TCP CONNECT ... ");              // Announce probe
  if (!probe.connect(hostIP, MQTT_PORT)) {              // Try to open TCP socket to broker
    Serial.println("NO TCP (wrong IP / firewall)");     // Failed connect (network or firewall)
    delay(1000);                                        // Backoff
    return;                                             // Exit to retry later
  }
  Serial.println("OK");                                 // TCP reachable
  probe.stop();                                         // Close probe socket

  mqtt.setServer(hostIP, MQTT_PORT);                    // Point MQTT client to resolved host/port
  mqtt.setSocketTimeout(4);                             // Shorter socket timeouts for snappier retries
  mqtt.setKeepAlive(20);                                // MQTT keepalive interval (seconds)

  String clientId = "esp32-env-" + String((uint32_t)ESP.getEfuseMac(), HEX); // Unique-ish clientID
  Serial.print("[MQTT] MQTT CONNECT ... ");             // Announce real MQTT connect
  bool ok = mqtt.connect(clientId.c_str(),              // Connect with LWT on env/status
                         MQTT_USER, MQTT_PASS,
                         TOPIC_ENV_STATUS, 0, true, "offline");
  if (!ok) {                                            // If connect failed
    Serial.printf("FAIL state=%d\n", mqtt.state());     // Print reason code for diagnostics
    delay(1000);                                        // Backoff
    return;                                             // Try again in next loop
  }

  Serial.println("OK");                                 // Connected to broker
  mqtt.subscribe(TOPIC_LIGHT_SET);                      // Subscribe to lights control topic
  mqtt.publish(TOPIC_LIGHT_STATUS, lightOn ? "ON" : "OFF", true); // Publish initial lights state (retained)
  mqtt.publish(TOPIC_ENV_STATUS, "online", true);       // Set status retained to "online"
}

void publish_env(float t, float h, float p) {           // Publish JSON snapshot and scalar topics
  char json[160];                                       // Temp buffer for JSON string
  snprintf(json, sizeof(json),                          // Compose compact JSON payload
           "{\"temp_c\":%.2f,\"humidity\":%.2f,\"pressure_hpa\":%.2f,\"rssi\":%d}",
           t, h, p, WiFi.RSSI());                       // Include Wi-Fi RSSI for quick diagnostics
  mqtt.publish(TOPIC_ENV_STATE, json, true);            // Publish retained JSON snapshot

  char buf[32];                                         // Buffer for scalar conversions
  dtostrf(t, 0, 2, buf); mqtt.publish(TOPIC_ENV_TEMP, buf, true); // Temperature as string
  dtostrf(h, 0, 2, buf); mqtt.publish(TOPIC_ENV_HUM,  buf, true); // Humidity as string
  dtostrf(p, 0, 2, buf); mqtt.publish(TOPIC_ENV_PRES, buf, true); // Pressure as string

  Serial.printf("[ENV] T=%.2f°C  RH=%.2f%%  P=%.2f hPa\n", t, h, p); // Log values for serial debug
}

bool init_bme_spi() {                                   // Initialize BME280 over software SPI
  if (!bme.begin()) return false;                       // Probe sensor; return false if not found
  uint8_t id = bme.sensorID();                          // Read chip ID to distinguish BME/BMP
  Serial.printf("[BME] sensorID=0x%02X (%s)\n",
                id, (id==0x60?"BME280":(id==0x58?"BMP280":"unknown"))); // Log detected device

  bme.setSampling(                                     // Configure oversampling/filter/standby
    Adafruit_BME280::MODE_NORMAL,                      // Continuous sampling mode
    Adafruit_BME280::SAMPLING_X2,                      // Temperature oversampling x2
    Adafruit_BME280::SAMPLING_X16,                     // Pressure oversampling x16
    Adafruit_BME280::SAMPLING_X1,                      // Humidity oversampling x1 (BME only)
    Adafruit_BME280::FILTER_X16,                       // IIR filter x16 for smoother values
    Adafruit_BME280::STANDBY_MS_500                    // 500 ms standby between measurements
  );
  return true;                                         // Sensor initialized
}

// ---------- Arduino ----------

void setup() {                                          
  Serial.begin(115200);                                 // Initialize UART for logs at 115200 baud
  delay(150);                                           // Small delay to let serial terminal attach
  Serial.println("\n[BOOT] ESP32 + MQTT + BME280 (SPI) + KWS + AutoFan"); // Banner

  pinMode(LED_PIN, OUTPUT);                             // Configure built-in LED as output
  digitalWrite(LED_PIN, LOW);                           // Start with auto-fan indicator OFF
  pinMode(LED_LIGHT_PIN, OUTPUT);                       // Configure external LED pin as output
  digitalWrite(LED_LIGHT_PIN, LOW);                     // Start with lights OFF
  pinMode(FAN_PIN, OUTPUT);                             // Configure fan driver pin as output
  digitalWrite(FAN_PIN, LOW);                           // Ensure fan is OFF at boot
  fanOn = false;                                        // Shadow state reset
  lightOn = false;                                      // Shadow state reset for lights

  ensureWiFi();                                         // Connect to Wi-Fi (blocking until connected)
  mqtt.setCallback(onMsg);                              // Register MQTT message handler
  ensureMqtt();                                         // Connect to MQTT and subscribe/publish LWT

  if (!init_bme_spi()) {                                // Try to start BME/BMP
    Serial.println("[ERROR] BME/BMP not found over SPI. Check SCK=18, MISO=19, MOSI=23, CS=5, power & GND."); // Wiring hint
  } else {
    Serial.println("[BME] Initialized.");               // Confirm sensor ready
  }

  mqtt.publish(TOPIC_LIGHT_STATUS, "OFF", true);        // Publish initial lights state (retained)
  mqtt.publish(TOPIC_ENV_STATUS, "online", true);       // Announce device online state (retained)

  kws_prepare_dsp();                                    // Precompute DSP constants (Hamming/mel/DCT)
  if (!kws_init_i2s()) {                                // Bring up the microphone interface
    Serial.println("[KWS] I2S init failed (check mic pins). Keyword spotting disabled."); // Fail notice
  } else {
    Serial.println("[KWS] Mic ready @16 kHz, MFCC online."); // Success notice
  }
}

void loop() {                                           
  ensureWiFi();                                         // Maintain Wi-Fi (reconnect if needed)
  ensureMqtt();                                         // Maintain MQTT (reconnect if needed)
  mqtt.loop();                                          // Service MQTT client (process packets)

  static unsigned long lastRead = 0;                    // Last sensor read timestamp
  const unsigned long now = millis();                   // Current time in ms

  if (now - lastRead >= 500) {                          // Read sensor every 500 ms
    lastRead = now;                                     // Update schedule

    float t = bme.readTemperature();                    // Read temperature (°C)
    float h = bme.readHumidity();                       // Read humidity (%RH) (NaN on BMP280)
    float p = bme.readPressure() / 100.0f;              // Read pressure (Pa) → convert to hPa

    if (!isnan(t)) current_temp_c = t;                  // Cache temperature for fan control

    if (!isnan(t) || !isnan(h) || !isnan(p)) {          // Only proceed if at least one value valid
      bool changed = false;                             // Flag if any delta exceeds epsilon
      if (isnan(last_t) || (!isnan(t) && fabsf(t - last_t) > MIN_DELTA_TEMP)) changed = true; // Temp delta
      if (isnan(last_h) || (!isnan(h) && fabsf(h - last_h) > MIN_DELTA_HUM))  changed = true; // Hum delta
      if (isnan(last_p) || (!isnan(p) && fabsf(p - last_p) > MIN_DELTA_PRES)) changed = true; // Press delta

      if (changed || (now - t_last_pub) >= PUBLISH_MS) { // Publish if changed enough or time elapsed
        publish_env(t, h, p);                           // Publish snapshot + scalars (retained)
        last_t = t; last_h = h; last_p = p;             // Update last values
        t_last_pub = now;                               // Reset rate limiter
      }
    }

    controlFanByTemp();                                 // Apply thermostat logic (auto mode only)
  }

  delay(10);                                            // Yield CPU to Wi-Fi/MQTT stacks
  kws_process_audio();                                  // Pump audio → MFCC → KWS state machine
}
