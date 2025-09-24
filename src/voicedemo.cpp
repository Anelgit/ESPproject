/*
#include <Arduino.h>
#include "driver/i2s.h"
#include "wakeword_templates.h"   // <-- from Colab export

// -------- I2S pins (your wiring) --------
static const i2s_port_t I2S_PORT = I2S_NUM_0;
static const int PIN_I2S_BCLK   = 26; // SCK
static const int PIN_I2S_LRCLK  = 25; // WS
static const int PIN_I2S_DOUT   = 33; // DO

// -------- Audio / MFCC params --------
static const uint32_t SR           = 16000;  // use 16 kHz for MFCC/DTW
static const int      WIN_LEN      = int(0.025f * SR);     // 25 ms -> 400
static const int      HOP_LEN      = int(0.010f * SR);     // 10 ms -> 160
static const int      N_FFT        = 512;
static const int      N_MELS       = 40;
static const int      CAPTURE_S    = 1;                    // 1 second window
static const int      CAPTURE_N    = SR * CAPTURE_S;

// -------- Buffers --------
static int16_t  pcm16[CAPTURE_N];
static float    mfcc_buf[ (CAPTURE_N - WIN_LEN) / HOP_LEN + 2 ][ KWS_N_MFCC ]; // ~98x13

// Util
static inline int32_t s24_to_s32(int32_t x) { return (x >> 8); }
static inline float   fast_absf(float x)     { return x < 0 ? -x : x; }

// ---------------- I2S init ----------------
void i2s_install() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SR,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  ESP_ERROR_CHECK(i2s_driver_install(I2S_PORT, &cfg, 0, nullptr));
  i2s_pin_config_t pins = {
    .bck_io_num = PIN_I2S_BCLK,
    .ws_io_num = PIN_I2S_LRCLK,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = PIN_I2S_DOUT
  };
  ESP_ERROR_CHECK(i2s_set_pin(I2S_PORT, &pins));
  i2s_zero_dma_buffer(I2S_PORT);
  // stereo framing (64*fs) keeps many MEMS mics happy
  ESP_ERROR_CHECK(i2s_set_clk(I2S_PORT, SR, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO));
}

// --------------- Simple VAD ---------------
bool is_voice_frame(const int16_t* x, int n) {
  // energy threshold on 20 ms
  uint64_t acc = 0;
  for (int i=0;i<n;i++) { int32_t s = x[i]; acc += (uint64_t)(s*s); }
  float rms = sqrtf( (float)acc / (float)n );
  static float avg = 0.f;
  avg = 0.98f*avg + 0.02f*rms;
  float th = avg*2.0f + 600.0f;  // tune if needed
  return rms > th;
}

// --------- Minimal MFCC on-device ---------
// (runtime builds mel filterbank and DCT once)
struct MelContext {
  bool   inited = false;
  float* mel_fb;    // [N_MELS * (N_FFT/2+1)]
  float* dct;       // [KWS_N_MFCC * N_MELS]
  float* win;       // [WIN_LEN]
  int    spec_bins;
} melctx;

float hz_to_mel(float f){ return 2595.f*log10f(1.f + f/700.f); }
float mel_to_hz(float m){ return 700.f*(powf(10.f, m/2595.f)-1.f); }

void build_mel_and_dct() {
  if (melctx.inited) return;
  melctx.spec_bins = N_FFT/2 + 1;
  melctx.mel_fb = (float*)heap_caps_malloc(N_MELS*melctx.spec_bins*sizeof(float), MALLOC_CAP_8BIT);
  melctx.dct    = (float*)heap_caps_malloc(KWS_N_MFCC*N_MELS*sizeof(float),        MALLOC_CAP_8BIT);
  melctx.win    = (float*)heap_caps_malloc(WIN_LEN*sizeof(float),                   MALLOC_CAP_8BIT);

  // Hann window
  for (int n=0;n<WIN_LEN;n++) melctx.win[n] = 0.5f - 0.5f*cosf(2*M_PI*n/WIN_LEN);

  // Mel filterbank
  float fmin=20.f, fmax=SR/2.f;
  float mmin = hz_to_mel(fmin), mmax = hz_to_mel(fmax);
  // N_MELS+2 points
  float mel_pts[N_MELS+2];
  for (int i=0;i<N_MELS+2;i++) mel_pts[i] = mmin + (mmax-mmin)*i/(N_MELS+1);
  float hz_pts[N_MELS+2];
  for (int i=0;i<N_MELS+2;i++) hz_pts[i] = mel_to_hz(mel_pts[i]);

  int bins[N_MELS+2];
  for (int i=0;i<N_MELS+2;i++) {
    float frac = hz_pts[i] / (SR/2.f);
    int   b    = (int)floorf(frac * (melctx.spec_bins-1));
    if (b<0) b=0; if (b>=melctx.spec_bins) b = melctx.spec_bins-1;
    bins[i] = b;
  }

  memset(melctx.mel_fb, 0, N_MELS*melctx.spec_bins*sizeof(float));
  for (int m=1; m<=N_MELS; m++) {
    int a = bins[m-1], b = bins[m], c = bins[m+1];
    if (b<=a) b=a+1;
    if (c<=b) c=b+1;
    for (int k=a; k<b; k++) melctx.mel_fb[(m-1)*melctx.spec_bins + k] = (float)(k-a)/(float)(b-a);
    for (int k=b; k<c; k++) melctx.mel_fb[(m-1)*melctx.spec_bins + k] = (float)(c-k)/(float)(c-b);
  }

  // DCT-II matrix (KWS_N_MFCC x N_MELS)
  for (int m=0;m<KWS_N_MFCC;m++){
    for (int n=0;n<N_MELS;n++){
      float val = sqrtf(2.f/N_MELS) * cosf(M_PI*(n+0.5f)*m / N_MELS);
      if (m==0) val *= 1.f/sqrtf(2.f);
      melctx.dct[m*N_MELS + n] = val;
    }
  }

  melctx.inited = true;
}

// real FFT using ESP32’s double-precision FFT via std::complex & Kiss-like fallback
// we’ll use simple Cooley–Tukey via std::complex FFT from Arduino core (sufficient at N=512)
#include <complex>
void rfft_mag(const float* x, int n, int nfft, float* mag_out) {
  // copy & zero-pad
  static std::complex<float> buf[N_FFT];
  for (int i=0;i<n;i++) buf[i] = std::complex<float>(x[i], 0.f);
  for (int i=n;i<nfft;i++)   buf[i] = std::complex<float>(0.f, 0.f);

  // FFT (naive iterative DFT is too slow; this is a small radix-2 impl)
  // Minimal iterative Cooley–Tukey (in-place)
  // bit-reverse
  int j=0;
  for (int i=1;i<nfft;i++){
    int bit = nfft>>1;
    for (; j & bit; bit >>=1) j &= ~bit;
    j |= bit;
    if (i<j) { auto t=buf[i]; buf[i]=buf[j]; buf[j]=t; }
  }
  for (int len=2; len<=nfft; len<<=1){
    float ang = -2.f*M_PI/len;
    std::complex<float> wlen(cosf(ang), sinf(ang));
    for (int i=0; i<nfft; i+=len){
      std::complex<float> w(1.f,0.f);
      for (int k=0;k<len/2;k++){
        auto u = buf[i+k];
        auto v = buf[i+k+len/2] * w;
        buf[i+k]         = u+v;
        buf[i+k+len/2]   = u-v;
        w *= wlen;
      }
    }
  }
  // magnitude of positive freqs
  int bins = nfft/2 + 1;
  for (int i=0;i<bins;i++) mag_out[i] = std::abs(buf[i]);
}

// Compute MFCCs into mfcc_buf; returns number of frames
int compute_mfcc(const int16_t* pcm, int n) {
  build_mel_and_dct();

  // pre-emphasis and framing
  static float frame[WIN_LEN];
  static float spec_mag[N_FFT/2+1];
  int frames = 0;

  for (int start=0; start+WIN_LEN <= n; start += HOP_LEN) {
    // pre-emphasis + window
    for (int i=0;i<WIN_LEN;i++){
      float s = (i==0) ? (float)pcm[start] : ((float)pcm[start+i] - 0.97f*(float)pcm[start+i-1]);
      frame[i] = s * melctx.win[i];
    }
    // FFT magnitude
    rfft_mag(frame, WIN_LEN, N_FFT, spec_mag);

    // mel projection
    static float melvec[N_MELS];
    for (int m=0;m<N_MELS;m++){
      const float* fb = &melctx.mel_fb[m* (N_FFT/2+1)];
      float acc = 0.f;
      for (int k=0;k<(N_FFT/2+1);k++) acc += spec_mag[k]*fb[k];
      if (acc < 1e-10f) acc = 1e-10f;
      melvec[m] = logf(acc);
    }
    // DCT to MFCC
    for (int c=0;c<KWS_N_MFCC;c++){
      const float* row = &melctx.dct[c*N_MELS];
      float z = 0.f;
      for (int m=0;m<N_MELS;m++) z += row[m]*melvec[m];
      mfcc_buf[frames][c] = z;
    }
    frames++;
  }
  return frames;
}

// --------------- DTW over MFCC ---------------
float dtw_distance(const float* A, int T1, const int16_t* Bq, int T2, int F, float scaleB) {
  // A: float [T1 x F]; B: int16 [T2 x F] scaled by scaleB
  // Memory: use 2 rows rolling to save RAM
  static float prev[200]; // adjust if your frames exceed 200 (for 1s @ 10ms hop ~98)
  static float curr[200];
  for (int j=0;j<=T2;j++) prev[j] = (j==0)?0.f:INFINITY;

  for (int i=1;i<=T1;i++){
    curr[0] = INFINITY;
    for (int j=1;j<=T2;j++){
      // cost = L2 between A[i-1,:] and dequantized Bq[j-1,:]
      float cost = 0.f;
      const float* a = &A[(i-1)*F];
      const int16_t* b = &Bq[(j-1)*F];
      for (int k=0;k<F;k++){
        float diff = a[k] - ( (float)b[k] * (scaleB/32767.f) );
        cost += diff*diff;
      }
      cost = sqrtf(cost);
      float v = prev[j];              // (i-1, j)
      if (curr[j-1] < v) v = curr[j-1];     // (i, j-1)
      if (prev[j-1] < v) v = prev[j-1];     // (i-1, j-1)
      curr[j] = cost + v;
    }
    // swap rows
    for (int j=0;j<=T2;j++) prev[j] = curr[j];
  }
  return prev[T2];
}

// ---- Capture 1 second into pcm16 ----
void capture_1s_into_pcm16() {
  size_t got = 0;
  while (got < CAPTURE_N) {
    int32_t buf32[1024];
    size_t br = 0;
    if (i2s_read(I2S_PORT, buf32, sizeof(buf32), &br, portMAX_DELAY) != ESP_OK || br==0) continue;
    size_t n = br / sizeof(int32_t);
    for (size_t i=0;i<n && got < CAPTURE_N;i++){
      int32_t s32 = s24_to_s32(buf32[i]);   // 24-bit in 32-bit slot
      int16_t s16 = (int16_t)(s32 >> 8);    // scale down
      pcm16[got++] = s16;
    }
  }
}

// ---- Simple “listen then detect” loop ----
bool wakeword_detected_once() {
  // 1) fill with 1s of audio (optionally gate with VAD)
  capture_1s_into_pcm16();

  // quick VAD: if <20% of frames are voiced, skip
  int voiced = 0, total = 0;
  for (int start=0; start+WIN_LEN <= CAPTURE_N; start += HOP_LEN) {
    voiced += is_voice_frame(&pcm16[start], WIN_LEN) ? 1 : 0;
    total++;
  }
  if (voiced < total/5) return false;

  // 2) MFCC
  int T = compute_mfcc(pcm16, CAPTURE_N);
  if (T <= 0) return false;

  // 3) DTW to all templates; take best distance
  float best = 1e30f;
  for (int ti=0; ti<KWS_NUM_TEMPLATES; ti++) {
    int T2   = KWS_TEMPL_FRAMES[ti];
    int F2   = KWS_TEMPL_COEFFS[ti];
    uint32_t off = KWS_TEMPL_OFFSET[ti];
    const int16_t* Bq = &KWS_TEMPL_DATA[off];
    float scale = KWS_TEMPL_SCALE[ti];
    if (F2 != KWS_N_MFCC) continue; // safety

    float d = dtw_distance(&mfcc_buf[0][0], T, Bq, T2, KWS_N_MFCC, scale);
    if (d < best) best = d;
  }

  // 4) Threshold — print distance to tune
  Serial.printf("DTW best distance: %.1f\n", best);
  // Start with a loose threshold and tighten after observing:
  // Talk in quiet room: expect ~200–900. Silence/noise: usually >> 1200.
  return (best < 900.0f);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nWakeword DTW demo @16k");
  i2s_install();
}

void loop() {
  if (wakeword_detected_once()) {
    Serial.println("word received");
    // debounce a bit
    delay(500);
  }
}





// test code for mic sanity check @48k

#include <Arduino.h>
#include "driver/i2s.h"

static const i2s_port_t I2S_PORT = I2S_NUM_0;
static const int PIN_I2S_BCLK   = 26; // Mic SCK
static const int PIN_I2S_LRCLK  = 25; // Mic WS
static const int PIN_I2S_DOUT   = 33; // Mic DO

// If SEL is tied to GND → LEFT channel
// If SEL is tied to 3V3 → RIGHT channel
// Make sure this matches your wiring!
#define USE_LEFT_CHANNEL true

static inline int32_t s24_to_s32(int32_t x) { 
    return (x >> 8); }

void i2s_install() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 48000,   // mic likes 48k
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = USE_LEFT_CHANNEL ?
                      I2S_CHANNEL_FMT_ONLY_LEFT :
                      I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  ESP_ERROR_CHECK(i2s_driver_install(I2S_PORT, &cfg, 0, nullptr));

  i2s_pin_config_t pins = {
    .bck_io_num = PIN_I2S_BCLK,
    .ws_io_num = PIN_I2S_LRCLK,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = PIN_I2S_DOUT
  };
  ESP_ERROR_CHECK(i2s_set_pin(I2S_PORT, &pins));
  i2s_zero_dma_buffer(I2S_PORT);
  ESP_ERROR_CHECK(i2s_set_clk(I2S_PORT, 48000, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO));
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nMic sanity test @48k");
  i2s_install();
}

void loop() {
  int32_t buf[1024];
  size_t br = 0;

  if (i2s_read(I2S_PORT, buf, sizeof(buf), &br, portMAX_DELAY) != ESP_OK) return;
  size_t n = br / sizeof(int32_t);

  int16_t mn = 32767, mx = -32768;
  for (size_t i=0; i<n; i++) {
    int32_t s32 = s24_to_s32(buf[i]);
    int16_t s16 = (int16_t)(s32 >> 8);
    if (s16 < mn) mn = s16;
    if (s16 > mx) mx = s16;
  }

  Serial.printf("PCM peek: min=%d max=%d\n", mn, mx);
  delay(200);
}
  

*/
/*******************************************************
 * Project: ESP32 Fan PWM Ramp Test
 * Author : Anel Hodza
 * Date   : Sept 2025
 *
 * Description:
 *   Ramps fan speed smoothly up and down using
 *   ESP32 hardware PWM (LEDC).
 *
 * Hardware:
 *   - ESP32 GPIO drives MOSFET gate (through 220 Ω + 100k pulldown)
 *   - Fan powered from 5 V or 12 V rail
 *   - Flyback diode + decoupling caps across fan
 *******************************************************/
/*
 #include <Arduino.h>

 // ---------- Pin/LEDC config ----------
 const int FAN_PIN = 19;        // GPIO connected to MOSFET gate
 const int PWM_CHANNEL = 0;     // use LEDC channel 0
 const int PWM_FREQ = 2000;     // 2 kHz PWM frequency
 const int PWM_RES = 8;         // 8-bit resolution (0–255)
 
 // ---------- Setup ----------
 void setup() {
   Serial.begin(115200);
 
   // Configure PWM channel
   ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
   // Attach fan pin to PWM channel
   ledcAttachPin(FAN_PIN, PWM_CHANNEL);
 
   Serial.println("Fan PWM ramp test starting...");
 }
 
 // ---------- Loop ----------
 void loop() {
   // Ramp up
   for (int duty = 0; duty <= 255; duty++) {
     ledcWrite(PWM_CHANNEL, duty);   // set fan speed
     delay(10);                      // ~2.5 s ramp up
   }
 
   // Hold full speed
   delay(1000);
 
   // Ramp down
   for (int duty = 255; duty >= 0; duty--) {
     ledcWrite(PWM_CHANNEL, duty);
     delay(10);                      // ~2.5 s ramp down
   }
 
   // Hold off
   delay(1000);
 }
 */
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <Adafruit_BME280.h>
// Keep for later voice work (not used here yet)
#include "wakeword_templates.h"

#include <math.h>
#include <driver/i2s.h>
#include <ArduinoFFT.h>

// ---------- Wi-Fi ----------

// Stores Wi-Fi network name (SSID) as a read-only C string in flash.
const char* WIFI_SSID = "FBI Surveillance van 4";
// Stores Wi-Fi password as a read-only C string in flash.
const char* WIFI_PASS = "anelshjem";

// ---------- MQTT ----------

// IP/hostname of the MQTT broker the ESP connects to.
 const char* MQTT_HOST = "192.168.1.2";
//const char* MQTT_HOST = "192.168.1.10";   // or your PC’s actual IP/hostname

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

void applyLight(bool on); // forward declaration
void ensureWiFi();
void ensureMqtt();

// ---------- Audio / KWS ----------
//
// Lightweight I²S mic + MFCC + template matching (approximate).
// - Uses arduinoFFT to compute spectra
// - 16 kHz mono, 25 ms window, 10 ms hop
// - 26 mel filters → 13 MFCCs (matches KWS_N_MFCC from wakeword_templates.h)
// - Classes assumed by template: 0 = wake word, 1 = "lights on", 2 = "lights off"
//
// I²S pin defaults for common INMP441 / ICS-43434 breakout.
// Change these three if your wiring differs.
#ifndef I2S_BCLK_PIN
#define I2S_BCLK_PIN 26
#endif
#ifndef I2S_WS_PIN
#define I2S_WS_PIN   25  // aka LRCL
#endif
#ifndef I2S_DOUT_PIN
#define I2S_DOUT_PIN 33  // mic data out -> ESP32 in
#endif

// Audio / feature params
static const int   KWS_SAMPLE_RATE   = 16000;
static const int   KWS_FFT_N         = 512;   // power of two
static const int   KWS_FRAME_LEN     = 400;   // 25 ms @ 16 kHz
static const int   KWS_HOP_LEN       = 160;   // 10 ms hop
static const int   KWS_N_FILT        = 26;    // mel filters
static const int   KWS_MAX_FRAMES    = 300;   // ~3.0 s of history (fits templates up to ~243f)

// === KWS debug/toggles ===
#define KWS_DEBUG 1              // 1 = print classifier/vad info
#define MIC_RIGHT_CHANNEL 0      // 0 = LEFT (L/R pin to GND), 1 = RIGHT (L/R pin to VCC)

// Utterance-level VAD & decisioning (helps avoid rapid flapping)
#define VAD_ON            0.0070f     // start speech when RMS rises above this
#define VAD_OFF           0.0035f     // consider silence when below this
#define SILENCE_HANG_MS   700        // how long silence must last to end an utterance
#define CLASS_COOLDOWN_MS 1500       // per-class deadtime to avoid repeats
#define MARGIN_RATIO      0.92f      // require best/second < this (tighter = fewer triggers)
#define GAP_MIN           50.0f     // min gap between best/second to accept (absolute)
#define ABS_DIST_CAP      1e9f      // absolute cap after CMN (safety)
#define MIN_SEG_FRAMES    30         // ignore super short segments (<~0.3 s)
#define MAX_SEG_FRAMES    280        // clamp overly long segments

// Class mapping (aligns with provided templates)
enum { KWS_CLS_WAKE = 0, KWS_CLS_LIGHTS_ON = 1, KWS_CLS_LIGHTS_OFF = 2 };

// DSP state
static float hamming[KWS_FRAME_LEN];
static float dct_m[KWS_N_MFCC * KWS_N_FILT];     // DCT-II matrix (row-major)
static uint16_t mel_bin_edges[KWS_N_FILT + 2];   // edges in FFT bins

// ring buffers for streaming MFCCs (most recent first index = last)
static float mfcc_seq[KWS_MAX_FRAMES][KWS_N_MFCC];
static int   mfcc_T = 0;  // number of valid frames in mfcc_seq

// Work buffers for FFT
static double vReal[KWS_FFT_N];
static double vImag[KWS_FFT_N];
static ArduinoFFT<double> FFT(vReal, vImag, KWS_FFT_N, KWS_SAMPLE_RATE);

// Utility: Hz<->Mel
static inline float hz2mel(float hz){ return 2595.0f * log10f(1.0f + hz / 700.0f); }
static inline float mel2hz(float m){ return 700.0f * (powf(10.0f, m/2595.0f) - 1.0f); }

// Prepare Hamming, mel triangles, DCT-II
void kws_prepare_dsp() {
  // Hamming window
  for (int n = 0; n < KWS_FRAME_LEN; ++n) {
    hamming[n] = 0.54f - 0.46f * cosf(2.0f * PI * n / (KWS_FRAME_LEN - 1));
  }
  // Mel filter bin edges (linearly spaced on mel scale)
  float mel_lo = hz2mel(0.0f);
  float mel_hi = hz2mel(KWS_SAMPLE_RATE / 2.0f);
  for (int m = 0; m < KWS_N_FILT + 2; ++m) {
    float mel = mel_lo + (mel_hi - mel_lo) * m / (KWS_N_FILT + 1);
    float hz  = mel2hz(mel);
    int bin   = (int)floor((KWS_FFT_N + 1) * hz / KWS_SAMPLE_RATE);
    if (bin < 0) bin = 0;
    if (bin > KWS_FFT_N/2) bin = KWS_FFT_N/2;
    mel_bin_edges[m] = (uint16_t)bin;
  }
  // DCT-II matrix to go from 26 log-mel → 13 MFCC (orthonormal-ish)
  for (int i = 0; i < KWS_N_MFCC; ++i) {
    for (int j = 0; j < KWS_N_FILT; ++j) {
      dct_m[i*KWS_N_FILT + j] = cosf(PI * i * (2*j + 1) / (2.0f * KWS_N_FILT));
    }
  }
}

// Compute 13-D MFCC for one frame of mono float samples (len=KWS_FRAME_LEN)
void kws_mfcc_frame(const float *frame, float *out13) {
  // zero-padded FFT buffer
  for (int i = 0; i < KWS_FFT_N; ++i) {
    if (i < KWS_FRAME_LEN) {
      vReal[i] = (double)(frame[i] * hamming[i]);
    } else {
      vReal[i] = 0.0;
    }
    vImag[i] = 0.0;
  }

  FFT.windowing(vReal, KWS_FFT_N, FFT_WIN_TYP_RECTANGLE, FFT_FORWARD); // window already applied
  FFT.compute(vReal, vImag, KWS_FFT_N, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, KWS_FFT_N);

  // Power spectrum bins 0..N/2
  // Triangular mel filters -> 26 energies
  float e[KWS_N_FILT];
  for (int m = 0; m < KWS_N_FILT; ++m) {
    int b0 = mel_bin_edges[m];
    int b1 = mel_bin_edges[m+1];
    int b2 = mel_bin_edges[m+2];
    float sum = 0.0f;
    if (b0 == b1) b0 = (b0>0?b0-1:b0);
    if (b1 == b2) b2 = (b2<KWS_FFT_N/2?b2+1:b2);

    // rising slope b0..b1
    for (int k = b0; k < b1; ++k) {
      float w = (k - b0) / (float)(b1 - b0);
      float p = (float)vReal[k];  // magnitude
      sum += (p * p) * w;
    }
    // falling slope b1..b2
    for (int k = b1; k < b2; ++k) {
      float w = (b2 - k) / (float)(b2 - b1);
      float p = (float)vReal[k];
      sum += (p * p) * w;
    }
    e[m] = logf(sum + 1e-8f);
  }

  // DCT-II to 13 coeffs, drop 0th if you like; here we keep c0..c12
  for (int i = 0; i < KWS_N_MFCC; ++i) {
    float acc = 0.0f;
    const float *row = &dct_m[i*KWS_N_FILT];
    for (int j = 0; j < KWS_N_FILT; ++j) acc += row[j] * e[j];
    out13[i] = acc;
  }
}

// Approximate template distance with CMN, aligning the **ends** and using min(L, T) frames.
// This avoids returning "huge" distances when the utterance is shorter than the template.
float kws_template_distance_align_end(const float *seq /*T*K*/, int T, int K,
                                      const int16_t *templ, int L, float scale) {
  int Lp = (T < L ? T : L);
  if (Lp <= 0) return 1e30f;

  const int startA = (T - Lp) * K;      // take last Lp frames from the utterance
  const int startB = (L - Lp) * K;      // take last Lp frames from the template
  const float invS = 1.0f / scale;

  // Means (CMN)
  float meanA[32]; float meanB[32];
  for (int k = 0; k < K; ++k) { meanA[k] = 0.0f; meanB[k] = 0.0f; }
  for (int t = 0; t < Lp; ++t) {
    const float   *a = &seq[startA + t*K];
    const int16_t *b = &templ[startB + t*K];
    for (int k = 0; k < K; ++k) {
      meanA[k] += a[k];
      meanB[k] += (float)b[k] * invS;
    }
  }
  for (int k = 0; k < K; ++k) { meanA[k] /= (float)Lp; meanB[k] /= (float)Lp; }

  // Distance
  float acc = 0.0f;
  for (int t = 0; t < Lp; ++t) {
    const float   *a = &seq[startA + t*K];
    const int16_t *b = &templ[startB + t*K];
    for (int k = 0; k < K; ++k) {
      float av = a[k] - meanA[k];
      float bv = (float)b[k] * invS - meanB[k];
      float d  = av - bv;
      acc += d * d;
    }
  }
  return acc / (float)(Lp * K);
}

// Classify a specific MFCC slice (utterance)
int kws_classify_segment(int startT, int endT, float *bestOut, float *secondOut) {
  int T = endT - startT;
  if (T < MIN_SEG_FRAMES) return -1;
  if (T > MAX_SEG_FRAMES) T = MAX_SEG_FRAMES; // clamp very long segments

  static float flat[KWS_MAX_FRAMES * KWS_N_MFCC];
  for (int t = 0; t < T; ++t) {
    for (int k = 0; k < KWS_N_MFCC; ++k) {
      flat[t*KWS_N_MFCC + k] = mfcc_seq[startT + t][k];
    }
  }

  float bestByClass[3] = {1e30f, 1e30f, 1e30f};
  for (int ti = 0; ti < KWS_NUM_TEMPLATES; ++ti) {
    int cls   = KWS_TEMPL_CLASS_IDX[ti];
    int L     = KWS_TEMPL_FRAMES[ti];
    int K     = KWS_TEMPL_COEFFS[ti];
    int off   = (int)KWS_TEMPL_OFFSET[ti];
    const int16_t *td = &KWS_TEMPL_DATA[off];
    float sc   = KWS_TEMPL_SCALE[ti];

    float d = kws_template_distance_align_end(flat, T, K, td, L, sc);
    if (d < bestByClass[cls]) bestByClass[cls] = d;
  }

  int winner = -1;
  float best = 1e30f, second = 1e30f;
  for (int c = 0; c < 3; ++c) {
    float d = bestByClass[c];
    if (d < best) { second = best; best = d; winner = c; }
    else if (d < second) { second = d; }
  }
  if (bestOut) *bestOut = best;
  if (secondOut) *secondOut = second;

  // Relative margin + absolute cap
// Relative + absolute gap acceptance
float ratio = (second > 0.0f) ? (best / second) : 0.0f;
float gap   = (second > 0.0f) ? (second - best) : 0.0f;

// Optional length sanity (helps avoid odd segments)
int seg_len = T;  // number of MFCC frames in the utterance
bool len_ok = true;
if (winner == KWS_CLS_LIGHTS_ON)  len_ok = (seg_len >= 40 && seg_len <= 120);
if (winner == KWS_CLS_LIGHTS_OFF) len_ok = (seg_len >= 45 && seg_len <= 140);
// Wake word stays unbounded

if (winner >= 0
    && len_ok
    && (ratio < MARGIN_RATIO || gap > GAP_MIN)
    && best < ABS_DIST_CAP) {
  return winner;
}
return -1;

}

// I²S mic init + non-blocking sample → frame → MFCC pipeline
static int   hop_accum = 0;
static float frame_buf[KWS_FRAME_LEN];   // sliding window time-domain
static float vad_rms = 0.0f;             // tiny VAD just to skip silence

// Utterance state (for endpointing)
static bool     in_speech = false;
static uint32_t silence_since = 0;
static int      seg_start_T = 0;
static uint32_t last_trigger_ms[3] = {0,0,0};

bool kws_init_i2s() {
  // I²S peripheral in RX, 32-bit samples (many I²S mics are 24-bit left-aligned)
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = KWS_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    // .channel_format = ...   // (intentionally omitted; we set it with i2s_set_clk)
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 6,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pins = {
    .bck_io_num   = I2S_BCLK_PIN,  // SCK/BCLK
    .ws_io_num    = I2S_WS_PIN,    // WS/LRCLK
    .data_out_num = -1,
    .data_in_num  = I2S_DOUT_PIN   // SD/DO from mic
  };

  if (i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr) != ESP_OK) {
    Serial.println("[KWS] i2s_driver_install failed");
    return false;
  }
  if (i2s_set_pin(I2S_NUM_0, &pins) != ESP_OK) {
    Serial.println("[KWS] i2s_set_pin failed");
    return false;
  }

  // Select mono (LEFT) or stereo (so we can pick RIGHT in software)
  if (MIC_RIGHT_CHANNEL) {
    i2s_set_clk(I2S_NUM_0, KWS_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO);
    Serial.printf("[KWS] I2S stereo @%d Hz (picking RIGHT)\n", KWS_SAMPLE_RATE);
  } else {
    i2s_set_clk(I2S_NUM_0, KWS_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
    Serial.printf("[KWS] I2S mono-left @%d Hz\n", KWS_SAMPLE_RATE);
  }

  i2s_zero_dma_buffer(I2S_NUM_0);
  return true;
}

// Read mic, build MFCC stream, do utterance-level classification
void kws_process_audio() {
  // Pull a small chunk each loop; convert 32-bit I²S sample to float [-1,1]
  int32_t raw[128];
  size_t nbytes = 0;
  if (i2s_read(I2S_NUM_0, (void*)raw, sizeof(raw), &nbytes, 0) != ESP_OK) return;
  int n = nbytes / sizeof(raw[0]);
  if (n <= 0) return;

  const int start  = MIC_RIGHT_CHANNEL ? 1 : 0;   // 0=LEFT (mono or stereo), 1=RIGHT (stereo)
  const int stride = MIC_RIGHT_CHANNEL ? 2 : 1;   // step by 2 if stereo interleaved

  for (int i = start; i < n; i += stride) {
    // 24-bit left-aligned in 32-bit frame (INMP441/ICS-43434 style)
    int32_t s24 = (raw[i] >> 8);                 // sign-extend 24-bit
    float   s   = (float)s24 / 8388607.0f;       // normalize to [-1,1]

    // simple clamp
    if (s > 1.5f) s = 1.5f;
    if (s < -1.5f) s = -1.5f;

    // push into sliding frame buffer with hop logic
    if (hop_accum < KWS_FRAME_LEN) {
      frame_buf[hop_accum] = s;
      hop_accum++;
    }

    if (hop_accum == KWS_FRAME_LEN) {
      // compute VAD on frame
      float rms = 0.0f;
      for (int k = 0; k < KWS_FRAME_LEN; ++k) rms += frame_buf[k]*frame_buf[k];
      rms = sqrtf(rms / KWS_FRAME_LEN);
      vad_rms = 0.95f*vad_rms + 0.05f*rms;

      // Always compute MFCCs for the frame; manage ring & seg_start shifts safely
      float coeffs[KWS_N_MFCC];
      kws_mfcc_frame(frame_buf, coeffs);
      if (mfcc_T < KWS_MAX_FRAMES) {
        for (int k = 0; k < KWS_N_MFCC; ++k) mfcc_seq[mfcc_T][k] = coeffs[k];
        mfcc_T++;
      } else {
        // shift left; keep seg_start_T anchored to new indices
        for (int t = 0; t < KWS_MAX_FRAMES-1; ++t) {
          memcpy(mfcc_seq[t], mfcc_seq[t+1], sizeof(mfcc_seq[t]));
        }
        memcpy(mfcc_seq[KWS_MAX_FRAMES-1], coeffs, sizeof(coeffs));
        if (in_speech && seg_start_T > 0) seg_start_T--;
      }

      // --- Utterance state machine (start/stop on VAD with hysteresis) ---
      uint32_t now = millis();
      if (!in_speech) {
        if (vad_rms >= VAD_ON) {
          in_speech = true;
          seg_start_T = max(0, mfcc_T - 1);  // start near this frame
          silence_since = 0;
        }
      } else {
        if (vad_rms < VAD_OFF) {
          if (silence_since == 0) silence_since = now;
          // if we've been silent long enough → end utterance and classify once
          if (now - silence_since >= SILENCE_HANG_MS) {
            int seg_end_T = mfcc_T; // up to current
            int seg_len   = seg_end_T - seg_start_T;

            float d1=0, d2=0;
            int cls = (seg_len >= MIN_SEG_FRAMES)
                      ? kws_classify_segment(seg_start_T, seg_end_T, &d1, &d2)
                      : -1;

            #if KWS_DEBUG
              Serial.printf("[KWS] SEG end: cls=%d best=%.2f second=%.2f frames=%d\n",
                            cls, d1, d2, seg_len);
            #endif

            if (cls >= 0) {
              // per-class cooldown
              if (now - last_trigger_ms[cls] >= CLASS_COOLDOWN_MS) {
                last_trigger_ms[cls] = now;
                if (cls == KWS_CLS_WAKE) {
                  Serial.println("ready");
                } else if (cls == KWS_CLS_LIGHTS_ON) {
                  applyLight(true);
                } else if (cls == KWS_CLS_LIGHTS_OFF) {
                  applyLight(false);
                }
              }
            }
            in_speech = false;
            silence_since = 0;
          }
        } else {
          // still speaking
          silence_since = 0;
        }
      }

      // slide by hop: move last (FRAME_LEN - HOP_LEN) samples down
      memmove(frame_buf, frame_buf + KWS_HOP_LEN, sizeof(float) * (KWS_FRAME_LEN - KWS_HOP_LEN));
      hop_accum = KWS_FRAME_LEN - KWS_HOP_LEN;
    }
  }
}

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

// Optional creds (leave as nullptr if anonymous)
#ifndef MQTT_USER
  #define MQTT_USER nullptr
  #define MQTT_PASS nullptr
#endif

void ensureMqtt() {
  if (mqtt.connected()) return;

  // Resolve hostname/IP → numeric IP (works for both IP strings & names)
  IPAddress hostIP;
  if (!WiFi.hostByName(MQTT_HOST, hostIP)) {
    Serial.printf("[MQTT] RESOLVE FAIL for '%s'\n", MQTT_HOST);
    delay(1000);
    return;
  }
  Serial.printf("[MQTT] HOST '%s' -> %s:%u\n",
                MQTT_HOST, hostIP.toString().c_str(), MQTT_PORT);

  // Quick TCP probe (firewall / reachability)
  WiFiClient probe;
  Serial.print("[MQTT] TCP CONNECT ... ");
  if (!probe.connect(hostIP, MQTT_PORT)) {
    Serial.println("NO TCP (wrong IP / firewall)");
    delay(1000);
    return;
  }
  Serial.println("OK");
  probe.stop();

  // Actual MQTT connect (auth)
  mqtt.setServer(hostIP, MQTT_PORT);
  mqtt.setSocketTimeout(4);
  mqtt.setKeepAlive(20);

  String clientId = "esp32-env-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  Serial.print("[MQTT] MQTT CONNECT ... ");
  bool ok = mqtt.connect(clientId.c_str(),
                         MQTT_USER, MQTT_PASS,
                         TOPIC_ENV_STATUS, 0, true, "offline");
  if (!ok) {
    Serial.printf("FAIL state=%d\n", mqtt.state()); // -4 timeout, -2 refused, 5 not authorized
    delay(1000);
    return;
  }

  Serial.println("OK");
  mqtt.subscribe(TOPIC_LIGHT_SET);
  mqtt.publish(TOPIC_LIGHT_STATUS, lightOn ? "ON" : "OFF", true);
  mqtt.publish(TOPIC_ENV_STATUS, "online", true);
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

  // ---- KWS bring-up ----
  kws_prepare_dsp();
  if (!kws_init_i2s()) {
    Serial.println("[KWS] I2S init failed (check mic pins). Keyword spotting disabled.");
  } else {
    Serial.println("[KWS] Mic ready @16 kHz, MFCC online.");
  }
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

  // ---- KWS processing ----
  kws_process_audio();
}
