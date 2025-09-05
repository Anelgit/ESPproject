/*
#include <Arduino.h>
#include "driver/i2s.h"

// ===== Pins (change if needed) =====
static const i2s_port_t I2S_PORT = I2S_NUM_0;
static const int PIN_I2S_BCLK   = 26; // SCK
static const int PIN_I2S_LRCLK  = 25; // WS
static const int PIN_I2S_DOUT   = 33; // DO -> data in

// ===== Audio config =====
static const uint32_t SAMPLE_RATE       = 16000;     // voice-friendly
static const size_t   SAMPLES_PER_READ  = 1024;      // DMA chunk

void i2s_install() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,        // 32-bit slots
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,         // read LEFT (SEL->GND)
    .communication_format = I2S_COMM_FORMAT_I2S,         // standard I2S
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = SAMPLES_PER_READ,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  ESP_ERROR_CHECK(i2s_driver_install(I2S_PORT, &cfg, 0, nullptr));

  i2s_pin_config_t pins = {
    .bck_io_num   = PIN_I2S_BCLK,
    .ws_io_num    = PIN_I2S_LRCLK,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = PIN_I2S_DOUT
  };
  ESP_ERROR_CHECK(i2s_set_pin(I2S_PORT, &pins));

  // Clear DMA buffers
  i2s_zero_dma_buffer(I2S_PORT);
  // Lock in exact clocks
  ESP_ERROR_CHECK(i2s_set_clk(I2S_PORT, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO));
}

// Most MEMS I2S mics output 24-bit left-justified in a 32-bit slot.
static inline int32_t s24_to_s32(int32_t x) { return (x >> 8); }

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nI2S mic test (16 kHz, mono/left)…");
  i2s_install();
  Serial.println("Speak/clap: RMS should change.");
}

void loop() {
  static int32_t buf[SAMPLES_PER_READ];
  size_t bytes_read = 0;
  esp_err_t ok = i2s_read(I2S_PORT, (void*)buf, sizeof(buf), &bytes_read, portMAX_DELAY);
  if (ok != ESP_OK || bytes_read == 0) return;

  const size_t n = bytes_read / sizeof(int32_t);

  int64_t acc = 0;
  for (size_t i = 0; i < n; ++i) {
    int32_t s = s24_to_s32(buf[i]);
    acc += (int64_t)s * (int64_t)s;
  }
  float rms = sqrtf((float)acc / (float)n);

  static uint32_t t0 = millis();
  if (millis() - t0 > 100) {         // print ~10 Hz
    Serial.printf("RMS: %.1f\n", rms);
    t0 = millis();
  }
}
  */ 