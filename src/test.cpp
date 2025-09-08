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

static inline int32_t s24_to_s32(int32_t x) { return (x >> 8); }

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
