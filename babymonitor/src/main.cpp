#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <driver/i2s.h>

#define LED 4
#define BCK_IO 7
#define WS_IO 5
#define DOUT_IO -1
#define DIN_IO 17
#define ESP_NOW_MAX_DATA_LEN 250

i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 11025,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, 
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = ESP_NOW_MAX_DATA_LEN*4,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0,
};

i2s_pin_config_t pin_config = {
    .bck_io_num = BCK_IO,   // Bit Clock.
    .ws_io_num = WS_IO,    // Word Select aka left/right clock aka LRCL.
    .data_out_num = DOUT_IO,
    .data_in_num = DIN_IO,  // Data-out of the mic. (someone used 23 on forums).
};

void setup() {
  Serial.begin(115200);

  pinMode(LED, OUTPUT);

  digitalWrite(LED, HIGH);
  // put your setup code here, to run once:
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

void loop() {
  // put your main code here, to run repeatedly:
  size_t bytesRead = 0;
  uint8_t buffer32[ESP_NOW_MAX_DATA_LEN * 4] = {0};
  i2s_read(I2S_NUM_0, &buffer32, sizeof(buffer32), &bytesRead, 100);
  int samplesRead = bytesRead / 4;

  int16_t buffer16[ESP_NOW_MAX_DATA_LEN] = {0};
  for (int i=0; i<samplesRead; i++) {
    uint8_t mid = buffer32[i * 4 + 2];
    uint8_t msb = buffer32[i * 4 + 3];
    uint16_t raw = (((uint32_t)msb) << 8) + ((uint32_t)mid);
    memcpy(&buffer16[i], &raw, sizeof(raw)); // Copy so sign bits aren't interfered with somehow.
    Serial.printf("%d\n", raw);
  }

}