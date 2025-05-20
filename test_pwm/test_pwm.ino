#include <Arduino.h>
#include <driver/ledc.h>

const int ledPin = 18;
const int freq = 5000;
const int resolution = 8;  // 8-bit: 0–255
const int pwmChannel = 0;

void setup() {
  Serial.begin(115200);

  // Timer config
  ledc_timer_config_t timer_config = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,
    .duty_resolution  = LEDC_TIMER_8_BIT,
    .timer_num        = LEDC_TIMER_0,
    .freq_hz          = freq,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer_config);

  // Channel config
  ledc_channel_config_t channel_config = {
    .gpio_num       = ledPin,
    .speed_mode     = LEDC_LOW_SPEED_MODE,
    .channel        = LEDC_CHANNEL_0,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = LEDC_TIMER_0,
    .duty           = 0,
    .hpoint         = 0
  };
  ledc_channel_config(&channel_config);
}

void loop() {
  // Fade up
  for (int duty = 0; duty <= 255; duty += 5) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    delay(20);
  }

  // Fade down
  for (int duty = 255; duty >= 0; duty -= 5) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    delay(20);
  }
}

