
#include <Arduino.h>
#include <driver/ledc.h>
#include <Arduino.h>

// Define the pin and PWM channel
const int piezoPin = 25;    // Any digital GPIO pin
const int pwmChannel = 0;    // PWM channel (0-15)
const int resolution = 8;    // PWM resolution (8-bit = 0-255)

void setup() {
  Serial.begin(115200);  // Initialize serial communication

  // Initial frequency (start low and increment gradually)
  int freq = 500;  // Start at 500 Hz for a manageable base frequency

  // Configure the LEDC timer
  ledc_timer_config_t timer_conf = {
      .speed_mode = LEDC_LOW_SPEED_MODE,    // Low speed mode
      .duty_resolution = static_cast<ledc_timer_bit_t>(resolution),  // Set resolution (8-bit)
      .timer_num = LEDC_TIMER_0,            // Timer 0
      .freq_hz = freq,                      // Set initial frequency (500 Hz)
      .clk_cfg = LEDC_AUTO_CLK              // Use automatic clock
  };
  ledc_timer_config(&timer_conf);

  // Configure the LEDC channel
  ledc_channel_config_t channel_conf = {
      .gpio_num = piezoPin,                 // GPIO pin for the piezo buzzer
      .speed_mode = LEDC_LOW_SPEED_MODE,    // Low speed mode
      .channel = static_cast<ledc_channel_t>(pwmChannel),  // Channel number (0-15)
      .timer_sel = LEDC_TIMER_0,            // Use timer 0
      .duty = 127,                          // Set duty cycle to 50% (127 out of 255)
      .hpoint = 0                           // No phase shift
  };
  ledc_channel_config(&channel_conf);

  // Start playing the tone (50% duty cycle)
  ledcWrite(pwmChannel, 127);  // Set duty cycle to 50% (127/255)

  // Print initial frequency
  Serial.print("Starting frequency: ");
  Serial.print(freq);
  Serial.println(" Hz");
}

void loop() {
  // Sweep frequency from 500 Hz to 50 kHz in small increments (e.g., 100 Hz)
  for (int freq = 500; freq <= 1000000; freq += 100) {
    // Reconfigure the timer with the new frequency
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,   
        .duty_resolution = static_cast<ledc_timer_bit_t>(resolution),
        .timer_num = LEDC_TIMER_0,
        .freq_hz = freq,                    
        .clk_cfg = LEDC_AUTO_CLK
    };

    // If the timer config is successful, apply the changes
    if (ledc_timer_config(&timer_conf) == ESP_OK) {
      // Adjust the duty cycle if needed (keeping 50% for simplicity)
      ledcWrite(pwmChannel, 127); // 50% duty cycle

      // Print the current frequency being transmitted
      Serial.print("Transmitting frequency: ");
      Serial.print(freq);
      Serial.println(" Hz");
      delay(100);                    // Delay for 100 ms before changing frequency
    }
  }

  delay(1000);  // Pause for 1 second before repeating the sweep
}

