#include <Arduino.h>
#include <BluetoothSerial.h>
#include <driver/ledc.h>

/*
	Tested on Writing digital pin on ESP
	and for reading analog value of a potentiometer
	(other funcs i was lacking equipment)
	libs used:
	BluetoothSerial 3.1.0   
	esp32:esp32   3.1.0   

	on attached fig.jpeg, we connect a led first to 
	32 pin, lighting it up and down while afterwards
	we connected a potentiometer and read some values 
        while modifying the potentiometer knob.
   */

BluetoothSerial SerialBT;
String response;

void setup() {
  Serial.begin(115200);         // USB serial monitor
  SerialBT.begin("ESP32_BT");   // Name seen by Bluetooth Terminal
  Serial.println("Bluetooth started. Waiting for data...");
}

void loop() {
  if (SerialBT.available()) {
    String received = SerialBT.readStringUntil('\n'); // Read until newline
    Serial.print("Received over Bluetooth: ");
    Serial.println(received);
    parseAndExecute(received);
    SerialBT.println(response); // Send response back to the phone
  }
}

void parseAndExecute(String cmd) {
  response = "OK";  // Default response

  if (cmd.startsWith("W")) {
    char type = cmd.charAt(1); // 'D' or 'A'
    int underscore = cmd.indexOf('_');
    int pin = cmd.substring(2, underscore).toInt();
    int val = cmd.substring(underscore + 1).toInt();

    if (type == 'D') {
      pinMode(pin, OUTPUT);
      digitalWrite(pin, val);
      } else if (type == 'A') {
      int pwmChannel = 0;
      int freq = 5000;
      int resolution = 8;
      
      // Configure LEDC timer
      ledc_timer_config_t timer_conf = {
          .speed_mode = LEDC_LOW_SPEED_MODE,
          .duty_resolution = static_cast<ledc_timer_bit_t>(resolution),
          .timer_num = LEDC_TIMER_0,
          .freq_hz = freq,
          .clk_cfg = LEDC_AUTO_CLK
      };
      ledc_timer_config(&timer_conf);
      
      // Configure LEDC channel
      ledc_channel_config_t channel_conf = {
          .gpio_num = pin,
          .speed_mode = LEDC_LOW_SPEED_MODE,
          .channel = static_cast<ledc_channel_t>(pwmChannel),
          .timer_sel = LEDC_TIMER_0,
          .duty = 0,
          .hpoint = 0
      };
      ledc_channel_config(&channel_conf);
      
      // Set duty cucle
      ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(pwmChannel), val);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(pwmChannel));
    }
  } else if (cmd.startsWith("R")) {
    char type = cmd.charAt(1); // 'D' or 'A'
    int pin = cmd.substring(2).toInt();

    if (type == 'D') {
      pinMode(pin, INPUT);
      response = String(digitalRead(pin));
    } else if (type == 'A') {
      pinMode(pin, INPUT);
      response = String(analogRead(pin));
    }
  } else {
    response = "Invalid command";
  }
}
