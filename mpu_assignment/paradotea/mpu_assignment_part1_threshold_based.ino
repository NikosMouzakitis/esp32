#include "mpu9250.h"
#include <WiFi.h>
#include <Arduino.h>
#include <PubSubClient.h>
#include <driver/ledc.h>


bool receivedInitialState = false;

// WiFi credentials
const char* ssid = "Test_net";
const char* password = "123456789";

// MQTT server
const char* mqtt_server = "192.168.88.247";
int mqtt_port = 1883;

// LED pins
const int led1Pin = 15;
const int led2Pin = 18;

// PWM settings
const int freq = 5000;
ledc_channel_t pwmChannel1 = LEDC_CHANNEL_0;
ledc_channel_t pwmChannel2 = LEDC_CHANNEL_1;
const int resolution = 8;

int pwmLevel = 128;  // Start at medium brightness

WiFiClient espClient;
PubSubClient client(espClient);
bfs::Mpu9250 imu;

// LED toggle states
bool led1State = false;
bool led2State = false;

// Edge detection flags
bool prevButton1State = false;
bool prevButton2State = false;

void setup() {
	Serial.begin(115200);
	Serial.println("operating");

	// LEDC timer config
	ledc_timer_config_t timer_conf = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.duty_resolution = LEDC_TIMER_8_BIT,
		.timer_num = LEDC_TIMER_0,
		.freq_hz = freq,
		.clk_cfg = LEDC_AUTO_CLK
	};
	ledc_timer_config(&timer_conf);

	// LED1 config
	ledc_channel_config_t channel_conf1 = {
		.gpio_num = led1Pin,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = LEDC_CHANNEL_0,
		.intr_type = LEDC_INTR_DISABLE,
		.timer_sel = LEDC_TIMER_0,
		.duty = 0,
		.hpoint = 0
	};
	ledc_channel_config(&channel_conf1);

	// LED2 config
	ledc_channel_config_t channel_conf2 = {
		.gpio_num = led2Pin,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = LEDC_CHANNEL_1,
		.intr_type = LEDC_INTR_DISABLE,
		.timer_sel = LEDC_TIMER_0,
		.duty = 0,
		.hpoint = 0
	};
	ledc_channel_config(&channel_conf2);

	// WiFi connect
	WiFi.begin(ssid, password);
	Serial.print("Connecting to wifi.");
	while (WiFi.status() != WL_CONNECTED) {
		Serial.print(".");
		delay(500);
	}
	Serial.print("\nWiFi connected - IP address: ");
	Serial.println(WiFi.localIP());
	delay(500);

	client.setServer(mqtt_server, 1883);
	client.setCallback(mqttCallback);

	Serial.println("connecting to MPU...");
	delay(1000);
	Wire.begin();
	Wire.setClock(400000);
	imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);

	if (!imu.Begin()) {
		Serial.println("IMU init failed");
		while (1) {}
	}
	if (!imu.ConfigSrd(19)) {
		Serial.println("SRD config failed");
		while (1) {}
	}


	//getting latest state.
	client.subscribe("esp32/restore_state");
	client.publish("esp32/restore_request", "get_latest");



// Connect to MQTT and wait until connected
	while (!client.connected()) {
		Serial.println("Connecting to MQTT...");
		if (client.connect("ESP32 client")) {
			Serial.println("MQTT connected");

			// Give time to establish connection
			delay(200);
			client.loop();

			// Subscribe first
			client.subscribe("esp32/restore_state");
			delay(100);

			// THEN send restore request
			client.publish("esp32/restore_request", "get_latest");
			Serial.println("Sent restore request");

			// Wait briefly to receive the response
			unsigned long startTime = millis();
			while (!receivedInitialState && millis() - startTime < 3000) {
				client.loop(); // Needed to process incoming message
				delay(10);
			}

			if (receivedInitialState) {
				Serial.println("Initial state restored successfully.");
			} else {
				Serial.println("Warning: Did not receive restore state in time.");
			}

		} else {
			Serial.print("MQTT connect failed, rc=");
			Serial.print(client.state());
			Serial.println(" trying again in 5 seconds");
			delay(5000);
		}
	}



}

void loop() {
	if (!client.connected()) {
		reconnect();
	}
	client.loop();

	if (imu.Read()) {
		Serial.print("\nAccX: ");
		float ax=imu.accel_x_mps2();
		Serial.print(ax);
		Serial.println(" ");
		Serial.print("AccY: ");
		float ay=imu.accel_y_mps2();
		Serial.print(ay);
		Serial.println(" ");
		Serial.print("AccZ: ");
		float az = imu.accel_z_mps2();
		Serial.print(az);
		Serial.println(" ");
		String accData = "ax:" + String(ax) + " ay:" + String(ay) + " az:" + String(az);
		Serial.println(accData);
		Serial.print("GyroX: ");
		float gx = imu.gyro_x_radps();
		Serial.print(gx);
		Serial.println(" ");
		Serial.print("GyroY: ");
		float gy = imu.gyro_y_radps();
		Serial.print(gy);
		Serial.println(" ");
		Serial.print("GyroZ: ");
		float gz=imu.gyro_z_radps();
		Serial.print(gz);
		Serial.println(" ");
		String gyroData = "gx:" + String(gx) + " gy:" + String(gy) + " gz:" + String(gz);
		Serial.println(gyroData);

		float mx, my, mz;
		mx = imu.mag_x_ut();
		my = imu.mag_y_ut();
		mz = imu.mag_z_ut();
		Serial.print("mx: ");
		Serial.print(mx);
		Serial.print(" my: ");
		Serial.print(my);
		Serial.print(" mz: ");
		Serial.print(mz);


		// Publish data
		client.publish("esp32/acc", accData.c_str());
		client.publish("esp32/gyro", gyroData.c_str());

		// Button logic for LED 1 (ay > 0.5)
		bool button1Pressed = (ay > 0.5);
		if (button1Pressed && !prevButton1State) {
			led1State = !led1State;
			Serial.println(led1State ? "LED1 toggled ON" : "LED1 toggled OFF");
		}
		prevButton1State = button1Pressed;

		// Button logic for LED 2 (ay < -10)
		bool button2Pressed = (ay < -10);
		if (button2Pressed && !prevButton2State) {
			led2State = !led2State;
			Serial.println(led2State ? "LED2 toggled ON" : "LED2 toggled OFF");
		}
		prevButton2State = button2Pressed;

		// PWM control via mx
		if (mx > 60) {
			pwmLevel = min(255, pwmLevel + 10);
		} else if (mx < 30) {
			pwmLevel = max(0, pwmLevel - 10);
		}
		Serial.print("PWM level now: ");
		Serial.println(pwmLevel);

		// LED1 control
		ledc_set_duty(LEDC_LOW_SPEED_MODE, pwmChannel1, led1State ? pwmLevel : 0);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, pwmChannel1);

		// LED2 control
		ledc_set_duty(LEDC_LOW_SPEED_MODE, pwmChannel2, led2State ? pwmLevel : 0);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, pwmChannel2);

		Serial.printf("ay=%.2f, gy=%.2f, LED1=%s, LED2=%s, PWM=%d\n", ay, gy,
		              led1State ? "ON" : "OFF", led2State ? "ON" : "OFF", pwmLevel);


		// Publish combined status message
		String statusMessage = "LED1:" + String(led1State ? "ON" : "OFF") +
		                       " LED2:" + String(led2State ? "ON" : "OFF") +
		                       " PWM:" + String(pwmLevel);
		client.publish("esp32/status", statusMessage.c_str());
		Serial.println("Published status: " + statusMessage);


	}

	delay(1000);
}

void reconnect() {
	while (!client.connected()) {
		Serial.println("Attempting MQTT connection...");
		if (client.connect("ESP32 client")) {
			Serial.println("Connected");
		} else {
			Serial.print(client.state());
			Serial.println(" Failed - retrying in 5 seconds");
			delay(5000);
		}
	}
}
//function that gives us the latest stored values of our last run from the influx DB.
void mqttCallback(char* topic, byte* payload, unsigned int length) {
	String incoming = "";
	for (unsigned int i = 0; i < length; i++) {
		incoming += (char)payload[i];
	}
	Serial.println("Received message on topic: " + String(topic));
	Serial.println("Payload: " + incoming);

	if (String(topic) == "esp32/restore_state") {
		Serial.println("RECEIVED RESTORE MESSAGE");
		int pwm;
		char led1[4], led2[4];
		if (sscanf(incoming.c_str(), "LED1:%3s LED2:%3s PWM:%d", led1, led2, &pwm) == 3) {
			led1State = (String(led1) == "ON");
			led2State = (String(led2) == "ON");
			pwmLevel = constrain(pwm, 0, 255);

			Serial.println("Restored state from InfluxDB");
			receivedInitialState = true;
		}
	}
}

