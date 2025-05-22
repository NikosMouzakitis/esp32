#include "mpu9250.h"
#include <WiFi.h>
#include <Arduino.h>
#include <PubSubClient.h>
#include <driver/ledc.h>
#include <math.h>
#include "CircularBuffer.h"
#include "LEDClassifier.h"
#include "PWMClassifier.h"


Eloquent::ML::Port::LEDSVM classifier;
Eloquent::ML::Port::PWMSVM PWMclassifier;

CircularBuffer featureBuffer(20);
CircularBuffer PWMfeatureBuffer(9);

bool receivedInitialState = false;
//led related classification.
const float led1_ref[] = {1.2644f, -0.1826f, 1.2624f, -0.2025f, 1.2611f, -0.4283f, 1.2824f, -0.0402f, 1.2319f, 0.1384f, 1.2348f, 0.0322f, 1.1921f, 0.5612f, 1.1313f, 0.1763f, 1.0968f, 0.5997f, 1.0932f, 0.0203f};
const float led2_ref[] = {-1.2262f, -0.0656f, -1.2462f, 0.0797f, -1.2499f, 0.1122f, -1.2516f, 0.1303f, -1.2603f, -0.2161f, -1.2802f, -0.0111f, -1.3104f, -0.2993f, -1.2949f, -0.1044f, -1.2933f, -0.1299f, -1.3144f, -0.1743f};
const float neutral_ref[] = {-0.0283f, 0.1839f, -0.0120f, 0.0909f, -0.0083f, 0.2342f, -0.0229f, -0.0667f, 0.0211f, 0.0575f, 0.0336f, -0.0156f, 0.0876f, -0.1940f, 0.1212f, -0.0533f, 0.1456f, -0.3480f, 0.1638f, 0.1141f};
// StandardScaler parameters
const float scaler_mean[] = {-3.2084f, 0.0766f, -3.3810f, 0.0975f, -3.4918f, 0.0891f, -3.3807f, 0.1169f, -3.6294f, 0.1043f, -3.8146f, 0.1236f, -4.1245f, 0.2030f, -4.2913f, 0.1094f, -4.3518f, 0.1715f, -4.4175f, 0.1130f};
const float scaler_scale[] = {6.7734f, 0.1838f, 7.1092f, 0.1949f, 7.0669f, 0.1462f, 7.0625f, 0.1584f, 7.3566f, 0.1566f, 7.2327f, 0.2770f, 6.9194f, 0.4276f, 6.8032f, 0.2529f, 6.7617f, 0.2310f, 6.5323f, 0.1716f};


//pwm related 9 feature buffer.
const float inc_ref[] = {0.7232f, 0.8878f, 0.9831f, 1.1029f, 1.1480f, 1.1479f, 1.1779f, 1.2011f, 1.2346f};
const float dec_ref[] = {-0.7754f, -0.8304f, -0.9497f, -1.0953f, -1.0931f, -1.0988f, -1.1310f, -1.1579f, -1.2060f};
const float pwm_neutral_ref[] = {0.2352f, 0.1621f, 0.2109f, 0.2678f, 0.2298f, 0.2359f, 0.2456f, 0.2552f, 0.2789f};
// StandardScaler parameters for PWM.
const float pwm_scaler_mean[] = {52.8201f, 52.9796f, 51.9225f, 50.1350f, 50.7688f, 50.5416f, 50.1524f, 49.7871f, 48.9675f};
const float pwm_scaler_scale[] = {10.9014f, 13.7405f, 15.9248f, 18.5846f, 19.4744f, 20.0321f, 21.0898f, 21.9032f, 22.8510f};



//function to scale on esp32.
void scaleFeatures(float* features) {
	for (int i = 0; i < 20; i++) {
		features[i] = (features[i] - scaler_mean[i]) / scaler_scale[i];
	}
}

//function to scale on esp32.
void PWMscaleFeatures(float* features) {
	for (int i = 0; i < 9; i++) {
		features[i] = (features[i] - pwm_scaler_mean[i]) / pwm_scaler_scale[i];
	}
}



unsigned long lastLed1,lastLed2;

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


//	client.setServer(mqtt_server, mqtt_port);
//	client.setCallback(mqttCallback);

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
	//invoke timer keeping.
	lastLed1 = millis();
	lastLed2 = millis();
}

void loop() {
	if (!client.connected()) {
		reconnect();
	}
	client.loop();


	if (imu.Read()) {
//		Serial.print("\nAccX: ");
		float ax=imu.accel_x_mps2();
//		Serial.print(ax);
//		Serial.println(" ");
//		Serial.print("AccY: ");
		float ay=imu.accel_y_mps2();
//		Serial.print(ay);
//		Serial.println(" ");
//		Serial.print("AccZ: ");
		float az = imu.accel_z_mps2();
//		Serial.print(az);
//		Serial.println(" ");
		String accData = "ax:" + String(ax) + " ay:" + String(ay) + " az:" + String(az);
//		Serial.println(accData);
//		Serial.print("GyroX: ");
		float gx = imu.gyro_x_radps();
		//	Serial.print(gx);	Serial.println(" ");	Serial.print("GyroY: ");
		float gy = imu.gyro_y_radps();
//		Serial.print(gy);
//		Serial.println(" ");
//		Serial.print("GyroZ: ");
		float gz=imu.gyro_z_radps();
//		Serial.print(gz);
//		Serial.println(" ");
		String gyroData = "gx:" + String(gx) + " gy:" + String(gy) + " gz:" + String(gz);
//		Serial.println(gyroData);

		float mx, my, mz;
		mx = imu.mag_x_ut();
		my = imu.mag_y_ut();
		mz = imu.mag_z_ut();
//		Serial.print("mx: "); Serial.print(mx); Serial.print(" my: ");	Serial.print(my);	Serial.print(" mz: ");	Serial.print(mz);


		// Publish data
		client.publish("esp32/acc", accData.c_str());
		client.publish("esp32/gyro", gyroData.c_str());


		//machine learning related.
		// Add new data
		featureBuffer.add(ay);
		featureBuffer.add(gy);
		PWMfeatureBuffer.add(mx);

		//classification should be done now.
		if (featureBuffer.getSize() == 20) {
			float features[20];
			featureBuffer.getFeatures(features);

			// 1. Scale the features
			scaleFeatures(features);

			// 2. Check distances (using scaled features)
			float dist_led1 = euclideanDistance(features, led1_ref, 20);
			float dist_led2 = euclideanDistance(features, led2_ref, 20);
			float dist_neutral = euclideanDistance(features, neutral_ref, 20);
			Serial.println("distances Euclid");
			Serial.println(dist_led1);	
			Serial.println(dist_led2);	
			Serial.println(dist_neutral);	
			// 3. Make decision
			if (dist_neutral < 2.0f) {  // Neutral threshold
				Serial.println("NEUTRAL by (distance)");
			} else if( (dist_led1 < 5) || (dist_led2 < 5) || (dist_neutral < 5) ) {
				// Use classifier
				String prediction = classifier.predictLabel(features);
				float confidence = classifier.predict(features);
				unsigned long curr_time = millis();
				if (confidence > 0.75) {
					Serial.println("Classifier: " + prediction);
					if (prediction == "NEUTRAL") {
						char dump_op = 'x';	
					} else if (prediction == "LED1_TOGGLE") {
						if(curr_time-lastLed1 > 900)
						{
							led1State = !led1State; // Toggle state
							Serial.printf("LED1 toggled to %s with (Confidence: %.1f%%)\n", led1State ? "ON" : "OFF", confidence*100);
							Serial.println("");
							lastLed1 = curr_time;
						}
					} else if (prediction == "LED2_TOGGLE") {
						if(curr_time - lastLed2  >900) {
							led2State = !led2State; // Toggle state
							Serial.printf("LED2 toggled to %s with (Confidence: %.1f%%)\n", led2State ? "ON" : "OFF", confidence*100);
							Serial.println("");
							lastLed2 = curr_time;
						}
					}
				} else {
					// Fallback to distances
					String prediction = classifier.predictLabel(features);
					Serial.println("Low confidence, using distance: " + prediction);
					
					if (prediction == "NEUTRAL") {
						char dump_op = 'x';	
					} else if (prediction == "LED1_TOGGLE") {
						Serial.println(curr_time - lastLed1);		
						if(curr_time - lastLed1 > 900) {
							led1State = !led1State; // Toggle state
							Serial.printf("LED1 toggled to %s\n", led1State ? "ON" : "OFF");
							Serial.println("");
							lastLed1 = curr_time;
						}
					} else if (prediction == "LED2_TOGGLE") {
						if(curr_time - lastLed2 > 900) {
							led2State = !led2State; // Toggle state
							Serial.printf("LED2 toggled to %s \n", led2State ? "ON" : "OFF");
							Serial.println("");
							lastLed2 = curr_time;
						}
					}
				}
			}
		}


		/*
		// PWM control via mx
		if (mx > 60) {
			pwmLevel = min(255, pwmLevel + 10);
		} else if (mx < 30) {
			pwmLevel = max(0, pwmLevel - 10);
		}
		*/
		//pwm via machine learning.
		
		if(PWMfeatureBuffer.getSize() == 9) {
			float pwmfeatures[9];
			PWMfeatureBuffer.getFeatures(pwmfeatures);
			PWMscaleFeatures(pwmfeatures);	

			float dist_inc = euclideanDistance(pwmfeatures,inc_ref,9);
			float dist_dec = euclideanDistance(pwmfeatures,dec_ref,9);
			float dist_norm = euclideanDistance(pwmfeatures,pwm_neutral_ref,9);

			Serial.println("distances Euclid");
			Serial.println(dist_inc);	
			Serial.println(dist_dec);	
			Serial.println(dist_norm);

			String prediction = PWMclassifier.predictLabel(pwmfeatures);
			Serial.println("PWM prediction");
			Serial.println(prediction);
			Serial.println("");
		}



			// LED1 control
		ledc_set_duty(LEDC_LOW_SPEED_MODE, pwmChannel1, led1State ? pwmLevel : 0);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, pwmChannel1);

		// LED2 control
		ledc_set_duty(LEDC_LOW_SPEED_MODE, pwmChannel2, led2State ? pwmLevel : 0);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, pwmChannel2);

		//Serial.printf("ay=%.2f, gy=%.2f, LED1=%s, LED2=%s, PWM=%d\n", ay, gy,led1State ? "ON" : "OFF", led2State ? "ON" : "OFF", pwmLevel);


		// Publish combined status message
		String statusMessage = "LED1:" + String(led1State ? "ON" : "OFF") +
		                       " LED2:" + String(led2State ? "ON" : "OFF") +
		                       " PWM:" + String(pwmLevel);
		client.publish("esp32/status", statusMessage.c_str());
		//	Serial.println("Published status: " + statusMessage);


	}

	delay(30);
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


float cosineSimilarity20(const float* a, const float* b) {
	float dot_product = 0.0f;
	float mag_a = 0.0f;
	float mag_b = 0.0f;

	// Unrolled loop for better performance on ESP32
	dot_product += a[0] * b[0];
	mag_a += a[0] * a[0];
	mag_b += b[0] * b[0];
	dot_product += a[1] * b[1];
	mag_a += a[1] * a[1];
	mag_b += b[1] * b[1];
	dot_product += a[2] * b[2];
	mag_a += a[2] * a[2];
	mag_b += b[2] * b[2];
	dot_product += a[3] * b[3];
	mag_a += a[3] * a[3];
	mag_b += b[3] * b[3];
	dot_product += a[4] * b[4];
	mag_a += a[4] * a[4];
	mag_b += b[4] * b[4];
	dot_product += a[5] * b[5];
	mag_a += a[5] * a[5];
	mag_b += b[5] * b[5];
	dot_product += a[6] * b[6];
	mag_a += a[6] * a[6];
	mag_b += b[6] * b[6];
	dot_product += a[7] * b[7];
	mag_a += a[7] * a[7];
	mag_b += b[7] * b[7];
	dot_product += a[8] * b[8];
	mag_a += a[8] * a[8];
	mag_b += b[8] * b[8];
	dot_product += a[9] * b[9];
	mag_a += a[9] * a[9];
	mag_b += b[9] * b[9];
	dot_product += a[10] * b[10];
	mag_a += a[10] * a[10];
	mag_b += b[10] * b[10];
	dot_product += a[11] * b[11];
	mag_a += a[11] * a[11];
	mag_b += b[11] * b[11];
	dot_product += a[12] * b[12];
	mag_a += a[12] * a[12];
	mag_b += b[12] * b[12];
	dot_product += a[13] * b[13];
	mag_a += a[13] * a[13];
	mag_b += b[13] * b[13];
	dot_product += a[14] * b[14];
	mag_a += a[14] * a[14];
	mag_b += b[14] * b[14];
	dot_product += a[15] * b[15];
	mag_a += a[15] * a[15];
	mag_b += b[15] * b[15];
	dot_product += a[16] * b[16];
	mag_a += a[16] * a[16];
	mag_b += b[16] * b[16];
	dot_product += a[17] * b[17];
	mag_a += a[17] * a[17];
	mag_b += b[17] * b[17];
	dot_product += a[18] * b[18];
	mag_a += a[18] * a[18];
	mag_b += b[18] * b[18];
	dot_product += a[19] * b[19];
	mag_a += a[19] * a[19];
	mag_b += b[19] * b[19];

	// Calculate magnitudes
	mag_a = sqrtf(mag_a);
	mag_b = sqrtf(mag_b);

	// Avoid division by zero
	if (mag_a == 0.0f || mag_b == 0.0f) {
		return 0.0f;
	}

	return dot_product / (mag_a * mag_b);
}

// Replace your cosineSimilarity20 function with this enhanced version:
void printSimilarityDiagnostics(const float* sample, const float* reference, const char* className) {
	Serial.print("\nDetailed similarity for ");
	Serial.println(className);

	float dot_product = 0.0f;
	float mag_sample = 0.0f;
	float mag_ref = 0.0f;

	Serial.println("Index | Sample | Reference | Product");
	Serial.println("-----------------------------------");

	for (int i = 0; i < 20; i++) {
		float product = sample[i] * reference[i];
		dot_product += product;
		mag_sample += sample[i] * sample[i];
		mag_ref += reference[i] * reference[i];

		Serial.printf("%2d | %7.3f | %7.3f | %7.3f\n",
		              i, sample[i], reference[i], product);
		Serial.println();
	}

	mag_sample = sqrtf(mag_sample);
	mag_ref = sqrtf(mag_ref);

	float similarity = (mag_sample == 0.0f || mag_ref == 0.0f)
	                   ? 0.0f
	                   : (dot_product / (mag_sample * mag_ref));

	Serial.println("-----------------------------------");
	Serial.printf("Dot product: %.4f\n", dot_product);
	Serial.println();
	Serial.printf("Magnitude sample: %.4f\n", mag_sample);
	Serial.println();
	Serial.printf("Magnitude reference: %.4f\n", mag_ref);
	Serial.println();
	Serial.printf("Final similarity: %.4f\n", similarity);
	Serial.println();
}


float euclideanDistance(const float* a, const float* b, int size) {
	float sum = 0.0f;
	for (int i = 0; i < size; i++) {
		sum += powf(a[i] - b[i], 2);
	}
	return sqrtf(sum);
}
