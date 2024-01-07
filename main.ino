#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <time.h> // Include the time library

const char* ssid = "EG8115V5";
const char* password = "alpha147147";
const char* mqtt_server = "192.168.18.186";
const char* ntp_server = "pool.ntp.org"; // NTP server to get the time
const char* mqtt_username = "your_mqtt_username"; // Replace with your MQTT username
const char* mqtt_password = "your_mqtt_password"; // Replace with your MQTT password

const int DHT_PIN = 5;  // DHT11 sensor on GPIO4
const int SOIL_MOISTURE_PIN_1 = 36;
const int SOIL_MOISTURE_PIN_2 = 39;
const int SOIL_MOISTURE_PIN_3 = 34; // Additional soil moisture sensor
const int SOIL_MOISTURE_PIN_4 = 35; // Additional soil moisture sensor
const int RAIN_SENSOR_PIN_1 = 32;
const int RAIN_SENSOR_PIN_2 = 33; // Additional rain sensor
const int RELAY_PIN_1 = 16; // GPIO pin for Relay 1
const int RELAY_PIN_2 = 17; // GPIO pin for Relay 2
const int RELAY_PIN_3 = 18; // Additional GPIO pin for Relay 3
const int RELAY_PIN_4 = 19; // Additional GPIO pin for Relay 4

DHT dht(DHT_PIN, DHT11);
PubSubClient client;

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Set the NTP server
  configTime(0, 0, ntp_server);

  // Connect to MQTT with username and password
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // Initialize sensors
  dht.begin();

  // Initialize Relay pins
  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);
  pinMode(RELAY_PIN_3, OUTPUT); // Additional GPIO pin for Relay 3
  pinMode(RELAY_PIN_4, OUTPUT); // Additional GPIO pin for Relay 4

  // Initial state: Relays are turned off
  digitalWrite(RELAY_PIN_1, LOW);
  digitalWrite(RELAY_PIN_2, LOW);
  digitalWrite(RELAY_PIN_3, LOW); // Initial state for Relay 3
  digitalWrite(RELAY_PIN_4, LOW); // Initial state for Relay 4
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Read sensor values and publish to specific MQTT topics
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (!isnan(temperature) && !isnan(humidity)) {
    publishSensorData("DHT11/temp", String(temperature));
    publishSensorData("DHT11/humi", String(humidity));
  }

  int soilMoisture1 = analogRead(SOIL_MOISTURE_PIN_1);
  int soilMoisture2 = analogRead(SOIL_MOISTURE_PIN_2);
  int soilMoisture3 = analogRead(SOIL_MOISTURE_PIN_3); // Additional soil moisture sensor
  int soilMoisture4 = analogRead(SOIL_MOISTURE_PIN_4); // Additional soil moisture sensor

  publishSensorData("solo1", String(soilMoisture1));
  publishSensorData("solo2", String(soilMoisture2));
  publishSensorData("solo3", String(soilMoisture3)); // Publish data for Additional soil moisture sensor
  publishSensorData("solo4", String(soilMoisture4)); // Publish data for Additional soil moisture sensor

  int rainSensor1 = digitalRead(RAIN_SENSOR_PIN_1);
  int rainSensor2 = digitalRead(RAIN_SENSOR_PIN_2);

  publishSensorData("chuva1", String(rainSensor1));
  publishSensorData("chuva2", String(rainSensor2));

  delay(5000); // Adjust the delay based on your application requirements
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Received message on topic: ");
  Serial.println(topic);

  payload[length] = '\0'; // Null-terminate the payload
  String payloadStr = String((char*)payload);

  Serial.println("Payload: " + payloadStr);

  // Check if the payload corresponds to controlling relays
  if (String(topic) == "lcabs1993/ESP32/rele") {
    // Control Relay 1
    if (payloadStr == "on") {
      digitalWrite(RELAY_PIN_1, HIGH);
    } else if (payloadStr == "off") {
      digitalWrite(RELAY_PIN_1, LOW);
    }
  } else if (String(topic) == "lcabs1993/ESP32/rele2") {
    // Control Relay 2
    if (payloadStr == "on") {
      digitalWrite(RELAY_PIN_2, HIGH);
    } else if (payloadStr == "off") {
      digitalWrite(RELAY_PIN_2, LOW);
    }
  } else if (String(topic) == "lcabs1993/ESP32/control/rele3") {
    // Control Relay 3
    if (payloadStr == "on") {
      digitalWrite(RELAY_PIN_3, HIGH);
    } else if (payloadStr == "off") {
      digitalWrite(RELAY_PIN_3, LOW);
    }
  } else if (String(topic) == "lcabs1993/ESP32/rele4") {
    // Control Relay 4
    if (payloadStr == "on") {
      digitalWrite(RELAY_PIN_4, HIGH);
    } else if (payloadStr == "off") {
      digitalWrite(RELAY_PIN_4, LOW);
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");

    if (client.connect("ESP32Client", mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT broker");

      // Get current time and adjust for GMT-3
      struct tm timeinfo;
      getLocalTime(&timeinfo);
      timeinfo.tm_hour -= 3; // Adjust for GMT-3

      char isoTime[20];
      strftime(isoTime, sizeof(isoTime), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);

      String connectionMessage = "Successfully connected to MQTT broker at " + String(isoTime);
      client.publish("lcabs1993/ESP32", connectionMessage.c_str());

      // Subscribe to control topics
      client.subscribe("lcabs1993/ESP32/rele1");
      client.subscribe("lcabs1993/ESP32/rele2");
      client.subscribe("lcabs1993/ESP32/rele3");
      client.subscribe("lcabs1993/ESP32/rele4");
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

void publishSensorData(const char* sensorName, const String& sensorValue) {
  String topic = "lcabs1993/ESP32/" + String(sensorName);
  client.publish(topic.c_str(), sensorValue.c_str());
}
