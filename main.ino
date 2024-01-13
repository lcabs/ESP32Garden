#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <time.h>

const char* ssid = "EG8115V5";
const char* password = "alpha147147";
const char* mqtt_server = "192.168.18.186";
const char* ntp_server = "pool.ntp.org";
const char* mqtt_username = "your_mqtt_username";
const char* mqtt_password = "your_mqtt_password";

const int DHT_PIN = 5;
const int LED_PIN = 2;  // GPIO 2 for the LED
const int SOIL_MOISTURE_PIN_1 = 36;
const int SOIL_MOISTURE_PIN_2 = 39;
const int SOIL_MOISTURE_PIN_3 = 34;
const int SOIL_MOISTURE_PIN_4 = 35;
const int RAIN_SENSOR_PIN_1 = 32;
const int RAIN_SENSOR_PIN_2 = 33;
const int RELAY_PIN_1 = 16;
const int RELAY_PIN_2 = 17;
const int RELAY_PIN_3 = 18;
const int RELAY_PIN_4 = 19;

DHT dht(DHT_PIN, DHT11);
WiFiClient espClient;
PubSubClient client(espClient);

int lastSoilMoisture1 = -1;
int lastSoilMoisture2 = -1;
int lastSoilMoisture3 = -1;
int lastSoilMoisture4 = -1;
int lastRainSensor1 = -1;
int lastRainSensor2 = -1;
int lastTemperature = -1;
int lastHumidity = -1;

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Received message on topic: ");
  Serial.println(topic);

  payload[length] = '\0'; // Null-terminate the payload
  String payloadStr = String((char*)payload);

  Serial.println("Payload: " + payloadStr);

  // Blink the LED on GPIO 2
  digitalWrite(LED_PIN, HIGH);
  delay(200);  // Adjust the blink duration as needed
  digitalWrite(LED_PIN, LOW);

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
    Serial.println("Reconnect: Attempting MQTT connection...");

    if (client.connect("ESP32Client", mqtt_username, mqtt_password)) {
      Serial.println("Reconnect: Connected to MQTT broker");

      // Blink the LED rapidly for three cycles
      for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
      }

      struct tm timeinfo;
      getLocalTime(&timeinfo);
      timeinfo.tm_hour -= 3;

      char isoTime[20];
      strftime(isoTime, sizeof(isoTime), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);

      String connectionMessage = "Successfully connected to MQTT broker at " + String(isoTime);
      client.publish("lcabs1993/ESP32", connectionMessage.c_str());

      client.subscribe("lcabs1993/ESP32/rele1");
      client.subscribe("lcabs1993/ESP32/rele2");
      client.subscribe("lcabs1993/ESP32/rele3");
      client.subscribe("lcabs1993/ESP32/rele4");
    } else {
      Serial.print("Reconnect: MQTT connection failed, rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

void publishSensorData(const char* sensorName, const String& sensorValue) {
  String topic = "lcabs1993/ESP32/" + String(sensorName);
  client.publish(topic.c_str(), sensorValue.c_str());
}

void setup() {
  Serial.begin(115200);
  Serial.println("Setup: Starting...");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Setup: Connecting to WiFi...");
  }
  Serial.println("Setup: Connected to WiFi");

  configTime(0, 0, "pool.ntp.org");

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  dht.begin();

  // Initialize Relay pins
  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);
  pinMode(RELAY_PIN_3, OUTPUT);
  pinMode(RELAY_PIN_4, OUTPUT);

  // Initial state: Relays are turned off
  digitalWrite(RELAY_PIN_1, LOW);
  digitalWrite(RELAY_PIN_2, LOW);
  digitalWrite(RELAY_PIN_3, LOW);
  digitalWrite(RELAY_PIN_4, LOW);

  pinMode(LED_PIN, OUTPUT);  // Set LED pin as output

  Serial.println("Setup: Completed.");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (!isnan(temperature) && !isnan(humidity)) {
    publishSensorData("DHT11/temp", String(temperature));
    publishSensorData("DHT11/humi", String(humidity));
  }

  // Read sensor values and publish to specific MQTT topics
  int soilMoisture1 = analogRead(SOIL_MOISTURE_PIN_1);
  int soilMoisture2 = analogRead(SOIL_MOISTURE_PIN_2);
  int soilMoisture3 = analogRead(SOIL_MOISTURE_PIN_3);
  int soilMoisture4 = analogRead(SOIL_MOISTURE_PIN_4);

  publishSensorData("solo1", String(soilMoisture1));
  publishSensorData("solo2", String(soilMoisture2));
  publishSensorData("solo3", String(soilMoisture3));
  publishSensorData("solo4", String(soilMoisture4));

  int rainSensor1 = digitalRead(RAIN_SENSOR_PIN_1);
  int rainSensor2 = digitalRead(RAIN_SENSOR_PIN_2);

  publishSensorData("chuva1", String(rainSensor1));
  publishSensorData("chuva2", String(rainSensor2));

  delay(50);  // Reduced delay to 50 milliseconds
}
