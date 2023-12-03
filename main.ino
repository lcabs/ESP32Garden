#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <PubSubClient.h>

/* Defines do Display */
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ADDR   0x3C

const char* ssid = "EG8115V5";
const char* password = "alpha147147";
const char* mqtt_server = "broker.hivemq.com";
const char* mqtt_topic1 = "lcabs1993/ESP32/remote";
const char* mqtt_topic2 = "lcabs1993/arduino";

const char* screens[] = {"debug", "weather", "animation", "clock", "sensor", "log"};
int currentScreen = 0;

WiFiClient espClient;
PubSubClient client(espClient);

//Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT);

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Connect to MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);




  // Initialize the OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  delay(2000);
  display.clearDisplay();
  display.display();

  // Initial screen
  drawScreen();
}


void loop() {
  if (!client.connected()) {
    reconnect();
  }

  client.loop();
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Received message on topic: ");
  Serial.println(topic);

  // Convert payload to a string
  payload[length] = '\0'; // Null-terminate the payload
  String payloadStr = String((char*)payload);

  // Check if payload matches any screen names
  for (int i = 0; i < sizeof(screens) / sizeof(screens[0]); i++) {
    if (payloadStr.equals(screens[i])) {
      currentScreen = i;
      Serial.print("Changing to screen: ");
      Serial.println(screens[i]);
      drawScreen();
      break;
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("Connected to MQTT broker");
      client.subscribe(mqtt_topic1);
      client.subscribe(mqtt_topic2);
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

void drawScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  // Draw the screen title
  display.print("Screen: ");
  display.println(screens[currentScreen]);

  // Add additional screen-specific content here

  display.display();
}
