#include <WiFi.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>  // Include the PubSubClient library for MQTT

// --------------------------------------------------------------------------------------------
//        UPDATE CONFIGURATION TO MATCH YOUR ENVIRONMENT
// --------------------------------------------------------------------------------------------

// GPIO pins for devices
#define RED_PIN 22       // GPIO pin connected to the Red channel of the RGB LED
#define GREEN_PIN 23     // GPIO pin connected to the Green channel of the RGB LED
#define BLUE_PIN 21      // GPIO pin connected to the Blue channel of the RGB LED
#define DHT_PIN 4        // GPIO pin the data line of the DHT sensor is connected to

// DHT sensor configuration
#define DHTTYPE DHT11

// Temperature thresholds
#define ALARM_COLD 0.0
#define ALARM_HOT 30.0
#define WARN_COLD 10.0
#define WARN_HOT 25.0

// WiFi credentials
char ssid[] = "Redmi Note 13 Prof";  // Network SSID (name)
char pass[] = "password123";         // Network password

// MQTT connection details
#define MQTT_HOST "192.168.251.131"   // IP address of your Mosquitto server 
#define MQTT_PORT 1883                // Default port for MQTT
#define MQTT_DEVICEID "esp32_888"     // Unique identifier for the ESP32
#define MQTT_USER ""                  // Leave blank for no authentication
#define MQTT_TOKEN ""                 // Leave blank for no authentication

// MQTT topics
#define MQTT_TOPIC "esp32_888/evt/status/fmt/json"        // Topic to publish status updates
#define MQTT_TOPIC_DISPLAY "esp32_888/cmd/display/fmt/json" // Topic to subscribe for commands

// --------------------------------------------------------------------------------------------
//        SHOULD NOT NEED TO CHANGE ANYTHING BELOW THIS LINE
// --------------------------------------------------------------------------------------------
DHT dht(DHT_PIN, DHTTYPE);

// MQTT client setup
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Variables to hold sensor data
StaticJsonDocument<200> jsonDoc;   // Increased JSON document size to accommodate additional data
static char msg[200];              // Adjusted size to fit JSON payload

float h = 0.0; // Humidity
float t = 0.0; // Temperature

// Callback function for incoming MQTT messages
void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0'; // Null-terminate the payload
  String message = String((char*)payload);
  
  // Parse JSON command
  StaticJsonDocument<100> jsonCommand;
  DeserializationError error = deserializeJson(jsonCommand, message);
  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  // Process command
  String command = jsonCommand["command"];
  String value = jsonCommand["value"];
  
  if (command == "set_led") {
    if (value == "red") {
      analogWrite(RED_PIN, 255);
      analogWrite(GREEN_PIN, 0);
      analogWrite(BLUE_PIN, 0);
    } else if (value == "green") {
      analogWrite(RED_PIN, 0);
      analogWrite(GREEN_PIN, 255);
      analogWrite(BLUE_PIN, 0);
    } else if (value == "blue") {
      analogWrite(RED_PIN, 0);
      analogWrite(GREEN_PIN, 0);
      analogWrite(BLUE_PIN, 255);
    }
  }
}

void setup() {
  // Start serial console
  Serial.begin(115200);
  Serial.println();
  Serial.println("ESP32 Sensor Application");

  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi Connected");

  // Initialize DHT sensor
  dht.begin();

  // Set RGB LED pins as outputs
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  // Setup MQTT
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(callback);

  // Attempt to connect to MQTT
  connectToMQTT();
}

// Function to connect to the MQTT broker
void connectToMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect(MQTT_DEVICEID, MQTT_USER, MQTT_TOKEN)) {
      Serial.println("connected");
      mqttClient.subscribe(MQTT_TOPIC_DISPLAY); // Subscribe to the command topic
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" trying again in 5 seconds");
      delay(5000);  // Retry every 5 seconds
    }
  }
}

// Function to set RGB LED color based on temperature thresholds
void setLEDColor(float temperature) {
  unsigned char r, g, b;
  b = (temperature < ALARM_COLD) ? 255 : ((temperature < WARN_COLD) ? 150 : 0);
  r = (temperature >= ALARM_HOT) ? 255 : ((temperature > WARN_HOT) ? 150 : 0);
  g = (temperature > ALARM_COLD) ? ((temperature <= WARN_HOT) ? 255 : ((temperature < ALARM_HOT) ? 150 : 0)) : 0;

  analogWrite(RED_PIN, r);
  analogWrite(GREEN_PIN, g);
  analogWrite(BLUE_PIN, b);
}

void loop() {
  // Maintain the MQTT connection
  if (!mqttClient.connected()) {
    connectToMQTT();
  }
  mqttClient.loop();

  // Read sensor data
  h = dht.readHumidity();
  t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {  
    setLEDColor(t);

    // Populate JSON payload
    jsonDoc.clear();
    jsonDoc["device_id"] = MQTT_DEVICEID;
    jsonDoc["temperature"] = t;
    jsonDoc["humidity"] = h;
    jsonDoc["status"] = (t >= WARN_HOT) ? "warning" : "normal";
    
    serializeJson(jsonDoc, msg);
    Serial.println(msg); // Print JSON data to Serial Monitor

    // Publish JSON payload to MQTT topic
    if (!mqttClient.publish(MQTT_TOPIC, msg)) {
      Serial.println("MQTT Publish failed");
    }
  }

  delay(10000); // Delay between readings
}
