#include <WiFi.h>
#include "esp_wpa2.h" //wpa2 library for connections to Enterprise networks
#include "ESP32MQTTClient.h"
#include <ArduinoJson.h>

// hotspot wifi
const char *ssid = "Hannah";
const char *pass = "andrewsux";

// Set up NUS WIFI 
// #define NUS_NET_IDENTITY "nusstu\e1036284"  //ie nusstu\e0123456
// #define NUS_NET_USERNAME "e1036284"
// #define NUS_NET_PASSWORD "142101Tey%" 

// const char* ssid = "NUS_STU"; // eduroam SSID

// MQTT
const char *server = "mqtt://128.199.83.151:1883"; // DO server IP
const char *subscribeTopic = "pressure/response";
const char *publishTopic = "pressure/input";
ESP32MQTTClient mqttClient; // all params are set later

String sendPayload;
String receivedPayload; 

#define PRESSURE_PIN 32 
#define PRESSURE_THRESHOLD 500 
volatile int pressure_val = 0;

#define ULTRASONIC_TRIG_PIN 18
#define ULTRASONIC_ECHO_PIN 19
#define ULTRASONIC_DELAY 1000
long duration;
int distance;
#define TRIGGER_DIST 20
#define WAKEUP_PIN 14 // use GPIO 14 to wake up the ESP32 CAM

#define DELAY 1000*60 // send sensor readings to remote server every minute
unsigned long t = millis();
unsigned long t_ultrasonic = millis();

void setup()
{
  Serial.begin(115200);
  
  // ultrasonic sensor
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);

  // pressure sensor
  pinMode(PRESSURE_PIN, INPUT); 

  // pin used to wake up ESP32-CAM
  pinMode(WAKEUP_PIN, OUTPUT); //INPUT_PULLUP
  digitalWrite(WAKEUP_PIN, HIGH); // by default, should not trigger anything

  //hotspot wifi setup
   WiFi.begin(ssid, pass); 
   WiFi.setHostname("c3test");

  //NUS Wifi setup
  // delay(10);
  // Serial.print(F("Connecting to network: "));
  // Serial.println(ssid);
  // WiFi.disconnect(true);  //disconnect from WiFi to set new WiFi connection
   
  // // Servers without Certificate like Bielefeld works this way, see https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/examples/WiFiClientEnterprise/WiFiClientEnterprise.ino Line 30.
  // WiFi.begin(ssid, WPA2_AUTH_PEAP, NUS_NET_IDENTITY, NUS_NET_USERNAME, NUS_NET_PASSWORD); 

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  } 
  log_i();
  log_i("setup, ESP.getSdkVersion(): ");
  log_i("%s", ESP.getSdkVersion());

  mqttClient.enableDebuggingMessages();

  mqttClient.setURI(server);
  mqttClient.enableLastWillMessage("lwt", "I am going offline");
  mqttClient.setKeepAlive(30);
  mqttClient.loopStart();
}
void loop() {
  // pressure sensor data collection
  if (pressure_val == 0){
    int pressure_read = analogRead(PRESSURE_PIN);
    if (pressure_read < PRESSURE_THRESHOLD){
      pressure_val = 0;
    }
    else {
      pressure_val = 1;
    } 
  }
  if (millis() - t >= DELAY){
    t = millis();
    // Create a JSON object to hold your data
    DynamicJsonDocument jsonData(256); // Adjust the size based on your data

    jsonData["pressure"] = String(pressure_val);
    // Serialize JSON to a string
    String payload;
    serializeJson(jsonData, payload);
    Serial.println(payload);
    mqttClient.publish(publishTopic, payload, 0, true);

    Serial.print("The force sensor value = ");
    Serial.print(pressure_val); // print pressure value sent to server
    pressure_val = 0; // reset pressure flag
  }

  // ultrasonic sensor used to trigger ESP32-CAM wakeup
  if (millis() -  t_ultrasonic >= ULTRASONIC_DELAY) {
    t_ultrasonic = millis();
    // Clears the trigPin
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(5);

    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2;
    // Prints the distance on the Serial Monitor
    Serial.printf("Distance: %d cm \n", distance);
    // Serial.println(distance);

    if (distance < TRIGGER_DIST){
      digitalWrite(WAKEUP_PIN, LOW);
      Serial.println("wakeup triggered");
      delay(10);
      digitalWrite(WAKEUP_PIN, HIGH); //reset the pin
    }
  }
}

void onConnectionEstablishedCallback(esp_mqtt_client_handle_t client)
{
    if (mqttClient.isMyTurn(client)) // can be omitted if only one client
    {
        mqttClient.subscribe(subscribeTopic, [](const String &payload)
                             { Serial.printf("%s: %s", subscribeTopic, payload.c_str()); });

    }
}

esp_err_t handleMQTT(esp_mqtt_event_handle_t event)
{
    mqttClient.onEventCallback(event);
    return ESP_OK;
}
