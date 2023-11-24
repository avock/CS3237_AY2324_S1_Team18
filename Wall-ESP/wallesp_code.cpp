// THIS CODE IS TO BE USED FOR THE CS3237 DEMO - WITH NUS WIFI
#include "Arduino.h"
#include "ESP32MQTTClient.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <WiFi.h> //Wifi library
#include "esp_wpa2.h" //wpa2 library for connections to Enterprise networks
byte mac[6];


// DEFINE ALL THE PINS FOR SENSORS 
Servo myservo;
const int SERVO_PIN = 17;
#define PIR_PIN 4

// Published values for SG90 servos; adjust if needed
int minUs = 600;
int maxUs = 2300;

int pir_state = LOW; 
#define ULTRASONIC_TRIG_PIN 14
#define ULTRASONIC_ECHO_PIN 12
long duration;
int distance;
#define PHOTORES_PIN 33 // needs analog pin
#define DHTPIN 16     // Digital pin connected to the DHT sensor 
#define DHTTYPE    DHT11     // DHT 11
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

// track the states 
int light_state = 0; 
int room_motion = 0; // for PIR 
int door_motion = 0; // for door


// Set up WIFI 
#define NUS_NET_IDENTITY "nusstu\eXXXXXX"  
#define NUS_NET_USERNAME "XXXXXX"
#define NUS_NET_PASSWORD "XXXXXX" 
const char* ssid = "NUS_STU"; // eduroam SSID

// SET UP MQTT PUB AND SUB
const char *server = "mqtt://128.199.83.151:1883"; // DO server IP

const char *subscribeTopic = "home/response";
const char *publishTopic = "home/input";
ESP32MQTTClient mqttClient; // all params are set later

String sendPayload;
String receivedPayload;


// DATETIME 
#include <NTPClient.h>
WiFiUDP ntpUDP;
#include "time.h"
const char* ntpServer = "sg.pool.ntp.org";
const long gmtOffset_sec = 8 * 3600; // GMT offset in seconds
NTPClient timeClient(ntpUDP, ntpServer, gmtOffset_sec);

// get the actual time
String getFormattedDateTime(const NTPClient& ntpClient) {
  String formattedDate = ntpClient.getFormattedTime(); // Get the time in "hh:mm:ss" format

  // Get the current epoch time (in seconds)
  unsigned long epochTime = ntpClient.getEpochTime();

  // Calculate the current date using epoch time
  int year, month, day, hour, minute, second;
  time_t rawtime = epochTime;
  struct tm *timeinfo;
  timeinfo = localtime(&rawtime);


  hour = timeinfo->tm_hour;
  minute = timeinfo->tm_min;

  // Format the date and time in ISO 8601 format
  String formattedDateTime = 
                            (hour < 10 ? "0" : "") + String(hour) + ":" +
                            (minute < 10 ? "0" : "") + String(minute) + ":" + // edit the time 
                            "00";

  return formattedDateTime;
}


void setup(){

    // CONNECT TO WIFI //
    Serial.begin(115200);
    delay(10);
    Serial.print(F("Connecting to network: "));
    Serial.println(ssid);
    WiFi.disconnect(true);  //disconnect from WiFi to set new WiFi connection
    
    WiFi.begin(ssid, WPA2_AUTH_PEAP, NUS_NET_IDENTITY, NUS_NET_USERNAME, NUS_NET_PASSWORD); 

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(F("."));
    }
    Serial.println("");
    Serial.println(F("WiFi is connected!"));
    Serial.println(F("IP address set: "));
    Serial.println(WiFi.localIP()); //print LAN IP
   

    // Configure pinmodes
    myservo.attach(SERVO_PIN, minUs, maxUs);
    pinMode(PIR_PIN,INPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);
    pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);


    // FOR THE TEMP + HUMIDITY SENSOR 
    dht.begin();
    sensor_t sensor;
    dht.temperature().getSensor(&sensor);
    dht.humidity().getSensor(&sensor);
    delayMS = sensor.min_delay / 1000;


    // FOR THE MQTT
    log_i();
    log_i("setup, ESP.getSdkVersion(): ");
    log_i("%s", ESP.getSdkVersion());
    mqttClient.enableDebuggingMessages();
    mqttClient.setURI(server);
    mqttClient.enableLastWillMessage("lwt", "I am going offline");
    mqttClient.setKeepAlive(60);
    WiFi.begin(ssid);
    WiFi.setHostname("c3test");


    // This code block will wait until the wifi connects before progressing further in your program.
    while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
    }
    mqttClient.loopStart();



}

void loop(){




    static unsigned long previousMillis = 0;  // will store last time data was sent
    const long interval = 1000 * 60;  // interval at which to send data (in milliseconds) // 1 min
    unsigned long currentMillis = millis();  // get the current time


    // READ THE PIR SENSOR - track motion in the minute//
    int pir_val = digitalRead(PIR_PIN);
    if (pir_val == 1){
        room_motion = 1; 
    }

    // OBTAIN DISTANCE FROM ULTRASONIC SENSOR - track motion in the minute  //
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG_PIN, HIGH); // Sets the trigPin on HIGH state for 10 micro seconds
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2;

    // if < 50cm (assuming a door is 80cm)
    if (distance <= 50){
        door_motion = 1; 
    } // if else


   // SEND THE DATA EVERY MINUTE
    DynamicJsonDocument jsonData(256); // Adjust the size based on your data
    String newformattedDateTime = getFormattedDateTime();
    jsonData["time"] = String(newformattedDateTime);
    jsonData["pir"] = String(room_motion);
    jsonData["light"] = String(light_state);
    jsonData["ultrasonic"] = String(door_motion);

    // READ TEMP AND HUMIDITY DATA - only read when you need to send to the DO
    delay(delayMS);
    sensors_event_t temp_event;
    dht.temperature().getEvent(&temp_event);
    if (isnan(temp_event.temperature)) {
        Serial.println(F("Error reading temperature!"));
    }
    sensors_event_t humidity_event;
    dht.humidity().getEvent(&humidity_event);
    if (isnan(humidity_event.relative_humidity)) {
        Serial.println(F("Error reading humidity!"));
    }

    jsonData["temperature"] = String(temp_event.temperature);
    jsonData["humidity"] = String(humidity_event.relative_humidity);
    
    // Serialize JSON to a string
    String payload;


    if (currentMillis - previousMillis >= interval) {
    // save the last time data was sent
    previousMillis = currentMillis;

    serializeJson(jsonData, payload);
    Serial.println(payload);
    mqttClient.publish(publishTopic, payload, 0, true);

    // reset the motion counts 
    door_motion = 0;
    room_motion = 0;

    } // SEND DATA EACH MINUTE     
}

void onConnectionEstablishedCallback(esp_mqtt_client_handle_t client)
{
    if (mqttClient.isMyTurn(client)) // can be omitted if only one client
    {

        mqttClient.subscribe("home/response", [](const String &topic, const String &payload)
        {
        
            receivedPayload = String(payload.c_str());

            if (receivedPayload == "1") {

                // TURN THE LIGHT SWITCH ON - switch moves up
                myservo.write(40);              
                
                // Change the light switch state 
                light_state  = 1; 

            } else if (receivedPayload == "0") {
                // TURN THE LIGHT SWITCH OFF - switch moves down.            
                myservo.write(0);  // tell servo to go to a particular angle
                delay(1000);

                // Change the light switch state 
                light_state  = 0; 


            } 
        }); 

    }
}

esp_err_t handleMQTT(esp_mqtt_event_handle_t event)
{
    mqttClient.onEventCallback(event);
    return ESP_OK;
}

