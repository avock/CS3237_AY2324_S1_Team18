/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-post-image-photo-server/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <Client.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"

// #include "esp_wpa2.h" //wpa2 library for connections to Enterprise networks
// byte mac[6];

// #define NUS_NET_IDENTITY "nusstu\e1036284"  //ie nusstu\e0123456
// #define NUS_NET_USERNAME "e1036284"
// #define NUS_NET_PASSWORD "142101Tey%" 

// const char* ssid = "NUS_STU"; // eduroam SSID

const char* ssid = "Hannah";
const char* password = "andrewsux";

String serverName = "172.20.10.3";   // REPLACE WITH YOUR Raspberry Pi IP ADDRESS
// String serverName = "https://666f764fe22ea1.lhr.life";   // OR REPLACE WITH YOUR DOMAIN NAME

String serverPath = "/image/upload";     // The default serverPath should be upload.php

const int serverPort = 8000;

// dashboard server
// String serverDashboard ="172.22.25.78";
// const int serverPortDashboard =;

WiFiClient client;

// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

int flashPin = 4; //on board camera flash

const int timerInterval = 1000;    // time between each HTTP POST image
unsigned long previousMillis = 0;   // last time image was sent
unsigned long wakeUpTime = 0; // time that camera has completed setup and starts sending images
unsigned long connectTime = 0; //time taken for reconnection
const int cameraOnInterval = 3*60*1000; //allow camera to run for 3 mins after each wakeup before sending to deep sleep
const int reconnectInterval = 10*1000; //wait 10s before restarting ESP32 if camera does not connect to WiFi

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void callback(){
  Serial.println("hello");
  digitalWrite(flashPin,HIGH);
  delay(1000);
  digitalWrite(flashPin,LOW);
  delay(1000);
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  Serial.begin(115200);
  
  pinMode(flashPin, OUTPUT); //flash camera to show camera is woken up
  //Print the wakeup reason for ESP32
  print_wakeup_reason();
  callback();
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_14,0); //1 = High, 0 = Low

  //hotspot wifi setup
WiFi.begin(ssid, password); 
WiFi.setHostname("c3test");

  // NUS WiFi setup
  // WiFi.mode(WIFI_STA);
  // Serial.println();
  // Serial.print("Connecting to ");
  // Serial.println(ssid);
  // WiFi.disconnect(true);  //disconnect from WiFi to set new WiFi connection
  // WiFi.begin(ssid, WPA2_AUTH_PEAP, NUS_NET_IDENTITY, NUS_NET_USERNAME, NUS_NET_PASSWORD);
  connectTime = millis();

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
    if (millis() - connectTime >= reconnectInterval){
      Serial.print("WiFi connection error, restarting ESP32-CAM");
      ESP.restart();
    }
  }
  Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }
  
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }

  // connect to server
  // Serial.println("Connecting to server: " + serverName);
  // client.connect(serverName.c_str(), serverPort);

  // sendPhoto(); 

  wakeUpTime = millis();
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= timerInterval) {
    sendPhoto();
    // sendToDashboard();
    previousMillis = currentMillis;
  }
  if (currentMillis - wakeUpTime >= cameraOnInterval){
    // Disconnect Client
    // client.stop();

    // Go to sleep now
    Serial.println("Going to sleep now");
    esp_deep_sleep_start();
  }
}

String sendPhoto() {
  String getAll;
  String getBody;

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
  }
  

  if (client.connect(serverName.c_str(), serverPort)) {
    Serial.println("Connection successful!");    
    String head = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--RandomNerdTutorials--\r\n";

    uint32_t imageLen = fb->len;
    uint32_t extraLen = head.length() + tail.length();
    uint32_t totalLen = imageLen + extraLen;
  
    client.println("POST " + serverPath + " HTTP/1.1");
    client.println("Host: " + serverName);
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
    client.println();
    client.print(head);
  
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n=0; n<fbLen; n=n+1024) {
      if (n+1024 < fbLen) {
        client.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen%1024>0) {
        size_t remainder = fbLen%1024;
        client.write(fbBuf, remainder);
      }
    }   
    client.print(tail);
    
    esp_camera_fb_return(fb);
    
    int timoutTimer = 10000;
    long startTimer = millis();
    boolean state = false;
    
    while ((startTimer + timoutTimer) > millis()) {
      Serial.print(".");
      delay(100);      
      while (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (getAll.length()==0) { state=true; }
          getAll = "";
        }
        else if (c != '\r') { getAll += String(c); }
        if (state==true) { getBody += String(c); }
        startTimer = millis();
      }
      if (getBody.length()>0) { break; }
    }
    Serial.println();
    client.stop();
    Serial.println(getBody);
  }
  else {
    getBody = "Connection to " + serverName +  " failed.";
    Serial.println(getBody);
    // ESP.restart();
  }
  return getBody;
}

// String sendToDashboard() {
//   String getAll;
//   String getBody;

//   camera_fb_t * fb = NULL;
//   fb = esp_camera_fb_get();
//   if(!fb) {
//     Serial.println("Camera capture failed");
//     delay(1000);
//     ESP.restart();
//   }
  

//   if (client.connect(serverDashboard.c_str(), serverPortDashboard)) {
//   // if (client.connected()) {
//     Serial.println("Connection successful!");    
//     String head = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
//     String tail = "\r\n--RandomNerdTutorials--\r\n";

//     uint32_t imageLen = fb->len;
//     uint32_t extraLen = head.length() + tail.length();
//     uint32_t totalLen = imageLen + extraLen;
  
//     client.println("POST " + serverPath + " HTTP/1.1");
//     client.println("Host: " + serverDasboard);
//     client.println("Content-Length: " + String(totalLen));
//     client.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
//     client.println();
//     client.print(head);
  
//     uint8_t *fbBuf = fb->buf;
//     size_t fbLen = fb->len;
//     for (size_t n=0; n<fbLen; n=n+1024) {
//       if (n+1024 < fbLen) {
//         client.write(fbBuf, 1024);
//         fbBuf += 1024;
//       }
//       else if (fbLen%1024>0) {
//         size_t remainder = fbLen%1024;
//         client.write(fbBuf, remainder);
//       }
//     }   
//     client.print(tail);
    
//     esp_camera_fb_return(fb);
    
//     int timoutTimer = 10000;
//     long startTimer = millis();
//     boolean state = false;
    
//     while ((startTimer + timoutTimer) > millis()) {
//       Serial.print(".");
//       delay(100);      
//       while (client.available()) {
//         char c = client.read();
//         if (c == '\n') {
//           if (getAll.length()==0) { state=true; }
//           getAll = "";
//         }
//         else if (c != '\r') { getAll += String(c); }
//         if (state==true) { getBody += String(c); }
//         startTimer = millis();
//       }
//       if (getBody.length()>0) { break; }
//     }
//     Serial.println();
//     client.stop();
//     Serial.println(getBody);
//   }
//   else {
//     getBody = "Connection to " + serverDashboard +  " failed.";
//     Serial.println(getBody);
//     // ESP.restart();
//   }
//   return getBody;
// }