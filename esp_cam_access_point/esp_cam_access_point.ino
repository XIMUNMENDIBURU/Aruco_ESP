#include "picture.h"
#include <WiFi.h>
#define WIFI true
const char* ssid = "Robot-Ximun";
const char* password = "123456789";

bool lastState = false;

void setup() {
  setWifiMode(WIFI);
  Serial.begin(115200);
  init_cam();
  if(getWifiMode()) {
    WiFi.softAP(ssid, password);
    startCameraServer();
  }
}

void loop() {
  if (!getWifiMode()) {
    video_handler();
  } else {
  }
}