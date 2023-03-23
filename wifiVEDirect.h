/*
   VE.Direct WiFi code.

   GITHUB Link

   MIT License

   Copyright (c) 2020 Ralf Lehmann


   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
*/
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti WiFiMultiElement;

// #include <WiFi.h>
// #include <WiFiMulti.h>
// WiFiMulti WiFiMultiElement;


int ssid_count = sizeof(ssid) / sizeof(ssid[0]);

// Set time via NTP, as required for x.509 validation
void setClock() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");  // UTC

  Serial.print("Waiting for NTP time sync: ");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    yield();
    delay(500);
    now = time(nullptr);
  }

  Serial.print("");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print("NTP time" + String(asctime(&timeinfo)));
}

String getClockTime() {
  time_t now = time(nullptr);
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  return String(asctime(&timeinfo));
}

boolean startWiFiMulti() {
  Serial.print("Number of ssid's" + String(ssid_count));
  // add all ssid's to WiFiMulti
  for (int i = 0; i < ssid_count; i++) {
    WiFiMultiElement.addAP(ssid[i], pw[i]);
  }
  if ((WiFiMultiElement.run() == WL_CONNECTED)) {
    Serial.print("WiFi connected");

    return true;
  }
  Serial.print("WiFi could not be started");
  return false;
}

// check if WiFi is still connected 
// if not, try to reconnect
boolean checkWiFi(){

  if ( WiFi.status() != WL_CONNECTED ){
    return startWiFiMulti();
  }
  return true;
}


