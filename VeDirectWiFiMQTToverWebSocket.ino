#include "config.h"

// #include <HardwareSerial.h>
// #include "VEDirect.h"
#include "wifiVEDirect.h"

// #include <Wire.h>           
// #include <LiquidCrystal_I2C.h> 
#ifndef ESP32
#include <SoftwareSerial.h>
#define rxPin D2 // PINs according a new NodeMCU V3 Board, with an ESP8266 Chipset.
#define txPin D3
#endif
#ifdef ESP32
#define rxPin 18 // PINs according a new NodeMCU V3 Board, with an ESP8266 Chipset.
#define txPin 19
#define LED_BUILTIN 2
#endif

#ifndef ESP32
EspSoftwareSerial::UART victronSerial;
#endif


int counter=0;
String label, val;
// 32 bit ints to collect the data from the device
int32_t VE_fw, VE_voltage, VE_current, VE_voltage_pv, VE_power_pv, VE_state, VE_mppt,
		VE_error, VE_yield_total, VE_yield_today, VE_power_max_today, VE_yield_yesterday, 
		VE_power_max_yesterday, VE_day_sequence_number, VE_serial_nr, VE_prod_id;

// Boolean to collect an ON/OFF value
uint8_t VE_load;

typedef struct keyAndValue_ {
   int key;
   char value[40];
} keyAndValue_t;

keyAndValue_t VEStatus[] = {
{0,"Off"},
{2,"Fault"},
{3,"Bulk"},
{4,"Absorption"},
{5,"Float"},
};

keyAndValue_t VEError[] = {
{0,"No Error"},
{2,"Battery voltage too high"},
{17,"Charger voltage too high"},
{18,"Charger over current"},
{20,"Bulk time limit exceeded"},
{21,"Current sensor issue"},
{26,"Terminals overheated"},
{33,"Input Voltage too high (solar panel)"},
{34,"Input current too high (solar panel)"},
{38,"Input shutdown (excessive bat volt.)"},
{116,"Factory calibration lost"},
{117,"invalied/incompatible firmware"},
{119,"User settings invalid"},
};
int mqtt_server_count = sizeof(mqtt_server) / sizeof(mqtt_server[0]);

void setup() {
  Serial.begin(19200);
  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED_BUILTIN pin as an output
#ifndef ESP32
  victronSerial.begin(19200, SWSERIAL_8N1, rxPin, txPin, false);
#endif
#ifdef ESP32
  Serial2.begin(19200, SERIAL_8N1, 18, 19); // Pins D18 and D19 accoring an ESP32 DEVKITV1
#endif

  if (startWiFiMulti()) {
    Serial.println("Wifi connected make next step...");
    Serial.println();  
  
    setClock();
    if ( startMQTT()) {
      return;
    }     
  }
#ifndef ESP32
  if (!victronSerial) { // If the object did not initialize, then its configuration is invalid
    Serial.println("Invalid EspSoftwareSerial pin configuration, check config"); 
    while (1) { // Don't continue with invalid configuration
      delay (1000);
    }
  }
#endif
  delay(30000);
  ESP.restart();
}


void loop() {
    espMQTT.update();  // should be called
    counter++;
    static uint32_t prev_ms = millis();

    if(Serial.available()){
      // For testing, send USB input data over bridged pins from TX to RX pin
#ifndef ESP32
      victronSerial.write(Serial.read());
#endif
#ifdef ESP32
      Serial2.write(Serial.read());
#endif
    }


    if (counter > 255) {
      counter = 0;
    }

#ifndef ESP32
    if (victronSerial.available() > 0) {
      Serial.println("Reading values from Victron Energy device using VE.Direct text mode");
      Serial.println();
      
      // Serial.println("Daten verfügbar");
      label = victronSerial.readStringUntil('\t');      
      val = victronSerial.readStringUntil('\r');
      // val = victronSerial.readStringUntil('\r\r\n');
#endif
#ifdef ESP32
    if (Serial2.available() > 0) {
      // Serial.println("Daten verfügbar");
      label = Serial2.readStringUntil('\t');      
      val = Serial2.readStringUntil('\r');
      // val = Serial2.readStringUntil('\r\r\n');
#endif
      digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);  // Turn the LED on (Note that LOW is the voltage level
      // Serial.println(label);
      // Serial.println(val);
      if(label == "PID") {
        // VE_prod_id = val;
      } else if (label == "FW") {
        // VE_fw = val;
      } else if (label == "SER#") {
        // VE_serial_nr = val;
      } else if (label == "V") {
        float temp = val.toFloat();
        temp = temp / 1000;
        VE_voltage = temp;
        // Serial.println(V);  
      } else if (label == "I") {
        VE_current = val.toFloat();
      } else if (label == "VPV") {
        float temp = val.toFloat();
        temp = temp / 1000;
        VE_voltage_pv = temp;
      } else if (label == "PPV") {
        VE_power_pv = val.toFloat();
      } else if (label == "CS") {
        VE_state = val.toInt();
      } else if (label == "MPPT") {
        // VE_mppt = val;
      // } else if (label == "OR") {
      //   OR = val;
      } else if (label == "ERR") {
        VE_error = val.toInt();
      } else if (label == "LOAD") {
        VE_load = val.toFloat();
      // } else if (label == "IL") {
        // IL = val.toFloat();
      } else if (label == "H19") {
        int temp = val.toInt();
        temp = temp * 10;
        VE_yield_total = temp;
      } else if (label == "H20") {
        int temp = val.toInt();
        temp = temp * 10;
        VE_yield_today = temp;
      } else if (label == "H21") {
        VE_power_max_today = val.toInt();
      } else if (label == "H22") {
        int temp = val.toInt();
        temp = temp * 10;
        VE_yield_yesterday = temp;
      } else if (label == "H23") {
        VE_power_max_yesterday = val.toInt();
      } else if (label == "HSDS") {
        VE_day_sequence_number = val.toInt();
      }  
    } 

    if (millis() > prev_ms + 10000) {
      if ( checkWiFi()) {

        // in case mqtt connection is lost, restart device
        if (!espMQTT.isConnected()) {
          delay(10000);
          // after 10s, check if wifi is available
          if ( checkWiFi()) {
            // try to reconnect to mqtt
            if (!startMQTT()) {
              Serial.print("MQTT Connection lost, restart system");
              ESP.restart(); 
            }

          } else {
            Serial.print("MQTT Connection lost, restart system");
            ESP.restart();          
          }
        }

        // Print each of the values
        Serial.print("Voltage                ");
        Serial.println(VE_voltage, DEC);
        Serial.print("Current                ");
        Serial.println(VE_current, DEC);
        Serial.print("Power PV               ");
        Serial.println(VE_power_pv, DEC);
        Serial.print("Voltage PV             ");
        Serial.println(VE_voltage_pv, DEC);
        Serial.print("Yield Total kWh        ");
        Serial.println(VE_yield_total, DEC);
        Serial.print("Yield Today kWh        ");
        Serial.println(VE_yield_today, DEC);
        Serial.print("Yield Yesterday kWh    ");
        Serial.println(VE_yield_yesterday, DEC);
        Serial.print("Max Power Today        ");
        Serial.println(VE_power_max_today, DEC);
        Serial.print("Max Power Yesterday    ");
        Serial.println(VE_power_max_yesterday, DEC);
        Serial.print("MPPT Code             ");
        Serial.println(VE_mppt, DEC);
        Serial.print("MPPT Firmware         ");
        Serial.println(VE_fw, DEC);
        Serial.print("Day Sequence Number   ");
        Serial.println(VE_day_sequence_number, DEC);
        Serial.print("Error Code             ");
        Serial.println(VE_error, DEC);
        Serial.print("Error Code             ");
        Serial.println(VEStatus[VE_state].value);
        Serial.print("State of operation     ");
        Serial.println(VE_state, DEC);
        Serial.print("State of operation     ");
        Serial.println(VEError[VE_error].value);
        Serial.println();

        Serial.print("Send MQTT data...");
        Serial.println();
        mqttSend("/victron/sensor/watt", String(counter));
        mqttSend("/victron/sensor/ve_power_pv", String(VE_power_pv));
        mqttSend("/victron/sensor/ve_voltage_pv", String(VE_voltage_pv));
        mqttSend("/victron/sensor/ve_yield_today", String(VE_yield_today));
        mqttSend("/victron/sensor/ve_yield_total", String(VE_yield_total));
        mqttSend("/victron/sensor/ve_power_max_today", String(VE_power_max_today));
        mqttSend("/victron/sensor/ve_voltage", String(VE_voltage));
        mqttSend("/victron/sensor/ve_current", String(VE_current));
        mqttSend("/victron/sensor/ve_state", VEStatus[VE_state].value);
        mqttSend("/victron/sensor/ve_error", VEError[VE_error].value);
        mqttSend("/victron/sensor/ve_last_update", getClockTime());
        mqttSend("/victron/sensor/ve_wifi_ssid", WiFi.SSID());
        prev_ms = millis();
      }
    }
}




