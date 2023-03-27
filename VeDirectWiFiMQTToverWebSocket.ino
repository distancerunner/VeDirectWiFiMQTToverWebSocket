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


uint8_t tickslower=0;
uint8_t tickfaster=0;
uint32_t oldPPVValue = 0;
String label, val, VE_prod_id, VE_fw, VE_serial_nr, VE_mppt, VE_OR;
// 32 bit ints to collect the data from the device
int32_t VE_state, VE_error, VE_yield_today, VE_power_max_today, VE_yield_yesterday, VE_power_pv,
		VE_power_max_yesterday, VE_day_sequence_number;

float VE_voltage, VE_current, VE_voltage_pv, VE_yield_total;

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
  Serial.println("Wait 30s and than restart...");
  delay(30000);
  ESP.restart();
}


void loop() {
    espMQTT.update();  // should be called
    // counter++;
    static uint32_t timerTicker = millis();
    static uint32_t timerTicker2 = millis();
    uint32_t sendingInterval = 10000;

    if(Serial.available()){
      // For testing, send USB input data over bridged pins from TX to RX pin
#ifndef ESP32
      victronSerial.write(Serial.read());
#endif
#ifdef ESP32
      Serial2.write(Serial.read());
#endif
    }

    if (tickslower > 1000) {
      tickslower = 0;
    }
    if (tickfaster > 1000) {
      tickfaster = 0;
    }

#ifndef ESP32
    if (victronSerial.available() > 0) {
      Serial.println("Reading values from Victron Energy device using VE.Direct text mode");
      Serial.println();
      
      label = victronSerial.readStringUntil('\t');      
      val = victronSerial.readStringUntil('\r');
#endif
#ifdef ESP32
    if (Serial2.available() > 0) {
      label = Serial2.readStringUntil('\t');
      val = Serial2.readStringUntil('\r');
#endif
      label.trim();

      if(label == "PID") {
        VE_prod_id = val;
      } else if (label.indexOf("FW") > 0) {
        VE_fw = val;
      } else if (label.indexOf("SER#") > 0) {
        VE_serial_nr = val;
      } else if (label == "VPV") {
        float temp = val.toFloat();
        temp = temp / 1000;
        VE_voltage_pv = temp;
      } else if (label == "PPV") {
        float temp = val.toInt();
        VE_power_pv = temp;
      } else if (label == "V") {
        val = val.substring(0, 4);
        float temp = val.toFloat();
        temp = temp / 100;
        VE_voltage = temp;
      } else if (label == "I") {
        val = val.substring(0, 4);
        float temp = val.toFloat();
        temp = temp / 1000;
        VE_current = temp;
      } else if (label == "CS") {
        VE_state = val.toInt();
      } else if (label == "MPPT") {
        VE_mppt = val;
      } else if (label == "OR") {
        VE_OR = val;
      } else if (label == "ERR") {
        VE_error = val.toInt();
      } else if (label == "LOAD") {
        VE_load = val.toFloat();
      // } else if (label == "IL") {
        // IL = val.toFloat();
      } else if (label == "H19") {
        val = val.substring(0, 4);
        float temp = val.toFloat();
        temp = temp / 100;
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

    // check dynamic values every second
    if (millis() > timerTicker2 + 1000) {
      // slow down, at night, if PV Voltage is lower than x Volts
      if (VE_voltage_pv < 14.00 && VE_voltage_pv > 4.00) {
        // slow down data sending;
        tickslower++;
        sendingInterval = 300000;
      }

      // speed up intervals, if there is a bigger change in power
      Serial.println(abs((int)oldPPVValue - (int)VE_power_pv));
      // if (abs((int)oldPPVValue - (int)VE_power_pv) > 15 && oldPPVValue > 0) {
      if (abs((int)oldPPVValue - (int)VE_power_pv) > 15) {
        // speed up data sending;
        tickfaster++;
        sendingInterval = 2000;
        oldPPVValue = VE_power_pv;
      }

      timerTicker2 = millis();
      // store "old" PV Power value for later comparison
    }

    if (millis() > timerTicker + sendingInterval) {
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
        Serial.println(String(VE_voltage));
        Serial.print("Current                ");
        Serial.println(String(VE_current));
        Serial.print("Power PV               ");
        Serial.println(String(VE_power_pv));
        Serial.print("Voltage PV             ");
        Serial.println(String(VE_voltage_pv));
        Serial.print("Yield Total kWh        ");
        Serial.println(String(VE_yield_total));
        Serial.print("Yield Today Wh         ");
        Serial.println(String(VE_yield_today));
        Serial.print("Yield Yesterday Wh     ");
        Serial.println(String(VE_yield_yesterday));
        Serial.print("Max Power Today W      ");
        Serial.println(String(VE_power_max_today));
        Serial.print("Max Power Yesterday W  ");
        Serial.println(String(VE_power_max_yesterday));
        Serial.print("MPPT Code              ");
        Serial.println(String(VE_mppt));
        Serial.print("MPPT Firmware          ");
        Serial.println(String(VE_fw));
        Serial.print("MPPT SerialNr          ");
        Serial.println(String(VE_serial_nr));
        Serial.print("Day Sequence Number    ");
        Serial.println(String(VE_day_sequence_number));
        Serial.print("State of operation     ");
        Serial.println(VEStatus[VE_state].value);
        Serial.print("Error Code             ");
        // Serial.println(VEError[VE_error].value);
        Serial.println(String(tickslower) + "/" + String(tickfaster));
        Serial.println();

        Serial.print("Send MQTT data...");
        Serial.println();
        // mqttSend("/victron/sensor/watt", String(counter));
        mqttSend("/victron/sensor/ve_power_pv", String(VE_power_pv));
        mqttSend("/victron/sensor/ve_voltage_pv", String(VE_voltage_pv));
        mqttSend("/victron/sensor/ve_yield_today", String(VE_yield_today));
        mqttSend("/victron/sensor/ve_yield_total", String(VE_yield_total));
        mqttSend("/victron/sensor/ve_power_max_today", String(VE_power_max_today));
        mqttSend("/victron/sensor/ve_voltage", String(VE_voltage));
        mqttSend("/victron/sensor/ve_current", String(VE_current));
        mqttSend("/victron/sensor/ve_state", VEStatus[VE_state].value);
        mqttSend("/victron/sensor/ve_error", VEError[VE_error].value + (String((int)tickslower) + "/" + String((int)tickfaster)));
        // mqttSend("/victron/sensor/ve_error", VEError[VE_error].value);
        mqttSend("/victron/sensor/ve_last_update", getClockTime());
        mqttSend("/victron/sensor/ve_wifi_ssid", WiFi.SSID());
        timerTicker = millis();
      }
    }
}




