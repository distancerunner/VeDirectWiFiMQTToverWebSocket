#include "config.h"

#include <HardwareSerial.h>
#include "VEDirect.h"
#include "wifiVEDirect.h"
#include <WebSocketsClient.h>  // include before MQTTPubSubClient.h
#include <MQTTPubSubClient.h>
#include <Wire.h>           
#include <LiquidCrystal_I2C.h> 

int counter=0;
// 32 bit ints to collect the data from the device
int32_t VE_fw, VE_voltage, VE_current, VE_voltage_pv, VE_power_pv, VE_state, VE_mppt,
		VE_error, VE_yield_total, VE_yield_today, VE_power_max_today, VE_yield_yesterday, 
		VE_power_max_yesterday, VE_day_sequence_number;

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
#define RXD2 16
#define TXD2 17

WebSocketsClient client;
MQTTPubSubClient espMQTT;

// Serial1 GPIO1 (TX) and GPIO3 (RX) -> 8266 RX und TX Pins am Board
// Serial1: RX1 on GPIO9, TX1 on GPIO10 -> ESP32 Wroom: RX1=16, TX1=17
// HardwareSerial VEConnect ( Serial1 ); 

// VEDirect instantiated with relevant serial object
// VEDirect myve(VEConnect);
// LiquidCrystal_I2C lcd(0x3F,16,2);   

void setup() {
    Serial.begin(115200);
    Serial2.begin(19200, SERIAL_8N1, RXD2, TXD2);

    if (startWiFiMulti()) {
      Serial.println("Wifi connected make next step...");
      Serial.println();  
    
      setClock();
      if ( startMQTT()) {
        return;
      }     
    }

    delay(30000);
    ESP.restart();
}


void loop() {
    // espUpdater();  // should be called
    espMQTT.update();  // should be called
    counter++;
    static uint32_t prev_ms = millis();

    if (counter > 255) {
      counter = 0;
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

        Serial.println("Reading values from Victron Energy device using VE.Direct text mode");
        Serial.println();
        
        if (Serial2.available()) {
          Serial.println("Read Serial");
          Serial.print(char(Serial2.read()));
        }

        // Read the data
        // if(myve.begin()) {		// test connection
        //   VE_fw = myve.read(VE_FW);
        //   VE_voltage = myve.read(VE_VOLTAGE);
        //   VE_current = myve.read(VE_CURRENT);
        //   VE_voltage_pv = myve.read(VE_VOLTAGE_PV);
        //   VE_power_pv = myve.read(VE_POWER_PV);
        //   VE_state = myve.read(VE_STATE);
        //   VE_mppt = myve.read(VE_MPPT);
        //   VE_error = myve.read(VE_ERROR);
        //   VE_load = myve.read(VE_LOAD);
        //   VE_yield_total = myve.read(VE_YIELD_TOTAL);
        //   VE_yield_today = myve.read(VE_YIELD_TODAY);
        //   VE_power_max_today = myve.read(VE_POWER_MAX_TODAY);
        //   VE_yield_yesterday = myve.read(VE_YIELD_YESTERDAY);
        //   VE_power_max_yesterday = myve.read(VE_POWER_MAX_YESTERDAY);
        //   VE_day_sequence_number = myve.read(VE_DAY_SEQUENCE_NUMBER);
          
        // } else {
        //   Serial.println("Could not open serial port to VE device");
        //   //while (1);
        // }

        // // Print each of the values
        // Serial.print("Voltage                ");
        // Serial.println(VE_voltage, DEC);
        // Serial.print("Current                ");
        // Serial.println(VE_current, DEC);
        // Serial.print("Power PV               ");
        // Serial.println(VE_power_pv, DEC);
        // Serial.print("Voltage PV             ");
        // Serial.println(VE_voltage_pv, DEC);
        // Serial.print("Yield Total kWh        ");
        // Serial.println(VE_yield_total, DEC);
        // Serial.print("Yield Today kWh        ");
        // Serial.println(VE_yield_today, DEC);
        // Serial.print("Yield Yesterday kWh    ");
        // Serial.println(VE_yield_yesterday, DEC);
        // Serial.print("Max Power Today        ");
        // Serial.println(VE_power_max_today, DEC);
        // Serial.print("Max Power Yesterday    ");
        // Serial.println(VE_power_max_yesterday, DEC);
        // Serial.print("MPPT Code             ");
        // Serial.println(VE_mppt, DEC);
        // Serial.print("MPPT Firmware         ");
        // Serial.println(VE_fw, DEC);
        // Serial.print("Day Sequence Number   ");
        // Serial.println(VE_day_sequence_number, DEC);
        // Serial.print("Error Code             ");
        // Serial.println(VE_error, DEC);
        // Serial.print("Error Code             ");
        // Serial.println(VEStatus[VE_state].value);
        // Serial.print("State of operation     ");
        // Serial.println(VE_state, DEC);
        // Serial.print("State of operation     ");
        // Serial.println(VEError[VE_error].value);
        // Serial.println();

        // // Copy the raw data stream (minus the \r) to Serial0
        // // Call read() with a token that won't match any VE.Direct labels
        // // Serial.println("All data from device:");
        // // myve.read(VE_DUMP);
        // Serial.println();

        Serial.print("Send MQTT data...");
        Serial.println();
        // delay(5000);

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

void espUpdater() {
    espMQTT.update();  // should be called
}

void mqttSend(String sensor, String value) {
    Serial.print("Sending " + sensor);
    Serial.println();

    espMQTT.publish(sensor, value);
}

// void mqttSubscribe() {
//     // subscribe callback which is called when every packet has come
//     espMQTT.subscribe([](const String& topic, const String& payload, const size_t size) {
//         Serial.println("mqtt received: " + topic + " - " + payload);
//     });

//     // subscribe topic and callback which is called when /hello has come
//     espMQTT.subscribe("/victron/sensor/watt", [](const String& payload, const size_t size) {
//         Serial.print("victron/sensor/watt ");
//         Serial.println(payload);
//     });

//     espMQTT.subscribe("/victron/sensor/ve_power_pv", [](const String& payload, const size_t size) {
//         Serial.print("victron/sensor/ve_power_pv ");
//         Serial.println(payload);
//     });
// }

//
// Startup
//
boolean startMQTT() {
    Serial.println("connecting to host...");
    client.begin(mqtt_server[0], mqtt_port[0], "/mqtt", "mqtt");  // "mqtt" is required
    client.setReconnectInterval(2000);

    // initialize mqtt client
    espMQTT.begin(client);
    delay(1000);
    int maxretries = 5;
    while (!espMQTT.isConnected() && maxretries-- > 0) {
      Serial.println(".");
      if (espMQTT.connect(mqtt_clientID[0], mqtt_username[0], mqtt_pw[0])) {
        return true;
      } else {
        delay(2000);
      }
    }
    return false;
}




