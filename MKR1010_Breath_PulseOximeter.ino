#include <PubSubClient.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include "SparkFunCCS811.h"
#include <ArduinoJson.h>

#define CCS811_ADDR 0x5A
CCS811 ccs811(CCS811_ADDR);

PulseOximeter pox;

const char* ssid = "LANTAI BAWAH 2";
const char* password = "ibudini17";
const char mqtt_server[] = "192.168.0.111";
const char publishTopic[] = "data/sensors/breath/detection";

WiFiClient mkr1010Client;
PubSubClient client(mkr1010Client);
long lastData = 0;
boolean state = false;
long stateControl_time = 0;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(5, OUTPUT);
  Serial.begin(9600);

  Wire.begin();

  if (ccs811.begin() == false) {
    Serial.print("CCS811 Error");
    while(1);
  }

  Serial.print("Initializing pulse oximeter...");

  //pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  
}


void loop()
{
  delay(10);

  if (!client.connected()) {

    reconnect();
  }

  client.loop();
  sendData();
}

void setup_wifi(){

  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while( WiFi.status() != WL_CONNECTED){

    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
}

void reconnect(){

  while(!client.connected()){

    Serial.print("Attemting MQTT connection ...");
    String clientID = "ESP32breath--";
    clientID += String(random(0xffff), HEX);

    if (client.connect(clientID.c_str())) {

      Serial.println("Connected to MQTT Broker");
      digitalWrite(LED_BUILTIN, HIGH);
      if (!pox.begin()) {
        
        Serial.println("Failed");
        for(;;);
          }

      else{
        
        Serial.println("Success");
       }
      pox.setOnBeatDetectedCallback(onBeatDetected);

    }

    else {
      Serial.print("Failed, rc= ");
      Serial.print(client.state());
      Serial.println("Try again in 5 second");
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}

void sendData(){
  
  long now = millis();
  pox.update();

  if (now - lastData > 5000) {

    uint16_t co2;
    uint16_t voc;

    lastData = now;
    if (ccs811.dataAvailable()) {

      co2 = ccs811.getCO2();
      voc = ccs811.getTVOC();
      ccs811.readAlgorithmResults(); 
    }
    
    float hr = pox.getHeartRate();
    uint16_t spo2 = pox.getSpO2();

    const size_t capacity = JSON_OBJECT_SIZE(4);
    DynamicJsonDocument doc(capacity);

    doc["CO2"] = co2;
    doc["VOC"] = voc;
    doc["HR"] = hr;
    doc["SPO2"] = spo2;

    char payload[256];
    serializeJson(doc, payload);

    client.publish(publishTopic, payload);
    Serial.println(payload);
  }

  if (millis() - stateControl_time > 60000) {

    stateControl_time = millis();
    digitalWrite(5, state);
    state = not state;
  }
}

void onBeatDetected(){

  Serial.println("Beat!");
}
