/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp8266-nodemcu-mqtt-publish-dht11-dht22-arduino/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include "DHT.h"
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include "Credentials.h" // contains WIFI_SSID and WIFI_PASSWORD (hidden away from git for obvious reasons)

// Wifi Credentials
#ifndef WIFI_SSID
#define WIFI_SSID "Your wifi SSID here"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "rchtsdrndmlkzr"
#endif


// Broker credentials (comment away if  you don't need a broker) Lower in this file
#ifndef BROKER_USER
#define BROKER_USER "your broker username"
#endif

#ifndef BROKER_PASSWORD
#define BROKER_PASSWORD "your broker password" 
#endif

// Raspberri Pi Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(192, 168, 178, 44)
// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1883

// Temperature MQTT Topics
#define MQTT_PUB_TEMP "homeassistant/dht/temperature"
#define MQTT_PUB_HUM "homeassistant/dht/humidity"

// Digital pin connected to the DHT sensor
#define DHTPIN 14  
#define LEDPIN 3
// Uncomment whatever DHT sensor type you're using
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)   

// Initialize DHT sensor
DHT dht(DHTPIN, DHTTYPE);

// Variables to hold sensor readings
float temp;
float hum;

bool intended_disconnect=false;

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}
void disconnectFromWifi(){
  Serial.println("Disconnecting from Wi-Fi..."); 
  WiFi.mode(WIFI_OFF);
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  if(!intended_disconnect){
  wifiReconnectTimer.once(2, connectToWifi);
  }
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

/*void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}*/

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  pinMode(LEDPIN,OUTPUT);
  digitalWrite(LEDPIN,LOW);
  dht.begin();
  
  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  mqttClient.setCredentials(BROKER_USER, BROKER_PASSWORD);
  
  connectToWifi();
}

void loop() {
  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds) 
  // it publishes a new MQTT message
  if ((currentMillis - previousMillis >= interval)&& mqttClient.connected()) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    // New DHT sensor readings
    hum = dht.readHumidity();
    // Read temperature as Celsius (the default)
    temp = dht.readTemperature();
    // Read temperature as Fahrenheit (isFahrenheit = true)
    //temp = dht.readTemperature(true);

    if(mqttClient.connected()){ // only attempt to send when connected
      digitalWrite(LEDPIN,HIGH);
      // Publish an MQTT message on topic esp/dht/temperature
      uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temp).c_str());                            
      Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_TEMP, packetIdPub1);
      Serial.printf("Message: %.2f \n", temp);
  
      // Publish an MQTT message on topic esp/dht/humidity
      uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 1, true, String(hum).c_str());                            
      Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM, packetIdPub2);
      Serial.printf("Message: %.2f \n", hum);
//      intended_disconnect=true;
      }
    
  }
  else if(digitalRead(LEDPIN)){
      digitalWrite(LEDPIN,LOW);
    }
  if(intended_disconnect && WiFi.isConnected()){
    disconnectFromWifi();
    intended_disconnect=false;
  }
}
