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
float temp_old;
float hum_old;
float temp_sent;
float hum_sent;
bool intended_disconnect=false;
int counter_disconnect=0;
int counter_connect=5;
AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
unsigned long previousMillis_hourly =0; // stores last time an hourly update was given (send values after an hour even when measurements were constant)
const long interval = 10000;        // Interval at which to publish sensor readings

void connectToWifi() {
  if(!intended_disconnect){
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
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
  digitalWrite(LEDPIN,HIGH);
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
  if (currentMillis - previousMillis >= interval) {
    Serial.printf("Counter Connect: %d\nCounter disconnect: %d\n",counter_connect,counter_disconnect);
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    // New DHT sensor readings
    hum = dht.readHumidity();
    // Read temperature as Celsius (the default)
    temp = dht.readTemperature();
    Serial.printf("Measured temp: %f, temp_old: %f, temp_sent: %f \nMeasured humidity: %f, hum_old: %f, hum_sent: %f\n",temp,temp_old,temp_sent,hum,hum_old,hum_sent);
    // Read temperature as Fahrenheit (isFahrenheit = true)
    //temp = dht.readTemperature(true);
      if(fabs(temp-temp_old)<0.1 && fabs(hum-hum_old)<0.1){
        if(counter_disconnect>=5){
          
          intended_disconnect=true;
          counter_disconnect=0;
          
         
        }
        else{
          counter_disconnect+=1;
          
        }
        counter_connect=0;
        }
      else{
        
        if(counter_connect >= 5){          
          intended_disconnect=false;
          counter_connect=0;
          temp_old=temp;
          hum_old=hum;
          
          
        }
        else{
          counter_connect+=1;
        }
        counter_disconnect=0;

       
       }
      if(mqttClient.connected()){ // only attempt to send when connected
        
        // Publish an MQTT message on topic esp/dht/temperature
        if(temp!=temp_sent){
        uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temp).c_str());                            
        Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_TEMP, packetIdPub1);
        Serial.printf("Message: %.2f \n", temp);
        temp_sent=temp;
        }
        // Publish an MQTT message on topic esp/dht/humidity
        if(hum!=hum_sent){
        uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 1, true, String(hum).c_str());                            
        Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM, packetIdPub2);
        Serial.printf("Message: %.2f \n", hum);
        hum_sent=hum;
        }
      }
       
    Serial.printf("Intended Disconnect: %d\n",intended_disconnect);

    

    if(intended_disconnect && WiFi.isConnected()){
    temp_sent=200;
    hum_sent=200;
    Serial.println("No temperature Change. Disconnecting from wifi");
    disconnectFromWifi();
  }
  else if(!intended_disconnect && !WiFi.isConnected()){
     wifiReconnectTimer.once(2, connectToWifi);
  }
  // Send an hourly update whether or not the measurements didn't change
    if(currentMillis-previousMillis_hourly>3.6e6){
      intended_disconnect=false;
      counter_connect=6;
      counter_disconnect=0;
      previousMillis_hourly=currentMillis;
      temp_sent=200;
      hum_sent=200;
      Serial.println("Sending Hourly Update...");
    }
  }
  else if(digitalRead(LEDPIN)){
      digitalWrite(LEDPIN,LOW);
    }
  
}
