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
// #include <DNSServer.h>
//#include <ESP8266WebServer.h>
#include <WiFiManager.h>  
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include "Credentials.h" // contains WIFI_SSID and WIFI_PASSWORD (hidden away from git for obvious reasons)

// Wifi Credentials
#ifndef __CREDENTIALS_H
#define AP_PASSWORD "Your accesspoint password  here"

// Broker credentials (comment away if  you don't need a broker) Lower in this file
#define BROKER_USER "your broker username"
#define BROKER_PASSWORD "your broker password" 
#endif

// Raspberri Pi Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(192, 168, 178, 44)
// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1883

// Temperature MQTT Topics
#define MQTT_PUB_TEMP "homeassistant/esp/temperature"
#define MQTT_PUB_HUM "homeassistant/esp/humidity"
#define MQTT_PUB_LIGHT "homeassistant/esp/light"
#define MQTT_PUB_MOTION "homeassistant/esp/motion"

// Digital pin connected to the DHT sensor
#define DHTPIN 14  
#define LEDPINWIFI 3
#define LEDPINMOTION 12
#define LIGHTSENSOR A0
#define PIRPIN 13 // motion sensor in 

// Uncomment whatever DHT sensor type you're using
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)   

// Initialize DHT sensor
DHT dht(DHTPIN, DHTTYPE);

// Variables to hold sensor readings
float temp;
float hum;
int light; 
float light_avg;
float light_outlier=300;
int counter_light_avg = 0;
float temp_old;
float temp_avg;
float hum_avg;
float hum_old;
int counter_dht_avg = 0;
float light_old;
float light_sent;
float temp_sent;
float hum_sent;
int motion_status = 3; 
// Variables for controlinig intentional disconnect
bool intended_disconnect=false;
int counter_disconnect=0;
int counter_connect=5;

// Wifi and MQTT:
AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
WiFiManager wifiManager;
// Ticker wifiReconnectTimer;

// Time settings
unsigned long previousMillis_dht = 0;   // Stores last time temperature was published
unsigned long previousMillis_send=0;
unsigned long previousMillis_forced =0; // stores last time an hourly update was given (send values after an hour even when measurements were constant)
unsigned long previousMillis_light = 0 ;
unsigned long previousMillis_motion =0 ;
unsigned long previousMillis_motion_detected = 0 ; 
const long interval_dht = 10000;        // Interval at which to publish sensor readings
const long interval_light = 2000;
const long forced_interval = 600000; //every 10mins
const long interval_send = 60000;
const long interval_motion = 2000;
const long interval_nomotion = 60000;
unsigned long currentMillis= 1;

void connectToWifi() {
  if(!intended_disconnect){
  Serial.println("Connecting to Wi-Fi...");
  // WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  wifiManager.autoConnect("ESPConnect",AP_PASSWORD);
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
  // wifiReconnectTimer.once(2, connectToWifi);
  connectToWifi();
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
  digitalWrite(LEDPINWIFI,HIGH);
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void get_light_readings(){
  light = analogRead(LIGHTSENSOR)*(3300/1024);
  delay(100);
  if(fabs(light)>2000){
    light_outlier=light;
    light=light_old;
    }
  light_avg=(light_avg*counter_light_avg+(float)light)/(counter_light_avg+1);
  counter_light_avg+=1;  
  Serial.printf("Measured light: %i, Average: %f [3.3/1024 V]\n",light,light_avg);
  if(light_outlier!=300){
    Serial.printf("Last light outlier: %f\n",light_outlier);
    }
  previousMillis_light=currentMillis;
}

void read_motion(){
  if(digitalRead(PIRPIN)){
    Serial.printf("Motion Detected\n");
    digitalWrite(LEDPINMOTION,HIGH);
    if(motion_status!=1){
      motion_status=1;
      publishToMqttBroker();
    }
    
    previousMillis_motion_detected=currentMillis;
  }
  else{
    Serial.printf("No Motion\n");
    digitalWrite(LEDPINMOTION,LOW);
    if(currentMillis-previousMillis_motion_detected>= interval_nomotion){
      motion_status=0;
    }
  }
  Serial.printf("Motion status: %d\n",motion_status);
}

void get_dht_readings(){
   
    // Save the last time a new reading was published
    previousMillis_dht = currentMillis;
    // New DHT sensor readings
    hum = dht.readHumidity();
    // Read temperature as Celsius (the default)
    temp = dht.readTemperature();
    delay(100);

    temp_avg=(temp_avg*counter_dht_avg+temp)/(counter_dht_avg+1);
    hum_avg=(hum_avg*counter_dht_avg+hum)/(counter_dht_avg+1);

    if(isnan(temp_avg)||isnan(hum_avg)){
      Serial.printf("dht error isnan. temp:%f, hum:%f,counter_dht_avg:%i\n",temp,hum,counter_dht_avg);
      temp_avg=temp_old;
      hum_avg=hum_old;
    }

    counter_dht_avg+=1;
    
    Serial.printf("Measured temp: %f, Average temp: %f, temp_old: %f, temp_sent: %f \nMeasured humidity: %f, Average Humidity: %f, hum_old: %f, hum_sent: %f\n",temp,temp_avg,temp_old,temp_sent,hum,hum_avg,hum_old,hum_sent);
    
}

void connect_intention(){
  if(fabs(temp_avg-temp_old)<0.2 && fabs(hum_avg-hum_old)<0.2 && fabs(light_avg-light_old)<6){
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
      temp_old=temp_avg;
      hum_old=hum_avg;
      light_old=light_avg;          
    }
    else{
      counter_connect+=1;
    }
    counter_disconnect=0;   
    }

  if(currentMillis-previousMillis_forced>forced_interval){ // forced regular updates
      intended_disconnect=false;
      counter_connect=6;
      counter_disconnect=0;
      previousMillis_forced=currentMillis;
      temp_sent=200;
      hum_sent=200;
      light_sent=0;
      Serial.println("Sending forced Update...");
    }

  Serial.printf("\nIntended Disconnect: %d\nCounter Connect: %d\nCounter Disconnect: %d\n",intended_disconnect,counter_connect,counter_disconnect);

  if(intended_disconnect && WiFi.isConnected()){
    temp_sent=200;
    hum_sent=200;
    light_sent=0;
    Serial.println("No temperature Change. Disconnecting from wifi");
    disconnectFromWifi();
  }
  else if(!intended_disconnect && !WiFi.isConnected()){
    //  wifiReconnectTimer.once(2, connectToWifi);
     connectToWifi();
  }
}

void publishToMqttBroker(){
  if(mqttClient.connected()){ // only attempt to send when connected
         // Publish an MQTT message light
        Serial.printf("Light avg send: %f, transformed: %f\n",light_avg,light_avg*((float)3300/(float)1024));
        uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_LIGHT, 1, true, String(light_avg*((float)3300/(float)1024)).c_str());                            
        Serial.printf("\nPublishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_LIGHT, packetIdPub3);
        Serial.printf("Message: %.2f [mV]\n", light_avg*((float)3300/(float)1024));
        light_sent=light_avg;
        
        // Publish an MQTT message temperature
        uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temp_avg).c_str());                            
        Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_TEMP, packetIdPub1);
        Serial.printf("Message: %.2f \n", temp_avg);
        temp_sent=temp_avg;
        
        // Publish an MQTT message humidity
        
        uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 1, true, String(hum_avg).c_str());                            
        Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM, packetIdPub2);
        Serial.printf("Message: %.2f \n", hum_avg);
        hum_sent=hum_avg;

        // Publish an MQTT message motion
        
        uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_MOTION, 1, true, String(motion_status).c_str());                            
        Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_MOTION, packetIdPub4);
        Serial.printf("Message: %d \n", motion_status);
        
      }
  else{
    Serial.printf("Not publishing because not connected to WiFi\n");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  pinMode(LEDPINWIFI,OUTPUT);
  pinMode(PIRPIN,INPUT);
  pinMode(LEDPINMOTION,OUTPUT);
  digitalWrite(LEDPINWIFI,LOW);
  dht.begin();
//  WiFi.disconnect();
//  WiFi.softAPdisconnect(true);
//  WiFi.mode(WIFI_STA);
  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
  wifiManager.setConnectTimeout(600);
//  wifiManager.resetSettings();// uncomment to reset stored wifi settings
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
  currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds) 
  // it publishes a new MQTT message
  if(currentMillis-previousMillis_light >= interval_light){
    get_light_readings();
  }

  if (currentMillis - previousMillis_dht >= interval_dht) {
    get_dht_readings();
  }

  if (currentMillis-previousMillis_send >= interval_send){
      
      previousMillis_send=currentMillis;
      connect_intention(); // checks whether or not board should connect to wifi and subsequently calls for connect/disconnect
      publishToMqttBroker();   

    counter_light_avg=0;
    counter_dht_avg=0;
    
  }
  if(currentMillis-previousMillis_motion >= interval_motion){
    previousMillis_motion=currentMillis;
    read_motion();

    

  }


  else if(digitalRead(LEDPINWIFI)){
      digitalWrite(LEDPINWIFI,LOW);
    }

}
