#include "DHT.h"
#include <ESP8266WiFi.h>
// #include <DNSServer.h>
//#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include "Credentials.h" // contains WIFI_SSID and WIFI_PASSWORD (hidden away from git for obvious reasons)
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
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

// MQTT Topics
#define MQTT_PUB_LIGHT "homeassistant/sensor/nodemcu1/light"
#define MQTT_PUB_LIGHT_CONFIG "homeassistant/sensor/nodemcu1/light/config"
#define MQTT_PUB_TEMP "homeassistant/sensor/nodemcu1/temperature"
#define MQTT_PUB_TEMP_CONFIG "homeassistant/sensor/nodemcu1/temperature/config"
#define MQTT_PUB_HUM "homeassistant/sensor/nodemcu1/humidity"
#define MQTT_PUB_HUM_CONFIG "homeassistant/sensor/nodemcu1/humidity/config"
#define MQTT_PUB_MOTION "homeassistant/binary_sensor/nodemcu1/motion"
#define MQTT_PUB_MOTION_CONFIG "homeassistant/binary_sensor/nodemcu1/motion/config"
#define MQTT_SUB_LED "homeassistant/espall/LED_command"
#define MQTT_PUB_LED "homeassistant/espall/LED_state"

// Digital pin connected to the DHT sensor
#define DHTPIN 14
#define LEDPINWIFI 3
#define LEDPINMOTION 12
#define LIGHTSENSOR A0
#define PIRPIN 13 // motion sensor in 

// Uncomment whatever DHT sensor type you're using
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Initialize DHT sensor
DHT dht(DHTPIN, DHTTYPE);

// Variables to hold sensor readings
float temp;
float hum;
int light;
float light_avg;
//float light_outlier = 300;
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
bool intended_disconnect = false;
bool LED_ON = true;
int counter_disconnect = 0;
int counter_connect = 5;
const char hostname[10] = "ESPNode-1";
// Wifi and MQTT:
AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
WiFiManager wifiManager;
// Ticker wifiReconnectTimer;

// Time settings
unsigned long previousMillis_dht = 0;   // Stores last time temperature was published
unsigned long previousMillis_send = 0;
unsigned long previousMillis_forced = 0; // stores last time an hourly update was given (send values after an hour even when measurements were constant)
unsigned long previousMillis_light = 0 ;
unsigned long previousMillis_motion = 0 ;
unsigned long previousMillis_motion_detected = 0 ;
const long interval_dht = 10000;        // Interval at which to publish sensor readings
const long interval_light = 2000;
const long forced_interval = 600000; //every 10mins
const long interval_send = 60000;
const long interval_motion = 2000;
const long interval_nomotion = 60000;
unsigned long currentMillis = 1;


void publish_Config(){
  
  // temperature config
  StaticJsonDocument<300> tempdoc;   
  tempdoc["dev_cla"] = "temperature";
  tempdoc["unit_of_meas"]= "°C";
  tempdoc["name"] = "temperature_sensor_1";
  tempdoc["stat_t"] = MQTT_PUB_TEMP; 
  tempdoc["stat_cla"] = "measurement";
  
  // humidity config
  StaticJsonDocument<300> humdoc;
  humdoc["dev_cla"] = "humidity";
  humdoc["unit_of_meas"]= "%";
  humdoc["name"] = "humidity_sensor_1";
  humdoc["stat_t"] = MQTT_PUB_HUM; 
  humdoc["stat_cla"] = "measurement";

  // motion config
  StaticJsonDocument<300> motiondoc;
  
  motiondoc["dev_cla"] = "motion";
  motiondoc["name"] = "motion_sensor_1";
  motiondoc["stat_t"] = MQTT_PUB_MOTION; 

  // light config
  StaticJsonDocument<300> lightdoc;
  lightdoc["dev_cla"] = "illuminance";
  lightdoc["unit_of_meas"]= "mV"; 
  lightdoc["name"] = "light_sensor_1";
  lightdoc["stat_t"] = MQTT_PUB_LIGHT;
  lightdoc["stat_cla"]  = "measurement";
  
  
  // publish
  char buffer[300];
  serializeJson(humdoc,buffer);
  Serial.println("Buffer: ");
  Serial.println(buffer);
  uint16_t packetIdPub0 = mqttClient.publish(MQTT_PUB_HUM_CONFIG, 1, true, buffer);
  Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_HUM_CONFIG, packetIdPub0);
  Serial.printf("Message: %s \n", buffer);

  memset(buffer, 0, sizeof(buffer));

  serializeJson(motiondoc,buffer);
  packetIdPub0 = mqttClient.publish(MQTT_PUB_MOTION_CONFIG, 1, true, buffer);
  Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_MOTION_CONFIG, packetIdPub0);
  Serial.printf("Message: %s \n", buffer);

  memset(buffer, 0, sizeof(buffer));
  
  serializeJson(tempdoc,buffer);
  packetIdPub0 = mqttClient.publish(MQTT_PUB_TEMP_CONFIG, 1, true, buffer);
  Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_TEMP_CONFIG, packetIdPub0);
  Serial.printf("Message: %s \n", buffer);
  
  memset(buffer,0,sizeof(buffer));

  serializeJson(lightdoc,buffer);
  packetIdPub0 = mqttClient.publish(MQTT_PUB_LIGHT_CONFIG, 1, true, buffer);
  Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_LIGHT_CONFIG, packetIdPub0);
  Serial.printf("Message: %s \n", buffer);
  
}

void connectToWifi() {
  if (!intended_disconnect) {
    Serial.println("Connecting to Wi-Fi...");
    // WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    wifiManager.autoConnect("ESPConnect", AP_PASSWORD);
  }
}
void disconnectFromWifi() {
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
  if (!intended_disconnect) {
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

  Serial.printf("Subscribing to %s\n", MQTT_SUB_LED);
  uint16_t packetIdSub = mqttClient.subscribe(MQTT_SUB_LED, 1);
  publish_Config();
}


void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}
//
void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
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
}

void onMqttPublish(uint16_t packetId) {
  if (LED_ON) {
    digitalWrite(LEDPINWIFI, HIGH);
  }
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

//void onMqttSubscribe(uint16_t packetId) {
//  if(LED_ON){
//  digitalWrite(LEDPINWIFI,HIGH);
//  }
//  Serial.print("Subscribe acknowledged.");
//  Serial.print("  packetId: ");
//  Serial.println(packetId);
//}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
  char checkplon[3];
  char checkploff[4];
  char led_state[4];
  checkplon[2] = '\0';
  checkploff[3] = '\0';
  led_state[3] = '\0';
  Serial.printf("payload: %s\n", payload);
  strncpy(checkplon, payload, 2);
  strncpy(checkploff, payload, 3);


  Serial.printf("topic1: %s, topic2: %s, checkpayload1: %s, checkpayload2: %s\n", topic, MQTT_SUB_LED, checkplon, checkploff);

  if (strcmp(topic, MQTT_SUB_LED) == 0) {
    if (strcmp(checkplon, "ON") == 0) {
      LED_ON = true;
      led_state[2] = '\0';
      strncpy(led_state, payload, 2);
    }
    else if (strcmp(checkploff, "OFF") == 0) {
      LED_ON = false;
      strncpy(led_state, payload, 3);

    }
        
        uint16_t packetIdPub20 = mqttClient.publish(MQTT_PUB_LED, 0, false, led_state);
        Serial.printf("\nPublishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_LED, packetIdPub20);
        Serial.printf("Message: %s\n",led_state);
  }
}



void get_light_readings() {
  light = analogRead(LIGHTSENSOR) * (float)(3300 / 1024);
  delay(100);
//  if (fabs(light) > 2000) {
//    light_outlier = light;
////    light = light_old;
//  }
  light_avg = (light_avg * counter_light_avg + (float)light) / (counter_light_avg + 1);
  counter_light_avg += 1;
  Serial.printf("Measured light: %i, Average: %f [3.3/1024 V]\n", light, light_avg);
//  if (light_outlier != 300) {
//    Serial.printf("Last light outlier: %f\n", light_outlier);
//  }
  previousMillis_light = currentMillis;
}

void read_motion() {
  if (digitalRead(PIRPIN)) {
    Serial.printf("Motion Detected\n");
    if (LED_ON) {
      digitalWrite(LEDPINMOTION, HIGH);
    }
    if (motion_status != 1) {
      motion_status = 1;
      intended_disconnect = false;
      counter_disconnect = 0;
      if (!WiFi.isConnected()) {
        connectToWifi();
      }
      publishToMqttBroker_motionOnly();
    }

    previousMillis_motion_detected = currentMillis;
  }
  else {
    Serial.printf("No Motion\n");
    digitalWrite(LEDPINMOTION, LOW);
    if (currentMillis - previousMillis_motion_detected >= interval_nomotion) {
      if (motion_status != 0) {
        motion_status = 0;
        intended_disconnect = false;
        counter_disconnect = 0;
        if (!WiFi.isConnected()) {
          connectToWifi();
        }
        publishToMqttBroker_motionOnly();
      }
    }
  }
  Serial.printf("Motion status: %d\n", motion_status);
}

void get_dht_readings() {

  // Save the last time a new reading was published
  previousMillis_dht = currentMillis;
  // New DHT sensor readings
  hum = dht.readHumidity();
  // Read temperature as Celsius (the default)
  temp = dht.readTemperature();
  delay(100);

  temp_avg = (temp_avg * counter_dht_avg + temp) / (counter_dht_avg + 1);
  hum_avg = (hum_avg * counter_dht_avg + hum) / (counter_dht_avg + 1);

  if (isnan(temp_avg) || isnan(hum_avg)) {
    Serial.printf("dht error isnan. temp:%f, hum:%f,counter_dht_avg:%i\n", temp, hum, counter_dht_avg);
    temp_avg = temp_old;
    hum_avg = hum_old;
  }

  counter_dht_avg += 1;

  Serial.printf("Measured temp: %f, Average temp: %f, temp_old: %f, temp_sent: %f \nMeasured humidity: %f, Average Humidity: %f, hum_old: %f, hum_sent: %f\n", temp, temp_avg, temp_old, temp_sent, hum, hum_avg, hum_old, hum_sent);

}

void connect_intention() {
  if (fabs(temp_avg - temp_old) < 0.2 && fabs(hum_avg - hum_old) < 0.2 && fabs(light_avg - light_old) < 6) {
    if (counter_disconnect >= 5) {
      intended_disconnect = true;
      counter_disconnect = 0;
    }
    else {
      counter_disconnect += 1;
    }
    counter_connect = 0;
  }
  else {
    if (counter_connect >= 5) {
      intended_disconnect = false;
      counter_connect = 0;
      temp_old = temp_avg;
      hum_old = hum_avg;
      light_old = light_avg;
    }
    else {
      counter_connect += 1;
    }
    counter_disconnect = 0;
  }

  if (currentMillis - previousMillis_forced > forced_interval) { // forced regular updates
    intended_disconnect = false;
    counter_connect = 6;
    counter_disconnect = 0;
    previousMillis_forced = currentMillis;
    temp_sent = 200;
    hum_sent = 200;
    light_sent = 0;
    Serial.println("Sending forced Update...");
  }

  Serial.printf("\nIntended Disconnect: %d\nCounter Connect: %d\nCounter Disconnect: %d\n", intended_disconnect, counter_connect, counter_disconnect);

  if (intended_disconnect && WiFi.isConnected()) {
    temp_sent = 200;
    hum_sent = 200;
    light_sent = 0;
    Serial.println("No temperature Change. Disconnecting from wifi");
    disconnectFromWifi();
  }
  else if (!intended_disconnect && !WiFi.isConnected()) {
    //  wifiReconnectTimer.once(2, connectToWifi);
    connectToWifi();
  }
}

void publishToMqttBroker() {
  if (mqttClient.connected()) { // only attempt to send when connected
    // Publish an MQTT message light
    Serial.printf("Light avg send: %f\n", light_avg );
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_LIGHT, 0, false, String(light_avg).c_str());
    Serial.printf("\nPublishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_LIGHT, packetIdPub3);
    Serial.printf("Message: %.2f [mV]\n", light_avg);
    light_sent = light_avg;

    // Publish an MQTT message temperature
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 0, false, String(temp_avg).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_TEMP, packetIdPub1);
    Serial.printf("Message: %.2f \n", temp_avg);
    temp_sent = temp_avg;

    // Publish an MQTT message humidity

    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 0, false, String(hum_avg).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM, packetIdPub2);
    Serial.printf("Message: %.2f \n", hum_avg);
    hum_sent = hum_avg;

    // Publish an MQTT message motion
    if(motion_status==1){
    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_MOTION, 0, false, "ON");
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_MOTION, packetIdPub4);
    Serial.printf("Message: %s \n", "ON");
    }
    else{
    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_MOTION, 0, false, "OFF");
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_MOTION, packetIdPub4);
    Serial.printf("Message: %s \n", "OFF");

    }

  }
  else {
    Serial.printf("Not publishing because not connected to WiFi\n");
  }
}



void publishToMqttBroker_motionOnly() {
  if (mqttClient.connected()) { // only attempt to send when connected
    // Publish an MQTT message light

    if(motion_status==1){
    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_MOTION, 0, false, "ON");
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_MOTION, packetIdPub4);
    Serial.printf("Message: %s \n", "ON");
    }
    else{
    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_MOTION, 0, false, "OFF");
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_MOTION, packetIdPub4);
    Serial.printf("Message: %s \n", "OFF");

    }

  }
  else {
    Serial.printf("Not publishing because not connected to WiFi\n");
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println();
  pinMode(LEDPINWIFI, OUTPUT);
  pinMode(PIRPIN, INPUT);
  pinMode(LEDPINMOTION, OUTPUT);
  digitalWrite(LEDPINWIFI, LOW);
  dht.begin();
  //  WiFi.disconnect();
  //  WiFi.softAPdisconnect(true);
  //  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname);
  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
  wifiManager.setConnectTimeout(600);
//    wifiManager.resetSettings();// uncomment to reset stored wifi settings
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  mqttClient.setCredentials(BROKER_USER, BROKER_PASSWORD);

  connectToWifi();

  ArduinoOTA.onStart([](){
  Serial.println("Start OTA Upload\n"); 
});

ArduinoOTA.onEnd([](){
  Serial.println("\nEnd OTA Update\n"); 
});

ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r\n", (progress / (total / 100)));
  });
ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("\n OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
    Serial.println("\n");
  });
ArduinoOTA.setHostname(hostname);
ArduinoOTA.setPassword(OTAPASSWORD);
ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());


}

void loop() {
  ArduinoOTA.handle();
  currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds)
  // it publishes a new MQTT message
  if (currentMillis - previousMillis_light >= interval_light) {
    get_light_readings();
  }

  if (currentMillis - previousMillis_dht >= interval_dht) {
    get_dht_readings();
  }

  if (currentMillis - previousMillis_send >= interval_send) {

    previousMillis_send = currentMillis;
    // connect_intention(); // checks whether or not board should connect to wifi and subsequently calls for connect/disconnect
    publishToMqttBroker();

    counter_light_avg = 0;
    counter_dht_avg = 0;

  }
  if (currentMillis - previousMillis_motion >= interval_motion) {
    previousMillis_motion = currentMillis;
    read_motion();
  }


  else {
    digitalWrite(LEDPINWIFI, LOW);
  }

}
