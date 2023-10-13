/***************************************************************************
  This is a library for the BMP280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BMEP280 Breakout
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
***************************************************************************/

#include <Wire.h>
#include <DHT.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <ArduinoJson.h>

#define DHTPIN 2
#define DHTTYPE DHT11

DHT dht(DHTPIN,DHTTYPE);

Adafruit_BMP280 bmp; //I2C
StaticJsonDocument<200> doc;

// Replace the next variables with your SSID/Password combination
const char* ssid = "MQTT-RASP-AP";
const char* password = "mqtt_haw_2023";

// Add your MQTT Broker IP address, example:
const char* mqtt_server = "192.168.2.1";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[100];
int value = 0;

void setup() {
  Serial.begin(9600);

  //Serial.println("Initiating BPM");
  //init_bmp();
  //delay(1000);
  dht.begin();
  Serial.println("Initiating WiFi");
  init_WiFi();

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  //Pin sensor DHT_11
  pinMode(34,INPUT);

  //Pins de luces led
  pinMode(17,OUTPUT);
  pinMode(16,OUTPUT);
  pinMode(18,OUTPUT);
  pinMode(19,OUTPUT);
  pinMode(5,OUTPUT);
}

void init_WiFi() {
  Serial.println("Try Connecting to ");
  Serial.println(ssid);

  // Connect to your wi-fi modem
  WiFi.begin(ssid, password);

  // Check wi-fi is connected to wi-fi network
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected successfully");
  Serial.print("Got IP: ");
  Serial.println(WiFi.localIP());  //Show ESP32 IP on serial
}

void init_bmp() {
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");

  String messageTemp;
  for (int i = 0; i < length; i++) {
    messageTemp += (char)payload[i];
  }

  Serial.println(messageTemp);

  if (String(topic) == "test-mqtt"){
    if(messageTemp == "test"){
      client.publish("test-result","ok");  
    }
  }
  
  if (String(topic) == "leds"){

    String state = messageTemp.substring(0, 3);  // Los primeros 3 caracteres son el estado
    int lightId = messageTemp.substring(3).toInt(); // Los caracteres restantes son la ID

    Serial.print("state: ");
    Serial.println(state);
    state.trim();
    Serial.print("light_id: ");
    Serial.println(lightId);
    
    if(state == "ON"){
       digitalWrite(lightId,HIGH);
       delay(1000);
       Serial.println("Led ON");  
     } 
     if(state == "OFF"){
       digitalWrite(lightId,LOW);
       delay(1000);
       Serial.println("Led OFF");  
     }
   }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    
    // Attempt to connect
    Serial.print("Attempting MQTT connection...");
    
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe to testing topic
      client.subscribe("test-mqtt");
      
      // Subscribe to sensor topics
      client.subscribe("humidity");
      client.subscribe("temperature");
      client.subscribe("pressure");
      client.subscribe("air_quality");
      client.subscribe("light");
      
      // Subscribe to actuator topics
      client.subscribe("leds");
      client.subscribe("door");
      client.subscribe("ventilation");
      
    } else {
      
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {
  
  if (!client.connected()) {
    reconnect();
  }
  
  client.loop();
 
  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;

    doc.clear();
    JsonObject device = doc.createNestedObject("ESP32 MQTT CLIENT");
    JsonObject value = device.createNestedObject("value");

    int light_sensor_value = analogRead(34);
    value["light_sensor"] = light_sensor_value;
        
    float humidity = dht.readHumidity();
    value["act_humidity"] = humidity;

    float temperature_celcius = dht.readTemperature();
    value["act_temperature"] = temperature_celcius;

    String jsonStringHumidity;
    serializeJson(doc["ESP32 MQTT CLIENT"]["value"]["act_humidity"], jsonStringHumidity);
    
    client.publish("humidity", jsonStringHumidity.c_str());

    String jsonStringTemperature;
    serializeJson(doc["ESP32 MQTT CLIENT"]["value"]["act_temperature"], jsonStringTemperature);
    client.publish("temperature", jsonStringTemperature.c_str());

    String jsonStringLightValue;
    serializeJson(doc["ESP32 MQTT CLIENT"]["value"]["light_sensor"], jsonStringLightValue);
    client.publish("light", jsonStringLightValue.c_str());
    
    delay(500);
  }  
   
}
