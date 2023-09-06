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
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <ArduinoJson.h>

Adafruit_BMP280 bmp; //I2C
StaticJsonDocument<200> doc;

float temperature = 0;
float pressure = 0;

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

  Serial.println("Initiating BPM");
  init_bmp();
  Serial.println("Initiating WiFi");
  init_WiFi();

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  delay(100);
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

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");

  String messageTemp;
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }

  Serial.println(messageTemp);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
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
    JsonObject device = doc.createNestedObject("BMP280");
    JsonObject status = device.createNestedObject("status");

    float temperature = bmp.readTemperature();
    status["temperature"] = temperature;

    float pressure = bmp.readPressure() / 100.0;
    status["pressure"] = pressure;

    String jsonString;
    serializeJson(doc, jsonString);

    client.publish("homewizard_mqtt", jsonString.c_str());
  }
}