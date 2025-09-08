#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Replace with your network credentials
const char* ssid = "Vodafone-C01140081";
const char* password = "SaveThePlanet@2023";
const char* mqtt_server = "192.168.1.9";

WiFiClient espClient;
PubSubClient client(espClient);

#define DHTPIN     19
#define DHTTYPE    DHT11

#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_WIDTH 128 // OLED display width, in pixels

const int photoresistorSensorPin = 18;
const int pirSensorPin = 5;
const int obstacleSensorPin = 4;

DHT dht(DHTPIN, DHTTYPE);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
long lastMsg;
const unsigned long updateInterval = 5000;

bool pirDetected = false;

int obstacleDetections = 0;
int lastStateObstacleSensor = HIGH;
int currentStateObstacleSensor;
unsigned long lastDetectionTime = 0;
const unsigned long debounceDelay = 200;

int page = 0;
long lastPageChange;
const unsigned long pageInterval = 10000;

float readDHTTemperature() {
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  //float t = dht.readTemperature(true);
  // Check if any reads failed and exit early (to try again).
  if (isnan(t)) {    
    Serial.println("Failed to read from DHT sensor!");
    return -1;
  }
  else {
    // Serial.println(t);
    return t;
  }
}

float readDHTHumidity() {
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  if (isnan(h)) {
    Serial.println("Failed to read from DHT sensor!");
    return -1;
  }
  else {
    // Serial.println(h);
    return h;
  }
}

int readPir() {
  int sensor_output = digitalRead(pirSensorPin);
  if(sensor_output == HIGH)
  {
    pirDetected = true;
  }
  return sensor_output;
}

int readPhotoresistor() {
  int photoresistorValue = digitalRead(photoresistorSensorPin);
  // Photoresistor LOW when luminosity goes over the threshold. Send a
  // 1 in this case, otherwise 0.
  if (photoresistorValue == LOW) {
    return 1;
  } else {
    return 0;
  }
}

void readObstacleIRModule() {
  currentStateObstacleSensor = digitalRead(obstacleSensorPin);
  if (lastStateObstacleSensor == HIGH && currentStateObstacleSensor == LOW) {
    unsigned long now = millis();
    if (now - lastDetectionTime > debounceDelay) {
      obstacleDetections++;
      lastDetectionTime = now;
    }
  }
  lastStateObstacleSensor = currentStateObstacleSensor;
}

void updateDisplay(StaticJsonDocument<80> doc) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  if (page == 0) {
    // linea orizzontale a metà
    display.drawLine(0, SCREEN_HEIGHT/2, SCREEN_WIDTH, SCREEN_HEIGHT/2, WHITE);
    // linea verticale a metà
    display.drawLine(SCREEN_WIDTH/2, 0, SCREEN_WIDTH/2, SCREEN_HEIGHT, WHITE);

    String temp = String(doc["t"]) + "C";
    String humidity = String(doc["h"]) + "%";
    String luminosity = doc["l"] == 1 ? "H" : "L";
    String motion = doc["pd"] == 1 ? "yes" : "no";

    // testo nel quadrante in alto a sinistra
    display.setCursor(5, 10);
    display.print("T: ");
    display.print(temp);

      // testo nel quadrante in alto a destra
    display.setCursor(SCREEN_WIDTH/2 + 5, 10);
    display.print("U: ");
    display.print(humidity);

    // testo nel quadrante in basso a sinistra
    display.setCursor(5, SCREEN_HEIGHT/2 + 10);
    display.print("Lum: ");
    display.print(luminosity);

    // testo nel quadrante in basso a destra
    display.setCursor(SCREEN_WIDTH/2 + 5, SCREEN_HEIGHT/2 + 10);
    display.print("Mot: ");
    display.print(motion);
  }
  else if (page == 1) {
    display.drawLine(0, SCREEN_HEIGHT/2, SCREEN_WIDTH, SCREEN_HEIGHT/2, WHITE);
    display.drawLine(SCREEN_WIDTH/2, 0, SCREEN_WIDTH/2, SCREEN_HEIGHT/2, WHITE);

    String noise = "3db"; // manca il sensore
    String obstacle = String(doc["od"]);

    display.setCursor(5, 10);
    display.print("Door: ");
    display.print(obstacle);

    display.setCursor(SCREEN_WIDTH/2 + 10, 10);
    display.print("N: ");
    display.print(noise);
  }

  display.display(); 
}

void setup() {
  delay(100);
  Serial.begin(115200);
  while (!Serial);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) 
  { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 30);

  // Display initial text
  display.println("Preparing...");
  display.display(); 
  delay(100);

  // Initialize DHT sensor
  dht.begin();

  pinMode(photoresistorSensorPin, INPUT);
  pinMode(pirSensorPin, INPUT);
  pinMode(obstacleSensorPin, INPUT);
  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Connect to MQTT server
  client.setServer(mqtt_server, 1883);

  Serial.println("PIR Warm Up Delay");
  delay(20000);
  Serial.println("Ready!");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }

  StaticJsonDocument<80> doc;
  char output[80];

  long now = millis();
  readPir();
  readObstacleIRModule();
  if (now - lastMsg > updateInterval) {
    lastMsg = now;
    float temp = readDHTTemperature();
    float humidity = readDHTHumidity();
    int luminosity = readPhotoresistor();

    int pirDetection;
    if (pirDetected) {
      // Serial.print("\nPerson detected by pir\n\n");
      pirDetected = false;
      pirDetection = 1;
    } else {
      pirDetection = 0;
    }

    // doc["t"] = temp;
    // doc["h"] = humidity;
    // doc["l"] = luminosity;
    // doc["pd"] = pirDetection;
    // doc["od"] = obstacleDetections;

    doc["t"] = 26.00;
    doc["h"] = 65.00;
    doc["l"] = 0;
    doc["pd"] = 0;
    doc["od"] = 15;

    obstacleDetections = 0;

    serializeJson(doc, output);

    String nowStr = String(now);         
    String outputStr = String(output);  
    Serial.println(nowStr + ": " + outputStr);

    updateDisplay(doc);

    client.publish("/home/sensors", output);
  }

  // check display page change
  if (now - lastPageChange > pageInterval) {
    page = (page + 1) % 2; // alternate between 0 and 1
    lastPageChange = now;
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}
