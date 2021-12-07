#include <ESP8266WiFi.h>
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient/releases/tag/v2.3
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>// https://github.com/bblanchon/ArduinoJson/releases/tag/v5.0.7

#define SEALEVELPRESSURE_HPA (1013.25)

//Adafruit_Si7021 sensor = Adafruit_Si7021();
//
//-------- Customise these values -----------
const char* ssid = "SSID";
const char* password = "PASS";
//long duration, distance;
//#define led1 D7
//#define led2 D5
//#define led3 D0
//#define trigPin D1
//#define echoPin D2
//unsigned char sent=0;
#define BUILTIN_LED D5
//unsigned char sent=0;
float latitude;
float longitude;
int fire;


#define ORG "ORGANIZATION ID"
#define DEVICE_TYPE "NAME OF THE DEVICE"
#define DEVICE_ID "ID OF THE DEVICE"
#define TOKEN "AUTHORIZED TOKEN"
//-------- Customise the above values --------

char server[] = ORG ".messaging.internetofthings.ibmcloud.com";
char authMethod[] = "use-token-auth";
char token[] = TOKEN;
char clientId[] = "d:" ORG ":" DEVICE_TYPE ":" DEVICE_ID;

const char eventTopic[] = "iot-2/evt/status/fmt/json";
const char cmdTopic[] = "iot-2/cmd/led/fmt/json";
static const int RX = D1, TX = D2;
static const uint32_t GPSBaud = 4800;
TinyGPSPlus gps;
SoftwareSerial ss(RX,TX);

WiFiClient wifiClient;
void callback(char* topic, byte* payload, unsigned int payloadLength) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < payloadLength; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if (payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}
PubSubClient client(server, 1883, callback, wifiClient);

int publishInterval = 1000; // millisec
long lastPublishMillis;

void setup() {
 //Serial.begin (9600);
   Serial.begin(115200);
    ss.begin(GPSBaud);
  // pinMode(led1,OUTPUT);
  // pinMode(led2,OUTPUT);
  // pinMode(led3,OUTPUT);
//  pinMode(trigPin, OUTPUT);
  //pinMode(echoPin, INPUT);
 //Serial.begin(9600);
  Serial.println();
  client.connect(clientId, authMethod, token);
  //  sensor.begin();
  pinMode(BUILTIN_LED, OUTPUT);
  wifiConnect();
  mqttConnect();
}

void loop() {
 while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }
  if (millis() - lastPublishMillis > publishInterval) {
    publishData();
    lastPublishMillis = millis();
  }

  if (!client.loop()) {
    mqttConnect();
  }
}

void wifiConnect() {
  Serial.print("Connecting to "); Serial.print(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("nWiFi connected, IP address: "); 
  Serial.println(WiFi.localIP());

}

void mqttConnect() {

  if (!!!client.connected()) {
    Serial.print("Reconnecting MQTT client to "); Serial.println(server);
    while (!!!client.connect(clientId, authMethod, token)) {
      Serial.print(".");
      delay(500);
    }
    if (client.subscribe(cmdTopic)) {
      Serial.println("subscribe to responses OK");
    } else {
      Serial.println("subscribe to responses FAILED");
    }
    Serial.println();
  }
}
//-------------------------------PUSH THE DATA--------------------------------------------------------------
void publishData() {
  StaticJsonDocument<200> doc;

  // create an object
  JsonObject payload = doc.to<JsonObject>();
  //sensorreading();
  //firesensoreading();
  char name1[10] = "bin_1";
  int distance = 32;
  int temp = 7;
  payload["name1"] = name1;
  payload["distance"] = distance;
  payload["fire"] = fire;
  payload["temp"] = temp;
  payload["latitude"] = latitude;
  payload["longitude"] = longitude;
  serializeJson(doc, Serial);
  String buffString;
  serializeJson(doc, buffString);
  Serial.print("Sending payload: ");
  //  Serial.println(payload);

  if (client.publish(eventTopic, (char*) buffString.c_str())) {
    Serial.println("Publish OK");
  } else {
    Serial.println("Publish FAILED");
  }
}
//-----------------------------------ULTRASONICSENSOR-------------------------------------------------------
//void sensorreading()
//{
//  digitalWrite(trigPin, LOW);
//  delayMicroseconds(2);
//  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(trigPin, LOW);
//  duration = pulseIn(echoPin, HIGH);
//  distance = (duration / 2) / 29.1;
//  Serial.print(distance);
//  Serial.println(" cm");
//     if (distance <= 30 )
//    {
//      digitalWrite(led1,HIGH);
//      Serial.println("bin is empty");
//    }
//    else
//    {
//      digitalWrite(led1,LOW);
//    }
//     if (distance < 20 )
//    {
//      digitalWrite(led2,HIGH);
//      Serial.println("bin is medium");
//    }
//    else
//    {
//      digitalWrite(led2,LOW);
//    }
//    if (distance < 10 && sent==0)
//    {
//      digitalWrite(led3,HIGH);
//      sent=1;
//    }
//    else
//    {
//      digitalWrite(led3,LOW);
//    }
//    if(distance > 30 || distance <= 0)
//    {
//      Serial.println("out of range");
//    }
//   else
//    {
// Serial.print(distance);
//    Serial.println(" cm");
//}
//-------------------------------------------------GPS-------------------------------------------------------
void displayInfo()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    latitude = gps.location.lat();
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
    longitude = gps.location.lng();
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}
void firesensoreading()
{
  int analogSensor = analogRead(smokeA0);

  Serial.print("Pin A0: ");
  Serial.println(analogSensor);
  if (analogSensor > sensorThres)
  {
    digitalWrite(fireled, HIGH);
     fire = 1;  
  }
else
{  
  digitalWrite(fireled,LOW);
  fire = 0;
  }
}
