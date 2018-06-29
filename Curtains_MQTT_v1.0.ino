/*
 * Author: Jonathan Caes
 * Subject: This code is written to control the stepper motor attached to an IKEA VIDGA curtain slider system via MQTT and OpenHAB.
 */
 
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <MQTTClient.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <AccelStepper.h>


const char* ssid = "Your Network SSID";
const char* password = "your Network Password";

const char* server = "MQTT server ip-address";
const char* mqttDeviceID = "MQTT device ID";                      //i.e.: BedroomCurtains

char* outTopic1 = "Outgoing topic for temperature";               //Outgoing topic for the temperature sensor on the PCB || i.e.: Home/Temp/Bedroom1
char* outTopic2 = "Outgoing topic for curtains";                  //Outgoing topic for the curtains || i.e.: Home/Curtains/Bedroom1

char* subscribeTopic1 = "Incoming topic for curtains";            //Incoming topic for the curtains, the same as the outgoing topic for the curtains || i.e.: Home/Curtains/Bedroom1 
char* subscribeTopic2 = "Incoming topic for OTA support";         //Incoming topic for OTA (Over The Air) support || i.e.: Home/Curtains/Bedroom1OTA
char* subscribeTopic3 = "Incoming topic to set max travel value"; //Incoming topic to set max travel distance || i.e.: Home/Curtains/Bedroom1SetMax
char* subscribeTopic4 = "Incoming topic to set start point";      //Incoming topic to set starting point || i.e.: Home/Curtains/Bedroom1SetMin
                                                                  //To set the starting point, send command "SET" to this topic

IPAddress ip(192,168,0,26);         //the desired IP Address
IPAddress gateway(192,168,0,1);     //set gateway to match your network
IPAddress subnet(255,255,255,0);    //set subnet mask to match your network

WiFiClient net;
MQTTClient client;

#define FULLSTEP 4
struct MotorPin {
  byte blue;      //Blue = IN1
  byte pink;      //Pink = IN2
  byte yellow;    //Yellow = IN3
  byte orange;    //Orange = IN4
};
MotorPin motorPin = {16, 14, 12, 13};         //The pins on which the motor is attached
long targetPosition = 5000;                   //The initialised maximum travel distance
long currentPosition;
AccelStepper Motor1(FULLSTEP, motorPin.blue, motorPin.yellow, motorPin.pink, motorPin.orange);
bool motorOn = true;

bool update_req = false;
bool start = true;
unsigned long Lasttime = 0;

#define TIME_BETWEEN_READING 300              //Time between temperature readings || Time in seconds (300s = 5min)
float temp;                                   //Stores temperature value
String s_temp;
char messTemp[10];
OneWire  ds(5);                               //DS18b20 on gpio5
DallasTemperature dsTemp(&ds);

int CurPos;
String s_CurPos;
char messCurPos[10];

#define buttonOPEN    10              //Pins on which the buttons are attached
#define buttonSTOP    4
#define buttonCLOSE   0  

void setup() {
  Serial.begin(115200);
  Serial.println("Start...");

  pinMode(buttonOPEN, INPUT);
  pinMode(buttonSTOP, INPUT);
  pinMode(buttonCLOSE, INPUT);

  Motor1.setMaxSpeed(700);            //The motor speed
  Motor1.setAcceleration(5000);       //The motor acceleration 
  
  connect();
  OTA_init();
}

void loop() {
  client.loop();
  
  if (!client.connected() || WiFi.status() != WL_CONNECTED) {
    connect();
  }
  if(update_req) {
    ArduinoOTA.handle();
  }
  if(!update_req && ((!motorOn && !start && ((millis() - Lasttime) > (TIME_BETWEEN_READING*1000))) || (start && (millis() > 30000) && !motorOn))) {
    dsTemp.requestTemperatures();
    if(start) 
      delay(2000);
    temp = dsTemp.getTempCByIndex(0);
    String s_temp = String(temp);
    s_temp.toCharArray(messTemp, s_temp.length()+1);
    client.publish(outTopic1, messTemp, true, 0);
    start = false;
    Lasttime = millis();
    Serial.println(temp);
  }

  Motor1.run();
  if((Motor1.distanceToGo() == 0) && motorOn) {
    Motor1.disableOutputs();
    motorOn = false;
  }

  int button1 = digitalRead(buttonOPEN);
  int button2 = digitalRead(buttonSTOP);
  int button3 = digitalRead(buttonCLOSE);
  if(!button1) {
    Motor1.moveTo(0); 
    Serial.println("OPEN");
    client.publish(outTopic2, "UP", true, 0);
    motorOn = true; 
    delay(200); 
  }
  else if(!button3) {
    Motor1.moveTo(-targetPosition); 
    Serial.println("CLOSED");
    client.publish(outTopic2, "DOWN", true, 0);
    motorOn = true;
    delay(200);
  }
  else if(!button2) {
    Motor1.stop();
    Motor1.disableOutputs();
    delay(1000);
    currentPosition = Motor1.currentPosition();
    CurPos = map(currentPosition, 0, -targetPosition, 0, 100);
    Serial.print("Pos %: "); Serial.println(CurPos);
    s_CurPos = String(CurPos);
    s_CurPos.toCharArray(messCurPos, s_CurPos.length()+1);
    client.publish(outTopic2, messCurPos, true, 0);
  }
  
}

void messageReceived(String &topic, String &payload) {
  String msgTopic = topic;
  String msgString = payload;
  Serial.println(msgTopic);
  Serial.println(msgString);

  if(msgTopic == subscribeTopic1) {
    if(msgString == "UP") {
      Motor1.moveTo(0); 
      Serial.println("OPEN");
      motorOn = true;
    }
    else if(msgString == "DOWN") {
      Motor1.moveTo(-targetPosition); 
      Serial.println("CLOSED");
      motorOn = true;
    }
    else if(msgString == "STOP") {
      Motor1.stop();
      Motor1.disableOutputs();
      currentPosition = Motor1.currentPosition();
      CurPos = map(currentPosition, 0, -targetPosition, 0, 100);
      Serial.print("Pos %: "); Serial.println(CurPos);
      s_CurPos = String(CurPos);
      s_CurPos.toCharArray(messCurPos, s_CurPos.length()+1);
      client.publish(outTopic2, messCurPos, true, 0);
    }
  }

  else if(msgTopic == subscribeTopic2) {
    if(msgString == "ON") {
      update_req = true;
    }
    else if(msgString == "OFF") {
      update_req = false;
    }
  }

  else if(msgTopic == subscribeTopic3) {
    targetPosition = msgString.toInt();
    Serial.print("New Maximum set: ");
    Serial.println(targetPosition);
  }

  else if(msgTopic == subscribeTopic4) {
    if(msgString == "SET") {
      Motor1.setCurrentPosition(0);
      Serial.println("New minimum set");
    }
  }
}

void connect() {
  Serial.println("Connecting...");
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  delay(500);
  WiFi.config(ip, gateway, subnet);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  client.begin(server, net);
  client.onMessage(messageReceived);

  for(int i=0; i<30; i++)
  {
    delay(300);
    if(WiFi.status() == WL_CONNECTED)
    {
      break;
    }
  }

  for(int i=0; i<5; i++)
  {
    delay(200);
    if(client.connect(mqttDeviceID))
    {
      break;
    }
  }
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected...");
  }
  client.subscribe(subscribeTopic1);            //These are the topics you want to subscribe to
  client.subscribe(subscribeTopic2);
  client.subscribe(subscribeTopic3);
  client.subscribe(subscribeTopic4);
}

void OTA_init() {
  //For more info, look at BasicOTA_Test
  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(mqttDeviceID);

  // No authentication by default
  ArduinoOTA.setPassword("OTA Password");       //This is the password you have to type in when arduino IDE uploads code OTA

  ArduinoOTA.begin();
}


