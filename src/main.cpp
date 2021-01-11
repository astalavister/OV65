#include <Arduino.h>
#include "secrets.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include "ACS712.h"
#include "RTClib.h"
#include <Bounce2.h>
#include <eeprom.h>
#include "DHT.h"
#include <BfButton.h>
//ESP32
#include "WiFi.h"
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <PubSubClient.h>
#include <time.h>
#include <sys/time.h>
extern "C" 
{
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

//#include "webpages.cpp"
extern const char* loginIndex;
extern const char* serverIndex;

#include "lcdletters.h"



void mqttPrintStatus();

WebServer server(80);
const char* host = "ov65";

const char * mqttHost = "mqtt.zabmail.ru";
char cstr[128]; //temp buf 

//#define MQTT_HOST IPAddress(45, 95, 184, 155)
#define MQTT_PORT 1883

AsyncMqttClient mqttClient;

TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;


EEPROMClass eData("eeprom0", 0x500);

const char* ntpServer = "ru.pool.ntp.org";
const long  gmtOffset_sec = 32400;
const int   daylightOffset_sec = 0;


DateTime dtStartIgnition;
DateTime dtStartHalfMotor;
DateTime dtStopIgnition;
DateTime sparkStartTime;
struct tm * timenow_tm = new tm();
DateTime  timeFreeRun;
DateTime timenow;


int wifiConnectedCount = 0; //wifi connected times counter

String sparkTime="  ";

bool timeAdjusted = false;

//By default, on ESP32 boards, the SDA pin is 21 . SCL pin is 22
#define DHT_PIN 23 //GPIO23 //20 not useable 
//Temperarure sensor
//#define DHTTYPE DHT21   // AM2301 
DHT dht(DHT_PIN,AM2301);
float CurrentTemp = -10.99;            // Глобальная переменная для хранения значение температуры с датчика DS18B20
float CurrentHum = 99.99;            // Глобальная переменная для хранения значение температуры с датчика DS18B20
byte neededTemp = 20;
//#define DS_PIN A7            // DS18B20 data pin //35ESP
#define CURRENT_IGN_PIN 34   //Current meter pin //34ESP
//#define BEEP_PIN 12              //Beeper
//#define BEEP_PIN_GROUND 13       //Beeper Ground
#define MOTOR_RELAY_PIN 15       //motor on/off relay //25
#define MOTOR_SPEED_RELAY_PIN 19 //motor speed relay (1/2) //26
#define FUEL_VALVE_RELAY_PIN 0   // fuel valve relay 
#define IGNITION_RELAY_PIN 16     // ingition relay //14
///KEYBOARD
#define BUTTON_A_PIN 32     
#define BUTTON_M_PIN 33
#define BUTTON_L_PIN 25     
#define BUTTON_R_PIN 14     
#define BUTTON_P_PIN 26     
#define BUTTON_HOT_PIN 18
#define BUTTON_ALARM_PIN 17
#define BUTTON_GROUND_PIN 27

BfButton btnA(BfButton::STANDALONE_DIGITAL, BUTTON_A_PIN);
BfButton btnM(BfButton::STANDALONE_DIGITAL, BUTTON_M_PIN);
BfButton btnL(BfButton::STANDALONE_DIGITAL, BUTTON_L_PIN);
BfButton btnR(BfButton::STANDALONE_DIGITAL, BUTTON_R_PIN);
BfButton btnP(BfButton::STANDALONE_DIGITAL, BUTTON_P_PIN);

#define NUM_BUTTONS 2 //HOT and OVERHEAT Inputs
const uint8_t BUTTON_PINS[NUM_BUTTONS] = {BUTTON_HOT_PIN, BUTTON_ALARM_PIN}; //ESP
Bounce * buttons = new Bounce[NUM_BUTTONS];

//ACS
//ACS712 ACS(CURRENT_IGN_PIN, 1, 4095, 33); //30 AMPERS sensor


ACS712 sensor(ACS712_30A, CURRENT_IGN_PIN);
long lastAcsUpdateTime;
const int ACS_UPDATE_TIME = 2000; // Определяем периодичность проверок



enum WorkMode
{
  ModeManual,
  ModeAuto
};
enum StartProcessStage
{
  IdleToStart,
  StartFuel, 
  StartIgnition, //wait 20 seconds
  StartHalfMotor, //wait 30 seconds
  StopIgnition, //wait for FIRE sensor
  StartFullMotor, 
  UnknownStartStage,
};
enum StopProcessStage
{
  HeaterStarted,
  StopFuel, 
  WaitToStopFire, //wait for sensor
  StopMotor,
  IdleStopped,
  UnknownStopStage,
};

WorkMode currentMode = ModeManual;
StartProcessStage currentStartStage = IdleToStart;
StopProcessStage currentStopStage = IdleStopped;

StartProcessStage currentStartStagePrev = UnknownStartStage;
StopProcessStage currentStopStagePrev = UnknownStopStage;


bool IsAlarm = false;
bool IsDeviceStopped = false;
bool IsNoIgnition = false;
bool IsFired = false;
bool IsIdle = true;
bool IgnoreSparkCurrent = true;

// set the LCD address to 0x27 for a 20 chars and 4 line display
LiquidCrystal_I2C display(0x27, 20, 4);
long lastLcdUpdateTime = 20000;  // Переменная для хранения времени ПОКАЗА НА lcd
const int LCD_UPDATE_TIME = 500; // Определяем периодичность ПОКАЗА НА lcd

bool curMotorRelayState = false;
int MotorSpeed = 1;
bool SparkRelayState = false;
int SparkCurrent = 0;
bool FuelRelayState = false;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        randomSeed(micros());
        wifiConnectedCount++;
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
		    xTimerStart(wifiReconnectTimer, 0);
        break;
    }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  
 // mqttClient.subscribe("iot/OV65/temperature", 2);
//  mqttClient.subscribe("iot/OV65/humidity", 2);
  mqttClient.subscribe("iot/OV65/startheater", 2);
  mqttClient.subscribe("iot/OV65/stopheater", 2);
  mqttClient.subscribe("iot/OV65/reset", 2);
  mqttClient.subscribe("iot/OV65/switchmode", 2);

  //Serial.print("Subscribing at QoS 2, packetId: ");
  //Serial.println(packetIdSub);
  //mqttClient.publish("test/lol", 0, true, "test 1");
  //Serial.println("Publishing at QoS 0");
 // uint16_t packetIdPub2 = mqttClient.publish("iot/OV65/humidity", 1, true, "33.3");
    //Serial.print("Publishing at QoS 1, packetId: ");
  //Serial.println(packetIdPub1);
  //uint16_t packetIdPub2 = mqttClient.publish("test/lol", 2, true, "test 3");
 // Serial.print("Publishing at QoS 2, packetId: ");
 // Serial.println(packetIdPub2);

  mqttPrintStatus();
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected())
   {
    xTimerStart(mqttReconnectTimer, 0);
   }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
 // Serial.println("Subscribe acknowledged.");
 // Serial.print("  packetId: ");
 // Serial.println(packetId);
 // Serial.print("  qos: ");
 // Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
 // Serial.println("Unsubscribe acknowledged.");
 // Serial.print("  packetId: ");
 // Serial.println(packetId);
}

void onMqttPublish(uint16_t packetId) {
//  Serial.println("Publish acknowledged.");
 // Serial.print("  packetId: ");
 // Serial.println(packetId);
}


char tmstr[64];
void mqttUpdateTime()
{
  if(mqttClient.connected())
  {
    sprintf(tmstr,"hh:mm:ss");
    Serial.println(timenow.toString(tmstr));
    mqttClient.publish("iot/OV65/currentTime", 1, true, tmstr );
  }
}


void mqttPrintStatus(){

  Serial.println("mqttPrintStatus()...");

  if(mqttClient.connected())
  {
  
    WiFi.localIP().toString().toCharArray(cstr,128);
    mqttClient.publish("iot/OV65/DeviceIp", 1, true, cstr);

    if(IsAlarm)
    {
      if(IsNoIgnition)
        mqttClient.publish("iot/OV65/DeviceError", 1, true, "Нет тока свечи");
      else
        mqttClient.publish("iot/OV65/DeviceError", 1, true, "Перегрев");
    } else
    {
      mqttClient.publish("iot/OV65/DeviceError", 1, true, "Нет");
    }
    sprintf(cstr, "%d", wifiConnectedCount);
    mqttClient.publish("iot/OV65/WiFiConnectionCount", 1, true, cstr);
    //fuel relay
    if (FuelRelayState == false)
    {
      mqttClient.publish("iot/OV65/fuelValve", 1, true, "Закрыт");
    } else
    {
      mqttClient.publish("iot/OV65/fuelValve", 1, true, "Открыт");
    }
    sprintf(cstr, "%d", SparkCurrent);
    mqttClient.publish("iot/OV65/ignitionCurrent", 1, true, cstr);
    //spark
    if (SparkRelayState == false)
    {
      mqttClient.publish("iot/OV65/ignition", 1, true, "Откл");
    } else
    {
      mqttClient.publish("iot/OV65/ignition", 1, true, "Вкл");
    }
     //engine
    if (curMotorRelayState == false)
    {
      mqttClient.publish("iot/OV65/engine", 1, true, "Откл");
      mqttClient.publish("iot/OV65/engineSpeed", 1, true, "0");
    } else
    {
      mqttClient.publish("iot/OV65/engine", 1, true, "Вкл");
      if(MotorSpeed==1)
        mqttClient.publish("iot/OV65/engineSpeed", 1, true, "1");
      else
        mqttClient.publish("iot/OV65/engineSpeed", 1, true, "2");
    }
 //mode
    if (currentMode == ModeManual)
    {
      mqttClient.publish("iot/OV65/autoMode", 1, true, "Ручной");
    } else
    {
      mqttClient.publish("iot/OV65/autoMode", 1, true, "Автоматический");
    }

    if (IsFired)
    {
      mqttClient.publish("iot/OV65/heatSensor", 1, true, "Горячий");
    } else
    {
      mqttClient.publish("iot/OV65/heatSensor", 1, true, "Холодный");
    }

    sprintf(cstr, "%d", neededTemp);
    mqttClient.publish("iot/OV65/neededTemp", 1, true, cstr);
    mqttUpdateTime();
  }
}


bool prevstate;
void ReadButtons()
{
  //Update the Bounce instance :
  buttons[0].update();
    // If it fell, flag the need to toggle the LED
  if ( buttons[0].fell()) 
  {
    IsFired = true;
    if(prevstate!=IsFired)
    {
      prevstate = IsFired;
      if(mqttClient.connected())
        mqttClient.publish("iot/OV65/heatSensor", 1, true, "Горячий");
    }
  }
  if ( buttons[0].rose()) 
  {
    IsFired = false;
    if(prevstate!=IsFired)
    {
      prevstate = IsFired;
      if(mqttClient.connected())
        mqttClient.publish("iot/OV65/heatSensor", 1, true, "Холодный");
    }
  }
  buttons[1].update();
  if(buttons[1].fell())
  {
    IsAlarm = true;
  }
  btnA.read();
  btnM.read();
  btnL.read();
  btnR.read();
  btnP.read();
}
void showTimeSpan(const char* txt, const TimeSpan& ts) {
    Serial.print(txt);
    Serial.print(" ");
    Serial.print(ts.days(), DEC);
    Serial.print(" days ");
    Serial.print(ts.hours(), DEC);
    Serial.print(" hours ");
    Serial.print(ts.minutes(), DEC);
    Serial.print(" minutes ");
    Serial.print(ts.seconds(), DEC);
    Serial.print(" seconds (");
    Serial.print(ts.totalseconds(), DEC);
    Serial.print(" total seconds)");
    Serial.println();
}
void showDate(const char* txt, const DateTime& dt) {
    Serial.print(txt);
    Serial.print(' ');
    Serial.print(dt.year(), DEC);
    Serial.print('/');
    Serial.print(dt.month(), DEC);
    Serial.print('/');
    Serial.print(dt.day(), DEC);
    Serial.print(' ');
    Serial.print(dt.hour(), DEC);
    Serial.print(':');
    Serial.print(dt.minute(), DEC);
    Serial.print(':');
    Serial.print(dt.second(), DEC);

    Serial.print(" = ");
    Serial.print(dt.unixtime());
    Serial.print("s / ");
    Serial.print(dt.unixtime() / 86400L);
    Serial.print("d since 1970");

    Serial.println();
}
//weather data reading task
float LastTemp = 0;
float LastHumidity = 0;
void weatherTask( void * pvParameters )
{
 dht.begin();
 
 /* main temptask loop */
 while (1)
 {
    vTaskDelay(15000 / portTICK_PERIOD_MS); //wait for conversion ready

     //update MQTT
    if(abs(CurrentTemp-LastTemp) >=1)
    {
      LastTemp=CurrentTemp;
      sprintf(cstr, "%f", CurrentTemp);
      if(mqttClient.connected())
        mqttClient.publish("iot/OV65/temperature", 1, true, cstr);
      Serial.print(F("T: "));
      Serial.print(LastTemp);
      Serial.println(F(" °C"));
    }
    if(abs(CurrentHum-LastHumidity)>=1)
    {
      LastHumidity=CurrentHum;
      sprintf(cstr, "%f", CurrentHum);
      if(mqttClient.connected())
        mqttClient.publish("iot/OV65/humidity", 1, true, cstr);
      Serial.print(F("H: "));
      Serial.print(LastHumidity);
      Serial.println(F(" %"));
    }

    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float t = dht.readTemperature();
    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t)) 
    {
      Serial.println(F("DHT sensor Failed!"));
      continue;
    }
    CurrentTemp = t;
    CurrentHum = h;
  
    Serial.print(F("H: "));
    Serial.print(h);
    Serial.print(F(" %  T: "));
    Serial.print(t);
    Serial.println(F(" °C"));
  }
}
void FuelRelayOn()
{
  Serial.println("Fuel is ON");
  if (FuelRelayState == false)
  {
    digitalWrite(FUEL_VALVE_RELAY_PIN, LOW);
    //tone(BEEP_PIN, 2800, 200);
    FuelRelayState = true;
  }
  mqttPrintStatus();
}
void FuelRelayOff()
{
  Serial.println("Fuel is OFF");
  if (FuelRelayState == true)
  {
    digitalWrite(FUEL_VALVE_RELAY_PIN, HIGH);
   // tone(BEEP_PIN, 2000, 200);
    FuelRelayState = false;
  }
  mqttPrintStatus();
}
void SparkRelayOn()
{
  Serial.println("Spark is ON");
  if (SparkRelayState == false)
  {
    digitalWrite(IGNITION_RELAY_PIN, LOW);
  //  tone(BEEP_PIN, 2000, 200);
    vTaskDelay(250 / portTICK_PERIOD_MS); //wait for conversion ready
    SparkRelayState = true;
    sparkStartTime = timenow;
  }
  mqttPrintStatus();
}
void SparkRelayOff()
{
  Serial.println("Spark is OFF");
  if (SparkRelayState == true)
  {
    digitalWrite(IGNITION_RELAY_PIN, HIGH);
 //   tone(BEEP_PIN, 1800, 200);
 //   tone(BEEP_PIN, 600, 200);
    SparkRelayState = false;
  }
  mqttPrintStatus();
}
void MotorSpeed2()
{
  Serial.println("Motor Speed 2");
  digitalWrite(MOTOR_SPEED_RELAY_PIN, LOW);
 // tone(BEEP_PIN, 2000, 200);
  MotorSpeed = 2;
  mqttPrintStatus();
}
void MotorSpeed1()
{
   Serial.println("Motor Speed 1");
   digitalWrite(MOTOR_SPEED_RELAY_PIN, HIGH);
   // tone(BEEP_PIN, 1800, 200);
    MotorSpeed = 1;
    mqttPrintStatus();
}
void MotorRelayOn()
{
  Serial.println("Motor ON");
  if (curMotorRelayState == false)
  {
    digitalWrite(MOTOR_RELAY_PIN, LOW);
  //  tone(BEEP_PIN, 1000, 200);
    curMotorRelayState = true;
  }
  mqttPrintStatus();
}
void MotorRelayOff()
{
  Serial.println("Motor OFF");
  if (curMotorRelayState == true)
  {
    MotorSpeed1();//off speed relay when idle
    digitalWrite(MOTOR_RELAY_PIN, HIGH);
  //  tone(BEEP_PIN, 600, 200);
    curMotorRelayState = false;
  }
  mqttPrintStatus();
}
void SetAutoMode()
{
  eData.writeByte(0, currentMode == ModeManual ? 0x00 : 0x01);
  //eeprom_write_byte(&modebyteAddr, currentMode == ModeManual ? 0x00 : 0x01);
  mqttPrintStatus();
}
void GetAutoMode()
{
  //byte readData = eeprom_read_byte(&modebyteAddr);
  byte readData = eData.readByte(0);
  currentMode = readData == 0 ? ModeManual : ModeAuto;
}
void SetNeededTemp()
{
  eData.writeByte(1,neededTemp);
 //eeprom_write_byte(&tempbyteAddr, neededTemp);
 mqttPrintStatus();
}
void GetNeededTemp()
{
  //neededTemp = eeprom_read_byte(&tempbyteAddr);
  neededTemp = eData.readByte(1);
}
void StartHeater()
{
  if(currentStartStage==IdleToStart)
  {
    IsIdle = false;
    Serial.println("StartHeater()");
    currentStartStage = StartFuel;
  }
  mqttPrintStatus();
}
void ChangeAutoMode()
{
      if (currentMode == ModeManual && IsIdle)
      {
        currentMode = ModeAuto;
        SetAutoMode();
        mqttPrintStatus();
        return;
      }
      if(currentMode == ModeAuto)
      {
        currentMode = ModeManual;
        SetAutoMode();
        mqttPrintStatus();
      }
}
void StopHeater()
{
  if(currentStopStage==IdleStopped)
  {
    currentStopStage = StopFuel;
    Serial.println("StopHeater()");
  } else
  {
    Serial.println("StopHeater() - Stopping Already");
  }
  mqttPrintStatus();
}
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
 Serial.print("Publish received.");
 Serial.print(" Topic: ");
 Serial.println(topic);
  //Serial.print("  qos: ");
  //Serial.println(properties.qos);
  //Serial.print("  dup: ");
  //Serial.println(properties.dup);
  //Serial.print("  retain: ");
  //Serial.println(properties.retain);
  //Serial.print("  len: ");
  //Serial.println(len);
  //Serial.print("  index: ");
  //Serial.println(index);
  //Serial.print("  total: ");
  //Serial.println(total);
  String tpc(topic);
  if(tpc.equals("iot/OV65/startheater"))
  {
     Serial.println("startheater");
     StartHeater();
  }
  if(tpc.equals("iot/OV65/stopheater"))
  {
     Serial.println("stopheater");
     StopHeater();
  }
  if(tpc.equals("iot/OV65/switchmode"))
  {
     Serial.println("switchmode");
     ChangeAutoMode();
  }
   if(tpc.equals("iot/OV65/reset"))
  {
     Serial.println("Resetting...");
     ESP.restart();
  }
}
void PressedEvent(int key)
{
    switch (key)
    {
    case BUTTON_A_PIN: //AUTO
      ChangeAutoMode();
      break;
    case BUTTON_M_PIN: //MENU
    if(!IgnoreSparkCurrent)
    {
      IgnoreSparkCurrent = true;
    }
    break;
    case BUTTON_L_PIN: //LEFT
      if (neededTemp > 11 && currentMode == ModeAuto)
      {
        neededTemp--;
        SetNeededTemp();
   //     tone(BEEP_PIN, 2000, 100);
      }
      else
      {
     //   tone(BEEP_PIN, 800, 100);
      }
      //menu.previous_screen();
      break;
    case BUTTON_R_PIN: //RIGHT
      if (neededTemp < 30 && currentMode == ModeAuto)
      {
        neededTemp++;
        SetNeededTemp();
   //     tone(BEEP_PIN, 2000, 100);
      }
      else
      {
  //      tone(BEEP_PIN, 800, 100);
      }
      break;
    case BUTTON_P_PIN: //POWER
      if (currentMode == ModeManual)
      {
        if(currentStartStage==IdleToStart && currentStopStage==IdleStopped)
        {
    //      tone(BEEP_PIN, 900, 100);
          StartHeater();
          return;
        }
 //       tone(BEEP_PIN, 600, 200);
        StopHeater();
      }
      break;
    default: //all other keys
      break;
    }
    
}
void pressHandler (BfButton *btn, BfButton::press_pattern_t pattern)
 {
  Serial.print(btn->getPin());
  switch (pattern)  {
    case BfButton::SINGLE_PRESS:
      Serial.println(" pressed.");
      if(IsAlarm)
        break;
      else
      PressedEvent(btn->getPin());
      break;
    case BfButton::DOUBLE_PRESS:
      Serial.println(" double pressed.");
      break;
    case BfButton::LONG_PRESS:
      Serial.println(" long pressed.");
      break;
  }
}
void printLocalTime()
{
  Serial.println(timenow_tm, "%A, %B %d %Y %H:%M:%S");
}
void SetRTC()
{

  Serial.println("Read time from NTP...");
  // get NTP time every time connect to wifi, not necessary but wont hurts
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  if (getLocalTime(timenow_tm))
  {
    Serial.println("NTP Ok, time is:");
  } else
  {
    Serial.println("NTP failed, set to prefefined time:");
    timeval tnow;
    tnow.tv_sec = 1609648461;
    timezone tzcur;
    tzcur.tz_minuteswest = 32400;
    settimeofday(&tnow, &tzcur);
  }
  timeAdjusted = true;
  getLocalTime(timenow_tm);
  printLocalTime();
}
void StopDevice()
{
  if(IsDeviceStopped)
    return;

  currentStopStagePrev = IdleStopped;
  currentStopStage = IdleStopped;
  currentStartStagePrev = IdleToStart;
  currentStartStage = IdleToStart;
    
  Serial.println("StopDevice()!!!!...");
  FuelRelayOff();
  vTaskDelay(500 / portTICK_PERIOD_MS); //wait for conversion ready
  SparkRelayOff();
  vTaskDelay(500 / portTICK_PERIOD_MS); //wait for conversion ready
  MotorRelayOff();
  vTaskDelay(500 / portTICK_PERIOD_MS); //wait for conversion ready
  IsDeviceStopped = true;

  mqttPrintStatus();

} 
void displayTask( void * pvParameters)
{

 // initialize the lcd
  display.init();
  // Print a message to the LCD.
  display.backlight();

  display.clear();
  display.createChar(1, bukva_P);  // Создаем символ под номером 1
  display.createChar(2, bukva_I);  // Создаем символ под номером 2
  display.createChar(3, bukva_CH); // Создаем символ под номером 3
  display.createChar(4, bukva_Y);  // Создаем символ под номером 4
  display.createChar(5, bukva_IY); // Создаем символ под номером 5
  display.createChar(6, bukva_G); // Создаем символ под номером 6
  display.createChar(7, bukva_L); // Создаем символ под номером 7
  display.createChar(8, thermometerIcon); // Создаем символ под номером 8
  display.createChar(9, kaplyaIcon); // Создаем символ под номером 9

  display.setCursor(4, 0);
  display.print(F("OV-65 Heater"));
  display.setCursor(5, 1);
  display.print(F("controller"));
  display.setCursor(6, 2);
  display.print(F("(c) 2020"));
  display.setCursor(3, 3);
  display.print(F("vasp@zabmail.ru"));

  vTaskDelay(3000 / portTICK_PERIOD_MS); //Show this 3 seconds

  //display.setCursor(0, 0); // установка позиции курсора
                          //
                          //  0,0 ------- 20,0
                          //   |
                          //   |
                          //  0,3
  while (1)
  {
    vTaskDelay(LCD_UPDATE_TIME / portTICK_PERIOD_MS);
    if(IsAlarm)
    {
      display.clear();
      display.setCursor(3, 1); // установка позиции курсора
      if(IsNoIgnition)
        display.print(F("CBE\3A HET TOKA"));
      else
        display.print(F("  OVERHEAT!!!"));
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }
    if (currentMode == ModeManual)
    {
      display.setCursor(0, 0); // установка позиции курсора
      display.print(F("P\4\3HO\5 "));
    }
    else
    {
      display.setCursor(0, 0); // установка позиции курсора
      display.print(F("ABTOMAT "));
    }

    if(currentStartStage==IdleToStart && currentStopStage == IdleStopped) //we are not running
    {
      display.setCursor(8, 0); // установка позиции курсора
      display.print(F("IDLE "));
    }
    else if(currentStopStage != IdleStopped) //we are not running
    {
      display.setCursor(8, 0); // установка позиции курсора
      display.print(F("OCTAH"));
    }else if(currentStartStage!=IdleToStart)
    {
      display.setCursor(8, 0); // установка позиции курсора
      display.print(F("START"));
    }

    display.setCursor(14, 0); // установка позиции курсора
    if (FuelRelayState == false)
    {
        display.print(F(" "));
    }
    else
    {
        display.print(F("F"));
    }

  //sprintf(timeresult, "%02d:%02d", timenow.hour(), timenow.minute());
    display.setCursor(17, 0); // установка позиции курсора
    if(IsFired)
      display.print(F("\6OP"));
    else
      display.print(F("XO\7"));

    display.setCursor(0, 1); // установка позиции курсора
    display.print(F("MOTOP"));
    display.setCursor(6, 1);
    if (!curMotorRelayState)
      display.print(F("OFF          "));
    else
    {
      display.print(F("ON "));
      display.setCursor(10, 1);
      display.print(F("CKOP "));
      display.setCursor(15, 1);
      display.print(MotorSpeed);
    }
    display.setCursor(0, 2); // установка позиции курсора
    display.print(F("CBE\3A"));
    display.setCursor(6, 2);
    if (!SparkRelayState)
      display.print(F("OFF           "));
    else
    {
      TimeSpan sparkWorked = timenow - sparkStartTime;
      display.print(F("ON "));
      display.setCursor(11, 2);
      display.print(F("TOK "));
      display.setCursor(15, 2);
      display.print(SparkCurrent);
      display.setCursor(18, 2);
      display.print(sparkWorked.totalseconds());
    }


    display.setCursor(0, 3);
    display.print(F("B"));
    display.setCursor(1, 3);
    display.print(CurrentHum,0);
    display.print(F("%"));

    display.setCursor(5, 3);
    display.print(F("\x08"));
    display.setCursor(6, 3);
    display.print(CurrentTemp,0);

 /* if (currentMode == ModeManual)
  {
    display.setCursor(16, 3);
    display.print(F("    "));
  }
  else
  {*/
    display.print(F("/"));
    display.print(neededTemp);
    display.setCursor(15, 3);
    display.print(timenow_tm, "%H:%M");
  //}
    display.display();
  }
}

int lastMinute = 0;

void ReadRTC()
{
  getLocalTime(timenow_tm);
  timenow = DateTime(timenow_tm->tm_year,timenow_tm->tm_mon,timenow_tm->tm_mday,timenow_tm->tm_hour,timenow_tm->tm_min,timenow_tm->tm_sec);
  
  if(timenow.minute()!=lastMinute)
  {
      lastMinute = timenow.minute();
      mqttUpdateTime();
  }
}
void ReadACS()
{
  if (!(millis() - lastAcsUpdateTime > ACS_UPDATE_TIME))
  {
    return;
  }
  lastAcsUpdateTime = millis();
  //ONLY CHECK IF Spark is on
  if(!SparkRelayState)
  {
    SparkCurrent = 0;
    return;
  }
  //SparkCurrent = (ACS.mA_DC());// / 1000; //read spark DC in amperes
  SparkCurrent = sensor.getCurrentDC();
  Serial.print("ACS: ");
  Serial.println(SparkCurrent, DEC);
  mqttPrintStatus();

}
bool CheckIgnition()
{
  ReadACS();

  if(SparkCurrent < 4)
  {
    if(!IgnoreSparkCurrent)
        IsNoIgnition = true; 
  } 
  else 
  {
    IsNoIgnition = false;
  }

 if(IsNoIgnition) //spark failed
    {
      IsAlarm = true; //no ignition!!!
      currentStartStage = IdleToStart;
      FuelRelayOff();
      SparkRelayOff();
      MotorRelayOff();
    }
    return !IsNoIgnition;
}
void StartFuelFn()
{
    Serial.println("StartFuelFn()");
    FuelRelayOn();
    SparkRelayOn();
    currentStartStage = StartIgnition;
    dtStartIgnition = timenow;
    vTaskDelay(1000 / portTICK_PERIOD_MS); 
}
void StartIgnitionFn()
{
    
    if(!CheckIgnition())
      return;

    TimeSpan sparkTimei = timenow - dtStartIgnition;

    //Serial.println(sparkTimei.totalseconds());
    //msg = sparkTime.totalseconds();
    if (sparkTimei.totalseconds() > 20 )
    {
      MotorSpeed1();
      vTaskDelay(1000 / portTICK_PERIOD_MS); 
      MotorRelayOn();
      vTaskDelay(1000 / portTICK_PERIOD_MS); 
      currentStartStage = StartHalfMotor;
      dtStartHalfMotor = timenow;
    }
}
void StartHalfMotorFn()
{
    if(!CheckIgnition())
      return;
    TimeSpan halfMotorTime = timenow - dtStartHalfMotor;
    //msg = halfMotorTime.totalseconds();
    if (halfMotorTime.totalseconds() > 20)
    {
      SparkRelayOff();
      currentStartStage = StopIgnition;
      dtStopIgnition = timenow;
    //  tone(BEEP_PIN, 4000, 100);
      vTaskDelay(1000 / portTICK_PERIOD_MS); 
    }

}
void StopIgnitionFn()
{
    //TimeSpan stopIgnTime = timenow - dtStopIgnition;
    if (IsFired) // we are heated already
    {
      MotorSpeed2();
      currentStartStage = StartFullMotor;
      vTaskDelay(1000 / portTICK_PERIOD_MS); 
    }
}
void ProcessStartup()
{
    if(currentStopStage!=IdleStopped) //we are stopping
    {
      return;
    }

    if( currentStartStage != currentStartStagePrev)
    {
      currentStartStagePrev = currentStartStage;
      if(mqttClient.connected())
      {
        switch (currentStartStage)
        {
          case IdleToStart:
            mqttClient.publish("iot/OV65/currentStartStage", 2, true, "Готов");
            break;
          case StartFuel:
            mqttClient.publish("iot/OV65/currentStartStage", 2, true, "Включение топлива");
            break;
          case StartIgnition:
            mqttClient.publish("iot/OV65/currentStartStage", 2, true, "Включение свечи");
            break;
          case StartHalfMotor:
              mqttClient.publish("iot/OV65/currentStartStage", 2, true, "Вентилятор 1 скорость");
            break;
          case StopIgnition:
            mqttClient.publish("iot/OV65/currentStartStage", 2, true, "Отключение свечи");
            break;
          case StartFullMotor:
            mqttClient.publish("iot/OV65/currentStartStage", 2, true, "Вентилятор 2 скорость");
            break;
          default:
            mqttClient.publish("iot/OV65/currentStartStage", 2, true, "Неизвестная стадия");
            break;
        }
      }
    }
    switch (currentStartStage)
    {
    case IdleToStart:
      break;
    case StartFuel:
      StartFuelFn();
      break;
    case StartIgnition:
      StartIgnitionFn();
      break;
    case StartHalfMotor:
      StartHalfMotorFn();
      break;
    case StopIgnition:
      StopIgnitionFn();
      break;
    case StartFullMotor:
      break;
    default:
      break;
    }
}
void ProcessShutDown()
{
  if(currentStartStage == IdleToStart) //we are not running
  {
    return;
  }

   if( currentStopStage != currentStopStagePrev)
    {
      currentStopStagePrev = currentStopStage;

      if(mqttClient.connected())
      {
        switch (currentStopStage)
        {
          case IdleStopped:
            mqttClient.publish("iot/OV65/currentStopStage", 2, true, "Ожидание");
            break;
          case StopFuel:
            mqttClient.publish("iot/OV65/currentStopStage", 2, true, "Отключение топлива");
            break;
          case WaitToStopFire:
            mqttClient.publish("iot/OV65/currentStopStage", 2, true, "Продувка для остывания");
            break;
          case StopMotor:
              mqttClient.publish("iot/OV65/currentStopStage", 2, true, "Отключение мотора");
            break;
          default:
            mqttClient.publish("iot/OV65/currentStopStage", 2, true, "Неизвестная стадия");
            break;
        }
      }
    }
  switch (currentStopStage)
  {
    break;
    case IdleStopped:
      break;
    case StopFuel:
      FuelRelayOff();
      SparkRelayOff();
      MotorRelayOn();
      MotorSpeed2();
      timeFreeRun = timenow;
      currentStopStage = WaitToStopFire;
      currentStartStage = StartFullMotor;
      break;
    case WaitToStopFire:
      if(!IsFired)
      {
        TimeSpan secs = timenow - timeFreeRun;
          if(secs.totalseconds() > 120)
            currentStopStage = StopMotor;
      }
      break;
  case StopMotor:
      MotorRelayOff();
      IsIdle = true;
      currentStopStage = IdleStopped;
      currentStartStage = IdleToStart;
       if(mqttClient.connected())
        mqttClient.publish("iot/OV65/currentStopStage", 2, true, "Ожидание");
      mqttPrintStatus();
      break;
    default:
    break;
  }
}
void setup()
{
  Serial.begin(115200);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  WiFi.onEvent(WiFiEvent);
  // If your broker requires authentication (username and password), set them below

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(mqttHost, MQTT_PORT);
  mqttClient.setCredentials(MQTT_NAME, MQTT_PASSWORD);
  connectToWifi();
  //temerature read task     
  xTaskCreatePinnedToCore(
    weatherTask,                       /* Function to implement the task */
    "WeatherTask",                    /* Name of the task */
    4000,                           /* Stack size in words */
    NULL,                           /* Task input parameter */
    255,                              /* Priority of the task */
    NULL,                           /* Task handle. */
    0);                             /* Core where the task should run */
  //Display Show task     
  xTaskCreatePinnedToCore(displayTask,"DisplayTask",4000,NULL,5,NULL,0);                             

  SetRTC();

  //relays (used modules are on with LOW!!! state)
  pinMode(MOTOR_RELAY_PIN, INPUT_PULLUP);
  pinMode(MOTOR_RELAY_PIN, OUTPUT);
  digitalWrite(MOTOR_RELAY_PIN, HIGH); //off at start

  pinMode(MOTOR_SPEED_RELAY_PIN, INPUT_PULLUP);
  pinMode(MOTOR_SPEED_RELAY_PIN, OUTPUT);
  digitalWrite(MOTOR_SPEED_RELAY_PIN, HIGH); //speed one at start

  pinMode(FUEL_VALVE_RELAY_PIN, INPUT_PULLUP);
  pinMode(FUEL_VALVE_RELAY_PIN, OUTPUT);
  digitalWrite(FUEL_VALVE_RELAY_PIN, HIGH); //off at start

  pinMode(IGNITION_RELAY_PIN, INPUT_PULLUP);
  pinMode(IGNITION_RELAY_PIN, OUTPUT);
  digitalWrite(IGNITION_RELAY_PIN, HIGH); //off at start

  //keyboard common pin
  pinMode(BUTTON_GROUND_PIN, OUTPUT);
  digitalWrite(BUTTON_GROUND_PIN, LOW); //off at start

  //inputs from heater sensors (LOW when they is ON)
  for (int i = 0; i < NUM_BUTTONS; i++) 
  {
    buttons[i].attach( BUTTON_PINS[i] , INPUT_PULLUP  );       //setup the bounce instance for the current button
    buttons[i].interval(125); // interval in ms
  }
  // calibrate() method calibrates zero point of sensor,
  // It is not necessary, but may positively affect the accuracy
  // Ensure that no current flows through the sensor at this moment
  // If you are not sure that the current through the sensor will not leak during calibration - comment out this method
  Serial.println("Calibrating ACS712");
  int zero = sensor.calibrate();
  Serial.print("Done! Zpoint: ");
  Serial.println(zero);
  
  GetNeededTemp();
  GetAutoMode();
  vTaskDelay(1500 / portTICK_PERIOD_MS); 

 // client.setServer(mqtt_server, 1883);
 // client.setCallback(callback);
  btnA.onPress(pressHandler)
     .onDoublePress(pressHandler) // default timeout
     .onPressFor(pressHandler, 1000); // custom timeout for 1 second
  btnL.onPress(pressHandler)
     .onDoublePress(pressHandler) // default timeout
     .onPressFor(pressHandler, 1000); // custom timeout for 1 second
  btnR.onPress(pressHandler)
     .onDoublePress(pressHandler) // default timeout
     .onPressFor(pressHandler, 1000); // custom timeout for 1 second
  btnM.onPress(pressHandler)
     .onDoublePress(pressHandler) // default timeout
     .onPressFor(pressHandler, 1000); // custom timeout for 1 second
  btnP.onPress(pressHandler)
     .onDoublePress(pressHandler) // default timeout
     .onPressFor(pressHandler, 1000); // custom timeout for 1 second

  //OTA Web
   /*use mdns for host name resolution*/
  if (!MDNS.begin(host)) { //http://ov65.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  /*return index page which is stored in serverIndex */
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  /*handling uploading firmware file */
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  server.begin();


}
void loop() //loop over all functions
{
  if(!timeAdjusted)
    SetRTC();

  ReadButtons();
  ReadRTC();
  if(IsAlarm)
    StopDevice();
  else
  {
    ReadACS();
    ProcessStartup();
    ProcessShutDown();
  }
  server.handleClient();
}