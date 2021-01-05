#include <Arduino.h>
//#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
//#include <Keypad.h> //keypad lib
#include "ACS712.h"
#include "RTClib.h"
//#include <I2C_EEPROM.h>
#include <Bounce2.h>
#include <eeprom.h>
#include "DHT.h"
#include <BfButton.h>

//ESP32
#include "WiFi.h"
#include <PubSubClient.h>
#include <time.h>
#include <sys/time.h>


const char* ssid = "zabmail2G";
const char* password =  "1234qwertY4321";

//const char* mqtt_server = "45.95.184.155";
//WiFiClient espClient;
//PubSubClient client(espClient);
//unsigned long lastMsg = 0;
//#define MSG_BUFFER_SIZE	(50)
//char msg[MSG_BUFFER_SIZE];

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


int wifiCount = 0; //wifi connect attept counter

bool timeAdjusted = false;

byte bukva_B[8] = {
    B11110,
    B10000,
    B10000,
    B11110,
    B10001,
    B10001,
    B11110,
    B00000,
}; // Буква "Б"
byte bukva_G[8] = {
    B11111,
    B10001,
    B10000,
    B10000,
    B10000,
    B10000,
    B10000,
    B00000,
}; // Буква "Г"
byte bukva_D[8] = {
    B01111,
    B00101,
    B00101,
    B01001,
    B10001,
    B11111,
    B10001,
    B00000,
}; // Буква "Д"
byte bukva_ZH[8] = {
    B10101,
    B10101,
    B10101,
    B11111,
    B10101,
    B10101,
    B10101,
    B00000,
}; // Буква "Ж"
byte bukva_Z[8] = {
    B01110,
    B10001,
    B00001,
    B00010,
    B00001,
    B10001,
    B01110,
    B00000,
}; // Буква "З"
byte bukva_I[8] = {
    B10001,
    B10011,
    B10011,
    B10101,
    B11001,
    B11001,
    B10001,
    B00000,
}; // Буква "И"
byte bukva_IY[8] = {
    B01110,
    B00000,
    B10001,
    B10011,
    B10101,
    B11001,
    B10001,
    B00000,
}; // Буква "Й"
byte bukva_L[8] = {
    B00011,
    B00111,
    B00101,
    B00101,
    B01101,
    B01001,
    B11001,
    B00000,
}; // Буква "Л"
byte bukva_P[8] = {
    B11111,
    B10001,
    B10001,
    B10001,
    B10001,
    B10001,
    B10001,
    B00000,
}; // Буква "П"
byte bukva_Y[8] = {
    B10001,
    B10001,
    B10001,
    B01010,
    B00100,
    B01000,
    B10000,
    B00000,
}; // Буква "У"
byte bukva_F[8] = {
    B00100,
    B11111,
    B10101,
    B10101,
    B11111,
    B00100,
    B00100,
    B00000,
}; // Буква "Ф"
byte bukva_TS[8] = {
    B10010,
    B10010,
    B10010,
    B10010,
    B10010,
    B10010,
    B11111,
    B00001,
}; // Буква "Ц"
byte bukva_CH[8] = {
    B10001,
    B10001,
    B10001,
    B01111,
    B00001,
    B00001,
    B00001,
    B00000,
}; // Буква "Ч"
byte bukva_Sh[8] = {
    B10101,
    B10101,
    B10101,
    B10101,
    B10101,
    B10101,
    B11111,
    B00000,
}; // Буква "Ш"
byte bukva_Shch[8] = {
    B10101,
    B10101,
    B10101,
    B10101,
    B10101,
    B10101,
    B11111,
    B00001,
}; // Буква "Щ"
byte bukva_Mz[8] = {
    B10000,
    B10000,
    B10000,
    B11110,
    B10001,
    B10001,
    B11110,
    B00000,
}; // Буква "Ь"
byte bukva_IYI[8] = {
    B10001,
    B10001,
    B10001,
    B11001,
    B10101,
    B10101,
    B11001,
    B00000,
}; // Буква "Ы"
byte bukva_Yu[8] = {
    B10010,
    B10101,
    B10101,
    B11101,
    B10101,
    B10101,
    B10010,
    B00000,
}; // Буква "Ю"
byte bukva_Ya[8] = {
    B01111,
    B10001,
    B10001,
    B01111,
    B00101,
    B01001,
    B10001,
    B00000,
}; // Буква "Я"
// иконка термометра:
byte thermometerIcon[8] = {
  B00100,
  B01010,
  B01010,
  B01010,
  B01010,
  B10001,
  B11111,
  B01110
};
// иконка капли:
byte kaplyaIcon[8] = {
  B00000,
  B01100,
  B01100,
  B11110,
  B11110,
  B11111,
  B11111,
  B01110
};

//By default, on ESP32 boards, the SDA pin is 21 . SCL pin is 22
/*
static const uint8_t SDA = 21;
static const uint8_t SCL = 22;
static const uint8_t SS    = 5;
static const uint8_t MOSI  = 23;
static const uint8_t MISO  = 19;
static const uint8_t SCK   = 18;
*/

#define DHT_PIN 23 //GPIO23 //20 not useable 
//Temperarure sensor
//#define DHTTYPE DHT21   // AM2301 
DHT dht(DHT_PIN,AM2301);
float CurrentTemp = 99;            // Глобальная переменная для хранения значение температуры с датчика DS18B20
float CurrentHum = 99;            // Глобальная переменная для хранения значение температуры с датчика DS18B20
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
const int ACS_UPDATE_TIME = 1000; // Определяем периодичность проверок



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
  StartFullMotor 
};
enum StopProcessStage
{
  HeaterStarted,
  StopFuel, 
  WaitToStopFire, //wait for sensor
  StopMotor,
  IdleStopped,
};

WorkMode currentMode = ModeManual;
StartProcessStage currentStartStage = IdleToStart;
StopProcessStage currentStopStage = IdleStopped;

bool IsAlarm = false;
bool IsDeviceStopped = false;
bool IsNoIgnition = false;
bool IsFired = false;
bool IsIdle = true;
bool IgnoreSparkCurrent = false;

// set the LCD address to 0x27 for a 20 chars and 4 line display
LiquidCrystal_I2C display(0x27, 20, 4);
long lastLcdUpdateTime = 20000;  // Переменная для хранения времени ПОКАЗА НА lcd
const int LCD_UPDATE_TIME = 500; // Определяем периодичность ПОКАЗА НА lcd

bool curMotorRelayState = false;
int MotorSpeed = 1;
bool SparkRelayState = false;
int SparkCurrent = 0;
bool FuelRelayState = false;

void ReadButtons()
{
  //Update the Bounce instance :
  buttons[0].update();
    // If it fell, flag the need to toggle the LED
  if ( buttons[0].fell()) 
  {
    IsFired = true;
  }
  if ( buttons[0].rose()) 
  {
    IsFired = false;
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
void weatherTask( void * pvParameters )
{
 dht.begin();
  /* main temptask loop */
 while (1)
 {
    vTaskDelay(10000 / portTICK_PERIOD_MS); //wait for conversion ready
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float t = dht.readTemperature();
    // Read temperature as Fahrenheit (isFahrenheit = true)
    //float f = dht.readTemperature(true);
    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t)) 
    {
      Serial.println(F("Failed to read from DHT sensor!"));
      continue;
  }
  CurrentTemp = t;
  CurrentHum = h;
  // Compute heat index in Fahrenheit (the default)
  //float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  //float hic = dht.computeHeatIndex(t, h, false);
  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.println(F("°C "));
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
}
void MotorSpeed2()
{
  Serial.println("Motor Speed 2");
  digitalWrite(MOTOR_SPEED_RELAY_PIN, LOW);
 // tone(BEEP_PIN, 2000, 200);
  MotorSpeed = 2;

}
void MotorSpeed1()
{
   Serial.println("Motor Speed 1");
   digitalWrite(MOTOR_SPEED_RELAY_PIN, HIGH);
   // tone(BEEP_PIN, 1800, 200);
    MotorSpeed = 1;
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
}
void SetAutoMode()
{
  EEPROM.writeByte(0, currentMode == ModeManual ? 0x00 : 0x01);
  //eeprom_write_byte(&modebyteAddr, currentMode == ModeManual ? 0x00 : 0x01);
}
void GetAutoMode()
{
  //byte readData = eeprom_read_byte(&modebyteAddr);

  byte readData = EEPROM.readByte(0);

  currentMode = readData == 0 ? ModeManual : ModeAuto;
}
void SetNeededTemp()
{
  EEPROM.writeByte(1,neededTemp);
 //eeprom_write_byte(&tempbyteAddr, neededTemp);
}
void GetNeededTemp()
{
  //neededTemp = eeprom_read_byte(&tempbyteAddr);
  neededTemp = EEPROM.readByte(1);
}
void StartHeater()
{
  if(currentStartStage==IdleToStart)
  {
    IsIdle = false;
    Serial.println("StartHeater()");
    currentStartStage = StartFuel;
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
}
void PressedEvent(int key)
{
    switch (key)
    {
    case BUTTON_A_PIN: //AUTO
      if (currentMode == ModeManual && IsIdle)
      {
    //    tone(BEEP_PIN, 550, 100);
        currentMode = ModeAuto;
        SetAutoMode();
        break;
      }
      if(currentMode == ModeAuto)
      {
    //    tone(BEEP_PIN, 1550, 100);
        currentMode = ModeManual;
        SetAutoMode();
        break;
      }
      break;
    case BUTTON_M_PIN: //MENU
    if(!IgnoreSparkCurrent)
    {
      IgnoreSparkCurrent = true;
   //   tone(BEEP_PIN, 300, 200);
      delay(200);
     // tone(BEEP_PIN, 200, 200);
      delay(200);
 //     tone(BEEP_PIN, 300, 200);
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
void ConnectWiFi()
{
  
  while(WiFi.status() != WL_CONNECTED) 
  {
    wifiCount++;
    Serial.print("Connecting to WiFi..., Attempt:");
    Serial.println(wifiCount);
    vTaskDelay(250 / portTICK_PERIOD_MS); //wait for conversion ready
    randomSeed(micros());
    if(wifiCount>5)
     { 
        wifiCount = 0;
       return;
       }
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  wifiCount = 0;
  
}
void StopDevice()
{

  if(IsDeviceStopped)
    return;

  Serial.println("StopDevice()!!!!...");
  
  FuelRelayOff();
        vTaskDelay(500 / portTICK_PERIOD_MS); //wait for conversion ready
  MotorRelayOff();
    vTaskDelay(500 / portTICK_PERIOD_MS); //wait for conversion ready
  SparkRelayOff();
    vTaskDelay(500 / portTICK_PERIOD_MS); //wait for conversion ready
  IsDeviceStopped = true;
} 
String sparkTime="  ";
void DisplayStatus()
{
  if (!(millis() - lastLcdUpdateTime > LCD_UPDATE_TIME))
  {
    return;
  }
  lastLcdUpdateTime = millis();


 // Serial.print("IsAlarm: ");
 // Serial.print(IsAlarm);
 // Serial.print(" IsFired: ");
 // Serial.println(IsFired);

  //display.setCursor(0, 0); // установка позиции курсора
                           //
                           //  0,0 ------- 20,0
                           //   |
                           //   |
                           //  0,3
                           //
  if(IsAlarm)
  {
    display.clear();
  //  tone(BEEP_PIN, 500, 100);
    StopDevice();
    display.setCursor(3, 1); // установка позиции курсора
    if(IsNoIgnition)
      display.print(F("CBE\3A HET TOKA"));
    else
      display.print(F("  OVERHEAT!!!"));
    return;
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
  //display.print(msg);

  if(currentStartStage==IdleToStart && currentStopStage == IdleStopped) //we are not running
  {
    display.setCursor(8, 0); // установка позиции курсора
    display.print(F("IDLE "));
  }
  else if(currentStopStage != IdleStopped) //we are not running
  {
    display.setCursor(8, 0); // установка позиции курсора
    display.print(F("STOP "));
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
void ReadRTC()
{
  getLocalTime(timenow_tm);
  timenow = DateTime(timenow_tm->tm_year,timenow_tm->tm_mon,timenow_tm->tm_mday,timenow_tm->tm_hour,timenow_tm->tm_min,timenow_tm->tm_sec);
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
     // return;
  }
  //SparkCurrent = (ACS.mA_DC());// / 1000; //read spark DC in amperes
  SparkCurrent = sensor.getCurrentDC();
  Serial.print("ACS: ");
  Serial.println(SparkCurrent, DEC);
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
  if(currentStartStage==IdleToStart) //we are not running
  {
    return;
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
      currentStartStage= IdleToStart;
      break;
    default:
    break;
  }
}

void setup()
{
  Serial.begin(115200);

 //dht.setup(DHT_PIN, DHTesp::DHT22);

  WiFi.begin(ssid, password);
  
  //temerature read task     
  xTaskCreatePinnedToCore(
    weatherTask,                       /* Function to implement the task */
    "WeatherTask",                    /* Name of the task */
    4000,                           /* Stack size in words */
    NULL,                           /* Task input parameter */
    255,                              /* Priority of the task */
    NULL,                           /* Task handle. */
    0);                             /* Core where the task should run */

  // initialize the lcd
  display.init();
  // Print a message to the LCD.
  display.backlight();


  display.setCursor(4, 0);
  display.print(F("OV-65 Heater"));
  display.setCursor(5, 1);
  display.print(F("controller"));
  display.setCursor(6, 2);
  display.print(F("(c) 2020"));
  display.setCursor(3, 3);
  display.print(F("vasp@zabmail.ru"));

  ConnectWiFi();
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
  //CurrentMeter setup
  //ACS.autoMidPoint();
  //ACS.getNoisemV();

  // calibrate() method calibrates zero point of sensor,
  // It is not necessary, but may positively affect the accuracy
  // Ensure that no current flows through the sensor at this moment
  // If you are not sure that the current through the sensor will not leak during calibration - comment out this method
  Serial.println("Calibrating ACS712");
  int zero = sensor.calibrate();
  Serial.print("Done! Zpoint: ");
  Serial.println(zero);
  //beep on start
 // pinMode(BEEP_PIN, OUTPUT);
 // pinMode(BEEP_PIN_GROUND, OUTPUT);
 // digitalWrite(BEEP_PIN_GROUND, LOW);
  //пищим
  //tone(BEEP_PIN, 1000, 100);
//  delay(250);
 // tone(BEEP_PIN, 1500, 100);
  //display.cursor();
  //display.blink();
  //display.noBacklight();
  //keypad.setDebounceTime(150);
  //keypad.setHoldTime(5000);
  GetNeededTemp();
  GetAutoMode();
  vTaskDelay(1500 / portTICK_PERIOD_MS); 
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
}
void loop() //loop over all functions
{
  if(!timeAdjusted)
    SetRTC();
  ReadButtons();
  ReadRTC();
  DisplayStatus();
  if(IsAlarm)
    StopDevice();
  else
  {
    ReadACS();
    ProcessStartup();
    ProcessShutDown();
  }
}