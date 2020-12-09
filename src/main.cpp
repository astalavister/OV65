#include <Arduino.h>
//#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Keypad.h> //keypad lib
#include "ACS712.h"
#include "RTClib.h"
#include <I2C_EEPROM.h>
#include <Bounce2.h>

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

#define DS_PIN A0            // DS18B20 data pin
#define CURRENT_IGN_PIN A1   //Current meter pin

#define NUM_BUTTONS 2
const uint8_t BUTTON_PINS[NUM_BUTTONS] = {A2, A3};
Bounce * buttons = new Bounce[NUM_BUTTONS];
//#define TEMP_WORK_PIN A2     //Working temp sensor
//#define TEMP_OVERHEAT_PIN A3 //Overheat sensor pin

#define BEEP_PIN 12              //Beeper
#define BEEP_PIN_GROUND 13       //Beeper Ground
#define MOTOR_RELAY_PIN 8       //motor on/off relay
#define MOTOR_SPEED_RELAY_PIN 9 //motor speed relay (1/2)
#define FUEL_VALVE_RELAY_PIN 10   // fuel valve relay
#define IGNITION_RELAY_PIN 11     // ingition relay

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
DateTime dtStartIgnition;
DateTime dtStartHalfMotor;
DateTime dtStopIgnition;

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

String msg = "";

//EEPROM
I2C_EEPROM memory(0x50); // on RTC board

//RTC
RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
DateTime dtStart;
DateTime timenow;
long lastRtcUpdateTime;
const int RTC_UPDATE_TIME = 1000; // Определяем периодичность проверок

//ACS
ACS712 ACS(CURRENT_IGN_PIN, 5.0, 1023, 66); //30 AMPERS sensor
long lastAcsUpdateTime;
const int ACS_UPDATE_TIME = 1000; // Определяем периодичность проверок

///KEYBOARD
const byte ROWS = 1; // строки
const byte COLS = 5; // столбца
char keys[ROWS][COLS] = {{'A', 'M', 'L', 'R', 'P'}};
byte rowPins[ROWS] = {3};             // подключить к выводам строк клавиатуры
byte colPins[COLS] = {7, 6, 5, 2, 4}; // подключить к выводам столбцов клавиатуры
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// set the LCD address to 0x27 for a 20 chars and 4 line display
LiquidCrystal_I2C display(0x27, 20, 4);
long lastLcdUpdateTime = 20000;  // Переменная для хранения времени ПОКАЗА НА lcd
const int LCD_UPDATE_TIME = 500; // Определяем периодичность ПОКАЗА НА lcd

//Temperarure sensor
OneWire oneWire(DS_PIN); // Создаем объект OneWire для шины 1-Wire, с помощью которого будет осуществляться работа с датчиком
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer, outsideThermometer;
float CurrentTemp = 99;            // Глобальная переменная для хранения значение температуры с датчика DS18B20
long lastUpdateTime = 20000;        // Переменная для хранения времени последнего считывания с датчика
const int TEMP_UPDATE_TIME = 15000; // Определяем периодичность проверок
byte neededTemp = 20;

char *timeresult = new char[7]{0, 0, 0, 0, 0, 0, 0};
void (*resetFunc)(void) = 0; //объявляем функцию reset с адресом 0

bool curMotorRelayState = false;
int MotorSpeed = 1;

bool SparkRelayState = false;
int SparkCurrent = 0;

bool FuelRelayState = false;

void FuelRelayOn()
{
  //Serial.println("Fuel is ON");
  if (FuelRelayState == false)
  {
    digitalWrite(FUEL_VALVE_RELAY_PIN, LOW);
    tone(BEEP_PIN, 2000, 200);
    delay(150);
    tone(BEEP_PIN, 1800, 200);
    FuelRelayState = true;
  }
}
void FuelRelayOff()
{
  //Serial.println("Fuel is OFF");
  if (FuelRelayState == true)
  {
    digitalWrite(FUEL_VALVE_RELAY_PIN, HIGH);
    tone(BEEP_PIN, 1800, 200);
    delay(150);
    tone(BEEP_PIN, 2000, 200);
    FuelRelayState = false;
  }
}
DateTime sparkStartTime;
void SparkRelayOn()
{
  //Serial.println("Spark is ON");
  if (SparkRelayState == false)
  {
    digitalWrite(IGNITION_RELAY_PIN, LOW);
    tone(BEEP_PIN, 2000, 200);
    delay(150);
    tone(BEEP_PIN, 1800, 200);
    SparkRelayState = true;
    sparkStartTime = timenow;
  }
}
void SparkRelayOff()
{
  //Serial.println("Spark is OFF");
  if (SparkRelayState == true)
  {
    digitalWrite(IGNITION_RELAY_PIN, HIGH);
    tone(BEEP_PIN, 1800, 200);
    delay(150);
    tone(BEEP_PIN, 2000, 200);
    SparkRelayState = false;
  }
}
void MotorRelayOn()
{
  //Serial.println("Motor ON");
  if (curMotorRelayState == false)
  {
    digitalWrite(MOTOR_RELAY_PIN, LOW);
    tone(BEEP_PIN, 2000, 200);
    delay(150);
    tone(BEEP_PIN, 1800, 200);
    curMotorRelayState = true;
  }
}
void MotorRelayOff()
{
  //Serial.println("Motor OFF");
  if (curMotorRelayState == true)
  {
    digitalWrite(MOTOR_RELAY_PIN, HIGH);
    tone(BEEP_PIN, 1800, 200);
    delay(150);
    tone(BEEP_PIN, 2000, 200);
    curMotorRelayState = false;
  }
}
void MotorSpeed2()
{
  //Serial.println("Motor Speed 2");
  digitalWrite(MOTOR_SPEED_RELAY_PIN, LOW);
  tone(BEEP_PIN, 2000, 200);
  MotorSpeed = 2;

}
void MotorSpeed1()
{
    //Serial.println("Motor Speed 1");
    digitalWrite(MOTOR_SPEED_RELAY_PIN, HIGH);
    tone(BEEP_PIN, 1800, 200);
    MotorSpeed = 1;
}
void SetAutoMode()
{
  memory.write(0x00, currentMode == ModeManual ? 0x00 : 0x01);
}
void GetAutoMode()
{
  byte readData = memory.read(0x00);
  currentMode = readData == 0 ? ModeManual : ModeAuto;
}
void SetNeededTemp()
{
  memory.write(0x01, neededTemp);
}
void GetNeededTemp()
{
  neededTemp = memory.read(0x01);
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
  if(currentStopStage==HeaterStarted)
  {
    currentStopStage = StopFuel;
    Serial.println("StopHeater()");
  }
}
void keypadEvent(Key key)
{
  switch (key.kstate)
  {
  case IDLE:
    //tone(BEEP_PIN, 1000, 330);
    break;
  case HOLD:
    //tone(BEEP_PIN, 1000, 330);
    //Serial.print("HOLD...");
    //Serial.println(key.kchar);
    if (key.kchar == 'P')
       resetFunc();
    break;
  case RELEASED:
    //tone(BEEP_PIN, 1000, 330);
    //Serial.print("PRESSES...");
    //Serial.println(key.kchar);
    break;
  case PRESSED:
        if(IsAlarm)
        break;
    //Serial.print("PRESSES...");
    //Serial.println(key.kchar);
    switch (key.kchar)
    {
    case 'A': //AUTO
      if (currentMode == ModeManual && IsIdle)
      {
        tone(BEEP_PIN, 550, 100);
        currentMode = ModeAuto;
        SetAutoMode();
        break;
      }
      if(currentMode == ModeAuto)
      {
        tone(BEEP_PIN, 1550, 100);
        currentMode = ModeManual;
        SetAutoMode();
        break;
      }
      break;
    case 'M': //MENU
      break;
    case 'L': //LEFT
      if (neededTemp > 11 && currentMode == ModeAuto)
      {
        neededTemp--;
        SetNeededTemp();
        tone(BEEP_PIN, 2000, 100);
      }
      else
      {
        tone(BEEP_PIN, 800, 100);
      }
      //menu.previous_screen();
      break;
    case 'R': //RIGHT
      if (neededTemp < 30 && currentMode == ModeAuto)
      {
        neededTemp++;
        SetNeededTemp();
        tone(BEEP_PIN, 2000, 100);
      }
      else
      {
        tone(BEEP_PIN, 800, 100);
      }
      break;
    case 'P': //POWER
      if (currentMode == ModeManual)
      {
        if(IsIdle)
        {
          tone(BEEP_PIN, 900, 100);
          StartHeater();
        } else
        {
          tone(BEEP_PIN, 900, 100);
          StopHeater();
        }
      }
      break;
    default: //all other keys
      break;
    }
    break;
  }
}
void ReadKeyboard()
{
  if (keypad.getKeys())
  {
    for (int i = 0; i < LIST_MAX; i++) // Scan the whole key list.
    {
      if (keypad.key[i].stateChanged) // Only find keys that have changed state.
      {
        keypadEvent(keypad.key[i]);
      }
    }
  }
}
void setup()
{
  memory.init();
  //timeresult = (char*)malloc(6);
  //Serial.begin(115200);
    // initialize the lcd
  display.init();
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
  delay(1000);
  //Serial.println("Starting RTC...");
  display.setCursor(3, 3);
  display.print(F("Starting RTC..."));
  //Serial.flush();
  if (!rtc.begin())
  {
    display.setCursor(3, 3);
    display.print(F("NO RTC!!!"));
    //Serial.println("Couldn't find RTC");
    //Serial.flush();
    abort();
  }
  if (!rtc.isrunning())
  {
    //Serial.println("RTC is NOT running, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  dtStart = rtc.now();
  delay(300);
  display.setCursor(1, 3);
  display.print(F("Starting DS Sensor"));

   // Start up the sensors library
  sensors.begin();
  if (!sensors.getAddress(insideThermometer, 0)) 
  {
    display.setCursor(3, 3);
    display.print(F("NO DS Sensor!!!"));
  //Serial.println("Couldn't find RTC");
  //Serial.flush();
    abort();
  }

 


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

  //inputs from heater sensors (LOW when they is ON)
  for (int i = 0; i < NUM_BUTTONS; i++) {
    buttons[i].attach( BUTTON_PINS[i] , INPUT_PULLUP  );       //setup the bounce instance for the current button
    buttons[i].interval(125); // interval in ms
  }
  //pinMode(TEMP_WORK_PIN, INPUT_PULLUP);
  //pinMode(TEMP_OVERHEAT_PIN, INPUT_PULLUP);

  // CurrentMeter setup
  ACS.autoMidPoint();

  //beep on start
  pinMode(BEEP_PIN, OUTPUT);
  pinMode(BEEP_PIN_GROUND, OUTPUT);
  digitalWrite(BEEP_PIN_GROUND, LOW);
  //пищим
  tone(BEEP_PIN, 1000, 100);
  delay(250);
  tone(BEEP_PIN, 1500, 100);
  //display.cursor();
  //display.blink();
  //display.noBacklight();
  keypad.setDebounceTime(100);
  keypad.setHoldTime(5000);
  GetNeededTemp();
  GetAutoMode();
  delay(1000);
  display.clear();
  display.createChar(1, bukva_P);  // Создаем символ под номером 1
  display.createChar(2, bukva_I);  // Создаем символ под номером 2
  display.createChar(3, bukva_CH); // Создаем символ под номером 3
  display.createChar(4, bukva_Y);  // Создаем символ под номером 4
  display.createChar(5, bukva_IY); // Создаем символ под номером 5
  display.createChar(6, bukva_G); // Создаем символ под номером 6
  display.createChar(7, bukva_L); // Создаем символ под номером 7
}
void StopDevice()
{
  if(IsDeviceStopped)
    return;
  FuelRelayOff();
    delay(500);
  MotorRelayOff();
    delay(500);
  SparkRelayOff();
    delay(500);
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
    tone(BEEP_PIN, 500, 100);
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
  display.print(msg);
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
  display.print(F("TEM\1EPAT\4PA"));
  display.setCursor(12, 3);
  display.print(CurrentTemp);

  if (currentMode == ModeManual)
  {
    display.setCursor(16, 3);
    display.print(F("    "));
  }
  else
  {
    display.setCursor(16, 3);
    display.print(F("["));
    display.print(neededTemp);
    display.print(F("]"));
  }

  display.display();
}

byte i;
//byte present = 0;
byte type_s;
byte data[12];
byte addr[8];

void detectTemperature()
{
  //float celsius, fahrenheit;
  if (millis() - lastUpdateTime > TEMP_UPDATE_TIME)
  {
    //Serial.println("Temp: Measuring...");
    lastUpdateTime = millis();
    sensors.requestTemperatures();
    float tempC = sensors.getTempC(insideThermometer);
    CurrentTemp = tempC;
  }
}
void ReadRTC()
{
  if (!(millis() - lastRtcUpdateTime > RTC_UPDATE_TIME))
  {
    return;
  }
  lastRtcUpdateTime = millis();
  timenow = rtc.now();
  //sprintf(timeresult, "%02d:%02d", timenow.hour(), timenow.minute());
  /* Serial.print(timenow.year(), DEC);
  Serial.print('/');
  Serial.print(timenow.month(), DEC);
  Serial.print('/');
  Serial.print(timenow.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[timenow.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(timenow.hour(), DEC);
  Serial.print(':');
  Serial.print(timenow.minute(), DEC);
  Serial.print(':');
  Serial.print(timenow.second(), DEC);
  Serial.println(); */
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
      return;
  }
  SparkCurrent = (ACS.mA_DC()) / 1000; //read spark DC in amperes
  //Serial.print("ACS: ");
  //Serial.println(SparkCurrent, DEC);
}
bool CheckIgnition()
{
  ReadACS();
  if(SparkCurrent < 4)
  {
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
    FuelRelayOn();
    SparkRelayOn();
    currentStartStage = StartIgnition;
    dtStartIgnition = timenow;
    delay(500); //make relay time to switch
}
void StartIgnitionFn()
{
    
    if(!CheckIgnition())
      return;

    TimeSpan sparkTime = timenow - dtStartIgnition;

    //msg = sparkTime.totalseconds();
    if (sparkTime.totalseconds() > 20 )
    {
      MotorSpeed1();
      delay(200);
      MotorRelayOn();
      delay(200);
      tone(BEEP_PIN, 4000, 100);
      delay(500);
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
      tone(BEEP_PIN, 4000, 100);
      delay(500);
    }

}
void StopIgnitionFn()
{
    //TimeSpan stopIgnTime = timenow - dtStopIgnition;
    if (IsFired) // we are heated already
    {
      MotorSpeed2();
      currentStartStage = StartFullMotor;
      delay(500);
    }
}
void ProcessStartup()
{
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
  switch (currentStopStage)
  {
    case IdleToStart:
      return;
      break;
    case StartFuel:
      break;
    default:
    break;
  }
}
void loop() //loop over all functions
{

  //check sensors
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

  ReadRTC();
  DisplayStatus();
  ReadKeyboard();
  if(IsAlarm)
    StopDevice();
  else
  {
    detectTemperature(); // Определяем температуру от датчика DS18b20
    ProcessStartup();
    ProcessShutDown();
  }
}