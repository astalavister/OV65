#include <Arduino.h>
//#include <EEPROM.h>
//#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <Keypad.h> //keypad lib
#include "ACS712.h"
#include "RTClib.h"
#include <I2C_EEPROM.h>



#define DS_PIN A0 // DS18B20 data pin
#define CURRENT_IGN_PIN A1 //Current meter pin
#define TEMP_WORK_PIN A2 //Working temp sensor
#define TEMP_OVERHEAT_PIN A3 //Overheat sensor pin

#define BEEP_PIN 12 //Beeper
#define BEEP_PIN_GROUND 13 //Beeper Ground
#define MOTOR_RELAY_PIN 11 //motor on/off relay
#define MOTOR_SPEED_RELAY_PIN 10 //motor speed relay (1/2)
#define FUEL_VALVE_RELAY_PIN 9 // fuel valve relay
#define IGNITION_RELAY_PIN 8 // ingition relay

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
ACS712  ACS(CURRENT_IGN_PIN, 5.0, 1023, 66);//30 AMPERS sensor
long lastAcsUpdateTime;
const int ACS_UPDATE_TIME = 1000; // Определяем периодичность проверок

///KEYBOARD
const byte ROWS = 1; // строки
const byte COLS = 5; // столбца
char keys[ROWS][COLS] = {{'A','M','L','R','P'}};
byte rowPins[ROWS] = {7}; // подключить к выводам строк клавиатуры
byte colPins[COLS] = {6, 5, 4, 3, 2}; // подключить к выводам столбцов клавиатуры
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

// set the LCD address to 0x27 for a 20 chars and 4 line display
LiquidCrystal_I2C display(0x27,20,4);  
long lastLcdUpdateTime = 20000; // Переменная для хранения времени ПОКАЗА НА lcd
const int LCD_UPDATE_TIME = 500; // Определяем периодичность ПОКАЗА НА lcd


//Temperarure sensor
OneWire ds(DS_PIN); // Создаем объект OneWire для шины 1-Wire, с помощью которого будет осуществляться работа с датчиком

float CurrentTemp = 99.99; // Глобальная переменная для хранения значение температуры с датчика DS18B20

long lastUpdateTime = 20000; // Переменная для хранения времени последнего считывания с датчика

const int TEMP_UPDATE_TIME = 5000; // Определяем периодичность проверок



void(* resetFunc) (void) = 0;//объявляем функцию reset с адресом 0

bool automode = false;

enum DisplayMode 
{
   ModeManual = 0,
   ModeAuto,
   ModeSetup,
};

bool curMotorRelayState = false;
int MotorSpeed = 1;

bool SparkState = false;
int SparkCurrent = 0;

void SetAutoMode(bool modeset)
{
    automode = modeset; 
    memory.write(0x00, automode == true ? 0x01 : 0x00);
}
bool GetAutoMode()
{
  //bool a = EEPROM.read(0);
  //Serial.print(a);
  //return EEPROM.read(0);
  byte readData = memory.read(0x00);
  automode = readData == 1;
  return automode;
}

void keypadEvent(KeypadEvent key)
{
  switch (keypad.getState())
  {
    case IDLE:
    // tone(Beep, 1000, 30);
    break;
    case HOLD:
    // tone(Beep, 1000, 30);
    break;
    case PRESSED:
    // tone(Beep, 1000, 30);
    break;
    case RELEASED:
    tone(BEEP_PIN, 800, 30);
      switch (key)
      {
        case 'A': //AUTO
          break;
        case 'M': //MENU
          break;
        case 'L': //LEFT
          break;
        case 'R': //RIGHT
          break;
        case 'P': //POWER
          break;
        default://all other keys
        break;
     }
  }
}
DisplayMode currDisplay = ModeManual;
DisplayMode prevDisplay = ModeManual;
char *timeresult;
void setup()
{

  memory.init();

  timeresult = (char*)malloc(6);
  Serial.begin(9600);
  delay(500);
  Serial.println("Starting RTC...");
  Serial.flush();



  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  if (! rtc.isrunning())
  {
    Serial.println("RTC is NOT running, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  dtStart = rtc.now();

  //relays (used modules are on with LOW!!! state)
  pinMode(MOTOR_RELAY_PIN, INPUT_PULLUP);
  pinMode(MOTOR_RELAY_PIN, OUTPUT);
  digitalWrite(MOTOR_RELAY_PIN, HIGH);//off at start

  pinMode(MOTOR_SPEED_RELAY_PIN, INPUT_PULLUP);
  pinMode(MOTOR_SPEED_RELAY_PIN, OUTPUT);
  digitalWrite(MOTOR_SPEED_RELAY_PIN, HIGH);//speed one at start

  pinMode(FUEL_VALVE_RELAY_PIN, INPUT_PULLUP);
  pinMode(FUEL_VALVE_RELAY_PIN, OUTPUT);
  digitalWrite(FUEL_VALVE_RELAY_PIN, HIGH);//off at start

  pinMode(IGNITION_RELAY_PIN, INPUT_PULLUP);
  pinMode(IGNITION_RELAY_PIN, OUTPUT);
  digitalWrite(IGNITION_RELAY_PIN, HIGH);//off at start

  //inputs from heater sensors (LOW when they is ON)
  pinMode(TEMP_WORK_PIN , INPUT_PULLUP);
  pinMode(TEMP_OVERHEAT_PIN , INPUT_PULLUP);
  
  // CurrentMeter setup
  ACS.autoMidPoint();

  // initialize the lcd 
  display.init();                      
  display.init();
  // Print a message to the LCD.
  display.backlight();
  display.setCursor(4,0);
  display.print("OV-65 Heater");
  display.setCursor(5,1);
  display.print("controller");
  display.setCursor(6,2);
  display.print("(c) 2020");
  display.setCursor(3,3);
  display.print("vasp@zabmail.ru");

  //beep on start
  pinMode(BEEP_PIN, OUTPUT);
  pinMode(BEEP_PIN_GROUND, OUTPUT);
  digitalWrite(BEEP_PIN_GROUND, LOW);
  //пищим
  tone(BEEP_PIN, 1000, 100);
  delay(250);
  tone(BEEP_PIN, 1200, 100);
  delay(250);
  tone(BEEP_PIN, 1500, 100);
  delay(350);
  tone(BEEP_PIN, 1800, 100); 
  display.clear();
  //display.cursor();
  //display.blink();
  //display.noBacklight();
  keypad.addEventListener(keypadEvent); // добавить слушателя события для данной клавиатуры
  keypad.setDebounceTime(150);
  keypad.setHoldTime(1000);
  automode = GetAutoMode();
}

int ignitorMA = 0;
void DisplayStatus()
{

  if (!(millis() - lastLcdUpdateTime > LCD_UPDATE_TIME))
  {
    return;
  }
  lastLcdUpdateTime = millis();

   // Serial.println(temperature); // Выводим полученное значение температуры
    
    if(prevDisplay!=currDisplay)
      display.clear();
      
    display.setCursor(0,0); // установка позиции курсора
        //
        //  0,0 ------- 20,0
        //   |
        //   |
        //  0,3
        //
    switch (currDisplay)
    {
      case ModeManual:
      {
          display.setCursor(0,0); // установка позиции курсора
          display.print("Manual"); 
 
          sprintf(timeresult, "%02d:%02d", timenow.hour(),timenow.minute());
          display.setCursor(15,0); // установка позиции курсора
          display.print(timeresult);

          display.setCursor(0,1); // установка позиции курсора
          display.print("BEHT :"); 
          display.setCursor(6,1); 
          if(!curMotorRelayState)
            display.print("OFF"); 
          else
            display.print("ON"); 
          display.setCursor(10,1); 
          display.print("CKOP:"); 
          display.setCursor(16,1); 
          display.print(MotorSpeed); 

          display.setCursor(0,2); // установка позиции курсора
          display.print("CBE4A:"); 
          display.setCursor(6,2); 
          if(!SparkState)
            display.print("OFF"); 
          else
            display.print("ON"); 
          display.setCursor(10,2); 
          display.print("TOK:"); 
          display.setCursor(15,2); 
          display.print(SparkCurrent); 
          display.setCursor(19,2); 
          display.print("A"); 

          display.setCursor(4,3);   
          display.print("TEMP:"); 
          display.setCursor(10,3); 
          display.print(CurrentTemp); 

      }
      break;
      /*case ModeAuto:
      {
        //get data first
        ReadWeather();
        display.setTextSize(1);  // установка размера шрифта
        display.println("Humidity:");
        display.setTextSize(2);  // установка размера шрифта
        display.print(h);
        display.println("%");
        display.setTextSize(1);  // установка размера шрифта
        display.println("Temperature:");
        display.setTextSize(2);  // установка размера шрифта
        display.print(t);
        display.println((char)247);
      }
      break;
       {
      case ModeSetup:
        display.setTextSize(2);  // установка размера шрифта
        display.print("Lm:");
        display.println(wflowrate);
        display.print(" L:");
        display.println(totalMilliLitres/1000);
      }
      break;*/
      default:
      break;
    }
    display.display();
}

void detectTemperature()
{
  byte i; 
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  //float celsius, fahrenheit;
  
  if (millis() - lastUpdateTime > TEMP_UPDATE_TIME)
  {
    Serial.println("Temp: Measuring...");
    lastUpdateTime = millis();
    
    if ( !ds.search(addr)) 
    {
      Serial.println("Temp: No Sensor.");
      Serial.println();
      ds.reset_search();
      //delay(250);
      //CurrentTemp = 99.99;
      return;
    }
    Serial.print("ROM =");
    for( i = 0; i < 8; i++) 
    {
      Serial.write(' ');
      Serial.print(addr[i], HEX);
    }
    if (OneWire::crc8(addr, 7) != addr[7]) 
    {
      Serial.println("Temp: CRC is not valid!");
      return;
    }
    Serial.println();
    // первый байт определяет чип
    switch (addr[0]) 
    {
      case 0x10:
      Serial.println(" Chip = DS18S20"); // или более старый DS1820
      type_s = 1;
      break;
      case 0x28:
      Serial.println(" Chip = DS18B20");
      type_s = 0;
      break;
      case 0x22:
      Serial.println(" Chip = DS1822");
      type_s = 0;
      break;
      default:
      Serial.println("Device is not a DS18x20 family device.");
      CurrentTemp = 99.99;
      return;
    }
    ds.reset();
    ds.select(addr);
    ds.write(0x44); // начинаем преобразование, используя ds.write(0x44,1) с "паразитным" питанием
    delay(1000); // 750 может быть достаточно, а может быть и не хватит
    // мы могли бы использовать тут ds.depower(), но reset позаботится об этом
    present = ds.reset();
    ds.select(addr);
    ds.write(0xBE);
    Serial.print(" Data = ");
    Serial.print(present, HEX);
    Serial.print(" ");
    for ( i = 0; i < 9; i++) 
    { // нам необходимо 9 байт
      data[i] = ds.read();
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.print(" CRC=");
    Serial.print(OneWire::crc8(data, 8), HEX);
    Serial.println();
    // конвертируем данный в фактическую температуру
    // так как результат является 16 битным целым, его надо хранить в
    // переменной с типом данных "int16_t", которая всегда равна 16 битам,
    // даже если мы проводим компиляцию на 32-х битном процессоре
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) 
    {
      raw = raw << 3; // разрешение 9 бит по умолчанию
      if (data[7] == 0x10) 
      {
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
     } else 
     {
        byte cfg = (data[4] & 0x60);
        // при маленьких значениях, малые биты не определены, давайте их обнулим
        if (cfg == 0x00) 
          raw = raw & ~7; // разрешение 9 бит, 93.75 мс
        else if 
          (cfg == 0x20) raw = raw & ~3; // разрешение 10 бит, 187.5 мс
        else if 
          (cfg == 0x40) raw = raw & ~1; // разрешение 11 бит, 375 мс
        //// разрешение по умолчанию равно 12 бит, время преобразования - 750 мс
      }
      CurrentTemp = (float)raw / 16.0;
      //fahrenheit = celsius * 1.8 + 32.0;
      Serial.print("Temperature = ");
      Serial.print(CurrentTemp);
      Serial.println(" Celsius");
     // Serial.print(fahrenheit);
     // Serial.println(" Fahrenheit");
     ds.depower();
    }  
}
void MotorRelayOn()
{
    if(curMotorRelayState==false)
    {
      digitalWrite(MOTOR_RELAY_PIN, LOW);
      curMotorRelayState = true;
      tone(BEEP_PIN, 2000, 200);
      delay(150);
      tone(BEEP_PIN, 1800, 200);
    } else
    {
      //already on
    }
}
void RelayOff()
{
    if(curMotorRelayState==true)
    {
      digitalWrite(MOTOR_RELAY_PIN, HIGH);
      curMotorRelayState = false;
      tone(BEEP_PIN, 1800, 200);
      delay(150);
      tone(BEEP_PIN, 2000, 200);
    } 
    else
    {
      //already off  
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
  SparkCurrent = (ACS.mA_DC()) / 1000; //read ignitor spark current
  Serial.print("ACS: ");
  Serial.println(SparkCurrent,DEC);
}
void loop()
{
  ReadRTC();
  DisplayStatus();
  detectTemperature(); // Определяем температуру от датчика DS18b20
  ReadACS();//ток свечи
}