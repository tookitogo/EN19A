// #define USE_MCP9808
#define USE_DS18
// #define USE_AVERAGES
#define USE_FILTER

#include <Arduino.h>
#include <math.h>
// #include <LiquidCrystal.h>

#include <IoAbstraction.h>
#include <IoLogging.h>
#include <TaskManagerIO.h>
#include <IoAbstractionWire.h>

#include <PCA9624.h>

#ifdef USE_DS18
#include <OneWire.h>
#include <DallasTemperature.h>
#endif

#ifdef USE_MCP9808
#include "Adafruit_MCP9808.h"
#endif

#ifdef USE_FILTER
#include <Ewma.h>
#endif

// pin mapping
#define BUT1 32
#define BUT2 33
#define BUT3 27
#define BUT4 4
#define ALS_INT 26
#define NTC1 34
#define NTC2 35
#define SCL2 18
#define SDA2 19

#ifdef USE_DS18
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
#define ONE_WIRE_BUS 5
#define DS18_RESOLUTION 12
#define DS18_FILTER_ALPHA 0.2
#define TEMP_READING_INTERVAL 200

#ifdef USE_AVERAGES
#define averages 20       // number of readings to average
float readings[averages]; // the readings from the sensor
int readIndex = 0;        // the index of the current reading
float total = 0;          // the running total
float average = 0;        // the average
#endif

#ifdef USE_FILTER
float reading = 0;
Ewma tempFilter(DS18_FILTER_ALPHA); // Less smoothing - faster to detect changes, but more prone to noise
#endif

OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature ds18b20sensor(&oneWire);
#endif

#ifdef USE_MCP9808
#define averages 10         // number of readings to average
#define mcp9808address 0x18 // I2C address of sensor

#define TEMP_READING_INTERVAL 200

float readings[averages]; // the readings from the analog input
int readIndex = 0;        // the index of the current reading
float total = 0;          // the running total
float average = 0;        // the average
// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
#endif

// LED driver I2C addresses
const int hundredsAddress = 0x1C;
const int tensAddress = 0x19;
const int onesAddress = 0x1A;

// 7 segment to LED output mapping
const int segA = 0, segB = 1, segC = 2, segD = 4, segE = 6, segF = 7, segG = 5, segDP = 3; // final PCB

// Setup PCA9624 drivers
PCA9624 ledDrvHnds(hundredsAddress);
PCA9624 ledDrvTens(tensAddress);
PCA9624 ledDrvOnes(onesAddress);

// uint8_t displaybrightness = 128;
uint8_t globalbrightness = 255;
uint8_t currentsegmentbrightness[4][9];
uint8_t newsegmentbrightness[4][9];

uint32_t fadeInLength = 600;   // time to fade in a segment in ms
uint32_t fadeOutLength = 1500; // time to fade out a segment in ms

bool displayIsStale = true; // stores the result of whether the display needs updating

enum displaydigit
{
  hundredsDigit = 1,
  tensDigit = 2,
  onesDigit = 3
};

int temperature = -123; // temp in hundredths of a degree
float tempC = 0;
float tempF = 0;
uint8_t hundredsValue, tensValue, onesValue, tenthsValue, hundredthsValue = 0;

bool testLED = 0;
bool errorflag = 0;

enum tempUnits
{
  Celsius,
  Fahrenheit
};

tempUnits tempUnit = Celsius; // default temperature units on power-on

enum brightnessLevels
{
  lowest = 2,
  lower = 4,
  low = 8,
  medium = 16,
  medhigh = 32,
  high = 64,
  highest = 128
};

// enum brightnessLevels
// {
//   lowest = 5,
//   lower  = 10,
//   low = 20,
//   medium = 40,
//   medhigh = 60,
//   high = 90,
//   highest = 120
// };

//set default brightness on power-on
// brightnessLevels displaybrightness = highest;
brightnessLevels displaybrightness = low;

// function prototypes
//  void updateLEDdisplay(void);
void fadeInLEDs(void);
void fadeOutLEDs(void);
bool checkDisplayStaleness(void);
void decode7seg(displaydigit, char);
void setDecimal(displaydigit, bool);
void updateFilter(void);
void tempFormatter(void);
void ds18b20read();
void getTemperature(void);
void toggleLED(void);
void onKeyDown(pinid_t, bool);
void onKeyUp(pinid_t, bool);

void setup()
{

  // Serial.begin(115200);  // ESP32 crashes without this, even if not actually needed

  // lcd.begin(16, 2);
  // taskManager.yieldForMicros(1000);

#ifdef USE_DS18

  // start 1Wire temp sensor
  ds18b20sensor.begin();

#ifdef USE_AVERAGES
  ds18b20sensor.setResolution(12); // force max resolution for initial reading

  // initialize readings array with initial reading:
  ds18b20read();
  total = averages * tempC;

  for (int thisReading = 0; thisReading < averages; thisReading++)
  {
    readings[thisReading] = tempC;
    }

  #endif
  #ifdef USE_FILTER
    ds18b20sensor.setResolution(12); // force max resolution for initial reading
    //initialize temperature with one unfiltered high-res reading:
    ds18b20read();

#endif
    ds18b20sensor.setResolution(DS18_RESOLUTION); //set final resolution for all subsequent readings

#endif

#ifdef USE_MCP9808
  // initialize all the readings with :
  for (int thisReading = 0; thisReading < averages; thisReading++)
  {
    readings[thisReading] = 0;
  }

  if (!tempsensor.begin(mcp9808address))
  {
    // Serial.println("Couldn't find MCP9808.");
    errorflag = 1;
    while (1)
      ;
  }

  // Serial.println("Found MCP9808.");
  errorflag = 0;

  tempsensor.setResolution(3); // sets the resolution mode of reading, the modes are defined in the table below:
  // Mode Resolution SampleTime
  //  0    0.5°C       30 ms
  //  1    0.25°C      65 ms
  //  2    0.125°C     130 ms
  //  3    0.0625°C    250 ms
#endif

  // set up IOAbstraction handler for button
  auto *deviceIo = internalDigitalIo();
  ioDevicePinMode(deviceIo, BUT1, INPUT);

  switches.init(deviceIo, SWITCHES_NO_POLLING, true);

  switches.addSwitch(BUT1, onKeyDown);
  switches.onRelease(BUT1, onKeyUp);

  // initialize brightness arrays
  for (int digit = 1; digit < 4; digit++)
  {
    for (int i = 0; i < 8; i++)
    {
      currentsegmentbrightness[digit][i] = 255; // functions as LED test by turning all to max brightness on power up
      newsegmentbrightness[digit][i] = 33;
    }
  }

  // calculate rate for LED updates in microseconds
  uint32_t fadeInPeriod = (fadeInLength * 1000) / 255;
  uint32_t fadeOutPeriod = (fadeOutLength * 1000) / 255;

  // set up LED drivers
  ledDrvHnds.begin();
  ledDrvTens.begin();
  ledDrvOnes.begin();
  ledDrvHnds.groupPwm(globalbrightness);
  ledDrvTens.groupPwm(globalbrightness);
  ledDrvOnes.groupPwm(globalbrightness);
  ledDrvHnds.allOn();
  ledDrvTens.allOn();
  ledDrvOnes.allOn();

  // schedule one-time tasks

  // schedule regular tasks
  taskManager.scheduleFixedRate(fadeInPeriod, fadeInLEDs, TIME_MICROS);
  taskManager.scheduleFixedRate(fadeOutPeriod, fadeOutLEDs, TIME_MICROS);
  taskManager.scheduleFixedRate(TEMP_READING_INTERVAL, getTemperature);
  // taskManager.scheduleFixedRate(100, updateFilter);
  taskManager.scheduleFixedRate(10, tempFormatter);

  // taskManager.scheduleFixedRate(2500000, toggleLED, TIME_MICROS);

  // taskManager.scheduleFixedRate(50, getTemperature);
  // taskManager.scheduleFixedRate(50, tempFormatter);

  // decode7seg(hundredsDigit, '2');
  // setDecimal(hundredsDigit, false);
  // decode7seg(tensDigit, '7');
  // setDecimal(tensDigit, true);
  // decode7seg(onesDigit, '5');
  // setDecimal(onesDigit, false);

  // clear display
  setDecimal(hundredsDigit, false);
  setDecimal(tensDigit, false);
  setDecimal(onesDigit, false);

  tempFormatter();

  // Serial.println("Setup complete.");
}



void loop()
{
  // put your main code here, to run repeatedly:
  taskManager.runLoop();

  // display formatting test sequence
  //  setDecimal(onesDigit, false);
  //  taskManager.yieldForMicros(5000000);
  //  setDecimal(onesDigit, true);
  //  taskManager.yieldForMicros(5000000);
  //  taskManager.yieldForMicros(5000000);

  // temperature = -9999;
  // taskManager.yieldForMicros(5000000);
  // temperature = -888;
  // taskManager.yieldForMicros(5000000);
  // temperature = -77;
  // taskManager.yieldForMicros(5000000);
  // temperature = -6;
  // taskManager.yieldForMicros(5000000);
  // temperature = -0;
  // taskManager.yieldForMicros(5000000);
  // temperature = 0;
  // taskManager.yieldForMicros(5000000);
  // temperature = 1;
  // taskManager.yieldForMicros(5000000);
  // temperature = 22;
  // taskManager.yieldForMicros(5000000);
  // temperature = 333;
  // taskManager.yieldForMicros(5000000);
  // temperature = 4444;
  // taskManager.yieldForMicros(5000000);
  // temperature = 55555;
  // taskManager.yieldForMicros(5000000);
}

// reads temperature into buffer

// version with averaging
#ifdef USE_DS18
void getTemperature()
{
  if (displayIsStale == false)
  {
#ifdef USE_AVERAGES
    // subtract the last reading:
    total = total - readings[readIndex];
    // read from the sensor:
    ds18b20sensor.requestTemperatures(); // Send the command to get temperatures
    readings[readIndex] = ds18b20sensor.getTempCByIndex(0);
    // add the reading to the total:
    // total = total + readings[readIndex];

    total = 0;
    for (int i = 0; i < averages; i++)
    {
      total = total + readings[i];
    }

    // advance to the next position in the array:
    readIndex = readIndex + 1;

    // if we're at the end of the array...
    if (readIndex >= averages)
    {
      // ...wrap around to the beginning:
      readIndex = 0;
    }

    // calculate the average:
    average = total / averages;

    tempC = average;

    if (readings[readIndex] != DEVICE_DISCONNECTED_C)
    {
      errorflag = 0;
    }
    else
    {
      errorflag = 1;
      temperature = -999999;
    }
#endif

#ifdef USE_FILTER

    // read from the sensor:
    ds18b20sensor.requestTemperatures(); // Send the command to get temperatures
    reading = ds18b20sensor.getTempCByIndex(0);

    tempC = tempFilter.filter(reading);

    if (reading != DEVICE_DISCONNECTED_C)
    {
      errorflag = 0;
    }
    else
    {
      errorflag = 1;
      temperature = -999999;
      tempFilter.reset();
    }

#endif

    // Serial.printf("display temp value: %d\n", temperature);
    // Serial.printf("DS18B20 temp: %f\n", tempC);
  }

  // void getTemperature()
  // {
  //   ds18b20read();
  // }
}
#endif

// original version without averaging
#ifdef USE_DS18
void ds18b20read()
{
  ds18b20sensor.requestTemperatures(); // Send the command to get temperatures

  tempC = ds18b20sensor.getTempCByIndex(0);
  // Check if reading was successful
  if (tempC != DEVICE_DISCONNECTED_C)
  {
    errorflag = 0;
  }
  else
  {
    errorflag = 1;
    temperature = -999999;
  }
  // Serial.printf("display temp value: %d\n", temperature);
  // Serial.printf("DS18B20 temp: %f\n", tempC);
}
#endif

#ifdef USE_MCP9808
void getTemperature()
{
  if (displayIsStale == false)
  {
    tempsensor.wake();

    // subtract the last reading:
    total = total - readings[readIndex];
    // read from the sensor:
    readings[readIndex] = tempsensor.readTempC();
    // add the reading to the total:
    total = total + readings[readIndex];
    // advance to the next position in the array:
    readIndex = readIndex + 1;

    // if we're at the end of the array...
    if (readIndex >= averages)
    {
      // ...wrap around to the beginning:
      readIndex = 0;
    }

    // calculate the average:
    average = total / averages;

    tempC = average;

    // //clear tempC to zero, then add up readings, then divide by number of readings to get average
    // tempC = 0;
    // for (int i = 0; i<averages; i++)
    // {
    //     tempC = tempC + tempsensor.readTempC();
    // }
    // tempC = tempC/averages;
    tempsensor.shutdown();
  }
}
#endif

bool checkDisplayStaleness(void)
{
  for (int digit = 1; digit < 4; digit++)
  {
    for (int i = 0; i < 8; i++)
    {
      if (newsegmentbrightness[digit][i] != currentsegmentbrightness[digit][i])
      {
        return true; // return true when stale
      }
    }
  }
  return false; // return false if all of the segments are identical
}

void updateFilter(void)
{
  #ifdef USE_FILTER
    if (displayIsStale == false)
    {
      tempC = tempFilter.filter(reading);
    }
   #endif
}

// converts temperature buffer to formatted number
void tempFormatter(void)
{
  switch (tempUnit)
  {
  case Celsius:
#ifdef USE_DS18
    temperature = round(tempC * 10) * 10; // originally used rint() here and in Fahrenheit. Was this to catch some edge case?? -- round to 1/10 degree here
#endif

#ifdef USE_MCP9808
    temperature = round(tempC * 100); // originally used rint() here and in Fahrenheit. Was this to catch some edge case?? -- round to 1/100 degree
#endif

    setDecimal(onesDigit, false); // turns off top-right annunciator
    break;
  case Fahrenheit:
    tempF = (tempC * 1.8) + 32;
    temperature = round(tempF * 100);
    setDecimal(onesDigit, true); // turns on top-right annunciator
    break;
  }

  // handle each digit (can probably simplify by sk)
  int inputtemp = temperature; // copying to new variable is probably not needed, I think I did it to double-buffer in case the temperature gets updated partway through
  hundredsValue = 0x30 + abs(inputtemp) / 10000;
  tensValue = 0x30 + (abs(inputtemp) % 10000) / 1000;
  onesValue = 0x30 + (abs(inputtemp) % 1000) / 100;
  tenthsValue = 0x30 + (abs(inputtemp) % 100 / 10);
  hundredthsValue = 0x30 + abs(inputtemp) % 10;

#ifdef USE_MCP9808
  // disconnected sensor after startup gives gigantic value, so catch it to set error flag
  if (inputtemp > 100000000) // temps above 1 million
  {
    errorflag = true;
  }
  else
  {
    errorflag = false;
  }
#endif

  if (inputtemp >= 10000) // temps 100 and above: nnn

  {
    // lcd.setCursor(0, 0);
    // // lcd.println("100 and above");
    // lcd.print(hundredsValue);
    // lcd.print(tensValue);
    // lcd.print(onesValue);
    // lcd.print("             ");

    decode7seg(hundredsDigit, hundredsValue);
    setDecimal(hundredsDigit, false);
    decode7seg(tensDigit, tensValue);
    setDecimal(tensDigit, false);
    decode7seg(onesDigit, onesValue);
  }
  else if (inputtemp >= 1000) // temps between 10 and 99.9: nn.n
  {
    // lcd.setCursor(0, 0);
    // // lcd.println("10 to 99.9");
    // lcd.print(tensValue);
    // lcd.print(onesValue);
    // lcd.print(".");
    // lcd.print(tenthsValue);
    // lcd.print("             ");

    decode7seg(hundredsDigit, tensValue);
    setDecimal(hundredsDigit, false);
    decode7seg(tensDigit, onesValue);
    setDecimal(tensDigit, true);
    decode7seg(onesDigit, tenthsValue);
  }

  // use two decimals of resolution on MCP9808
#ifdef USE_MCP9808
  else if (inputtemp >= 0) // positive temps 9.99 and below -- version with two decimals of resolution: n.nn
  {
    // lcd.setCursor(0, 0);
    // // lcd.println("0 to 9.99");
    // lcd.print(onesValue);
    // lcd.print(".");
    // lcd.print(tenthsValue);
    // lcd.print(hundredthsValue);
    // lcd.print("             ");

    decode7seg(hundredsDigit, onesValue);
    setDecimal(hundredsDigit, true);
    decode7seg(tensDigit, tenthsValue);
    setDecimal(tensDigit, false);
    decode7seg(onesDigit, hundredthsValue);
  }
#endif

// use one decimal of resolution on DS18
#ifdef USE_DS18
  else if (inputtemp >= 0) // positive temps 9.99 and below -- version with just one decimal of resolution: n.n
  {
    //   lcd.setCursor(0, 0);
    //   // lcd.println("0 to 9.99");
    //   lcd.print(" ");
    //   lcd.print(onesValue);
    //   lcd.print(".");
    //   lcd.print(tenthsValue);
    //   lcd.print("             ");

    decode7seg(hundredsDigit, 0);
    setDecimal(hundredsDigit, false);
    decode7seg(tensDigit, onesValue);
    setDecimal(tensDigit, true);
    decode7seg(onesDigit, tenthsValue);
  }
#endif

  else if (inputtemp > -1000) // negative temps above -10: -n.n
  {
    // lcd.setCursor(0, 0);
    // // lcd.println("-0.1 to -9.9");
    // lcd.print("-");
    // lcd.print(onesValue);
    // lcd.print(".");
    // lcd.print(tenthsValue);
    // lcd.print("             ");

    decode7seg(hundredsDigit, '-');
    setDecimal(hundredsDigit, false);
    decode7seg(tensDigit, onesValue);
    setDecimal(tensDigit, true);
    decode7seg(onesDigit, tenthsValue);
  }

  else if (inputtemp <= -1000) // negative temps -10 and below: -nn
  {
    // lcd.setCursor(0, 0);
    // // lcd.println ("-10 and below");
    // lcd.print("-");
    // lcd.print(tensValue);
    // lcd.print(onesValue);
    // lcd.print("             ");

    decode7seg(hundredsDigit, '-');
    setDecimal(hundredsDigit, false);
    decode7seg(tensDigit, tensValue);
    setDecimal(tensDigit, false);
    decode7seg(onesDigit, onesValue);
  }

  if (errorflag) // overwrite prior with error message if applicable
  {
    decode7seg(hundredsDigit, 'E');
    setDecimal(hundredsDigit, false);
    decode7seg(tensDigit, 'r');
    setDecimal(tensDigit, false);
    decode7seg(onesDigit, 'r');
  }
}

// decode input text into 7-segment display
void decode7seg(displaydigit digit, char value)
{
  switch (value)
  {
  case '0':
    newsegmentbrightness[digit][segA] = displaybrightness;
    newsegmentbrightness[digit][segB] = displaybrightness;
    newsegmentbrightness[digit][segC] = displaybrightness;
    newsegmentbrightness[digit][segD] = displaybrightness;
    newsegmentbrightness[digit][segE] = displaybrightness;
    newsegmentbrightness[digit][segF] = displaybrightness;
    newsegmentbrightness[digit][segG] = 0;
    break;

  case '1':
    newsegmentbrightness[digit][segA] = 0;
    newsegmentbrightness[digit][segB] = displaybrightness;
    newsegmentbrightness[digit][segC] = displaybrightness;
    newsegmentbrightness[digit][segD] = 0;
    newsegmentbrightness[digit][segE] = 0;
    newsegmentbrightness[digit][segF] = 0;
    newsegmentbrightness[digit][segG] = 0;
    break;

  case '2':
    newsegmentbrightness[digit][segA] = displaybrightness;
    newsegmentbrightness[digit][segB] = displaybrightness;
    newsegmentbrightness[digit][segC] = 0;
    newsegmentbrightness[digit][segD] = displaybrightness;
    newsegmentbrightness[digit][segE] = displaybrightness;
    newsegmentbrightness[digit][segF] = 0;
    newsegmentbrightness[digit][segG] = displaybrightness;
    break;

  case '3':
    newsegmentbrightness[digit][segA] = displaybrightness;
    newsegmentbrightness[digit][segB] = displaybrightness;
    newsegmentbrightness[digit][segC] = displaybrightness;
    newsegmentbrightness[digit][segD] = displaybrightness;
    newsegmentbrightness[digit][segE] = 0;
    newsegmentbrightness[digit][segF] = 0;
    newsegmentbrightness[digit][segG] = displaybrightness;
    break;

  case '4':
    newsegmentbrightness[digit][segA] = 0;
    newsegmentbrightness[digit][segB] = displaybrightness;
    newsegmentbrightness[digit][segC] = displaybrightness;
    newsegmentbrightness[digit][segD] = 0;
    newsegmentbrightness[digit][segE] = 0;
    newsegmentbrightness[digit][segF] = displaybrightness;
    newsegmentbrightness[digit][segG] = displaybrightness;
    break;

  case '5':
    newsegmentbrightness[digit][segA] = displaybrightness;
    newsegmentbrightness[digit][segB] = 0;
    newsegmentbrightness[digit][segC] = displaybrightness;
    newsegmentbrightness[digit][segD] = displaybrightness;
    newsegmentbrightness[digit][segE] = 0;
    newsegmentbrightness[digit][segF] = displaybrightness;
    newsegmentbrightness[digit][segG] = displaybrightness;
    break;

  case '6':
    newsegmentbrightness[digit][segA] = displaybrightness;
    newsegmentbrightness[digit][segB] = 0;
    newsegmentbrightness[digit][segC] = displaybrightness;
    newsegmentbrightness[digit][segD] = displaybrightness;
    newsegmentbrightness[digit][segE] = displaybrightness;
    newsegmentbrightness[digit][segF] = displaybrightness;
    newsegmentbrightness[digit][segG] = displaybrightness;
    break;

  case '7':
    newsegmentbrightness[digit][segA] = displaybrightness;
    newsegmentbrightness[digit][segB] = displaybrightness;
    newsegmentbrightness[digit][segC] = displaybrightness;
    newsegmentbrightness[digit][segD] = 0;
    newsegmentbrightness[digit][segE] = 0;
    newsegmentbrightness[digit][segF] = 0;
    newsegmentbrightness[digit][segG] = 0;
    break;

  case '8':
    newsegmentbrightness[digit][segA] = displaybrightness;
    newsegmentbrightness[digit][segB] = displaybrightness;
    newsegmentbrightness[digit][segC] = displaybrightness;
    newsegmentbrightness[digit][segD] = displaybrightness;
    newsegmentbrightness[digit][segE] = displaybrightness;
    newsegmentbrightness[digit][segF] = displaybrightness;
    newsegmentbrightness[digit][segG] = displaybrightness;
    break;

  case '9':
    newsegmentbrightness[digit][segA] = displaybrightness;
    newsegmentbrightness[digit][segB] = displaybrightness;
    newsegmentbrightness[digit][segC] = displaybrightness;
    newsegmentbrightness[digit][segD] = displaybrightness;
    newsegmentbrightness[digit][segE] = 0;
    newsegmentbrightness[digit][segF] = displaybrightness;
    newsegmentbrightness[digit][segG] = displaybrightness;
    break;

  case 'A':
    newsegmentbrightness[digit][segA] = displaybrightness;
    newsegmentbrightness[digit][segB] = displaybrightness;
    newsegmentbrightness[digit][segC] = displaybrightness;
    newsegmentbrightness[digit][segD] = 0;
    newsegmentbrightness[digit][segE] = displaybrightness;
    newsegmentbrightness[digit][segF] = displaybrightness;
    newsegmentbrightness[digit][segG] = displaybrightness;
    break;

  case 'B':
    newsegmentbrightness[digit][segA] = 0;
    newsegmentbrightness[digit][segB] = 0;
    newsegmentbrightness[digit][segC] = displaybrightness;
    newsegmentbrightness[digit][segD] = displaybrightness;
    newsegmentbrightness[digit][segE] = displaybrightness;
    newsegmentbrightness[digit][segF] = displaybrightness;
    newsegmentbrightness[digit][segG] = displaybrightness;
    break;

  case 'C':
    newsegmentbrightness[digit][segA] = displaybrightness;
    newsegmentbrightness[digit][segB] = 0;
    newsegmentbrightness[digit][segC] = 0;
    newsegmentbrightness[digit][segD] = displaybrightness;
    newsegmentbrightness[digit][segE] = displaybrightness;
    newsegmentbrightness[digit][segF] = displaybrightness;
    newsegmentbrightness[digit][segG] = 0;
    break;

  case 'D':
    newsegmentbrightness[digit][segA] = 0;
    newsegmentbrightness[digit][segB] = displaybrightness;
    newsegmentbrightness[digit][segC] = displaybrightness;
    newsegmentbrightness[digit][segD] = displaybrightness;
    newsegmentbrightness[digit][segE] = 0;
    newsegmentbrightness[digit][segF] = displaybrightness;
    newsegmentbrightness[digit][segG] = displaybrightness;
    break;

  case 'E':
    newsegmentbrightness[digit][segA] = displaybrightness;
    newsegmentbrightness[digit][segB] = 0;
    newsegmentbrightness[digit][segC] = 0;
    newsegmentbrightness[digit][segD] = displaybrightness;
    newsegmentbrightness[digit][segE] = displaybrightness;
    newsegmentbrightness[digit][segF] = displaybrightness;
    newsegmentbrightness[digit][segG] = displaybrightness;
    break;

  case 'F':
    newsegmentbrightness[digit][segA] = displaybrightness;
    newsegmentbrightness[digit][segB] = 0;
    newsegmentbrightness[digit][segC] = 0;
    newsegmentbrightness[digit][segD] = 0;
    newsegmentbrightness[digit][segE] = displaybrightness;
    newsegmentbrightness[digit][segF] = displaybrightness;
    newsegmentbrightness[digit][segG] = displaybrightness;
    break;

  case 'i':
    newsegmentbrightness[digit][segA] = 0;
    newsegmentbrightness[digit][segB] = 0;
    newsegmentbrightness[digit][segC] = displaybrightness;
    newsegmentbrightness[digit][segD] = 0;
    newsegmentbrightness[digit][segE] = 0;
    newsegmentbrightness[digit][segF] = 0;
    newsegmentbrightness[digit][segG] = 0;
    break;

  case 'n':
    newsegmentbrightness[digit][segA] = 0;
    newsegmentbrightness[digit][segB] = 0;
    newsegmentbrightness[digit][segC] = displaybrightness;
    newsegmentbrightness[digit][segD] = 0;
    newsegmentbrightness[digit][segE] = displaybrightness;
    newsegmentbrightness[digit][segF] = 0;
    newsegmentbrightness[digit][segG] = displaybrightness;
    break;

  case 'o':
    newsegmentbrightness[digit][segA] = 0;
    newsegmentbrightness[digit][segB] = 0;
    newsegmentbrightness[digit][segC] = displaybrightness;
    newsegmentbrightness[digit][segD] = displaybrightness;
    newsegmentbrightness[digit][segE] = displaybrightness;
    newsegmentbrightness[digit][segF] = 0;
    newsegmentbrightness[digit][segG] = displaybrightness;
    break;

  case 'r':
    newsegmentbrightness[digit][segA] = 0;
    newsegmentbrightness[digit][segB] = 0;
    newsegmentbrightness[digit][segC] = 0;
    newsegmentbrightness[digit][segD] = 0;
    newsegmentbrightness[digit][segE] = displaybrightness;
    newsegmentbrightness[digit][segF] = 0;
    newsegmentbrightness[digit][segG] = displaybrightness;
    break;

  case 't':
    newsegmentbrightness[digit][segA] = 0;
    newsegmentbrightness[digit][segB] = 0;
    newsegmentbrightness[digit][segC] = 0;
    newsegmentbrightness[digit][segD] = displaybrightness;
    newsegmentbrightness[digit][segE] = displaybrightness;
    newsegmentbrightness[digit][segF] = displaybrightness;
    newsegmentbrightness[digit][segG] = displaybrightness;
    break;
  case 'u':
    newsegmentbrightness[digit][segA] = 0;
    newsegmentbrightness[digit][segB] = 0;
    newsegmentbrightness[digit][segC] = displaybrightness;
    newsegmentbrightness[digit][segD] = displaybrightness;
    newsegmentbrightness[digit][segE] = displaybrightness;
    newsegmentbrightness[digit][segF] = 0;
    newsegmentbrightness[digit][segG] = 0;
    break;

  case '-':
    newsegmentbrightness[digit][segA] = 0;
    newsegmentbrightness[digit][segB] = 0;
    newsegmentbrightness[digit][segC] = 0;
    newsegmentbrightness[digit][segD] = 0;
    newsegmentbrightness[digit][segE] = 0;
    newsegmentbrightness[digit][segF] = 0;
    newsegmentbrightness[digit][segG] = displaybrightness;
    break;

  default:
    newsegmentbrightness[digit][segA] = 0;
    newsegmentbrightness[digit][segB] = 0;
    newsegmentbrightness[digit][segC] = 0;
    newsegmentbrightness[digit][segD] = 0;
    newsegmentbrightness[digit][segE] = 0;
    newsegmentbrightness[digit][segF] = 0;
    newsegmentbrightness[digit][segG] = 0;
    break;
  }
}

// control decimal points and onesDigit annunciator
void setDecimal(displaydigit digit, bool value) // also used for onesDigit annunciator
{
  if (value == true)
  {
    newsegmentbrightness[digit][segDP] = displaybrightness * 70 / 100;
  }
  else
  {
    newsegmentbrightness[digit][segDP] = 0;
  }
}

// increment display brightness changes at same speed on in/out

// void updateLEDdisplay(void)
// {
//   updateLEDdigit(hundredsDigit);
//   updateLEDdigit(tensDigit);
//   updateLEDdigit(onesDigit);

// }

// void updateLEDdigit(displaydigit digit)
// {
//   for(int i = 0; i < 8; i++)
//   {
//       if (newsegmentbrightness[digit][i] > currentsegmentbrightness[digit][i])
//       {
//       currentsegmentbrightness[digit][i]++;
//       ledDrvHnds.pwm(i,currentsegmentbrightness[digit][i]);
//       ledDrvTens.pwm(i,currentsegmentbrightness[digit][i]);
//       ledDrvOnes.pwm(i,currentsegmentbrightness[digit][i]);
//       }
//       else if (newsegmentbrightness[digit][i] < currentsegmentbrightness[digit][i])
//       {
//       currentsegmentbrightness[digit][i]--;
//       ledDrvHnds.pwm(i,currentsegmentbrightness[digit][i]);
//       ledDrvTens.pwm(i,currentsegmentbrightness[digit][i]);
//       ledDrvOnes.pwm(i,currentsegmentbrightness[digit][i]);
//       }
//   }
// }

// separate into two paths for fading up/down to allow different fade speeds
void fadeInLEDs(void)
{
  displayIsStale = checkDisplayStaleness();
  if (displayIsStale == true)
  {
    for (int digit = 1; digit < 4; digit++)
    {
      for (int i = 0; i < 8; i++)
      {
        if (newsegmentbrightness[digit][i] > currentsegmentbrightness[digit][i])
        {
          currentsegmentbrightness[digit][i]++;
          switch (digit)
          {
          case 1:
            ledDrvHnds.pwm(i, currentsegmentbrightness[digit][i]);
            break;
          case 2:
            ledDrvTens.pwm(i, currentsegmentbrightness[digit][i]);
            break;
          case 3:
            ledDrvOnes.pwm(i, currentsegmentbrightness[digit][i]);
            break;
          }
        }
      }
    }
  }
}

void fadeOutLEDs(void)
{
  displayIsStale = checkDisplayStaleness();
  if (displayIsStale == true)
  {
    for (int digit = 1; digit < 4; digit++)
    {
      for (int i = 0; i < 8; i++)
      {
        if (newsegmentbrightness[digit][i] < currentsegmentbrightness[digit][i])
        {
          currentsegmentbrightness[digit][i]--;
          switch (digit)
          {
          case 1:
            ledDrvHnds.pwm(i, currentsegmentbrightness[digit][i]);
            break;
          case 2:
            ledDrvTens.pwm(i, currentsegmentbrightness[digit][i]);
            break;
          case 3:
            ledDrvOnes.pwm(i, currentsegmentbrightness[digit][i]);
            break;
          }
        }
      }
    }
  }
}

// LED blinker display test
void toggleLED(void)
{
  setDecimal(onesDigit, testLED);

  testLED = !testLED;
}

void onKeyUp(pinid_t key, bool held)
{
  switch (key)
  {
  case BUT1:
    if (held)
    { // no action on long key up
    }
    else
    {
      switch (displaybrightness)
      {
      case lowest:
        displaybrightness = lower;
        break;
      case lower:
        displaybrightness = low;
        break;
      case low:
        displaybrightness = medium;
        break;
      case medium:
        displaybrightness = medhigh;
        break;
      case medhigh:
        displaybrightness = high;
        break;
      case high:
        displaybrightness = highest;
        break;
      case highest:
        displaybrightness = lowest;
        break;
      }
      tempFormatter;
    }
    break;
  case BUT2:
    break;
  case BUT3:
    break;
  case BUT4:
    break;
  default:
    break;
  }
}

void onKeyDown(pinid_t key, bool held)
{
  switch (key)
  {
  case BUT1:
    if (held) // toggle temperature unit on long hold
    {
      if (tempUnit == Fahrenheit)
      {
        tempUnit = Celsius;
      }
      else
      {
        tempUnit = Fahrenheit;
      }
      tempFormatter();
    }
    else
    {
    }
    break;
  case BUT2:
    break;
  case BUT3:
    break;
  case BUT4:
    break;
  default:
    break;
  }
}

void Task1Code(void *pvParameters)
{
  // taskManager.runLoop();
}

void Task2Code(void *pvParameters)
{
  // displayThread.runLoop();
}