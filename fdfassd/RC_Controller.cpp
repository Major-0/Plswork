#include <Arduino.h>
#include "RC_Controller.h"
#include "Settings.h"
#include "DBW_Pins.h"
#include <SPI.h>
#include "Vehicle.h"
#include <SD.h>
#include <RTClib.h>
#include <Wire.h>
#include <DS1307RTC.h>
#include "Can_Protocol.h"

#include <Arduino.h>
#include "DBW_Pins.h"
unsigned volatile long RC_Controller::riseTime[RC_NUM_SIGNALS] = { 0 };     // rising edge, beginning of pulse
unsigned volatile long RC_Controller::elapsedTime[RC_NUM_SIGNALS] = { 0 };  // falling edge, end of pulse

unsigned long RC_Controller::RC_RISE[RC_NUM_SIGNALS] = { 0 };     // stores the beginning of pulse
unsigned long RC_Controller::RC_ELAPSED[RC_NUM_SIGNALS] = { 0 };  // stores the total pulse width
const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};
tmElements_t tm;

const int chipSelect  =53; //chipSelect pin for the SD card Reader
File logfile; //Data object you will write your sesnor data

RC_Controller::RC_Controller() {

  // Setup input for RC reciever
  pinMode(STEERING_CH1_PIN, INPUT);
  pinMode(THROTTLE_BR_CH2_PIN, INPUT);

  // Interrupts to pins and ISR functions
  attachInterrupt(digitalPinToInterrupt(STEERING_CH1_PIN), ISR_STEERING_RISE, RISING);
  attachInterrupt(digitalPinToInterrupt(THROTTLE_BR_CH2_PIN), ISR_THROTTLE_RISE, RISING);

  // Initialize default values
  for (int i = 0; i < RC_NUM_SIGNALS; i++) {
    RC_VALUES_MAPPED[i] = 0;  // keeps track of mapped values
    previousTime[i] = 0;      // keep track of the time for each signal
  }
  prevSteering = 0;       // keep track of steering data
  prevThrottleBrake = 0;  // keep track of throttle data
}


/*******************************
Initilze SD Card
******************/ 

void RC_Controller::initalize(){
bool parse=false;
  bool config=false;

  // get the date and time the compiler was run
  if (getDate(__DATE__) && getTime(__TIME__)) {
    parse = true;
    // and configure the RTC with this info
    if (RTC.write(tm)) {
      config = true;
    }
  }

  if (parse && config) {
    Serial.print("DS1307 configured Time=");
    Serial.print(__TIME__);
    Serial.print(", Date=");
    Serial.println(__DATE__);
  } else if (parse) {
    Serial.println("DS1307 Communication Error :-{");
    Serial.println("Please check your circuitry");
  } else {
    Serial.print("Could not parse info from the compiler, Time=\"");
    Serial.print(__TIME__);
    Serial.print("\", Date=\"");
    Serial.print(__DATE__);
    Serial.println("\"");
  }
      if (RTC.read(tm)) {
    Serial.print("Ok, Time = ");
    print2digits(tm.Hour);
    Serial.write(':');
    print2digits(tm.Minute);
    Serial.write(':');
    print2digits(tm.Second);
    Serial.print(", Date (D/M/Y) = ");
    Serial.print(tm.Day);
    Serial.write('/');
    Serial.print(tm.Month);
    Serial.write('/');
    Serial.print(tmYearToCalendar(tm.Year));
    Serial.println();
  } else {
    if (RTC.chipPresent()) {
      Serial.println("The DS1307 is stopped.  Please run the SetTime");
      Serial.println("example to initialize the time and begin running.");
      Serial.println();
    } else {
      Serial.println("DS1307 read error!  Please check the circuitry.");
      Serial.println();
    }
    //delay(9000);
  }
 // delay(2000);


// initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(53, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(53)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  
  // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  
  if (! logfile) {
    error("couldnt create file");
  }
  
  Serial.print("Logging to: ");
  Serial.println(filename);
logfile.println("time_ms, desired_speed_ms, desired_break,Desired_angle, current_speed, current_break, current_angle,throttle_pulse,steerpulse, break_hold");


}


RC_Controller::~RC_Controller() {initalize();}

// Obtain the mapped values for each channel
long RC_Controller::getMappedValue(int channel) {
  return RC_VALUES_MAPPED[channel];
}

// Calls other function to map pulse widths accordingly
void RC_Controller::mapValues() {
  mapThrottleBrake();
  mapSteering();
}

// Maps value into steering
// 779 (Left - 1020 us), 722 (Straight - 1440us), 639 (Right - 1860us)
void RC_Controller::mapSteering() {
  unsigned long currentTime = micros();

  // smooth steering, can change value 2000 is arbitrary value
  if (currentTime - previousTime[RC_CH1_STEERING] >= 2000) {

    int steeringValue = 0;
    long pulseWidth = RC_ELAPSED[RC_CH1_STEERING];
    if (pulseWidth < 1000 || pulseWidth > 2000) {  // invalid data, out of range
      return;
    }

    // filtering pulse widths
    if (prevSteering == pulseWidth) {
      if (pulseWidth > 1500) {  // calibrated steering values
        steeringValue = map(pulseWidth, 1012, 1440, 779, 722);
      } else {
        steeringValue = map(pulseWidth, 1440, 1860, 722, 639);
      }
      
      RC_VALUES_MAPPED[RC_CH1_STEERING] = steeringValue;
    }

    Serial.println("Steering:" + String(prevSteering));
    
    previousTime[RC_CH1_STEERING] = currentTime;  // update time
    prevSteering = pulseWidth;                    // steering data
    steeringFlag = 1;                             // valid data
  }
}

// Maps values for throttle (0 - 255); >= 1500us pulse width
// Also calls brake (<1100 us pulse width)
void RC_Controller::mapThrottleBrake() {
  unsigned long currentTime = micros();

  // filtering pulse width at next period cycle (~16.6ms)
  if (currentTime - previousTime[RC_CH2_THROTTLE_BR] >= 16600) {

    long pulseWidth = RC_ELAPSED[RC_CH2_THROTTLE_BR];
    if (pulseWidth < 1000 || pulseWidth > 2000) {  // invalid data, out of range
      return;
    }

    // filter pulse widths
    if (prevThrottleBrake == pulseWidth) {
      if (pulseWidth < 1100) {  // brakes
        RC_VALUES_MAPPED[RC_CH2_THROTTLE_BR] = -1;
        Serial.println("Throttle:" + String(pulseWidth));
       
      } else if(pulseWidth >= 1500) {                                                    // throttle
        int throttleValue = map(pulseWidth, 1500, 2000, 0, 120);  // maximum to 120 counts, increase if needed
        RC_VALUES_MAPPED[RC_CH2_THROTTLE_BR] = throttleValue;
      }
    }

    Serial.println("Throttle:" + String(pulseWidth));
LogSD();
    previousTime[RC_CH2_THROTTLE_BR] = currentTime;  // update time
    prevThrottleBrake = pulseWidth;                  // throttle or brake
    throttleBrakeFlag = 1;                           // valid data
  }
}

// Checks for valid data for steering, throttle, and brakes
bool RC_Controller::checkValidData() {
  return (throttleBrakeFlag && steeringFlag);
}

// Clears flag after data processing
void RC_Controller::clearFlag() {
  throttleBrakeFlag = 0;
  steeringFlag = 0;
}


/* Interrupt Service Routine */
void RC_Controller::ISR_STEERING_RISE() {
  if (digitalRead(STEERING_CH1_PIN) == HIGH) {  // filter pulse widths
    noInterrupts();
    riseTime[RC_CH1_STEERING] = micros();
    RC_RISE[RC_CH1_STEERING] = riseTime[RC_CH1_STEERING];
    attachInterrupt(digitalPinToInterrupt(STEERING_CH1_PIN), ISR_STEERING_FALL, FALLING);
    interrupts();
  }
}

void RC_Controller::ISR_STEERING_FALL() {
  if (digitalRead(STEERING_CH1_PIN) == LOW) {  // filter pulse widths
    noInterrupts();
    elapsedTime[RC_CH1_STEERING] = micros() - RC_RISE[RC_CH1_STEERING];
    RC_ELAPSED[RC_CH1_STEERING] = elapsedTime[RC_CH1_STEERING];
    attachInterrupt(digitalPinToInterrupt(STEERING_CH1_PIN), ISR_STEERING_RISE, RISING);
    interrupts();
  }
}

void RC_Controller::ISR_THROTTLE_RISE() {
  if (digitalRead(THROTTLE_BR_CH2_PIN) == HIGH) {  // filter pulse widths
    noInterrupts();
    riseTime[RC_CH2_THROTTLE_BR] = micros();
    RC_RISE[RC_CH2_THROTTLE_BR] = riseTime[RC_CH2_THROTTLE_BR];
    attachInterrupt(digitalPinToInterrupt(THROTTLE_BR_CH2_PIN), ISR_THROTTLE_FALL, FALLING);
    interrupts();
  }
}

void RC_Controller::ISR_THROTTLE_FALL() {
  if (digitalRead(THROTTLE_BR_CH2_PIN) == LOW) {  // filter pulse widths
    noInterrupts();
    elapsedTime[RC_CH2_THROTTLE_BR] = micros() - RC_RISE[RC_CH2_THROTTLE_BR];
    RC_ELAPSED[RC_CH2_THROTTLE_BR] = elapsedTime[RC_CH2_THROTTLE_BR];
    attachInterrupt(digitalPinToInterrupt(THROTTLE_BR_CH2_PIN), ISR_THROTTLE_RISE, RISING);
    interrupts();
  }
}

void RC_Controller::LogSD()
{
 
  logfile.print(millis());logfile.print(", ");
  logfile.print(prevSteering);logfile.print(", ");
  logfile.print(pulseWidth);logfile.print(", ");
  logfile.flush();


}

void RC_Controller::error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);

  while(1);
}




void RC_Controller::print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}


bool RC_Controller::getTime(const char *str)
{
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm.Hour = Hour;
  tm.Minute = Min;
  tm.Second = Sec;
  return true;
}


bool RC_Controller::getDate(const char *str)
{
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  tm.Day = Day;
  tm.Month = monthIndex + 1;
  tm.Year = CalendarYrToTm(Year);
  return true;
}
