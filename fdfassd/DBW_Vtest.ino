/* Drive_By_Wire //DATALOGGER
 *  Accept commands over CAN bus
 *  Implement vehicle throttle, brakes and steering as commanded
 *  Report status over CAN bus
 */
#include "DBW_Pins.h"
#include <SPI.h>
#include "Vehicle.h"
#include <SD.h>
#include <RTClib.h>
#include <Wire.h>
#include <DS1307RTC.h>

#define baud 115200  // baudrate for debugging with a host PC over USB serial

Vehicle *myTrike;





// A simple data logger for the Arduino analog pins
#define LOG_INTERVAL  1000 // mills between entries

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 1000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()



// The analog pins that connect to the sensors
#define voltPin A0           // analog A0



// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 53;

// the logging file
//File logfile;

//Timing stuff
#define LOOP_TIME_MS 100
uint32_t nextTime;
uint32_t endTime;
uint32_t delayTime;
#define ULONG_MAX 0x7FFFFFFF

void setup(void) {
  Serial.begin(baud);
  while (!Serial) ; // wait for serial
  delay(2000);

  if (DEBUG) {
    Serial.println("main Setup complete");
  }

   myTrike = new Vehicle();
  Serial.println("2");
}

void loop() {

  //delay(2000);
  

 



  //myTrike->update();
  myTrike->updateRC();



}

