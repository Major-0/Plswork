#include <Arduino.h>
#include "PID_v1.h"
#include "DBW_Pins.h"
#include "Settings.h"
#include "SteeringController.h"

SteeringController::SteeringController()
  : steerPID(&steerAngleUS, &PIDSteeringOutput_us, &desiredTurn_us, proportional_steering, integral_steering, derivative_steering, DIRECT) {
  //Hacky fix for not burning servo circuit
  pinMode(STEER_ON_PIN, OUTPUT);

  steerPID.SetOutputLimits(MIN_TURN_MS, MAX_TURN_MS);
  steerPID.SetSampleTime(PID_CALCULATE_TIME);
  steerPID.SetMode(AUTOMATIC);

  Steer_Servo.attach(STEER_PULSE_PIN);
  delay(1);
  //Steer_Servo.write(90);
  if (DEBUG) {
    Serial.println("Steering Setup Complete");
  }
}

SteeringController::~SteeringController() {
}

/** 
 *  
 *  @param desiredAngle 
 *  !UPDATE THIS OBSERVED INFO!

Current Calibration: (Last Update: May 14, 2023)
779 - 1000us (left)
722 - 1500us (middle)
639 - 1850us (right)

Main function to call the steering
*/

int32_t SteeringController::update(int32_t desiredAngle) {
  // desiredAngle = map(desiredAngle, MIN_TURN_Mdegrees, MAX_TURN_Mdegrees, MIN_TURN_MS, MAX_TURN_MS); // old settings
  int32_t mappedAngle = desiredAngle;

  if (desiredAngle > Straight_Read) { 
    mappedAngle = map(desiredAngle*100, Right_Read_at_MIN_TURN, Straight_Read, MIN_TURN_MS, 1500); //right turn
  } else {
    mappedAngle = map(desiredAngle*100, Straight_Read, Right_Read_at_MAX_TURN, 1500, MAX_TURN_MS); //left turn
  }

  if (USE_PIDS) {
    SteeringPID(mappedAngle);
  } else {
    engageSteering(mappedAngle);
  }
  delay(1);

  //return map(currentSteeringUS, MIN_TURN_MS,MAX_TURN_MS,MIN_TURN_Kdegrees,MAX_TURN_Kdegrees); // old settings

  return desiredAngle;
  //steerAngleUS = computeAngleRight(); // uncomment when testing with sensors
  //return steerAngleUS;
}


void SteeringController::engageSteering(int32_t desiredAmount) {
turn_left (desiredAmount);
turn_right (desiredAmount);

}


//Private
void SteeringController::SteeringPID(int32_t input) {
  desiredTurn_us = input;
  //no need for steerPID.compute() ???
  if (PIDSteeringOutput_us != currentSteeringUS) {
    Steer_Servo.writeMicroseconds(PIDSteeringOutput_us);
    currentSteeringUS = PIDSteeringOutput_us;
  }
}

// Outputs a PWM based on input (1ms - 1.85ms)
/*
void SteeringController::engageSteering(int32_t input) {
  if (input > MAX_TURN_MS)
    input = MAX_TURN_MS;
  else if (input < MIN_TURN_MS)
    input = MIN_TURN_MS;
  if (currentSteeringUS != input) {
    if (DEBUG) {
      Serial.print("MAP Steering: ");
      Serial.println(input);
    }
    currentSteeringUS = input;
  }
  Steer_Servo.writeMicroseconds(input);
  delay(1);
}
*/

////Coding Movements
// Calculates the angle from left sensor
int32_t SteeringController::computeAngleLeft() { // issues with sensor
  int32_t val = analogRead(L_SENSE_PIN);
  val = map(val, Left_Read_at_MIN_TURN, Left_Read_at_MAX_TURN, MIN_TURN_MS, MAX_TURN_MS);
  /*if(DEBUG){
    Serial.print("Left sensor: ");
    Serial.println(val);
  } */
  return val;
}

// Calculates the angle from right sensor
int32_t SteeringController::computeAngleRight() {
  int32_t val = analogRead(R_SENSE_PIN);
  val = map(val, Right_Read_at_MIN_TURN, Right_Read_at_MAX_TURN, MIN_TURN_MS, MAX_TURN_MS);
  /* if(DEBUG){
 Serial.print("Right sensor: ");
 Serial.println(val);
 */
  return val;
}

//Using the analog inputs, find the one to turn left/right
short int turn_left(int amount) {
  if (amount < 0 || amount >100 )
    return;
int reading_range = Right_Read_at_MAX_TURN - Straight_Read;
int actual_read = analogRead (R_TURN_PIN);
int desired_read = (amount * reading_range)/100;
  if (desired_read == actual_read){
        digitalWrite(L_TURN_PIN, HIGH);
        digitalWrite(R_TURN_PIN, HIGH);
         return;
  }
   if (desired_read > actual_read){
        digitalWrite(L_TURN_PIN, HIGH);
        digitalWrite(R_TURN_PIN, LOW);
      }
       else {
        digitalWrite(L_TURN_PIN, LOW);
        digitalWrite(R_TURN_PIN, HIGH);
       }
}

short int turn_right(int amount) {
  amount = -amount;
  if (amount < 0 || amount >100 )
    return;
int reading_range = Right_Read_at_MIN_TURN - Straight_Read;
int actual_read = analogRead (L_TURN_PIN);
int desired_read = (amount * reading_range)/100;
  if (desired_read == actual_read){
        digitalWrite(L_TURN_PIN, HIGH);
        digitalWrite(R_TURN_PIN, HIGH);
         return;
  }
   if (desired_read > actual_read){
        digitalWrite(L_TURN_PIN, LOW);
        digitalWrite(R_TURN_PIN, HIGH);
      }
       else {
        digitalWrite(L_TURN_PIN, HIGH);
        digitalWrite(R_TURN_PIN, LOW);
       }
}

//troubleshoot RC (serial monitor print readings) Serial monitors and verying voltages (friday)
