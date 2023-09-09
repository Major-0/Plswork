#pragma once
#include <Servo.h>
#include "PID_v1.h"

class SteeringController {
  Servo Steer_Servo;
  PID steerPID;
  double steerAngleUS;
  double PIDSteeringOutput_us;
  double desiredTurn_us;
  int32_t currentSteeringUS = 0;
  void SteeringPID(int32_t input);
  int32_t computeAngleLeft();
  short int turn_left(int amount); //amount is % of turn 100% is full turn
  int32_t computeAngleRight();
  short int turn_right(int amount);
  void engageSteering(int32_t input);
public:
  SteeringController();
  ~SteeringController();
  int32_t update(int32_t desiredAngle);
};
