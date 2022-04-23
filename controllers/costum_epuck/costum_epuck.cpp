#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

#include <iostream>

#define TIME_STEP 64
#define MAX_SPEED 6.28

using namespace webots;
using namespace std;

int main (int argc, char** argv){
  cout << "costum epuck" << endl;
  cout << "hey hey hey" << endl;
  Robot* robot = new Robot();

  Motor *leftMotor = robot->getMotor("left wheel motor");
  leftMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  
  Motor *rightMotor = robot->getMotor("right wheel motor");
  rightMotor->setPosition(INFINITY);
  rightMotor->setVelocity(0.0);

  while (robot->step(TIME_STEP) != -1) {
    double speed = MAX_SPEED;

    leftMotor->setVelocity(speed);
    rightMotor->setVelocity(speed);
  }

  delete robot;
  return 0;
}