#include "motion.hpp"
#include "algebra.hpp"

#include <Action.h>
#include <MX28.h>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

#include <cmath>
#include <iostream>
#include <string>

using namespace Robot;
using namespace webots;
using namespace std;

const string motor_name[20] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
  "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
  "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
  "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/
};

MotionRobot::MotionRobot(webots::Robot* robot, string motion_file){
  myRobot = robot;
  timeStep = myRobot->getBasicTimeStep();

  for (int i = 0; i < 20; i++) {
    current_position[i] = 0.0;
    motor[i] = myRobot->getMotor(motor_name[i]);
    string sensor_name = motor_name[i];
    sensor_name.push_back('S');
    position_sensor[i] = myRobot->getPositionSensor(sensor_name);
    position_sensor[i]->enable(timeStep);
    min_motor_position[i] = motor[i]->getMinPosition();
    max_motor_position[i] = motor[i]->getMaxPosition();
  }

  action = Action::GetInstance();
  string path = "data/motion/" + motion_file;
  if (action->LoadFile((char *)path.c_str()) == false) {
    cerr << "failed to load motion file" << endl;
    return;
  }
}

MotionRobot::~MotionRobot(){
}

void MotionRobot::playPage(int id) {
  Action::PAGE page;
  if (action->LoadPage(id, &page)) {
    for (int i = 0; i < page.header.repeat; i++) {
      for (int j = 0; j < page.header.stepnum; j++) {
        for (int k = 0; k < 20; k++)
          target_position[k] = valueToPosition(page.step[j].position[k + 1]);
        achieveTarget(8 * page.step[j].time);
        wait(8 * page.step[j].pause);
      }
    }
    if (page.header.next != 0)
      playPage(page.header.next);
  } else
    cerr << "Cannot load the page" << endl;
}

void MotionRobot::myStep(){
  if (myRobot->step(timeStep) == -1) exit(EXIT_SUCCESS);
}

void MotionRobot::wait(int duration) {
  double start = myRobot->getTime();
  double time = (double)duration / 1000.0;
  while (start + time >= myRobot->getTime()) myStep();
}

void MotionRobot::achieveTarget(int timeToAchieveTarget) {
  int stepNumberToAchieveTarget = timeToAchieveTarget / timeStep;

  myStep();
  for (int i = 0; i < 20; i++){
    if (position_sensor[i]->getSamplingPeriod() <= 0) {
      cerr << "position sensor not enabled" << endl;
    } else {
      current_position[i] = position_sensor[i]->getValue();
    }
  }

  while (stepNumberToAchieveTarget > 0) {
    for (int i = 0; i < 20; i++) {
      double dX = target_position[i] - current_position[i];
      double newPosition = current_position[i] + dX / stepNumberToAchieveTarget;
      newPosition = alg::clampValue(newPosition, min_motor_position[i], max_motor_position[i]);
      current_position[i] = newPosition;
      motor[i]->setPosition(current_position[i]);
    }
    myStep();
    stepNumberToAchieveTarget--;
  }
}

double MotionRobot::valueToPosition(unsigned short value) {
  double degree = MX28::Value2Angle(value);
  double position = degree / 180.0 * M_PI;
  return position;
}