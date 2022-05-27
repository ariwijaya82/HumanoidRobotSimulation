#include "motion.hpp"
#include "algebra.hpp"

#include <MX28.h>

#include <fstream>

std::string motorNames[DMM_NMOTORS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
  "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
  "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
  "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/
};

MotionRobot::MotionRobot(webots::Robot* robot) :
mRobot(robot)
{
  if (!mRobot) {
    std::cerr << "robot instance is required" << std::endl;
    return;
  }
  mBasicTimeStep = mRobot->getBasicTimeStep();

  myStep();
  for (int i = 0; i < DMM_NMOTORS; i++) {
    mCurrentPositions[i] = 0.0;
    mMotors[i] = mRobot->getMotor(motorNames[i]);
    std::string sensorName = motorNames[i];
    sensorName.push_back('S');
    mPositionSensors[i] = mRobot->getPositionSensor(sensorName);
    mPositionSensors[i]->enable(mBasicTimeStep);
    minMotorPositions[i] = mMotors[i]->getMinPosition();
    maxMotorPositions[i] = mMotors[i]->getMaxPosition();
  }
}

MotionRobot::~MotionRobot() {
}

void MotionRobot::playMotion(std::string file_name) {
  std::string path = "data/motion/" + file_name;
  std::ifstream file(path);
  nlohmann::json data = nlohmann::json::parse(file);
  for (int i = 0; i < data["poses"].size(); i++) {
    for (int j = 0; j < DMM_NMOTORS; j++) {
      mTargetPositions[j] = data["poses"][i]["joints"][motorNames[j]].get<float>() / 180.0 * M_PI;
    }
    achieveTarget(mBasicTimeStep * data["poses"][i]["speed"].get<float>());
    wait(mBasicTimeStep * data["poses"][i]["pause"].get<float>());
  }
}

void MotionRobot::myStep() {
  int ret = mRobot->step(mBasicTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

void MotionRobot::wait(int duration) {
  double start = mRobot->getTime();
  double s = (double)duration / 1000.0;
  while (s + start >= mRobot->getTime())
    myStep();
}

void MotionRobot::achieveTarget(int timeToAchieveTarget) {
  int stepNumberToAchieveTarget = timeToAchieveTarget / mBasicTimeStep;

  myStep();
  for (int i = 0; i < DMM_NMOTORS; i++) {
    mCurrentPositions[i] = mPositionSensors[i]->getValue();
  }
  
  while (stepNumberToAchieveTarget > 0) {
    for (int i = 0; i < DMM_NMOTORS; i++) {
      double dX = mTargetPositions[i] - mCurrentPositions[i];
      double newPosition = mCurrentPositions[i] + dX / stepNumberToAchieveTarget;
      newPosition = alg::clampValue(newPosition, minMotorPositions[i], maxMotorPositions[i]);
      mCurrentPositions[i] = newPosition;
      mMotors[i]->setPosition(newPosition);
    }
    myStep();
    stepNumberToAchieveTarget--;
  }
}