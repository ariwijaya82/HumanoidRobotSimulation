#include "motion.hpp"

const std::string motorNames[DMM_NMOTORS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
  "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
  "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
  "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/
};

static double minMotorPositions[DMM_NMOTORS];
static double maxMotorPositions[DMM_NMOTORS];

static double clamp(double value, double min, double max) {
  if (min > max) {
    double temp = min;
    min = max;
    max = min;
  }
  return value < min ? min : value > max ? max : value;
}

MotionRobot::MotionRobot(webots::Robot* robot) :
mRobot(robot)
{
  if (!mRobot) {
    std::cerr << "robot instance is required" << std::endl;
    return;
  }
  mBasicTimeStep = mRobot->getBasicTimeStep();
  mMotionPlaying = false;

  for (int i = 0; i < DMM_NMOTORS; i++) {
    mCurrentPositions[i] = 0.0;
    mMotors[i] = mRobot->getMotor(motorNames[i]);
    std::string sensorName = motorNames[i];
    sensorName.push_back('S');
    mPositionSensors[i] = mRobot->getPositionSensor(sensorName);
    mPositionSensors[i]->enable(true);
    minMotorPositions[i] = mMotors[i]->getMinPosition();
    maxMotorPositions[i] = mMotors[i]->getMaxPosition();
  }

  mAction = Robot::Action::GetInstance();
}

MotionRobot::~MotionRobot() {
  if (mAction && mAction->IsRunning())
    mAction->Stop();
}

void MotionRobot::playMotion(std::string yaml_file) {
  std::string path = "data/motion/" + yaml_file;
  std::ifstream file(path);
  nlohmann::json data = nlohmann::json::parse(file);
  for (int i = 0; i < data["poses"].size(); i++) {
    for (int j = 0; j < 20; j++) {
      mTargetPositions[j] = data["poses"][i]["joints"][motorNames[i]].get<float>() / 180.0 * M_PI;
    }
    achieveTarget(8*data["poses"][i]["speed"].get<float>());
    wait(8*data["poses"][i]["pause"].get<float>());
  }
  mMotionPlaying = false;
}

void MotionRobot::myStep() {
  mMotionPlaying = true;
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
  bool stepNeeded = false;

  for (int i = 0; i < DMM_NMOTORS; i++) {
    if (mPositionSensors[i]->getSamplingPeriod() <= 0) {
      std::cerr << "position feedback " << motorNames[i] << " is not enabled" << std::endl;
      mPositionSensors[i]->enable(mBasicTimeStep);
      stepNeeded = true;
    }
    if (stepNeeded) myStep();
    mCurrentPositions[i] = mPositionSensors[i]->getValue();
  }

  while (stepNumberToAchieveTarget > 0) {
    for (int i = 0; i < DMM_NMOTORS; i++) {
      double dX = mTargetPositions[i] - mCurrentPositions[i];
      double newPosition = mCurrentPositions[i] + dX / stepNumberToAchieveTarget;
      newPosition = clamp(newPosition, minMotorPositions[i], maxMotorPositions[i]);
      mCurrentPositions[i] = newPosition;
      mMotors[i]->setPosition(newPosition);
    }
    myStep();
    stepNumberToAchieveTarget--;
  }
}