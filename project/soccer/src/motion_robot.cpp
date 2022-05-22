#include "motion_robot.hpp"

static const std::string motorNames[DMM_NMOTORS] = {
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

MotionRobot::MotionRobot(webots::Robot* robot, const std::string &costumMotionFile) :
mRobot(robot),
mCorrectlyInitialized(true)
{
  if (!mRobot) {
    std::cerr << "robot instance is required" << std::endl;
    mCorrectlyInitialized = false;
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
    minMotorPositions[i] = mMotors[i]->getMinPosition();
    maxMotorPositions[i] = mMotors[i]->getMaxPosition();
  }

  mAction = Robot::Action::GetInstance();
  if (mAction->LoadFile((char*)costumMotionFile.c_str()) == false) {
    std::cerr << "cannot load motion file" << std::endl;
    mCorrectlyInitialized = false;
    mAction = NULL;
    return;
  }
}

MotionRobot::~MotionRobot() {
  if (mAction && mAction->IsRunning())
    mAction->Stop();
}

void MotionRobot::playPage(int id, bool sync) {
  if (!mCorrectlyInitialized)
    return;

  if (sync) {
    Robot::Action::PAGE page;
    if (mAction->LoadPage(id, &page)) {
      for (int i = 0; i < page.header.repeat; i++) {
        for (int j = 0; j < page.header.stepnum; j++) {
          for (int k = 0; k < DMM_NMOTORS; k++)
            mTargetPositions[k] = valueToPosition(page.step[j].position[k+1]);
          achieveTarget(8*page.step[j].time);
          wait(8*page.step[j].pause);
        }
      }
      if (page.header.next != 0)
        playPage(page.header.next);
    } else 
      std::cerr << "cannot load the page" << std::endl;
  } else {
    InitMotionAsync();
    mPage = new Robot::Action::PAGE;
    if (!(mAction->LoadPage(id, (Robot::Action::PAGE*)mPage)))
      std::cerr << "cannot load the page" << std::endl;
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

double MotionRobot::valueToPosition(unsigned short value) {
  double degree = Robot::MX28::Value2Angle(value);
  double position = degree / 180.0 * M_PI;
  return position;
}

void MotionRobot::InitMotionAsync() {
  bool stepNeeded = false;
  for (int i = 0; i < DMM_NMOTORS; i++) {
    if (mPositionSensors[i]->getSamplingPeriod() <= 0) {
      std::cerr << "position feedback " << motorNames[i] << " is not enabled" << std::endl;
      mPositionSensors[i]->enable(mBasicTimeStep);
      stepNeeded = true;
    }
    if (stepNeeded)
      myStep();
    mCurrentPositions[i] = mPositionSensors[i]->getValue();
  }
  mStepnum = 0;
  mRepeat = 1;
  mStepNumberToAchieveTarget = 0;
  mWait = 0;
  mMotionPlaying = true;
}

void MotionRobot::step(int duration) {
  if (mStepNumberToAchieveTarget > 0) {
    for (int i = 0; i < DMM_NMOTORS; i++) {
      double dX = mTargetPositions[i] - mCurrentPositions[i];
      double newPosition = mCurrentPositions[i] + dX / mStepNumberToAchieveTarget;
      mCurrentPositions[i] = newPosition;
      mMotors[i]->setPosition(newPosition);
    }
    mStepNumberToAchieveTarget--;
  } else if (mWait > 0) mWait--;
  else {
    if (mStepnum < ((Robot::Action::PAGE*)mPage)->header.stepnum) {
      for (int k = 0; k < DMM_NMOTORS; k++) 
        mTargetPositions[k] = valueToPosition(((Robot::Action::PAGE*)mPage)->step[mStepnum].position[k+1]);
      mStepNumberToAchieveTarget = (8*((Robot::Action::PAGE*)mPage)->step[mStepnum].time) / mBasicTimeStep;
      if (mStepNumberToAchieveTarget == 0)
        mStepNumberToAchieveTarget = 1;
      mWait = (8*((Robot::Action::PAGE*)mPage)->step[mStepnum].pause) / mBasicTimeStep + 0.5;
      mStepnum++;
      step(duration);
    } else if (mRepeat < (((Robot::Action::PAGE*)mPage)->header.repeat)) {
      mRepeat++;
      mStepnum = 0;
      step(duration);
    } else if (((Robot::Action::PAGE*)mPage)->header.next != 0)
      playPage(((Robot::Action::PAGE*)mPage)->header.next, true);
    else 
      mMotionPlaying = true;
  }
}