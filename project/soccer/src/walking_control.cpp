#include "walking_control.hpp"

static const std::string sotorNames[DGM_NMOTORS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
  "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
  "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
  "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/
};

WalkingRobot::WalkingRobot(webots::Robot* robot, const std::string& iniFilename) :
mRobot(robot),
mCorrectlyInitialized(true),
mXAmplitude(0.0),
mYAmplitude(0.0),
mAAmplitude(0.0),
mMoveAimOn(false),
mBalanceEnable(false),
mIsWalking(false)
{
    if (!mRobot) {
        std::cerr << "robot instance is required" << std::endl;
        mCorrectlyInitialized = false;
        return;
    }
    mBasicTimeStep = mRobot->getBasicTimeStep();

    for (int i = 0; i < DGM_NMOTORS; i++)
        mMotors[i] = mRobot->getMotor(sotorNames[i]);

    minIni ini(iniFilename.c_str());
    mWalking = Robot::Walking::GetInstance();
    mWalking->Initialize();
    mWalking->LoadINISettings(&ini);
}

WalkingRobot::~WalkingRobot(){}

void WalkingRobot::step(int step) {
    if (step < 8) {
        std::cerr << "walking robot: steps of less than 8ms are not supported" << std::endl;
        return;
    }

    if (mIsWalking) {
        mWalking->X_MOVE_AMPLITUDE = mXAmplitude;
        mWalking->A_MOVE_AMPLITUDE = mAAmplitude;
        mWalking->Y_MOVE_AMPLITUDE = mYAmplitude;
        mWalking->A_MOVE_AIM_ON = mMoveAimOn;
        mWalking->BALANCE_ENABLE = mBalanceEnable;
    }

    int numberOfStepToProcess = step / 8;
    if (mBalanceEnable && (mRobot->getGyro("Gyro")->getSamplingPeriod() <= 0)) {
        std::cerr << "gyro is not enabled" << std::endl;
        mRobot->getGyro("Gyro")->enable(mBasicTimeStep);
        myStep();
    }

    for (int i = 0; i < numberOfStepToProcess; i++) {
        if (mBalanceEnable) {
            const double* gyro = mRobot->getGyro("Gyro")->getValues();
            Robot::MotionStatus::RL_GYRO = gyro[0] - 512;
            Robot::MotionStatus::FB_GYRO = gyro[1] - 512;
        }
        mWalking->Process();
    }

    for (int i = 0; i < (DGM_NMOTORS - 2); i++) 
        mMotors[i]->setPosition(valueToPosition(mWalking->m_Joint.GetValue(i + 1)));
}

void WalkingRobot::stop() {
    mIsWalking = false;
    mWalking->Stop();
    while (mWalking->IsRunning())
        this->step(8);
}

void WalkingRobot::start() {
    mIsWalking = true;
    mWalking->Start();
}

double WalkingRobot::valueToPosition(unsigned short value) {
    double degree = Robot::MX28::Value2Angle(value);
    double position = degree / 180.0 * M_PI;
    return position;
}

void WalkingRobot::myStep() {
    int ret = mRobot->step(mBasicTimeStep);
    if (ret == -1)
        exit(EXIT_SUCCESS);
}