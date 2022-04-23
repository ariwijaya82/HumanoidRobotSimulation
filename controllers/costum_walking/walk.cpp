#include "walk.hpp"

#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

using namespace webots;
using namespace managers;
using namespace std;

static const char *motorNames[NMOTORS] = {
    "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
    "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
    "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
    "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/
};

Walk::Walk() : Robot() {
    mTimeStep = getBasicTimeStep();

    getLED("HeadLed")->set(0xFF0000);
    getLED("EyeLed")->set(0x00FF00);
    mAccelerometer = getAccelerometer("Accelerometer");
    mAccelerometer->enable(mTimeStep);

    getGyro("Gyro")->enable(mTimeStep);

    for (int i = 0; i < NMOTORS; i++){
        mMotors[i] = getMotor(motorNames[i]);
        string sensorName = motorNames[i];
        sensorName.push_back('S');
        mPositionSensors[i] = getPositionSensor(sensorName);
        mPositionSensors[i]->enable(mTimeStep);
    }

    mKeyboard = getKeyboard();
    mKeyboard->enable(mTimeStep);

    mMotionManager = new RobotisOp2MotionManager(this);
    mGaitManager = new RobotisOp2GaitManager(this, "config.ini");
}

Walk::~Walk(){
}

void Walk::myStep(){
    int ret = step(mTimeStep);
    if (ret == -1) {
        exit(EXIT_SUCCESS);
    }
}

void Walk::wait(int ms){
    double startTime = getTime();
    double s = (double)ms / 1000.0;
    while (s + startTime >= getTime())
        myStep();
}

void Walk::run() {
    cout << "Costum walking" << endl;

    myStep();

    mMotionManager->playPage(9);
    wait(200);

    bool isWalking = false;

    while (true) {
        checkIfFallen();

        mGaitManager->setXAmplitude(0.0);
        mGaitManager->setAAmplitude(0.0);

        int key = 0;
        while ((key = mKeyboard->getKey()) >= 0) {
            switch (key) {
                case ' ':
                    if (isWalking){
                        mGaitManager->stop();
                        isWalking = false;
                        wait(200);   
                    } else {
                        mGaitManager->start();
                        isWalking = true;
                        wait(200);
                    }
                    break;
                case Keyboard::UP:
                    mGaitManager->setXAmplitude(1.0);
                    break;
                case Keyboard::DOWN:
                    mGaitManager->setXAmplitude(-1.0);
                    break;
                case Keyboard::RIGHT:
                    mGaitManager->setAAmplitude(-0.5);
                    break;
                case Keyboard::LEFT:
                    mGaitManager->setAAmplitude(0.5);
                    break;
            }
        }

        mGaitManager->step(mTimeStep);
        myStep();
    }
}

void Walk::checkIfFallen(){
    static int fup = 0;
    static int fdown = 0;
    static const double acc_tolarance = 80.0;
    static const double acc_step = 100;

    const double *acc = mAccelerometer->getValues();
    if (acc[1] < 512.0 - acc_tolarance)
        fup++;
    else 
        fup = 0;

    if (acc[1] > 512.0 + acc_tolarance)
        fdown++;
    else 
        fdown = 0;

    if (fup > acc_step){
        mMotionManager->playPage(10);
        mMotionManager->playPage(9);
        fup = 0;
    } else if (fdown > acc_step){
        mMotionManager->playPage(11);
        mMotionManager->playPage(9);
        fdown = 0;
    }
}