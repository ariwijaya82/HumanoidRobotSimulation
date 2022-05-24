#include "motion.hpp"
#include "walking_control.hpp"

#include <webots/Robot.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>

#include <string>
#include <iostream>

static const char *motorNames[20] = {
    "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
    "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
    "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
    "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/
};

void myStep(webots::Robot* myRobot, int timeStep){
    int ret = myRobot->step(timeStep);
    if (ret == -1) exit(EXIT_SUCCESS);
}

void wait(webots::Robot* myRobot, int timeStep, int ms) {
    double start = myRobot->getTime();
    double s = (double)ms / 1000.0;
    while (s + start >= myRobot->getTime()) myStep(myRobot, timeStep);
}

// void checkFallen(webots::Accelerometer* accel, MotionRobot* motion){
//     static int fup = 0;
//     static int fdown = 0;
//     const double acc_tolerance = 80.0;
//     const int acc_step = 100;

//     const double *val = accel->getValues();
//     if (val[1] < 512.0 - acc_tolerance) fup++;
//     else fup = 0;

//     if (val[1] > 512.0 + acc_tolerance) fdown++;
//     else fdown = 0;

//     if (fup > acc_step) {
//         motion->playPage(10);
//         motion->playPage(9);
//         fup = 0;
//     } else if (fdown > acc_step) {
//         motion->playPage(11);
//         motion->playPage(9);
//         fdown = 0;
//     }
// }

int main() {
    webots::Robot* myRobot = new webots::Robot();
    int timeStep = myRobot->getBasicTimeStep();
    // std::cout << timeStep << std::endl;
    // std::cout << "initialize robot" << std::endl;
    // webots::Accelerometer* accel = myRobot->getAccelerometer("Accelerometer");
    // webots::Gyro* gyro = myRobot->getGyro("Gyro");
    // accel->enable(timeStep);
    // gyro->enable(timeStep);

    // webots::Motor* motors[20];
    // webots::PositionSensor* position_sensors[20];
    // for (int i = 0; i < 20; i++) {
    //     motors[i] = myRobot->getMotor(motorNames[i]);
    //     std::string name = motorNames[i];
    //     name.push_back('S');
    //     position_sensors[i] = myRobot->getPositionSensor(name);
    //     position_sensors[i]->enable(timeStep);
    // }

    // webots::Keyboard* keyboard = myRobot->getKeyboard();
    // keyboard->enable(timeStep);
    // std::cout << "Initialize motion" << std::endl;
    MotionRobot* motion = new MotionRobot(myRobot);
    // WalkingRobot* walking = new WalkingRobot(myRobot, "data/walking.ini");
    myRobot->step(timeStep);
    motion->playMotion("walkready.json");
    // motion->playPage(9);
    wait(myRobot, timeStep, 200);

    // bool isWalking = false;
    // while(true) {
    //     checkFallen(accel, motion);
    //     walking->setXAmplitude(0.0);
    //     walking->setAAmplitude(0.0);

    //     int key = 0;
    //     while((key = keyboard->getKey()) >= 0) {
    //         switch (key) {
    //             case ' ':
    //                 if (isWalking) {
    //                     walking->stop();
    //                     isWalking = false;
    //                     wait(myRobot, timeStep, 200);
    //                 } else {
    //                     walking->start();
    //                     isWalking = true;
    //                     wait(myRobot, timeStep, 200);
    //                 }
    //                 break;
    //             case webots::Keyboard::UP:
    //                 walking->setXAmplitude(20.0);
    //                 break;
    //             case webots::Keyboard::DOWN:
    //                 walking->setXAmplitude(-20.0);
    //                 break;
    //             case webots::Keyboard::LEFT:
    //                 walking->setAAmplitude(10.0);
    //                 break;
    //             case webots::Keyboard::RIGHT:
    //                 walking->setAAmplitude(-10.0);
    //                 break;
    //         }
    //     }
    //     walking->step(timeStep);
    //     myStep(myRobot, timeStep);
    // }

    return 0;
}