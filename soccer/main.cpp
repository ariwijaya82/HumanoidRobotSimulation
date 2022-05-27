#include "motion.hpp"
#include "walking.hpp"

#include <webots/Robot.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>

#include <string>
#include <iostream>

const char *motorNames[20] = {
    "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
    "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
    "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
    "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/
};

void wait (int ms, webots::Robot* myRobot, int timeStep) {
    double start = myRobot->getTime();
    double time = (double)ms / 1000.0;
    while (start + time >= myRobot->getTime()) myRobot->step(timeStep);
}

int main() {
    std::cout << "Soccer program" << std::endl;
    webots::Robot* myRobot = new webots::Robot();
    int timeStep = myRobot->getBasicTimeStep();
    
    webots::Accelerometer* accel = myRobot->getAccelerometer("Accelerometer");
    webots::Gyro* gyro = myRobot->getGyro("Gyro");
    accel->enable(timeStep);
    gyro->enable(timeStep);

    webots::Motor* motors[20];
    webots::PositionSensor* position_sensors[20];
    for (int i = 0; i < 20; i++) {
        motors[i] = myRobot->getMotor(motorNames[i]);
        std::string name = motorNames[i];
        name.push_back('S');
        position_sensors[i] = myRobot->getPositionSensor(name);
        position_sensors[i]->enable(timeStep);
    }

    webots::Keyboard* keyboard = myRobot->getKeyboard();
    keyboard->enable(timeStep);
    
    MotionRobot* motion = new MotionRobot(myRobot, "motion.bin");
    WalkingRobot* walking = new WalkingRobot(myRobot);
    walking->Initialize();
    walking->LoadJSON("walking.json");
    
    motion->playPage(9);
    wait(200, myRobot, timeStep);

    bool isWalking = false;
    while(true) {
        int key = 0;
        while((key = keyboard->getKey()) >= 0) {
            switch (key) {
                case ' ':
                    if (isWalking) {
                        walking->Stop();
                        isWalking = false;
                        wait(200, myRobot, timeStep);
                    } else {
                        walking->Start();
                        isWalking = true;
                        wait(200, myRobot, timeStep);
                    }
                    break;
                case webots::Keyboard::UP:
                    walking->X_MOVE_AMPLITUDE = 20.0;
                    break;
                case webots::Keyboard::DOWN:
                    walking->X_MOVE_AMPLITUDE = -20.0;
                    break;
                case webots::Keyboard::LEFT:
                    walking->A_MOVE_AMPLITUDE = 10.0;
                    break;
                case webots::Keyboard::RIGHT:
                    walking->A_MOVE_AMPLITUDE = -10.0;
                    break;
            }
        }
        walking->step(timeStep);
        walking->X_MOVE_AMPLITUDE = 0.0;
        walking->A_MOVE_AMPLITUDE = 0.0;
        
        int ret = myRobot->step(timeStep);
        if (ret == -1) exit(EXIT_SUCCESS);
    }

    return 0;
}