#include "locomotion.hpp"
#include "algebra.hpp"
#include <iostream>

Locomotion::Locomotion(webots::Robot* robot, int camera_width, int camera_height){
    myRobot = robot;
    timeStep = myRobot->getBasicTimeStep();

    for (int i = 0; i < 20; i++) {
        motors[i] = myRobot->getMotor(motor_name[i]);
        std::string name = motor_name[i];
        name.push_back('S');
        position_sensors[i] = myRobot->getPositionSensor(name);
        position_sensors[i]->enable(timeStep);
    }

    webots::Accelerometer* accel = myRobot->getAccelerometer("Accelerometer");
    webots::Gyro* gyro = myRobot->getGyro("Gyro");
    accel->enable(timeStep);
    gyro->enable(timeStep);

    width = camera_width;
    height = camera_height;
}

void Locomotion::gait(KinematicRobot* kinematic) {
    int step = timeStep / 8;

    for (int i = 0; i < step; i++) {
        kinematic->Process();
    }

    for (int i = 0; i < 18; i++) {
        motors[i]->setPosition(kinematic->getJointValue(i));
    }
}

void Locomotion::head(const double& x, const double& y, double& prev_x, double& prev_y) {
    double err_x = (2 * x / width) - 1.0;
    double err_y = (2 * y / height) - 1.0;

    double delta_err_x = prev_x - err_x;
    double delta_err_y = prev_y - err_y;
    prev_x = err_x;
    prev_y = err_y;

    double diff_x = err_x * 0.4 + 0 * delta_err_x;
    double diff_y = err_y * 0.4 + 0 * delta_err_y;

    double target_neck = alg::clampValue(-diff_x, motors[18]->getMinPosition(), motors[18]->getMaxPosition());
    double target_head = alg::clampValue(-diff_y, motors[19]->getMinPosition(), motors[19]->getMaxPosition());

    double current_neck = position_sensors[18]->getValue();
    double current_head = position_sensors[19]->getValue();

    motors[18]->setPosition(current_neck + target_neck);
    motors[19]->setPosition(current_head + target_head);
}