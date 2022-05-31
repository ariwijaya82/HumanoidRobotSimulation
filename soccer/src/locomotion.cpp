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

    accel = myRobot->getAccelerometer("Accelerometer");
    gyro = myRobot->getGyro("Gyro");
    accel->enable(timeStep);
    gyro->enable(timeStep);

    width = camera_width;
    height = camera_height;
}

void Locomotion::InitFuzzyWalking() {
    engine_fb = fl::FllImporter().fromFile("data/hip_pitch.fll");
    engine_lr = fl::FllImporter().fromFile("data/hip_roll.fll");
        
    std::string status1, status2;
    if (not engine_fb->isReady(&status1) || not engine_lr->isReady(&status2)){
        throw fl::Exception("[engine error] engine is not ready:\n" + status1, FL_AT);
        throw fl::Exception("[engine error] engine is not ready:\n" + status2, FL_AT);
    }

    accel_y = engine_fb->getInputVariable("accel_y");
    gyro_y = engine_fb->getInputVariable("gyro_y");
    accel_x = engine_lr->getInputVariable("accel_x");
    gyro_x = engine_lr->getInputVariable("gyro_x");
    angle_pitch = engine_fb->getOutputVariable("angle");
    angle_roll = engine_lr->getOutputVariable("angle");
}

void Locomotion::gait(KinematicRobot* kinematic) {
    int step = timeStep / 8;
    for (int i = 0; i < step; i++) {
        kinematic->Process();
    }

    // fuzzy control
    if (fuzzy_flag) {
        // const double *acc = accel->getValues();
        // const double *gy = gyro->getValues();
        
        // accel_y->setValue(acc[1]);
        // gyro_y->setValue(gy[1]);
        // accel_x->setValue(acc[0]);
        // gyro_x->setValue(gy[0]);
        // engine_fb->process();
        // engine_lr->process();

        // double pitch_deg = angle_pitch->getValue();
        // double roll_deg = angle_roll->getValue();
        
        // kinematic->setJointValue(10, kinematic->getJointValue(10) - pitch_deg);
        // kinematic->setJointValue(11, kinematic->getJointValue(11) + pitch_deg);
        // if (roll_deg < 0){
        //     kinematic->setJointValue(2, kinematic->getJointValue(2) - roll_deg);
        // } else {
        //     kinematic->setJointValue(3, kinematic->getJointValue(3) - roll_deg);
        // }
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

void Locomotion::InitFuzzyTracking(){

}

void Locomotion::tracking(KinematicRobot* kinematic) {
    double x_move;
    double a_move;

    double head_pan = position_sensors[18]->getValue() * alg::rad2Deg();
    double head_tilt = position_sensors[19]->getValue() * alg::rad2Deg();
    if (head_pan < 20 && head_pan > -20 &&
        head_tilt > -10 && head_tilt < 0){
        kinematic->Stop();
    }
    else {
        kinematic->Start();
        kinematic->X_MOVE_AMPLITUDE = x_move;
        kinematic->A_MOVE_AMPLITUDE = a_move;
    }
}