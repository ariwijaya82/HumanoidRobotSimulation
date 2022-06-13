#include "locomotion.hpp"
#include "algebra.hpp"
#include <iostream>
#include <cmath>

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
    engine_walk = fl::FllImporter().fromFile("data/walking.fll");
        
    std::string status;
    if (not engine_walk->isReady(&status)){
        throw fl::Exception("[engine error] engine is not ready:\n" + status, FL_AT);
    }

    accel_fuzzy = engine_walk->getInputVariable("accel");
    gyro_fuzzy = engine_walk->getInputVariable("gyro");
    angle = engine_walk->getOutputVariable("angle");
}

void Locomotion::gait(KinematicRobot* kinematic, double *data) {
    int step = timeStep / 8;
    for (int i = 0; i < step; i++) {
        kinematic->Process();
    }

    // fuzzy control
    if (fuzzy_flag) {
        const double *acc = accel->getValues();
        const double *gy = gyro->getValues();

        double acc_y = (acc[1] - 486);
        double gy_y = (gy[1] - 512.0);

        acc_y = alg::clampValue(acc_y, -50, 50);
        gy_y = alg::clampValue(gy_y, -50, 50);
        
        if (data) {
            data[0] = acc_y;
            data[1] = gy_y;
        }

        accel_fuzzy->setValue(acc_y);
        gyro_fuzzy->setValue(gy_y);
        engine_walk->process();

        double pitch_deg = angle->getValue();
        // if (isnan(pitch_deg)) pitch_deg = 0;
        // printf("acc: %lf, gyro: %lf, pitch: %lf\n", acc_y, gy_y, pitch_deg);
        if (data) {
            data[2] = pitch_deg;
        }
        pitch_deg *=  alg::deg2Rad();
        
        kinematic->setJointValue(10, kinematic->getJointValue(10) - pitch_deg);
        kinematic->setJointValue(11, kinematic->getJointValue(11) + pitch_deg);
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

    // std::cout << "x: " << diff_x << ", y: " << diff_y << std::endl;
    double target_neck = alg::clampValue(-diff_x + position_sensors[18]->getValue(), motors[18]->getMinPosition(), motors[18]->getMaxPosition());
    double target_head = alg::clampValue(-diff_y + position_sensors[19]->getValue(), motors[19]->getMinPosition(), motors[19]->getMaxPosition());
    // std::cout << "neck: " << target_neck * alg::rad2Deg() << ", head: " << target_head * alg::rad2Deg() << std::endl;
    motors[18]->setPosition(target_neck);
    motors[19]->setPosition(target_head);
}

void Locomotion::InitFuzzyTracking(){
    engine_nav = fl::FllImporter().fromFile("data/navigation.fll");
        
    std::string status;
    if (not engine_nav->isReady(&status)){
        throw fl::Exception("[engine error] engine is not ready:\n" + status, FL_AT);
    }

    pan_fuzzy = engine_nav->getInputVariable("pan");
    tilt_fuzzy = engine_nav->getInputVariable("tilt");
    a_move = engine_nav->getOutputVariable("a_move");
    x_move = engine_nav->getOutputVariable("x_move");
}

void Locomotion::tracking(KinematicRobot* kinematic) {
    double head_pan = position_sensors[18]->getValue() * alg::rad2Deg();
    double head_tilt = position_sensors[19]->getValue() * alg::rad2Deg();

    head_pan = alg::clampValue(head_pan, -70, 70);
    pan_fuzzy->setValue(head_pan);
    head_tilt = alg::clampValue(head_tilt, -10, 15);
    tilt_fuzzy->setValue(head_tilt);
    engine_nav->process();

    double x = x_move->getValue();
    double a = a_move->getValue();

    if (isnan(x) || isnan(a)){
        x = 0;
        a = 0;
    }

    // std::cout << "head_pan: " << head_pan << ", head_tilt: " << head_tilt << std::endl;
    // std::cout << "a_move: " << a << ", x_move: " << x << std::endl;

    if (head_pan < 10 && head_pan > -10 &&
        head_tilt > -10 && head_tilt < 0){
        std::cout << "done" << std::endl;
        kinematic->Stop();
    } else {
        kinematic->X_MOVE_AMPLITUDE = x;
        kinematic->A_MOVE_AMPLITUDE = a;
    }
}