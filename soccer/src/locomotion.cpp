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
        const double *acc = accel->getValues();
        const double *gy = gyro->getValues();

        double acc_x = acc[0] - 512.0;
        double acc_y = acc[1] - 483.2;
        double gy_x = gy[0] - 512.0;
        double gy_y = gy[1] - 512.0;
        
        // std::cout << "acc_x: " << acc_x << std::endl
        //           << "acc_y: " << acc_y << std::endl
        //           << "gy_x: " << gy_x << std::endl
        //           << "gy_y: " << gy_y << std::endl;

        acc_y = alg::clampValue(acc_y, -50, 50);
        gy_y = alg::clampValue(gy_y, -20, 20);
        accel_y->setValue(acc_y);
        gyro_y->setValue(gy_y);
        engine_fb->process();

        // acc_x = alg::clampValue(acc_x, -80, 80);
        // accel_x->setValue(acc_x);
        // gyro_x->setValue(gy_x);
        // engine_lr->process();

        // std::cout << "angle: " << angle_roll->getValue() << std::endl;
        double pitch_deg = angle_pitch->getValue() * alg::deg2Rad();
        // double roll_deg = angle_roll->getValue() * alg::deg2Rad();
        
        kinematic->setJointValue(10, kinematic->getJointValue(10) - pitch_deg);
        kinematic->setJointValue(11, kinematic->getJointValue(11) + pitch_deg);
        // if (roll_deg < 0){
        //     kinematic->setJointValue(3, kinematic->getJointValue(2) - roll_deg);
        // } else {
        //     kinematic->setJointValue(2, kinematic->getJointValue(3) - roll_deg);
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

    // std::cout << "x: " << diff_x << ", y: " << diff_y << std::endl;
    double target_neck = alg::clampValue(-diff_x + position_sensors[18]->getValue(), motors[18]->getMinPosition(), motors[18]->getMaxPosition());
    double target_head = alg::clampValue(-diff_y + position_sensors[19]->getValue(), motors[19]->getMinPosition(), motors[19]->getMaxPosition());
    // std::cout << "neck: " << target_neck * alg::rad2Deg() << ", head: " << target_head * alg::rad2Deg() << std::endl;
    motors[18]->setPosition(target_neck);
    motors[19]->setPosition(target_head);
}

void Locomotion::InitFuzzyTracking(){
    navigation = fl::FllImporter().fromFile("data/navigation.fll");
        
    std::string status;
    if (not navigation->isReady(&status)){
        throw fl::Exception("[engine error] engine is not ready:\n" + status, FL_AT);
    }

    pan_fuzzy = navigation->getInputVariable("pan");
    tilt_fuzzy = navigation->getInputVariable("tilt");
    a_move = navigation->getOutputVariable("a_move");
    x_move = navigation->getOutputVariable("x_move");
}

void Locomotion::tracking(KinematicRobot* kinematic) {
    double head_pan = position_sensors[18]->getValue() * alg::rad2Deg();
    double head_tilt = position_sensors[19]->getValue() * alg::rad2Deg();

    head_pan = alg::clampValue(head_pan, -30, 30);
    pan_fuzzy->setValue(head_pan);
    head_tilt = alg::clampValue(head_tilt, -15, 15);
    tilt_fuzzy->setValue(head_tilt);
    navigation->process();

    double x = x_move->getValue();
    double a = a_move->getValue();

    std::cout << "head_pan: " << head_pan << ", head_tilt: " << head_tilt << std::endl;
    std::cout << "x_move: " << x << ", a_move: " << a << std::endl;

    if (head_pan < 10 && head_pan > -10 &&
        head_tilt > -20 && head_tilt < -10){
        // std::cout << "done" << std::endl;
        kinematic->Stop();
    } else {
        // std::cout << "start" << std::endl;
        kinematic->Start();
        kinematic->X_MOVE_AMPLITUDE = x;
        kinematic->A_MOVE_AMPLITUDE = a;
    }

    fuzzy_flag = false;
    gait(kinematic);
}