#include "motion.hpp"
#include "kinematic.hpp"
#include "vision.hpp"
#include "locomotion.hpp"

#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Camera.hpp>

#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

#include <string>
#include <iostream>

#include <fl/Headers.h>

void wait (int ms, webots::Robot* myRobot, int timeStep) {
    double start = myRobot->getTime();
    double time = (double)ms / 1000.0;
    while (start + time >= myRobot->getTime()) myRobot->step(timeStep);
}

int main(int argc, char** argv) {
    if (argc != 2){
        std::cout << "usage: ./run <mode>" << std::endl;
        exit(EXIT_SUCCESS);
    }

    std::cout << "humanoid robot" << std::endl;
    webots::Robot* myRobot = new webots::Robot();
    int timeStep = myRobot->getBasicTimeStep();

    webots::Keyboard* keyboard = myRobot->getKeyboard();
    keyboard->enable(timeStep);

    webots::Camera* camera = myRobot->getCamera("Camera");
    int camera_width = camera->getWidth();
    int camera_height = camera->getHeight();
    camera->enable(2*timeStep);
    
    MotionRobot* motion = new MotionRobot(myRobot, "motion.bin");
    KinematicRobot* kinematic = new KinematicRobot();
    kinematic->LoadJSON("kinematic.json");
    kinematic->Initialize();
    VisionRobot* vision = new VisionRobot(camera_width, camera_height);
    Locomotion* locomotion = new Locomotion(myRobot, camera_width, camera_height);

    motion->playPage(9);
    wait(200, myRobot, timeStep);

    std::string mode = argv[1];
    if (mode == "walking"){
        bool isWalking = false;
        while( myRobot->step(timeStep) != -1 ) {
            int key = 0;
            while((key = keyboard->getKey()) >= 0) {
                switch (key) {
                    case ' ':
                        if (isWalking) {
                            kinematic->Stop();
                            isWalking = false;
                            wait(200, myRobot, timeStep);
                        } else {
                            kinematic->Start();
                            isWalking = true;
                            wait(200, myRobot, timeStep);
                        }
                        break;
                    case webots::Keyboard::UP:
                        kinematic->X_MOVE_AMPLITUDE = 20.0;
                        break;
                    case webots::Keyboard::DOWN:
                        kinematic->X_MOVE_AMPLITUDE = -20.0;
                        break;
                    case webots::Keyboard::LEFT:
                        kinematic->A_MOVE_AMPLITUDE = 20.0;
                        break;
                    case webots::Keyboard::RIGHT:
                        kinematic->A_MOVE_AMPLITUDE = -20.0;
                        break;
                }
            }
            locomotion->gait(kinematic);
        }
    }
    else if (mode == "head") {
        double pos_x, pos_y, prev_x = 0, prev_y = 0;
        while(myRobot->step(timeStep) != -1){
            const unsigned char* frame = camera->getImage();
            bool isDetected = vision->getBallCenter(pos_x, pos_y, frame);

            locomotion->head(pos_x, pos_y, prev_x, prev_y);
        }
    }
    else if (mode == "fuzzy") {
        fl::Engine* engine = fl::FllImporter().fromFile("data/test.fll");
        
        std::string status;
        if (not engine->isReady(&status))
            throw fl::Exception("[engine error] engine is not ready:\n" + status, FL_AT);

        fl::InputVariable* obstacle = engine->getInputVariable("obstacle");
        fl::OutputVariable* steer = engine->getOutputVariable("mSteer");

        for (int i = 0; i <= 50; ++i){
            fl::scalar location = obstacle->getMinimum() + i * (obstacle->range() / 50);
            obstacle->setValue(location);
            engine->process();
            FL_LOG("obstacle.input = " << fl::Op::str(location) << 
                " => " << "steer.output = " << fl::Op::str(steer->getValue()));

            std::cout << "value: " << steer->getValue() << std::endl;
        }
    }
    return 0;
}