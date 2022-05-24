#ifndef MOTION_ROBOT_HPP
#define MOTION_ROBOT_HPP

#include <Action.h>
#include <MX28.h>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

#include <nlohmann/json.hpp>
#include <string>
#include <fstream>

const int DMM_NMOTORS = 20;

class MotionRobot {
    public:
        MotionRobot(webots::Robot* robot);
        virtual ~MotionRobot();
        void playMotion(std::string yaml_file);;
        bool isMotionPlaying() { return mMotionPlaying; }
    
    private:
        webots::Robot *mRobot;
        Robot::Action* mAction;
        int mBasicTimeStep;
        bool mMotionPlaying;

        void myStep();
        void wait(int duration);
        void achieveTarget(int timeToAchieveTarget);

        webots::Motor* mMotors[DMM_NMOTORS];
        webots::PositionSensor* mPositionSensors[DMM_NMOTORS];
        double mTargetPositions[DMM_NMOTORS];
        double mCurrentPositions[DMM_NMOTORS];
        int mStepNumberToAchieveTarget;
};

#endif