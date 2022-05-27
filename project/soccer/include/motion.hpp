#ifndef MOTION_ROBOT_HPP
#define MOTION_ROBOT_HPP

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

#include <nlohmann/json.hpp>
#include <string>

#define DMM_NMOTORS 20

class MotionRobot {
    public:
        MotionRobot(webots::Robot* robot);
        virtual ~MotionRobot();
        void playMotion(std::string file_name);
    
    private:
        webots::Robot *mRobot;
        int mBasicTimeStep;

        void myStep();
        void wait(int duration);
        void achieveTarget(int timeToAchieveTarget);

        webots::Motor* mMotors[DMM_NMOTORS];
        webots::PositionSensor* mPositionSensors[DMM_NMOTORS];
        double mTargetPositions[DMM_NMOTORS];
        double mCurrentPositions[DMM_NMOTORS];
        double minMotorPositions[DMM_NMOTORS];
        double maxMotorPositions[DMM_NMOTORS];
        int mStepNumberToAchieveTarget;
};

#endif