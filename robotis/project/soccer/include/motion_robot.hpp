#ifndef MOTION_ROBOT_HPP
#define MOTION_ROBOT_HPP

#include <Action.h>
#include <MX28.h>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

#define DMM_NMOTORS 20

class MotionRobot {
    public:
        MotionRobot(webots::Robot* robot, const std::string &costumMotionFile);
        virtual ~MotionRobot();
        bool isCorrectlyInitialized() { return mCorrectlyInitialized; }
        void playPage(int id, bool sync = true);
        void step(int duration);
        bool isMotionPlaying() { return mMotionPlaying; }
    
    private:
        webots::Robot *mRobot;
        bool mCorrectlyInitialized;
        Robot::Action* mAction;
        int mBasicTimeStep;
        bool mMotionPlaying;

        void myStep();
        void wait(int duration);
        void achieveTarget(int timeToAchieveTarget);
        double valueToPosition(unsigned short value);
        void InitMotionAsync();

        webots::Motor* mMotors[DMM_NMOTORS];
        webots::PositionSensor* mPositionSensors[DMM_NMOTORS];
        double mTargetPositions[DMM_NMOTORS];
        double mCurrentPositions[DMM_NMOTORS];
        int mRepeat;
        int mStepnum;
        int mWait;
        int mStepNumberToAchieveTarget;
        void *mPage;
};

#endif