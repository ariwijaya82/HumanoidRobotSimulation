#ifndef WALKING_ROBOT_HPP
#define WALKING_ROBOT_HPP

#include <Walking.h>
#include <MX28.h>
#include <minIni.h>
#include <MotionStatus.h>
#include <algebra.hpp>

#include <webots/Gyro.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#define DGM_NMOTORS 20

class WalkingRobot {
    public:
        WalkingRobot(webots::Robot* robot, const std::string &iniFilename);
        virtual ~WalkingRobot();
        bool isCorrectlyInitialized() { return mCorrectlyInitialized; }

        void setXAmplitude(double x) { mXAmplitude = x; }
        void setYAmplitude(double x) { mYAmplitude = x; }
        void setAAmplitude(double x) { mAAmplitude = x; }
        void setMoveAimOn(bool q) { mMoveAimOn = q; }
        void setBalanceEnable(bool q) { mBalanceEnable = q; }

        void start();
        void step(int duration);
        void stop();

    private:
        webots::Robot* mRobot;
        bool mCorrectlyInitialized;
        Robot::Walking* mWalking;
        int mBasicTimeStep;
        double mXAmplitude;
        double mYAmplitude;
        double mAAmplitude;
        bool mMoveAimOn;
        bool mBalanceEnable;
        bool mIsWalking;

        void myStep();
        double valueToPosition(unsigned short value);
        webots::Motor* mMotors[DGM_NMOTORS];

};

#endif