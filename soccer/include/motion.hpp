#ifndef MOTION_ROBOT_HPP
#define MOTION_ROBOT_HPP

#include <Action.h>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <string>

class MotionRobot {
    public:
        MotionRobot(webots::Robot* robot, std::string motion_file);
        virtual ~MotionRobot();
        void playPage(int id);
    
    private:
        webots::Robot *myRobot;
        Robot::Action* action;
        int timeStep;

        void myStep();
        void wait(int duration);
        void achieveTarget(int timeToAchieveTarget);
        double valueToPosition(unsigned short value);

        webots::Motor* motors[20];
        webots::PositionSensor* position_sensors[20];
        double target_position[20];
        double current_position[20];
        double min_motor_position[20];
        double max_motor_position[20];
};

#endif