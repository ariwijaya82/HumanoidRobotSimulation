#ifndef LOCOMOTION_HPP
#define LOCOMOTION_HPP

#include "kinematic.hpp"
#include "algebra.hpp"

#include <webots/Robot.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

class Locomotion {
    public:
        Locomotion(webots::Robot* robot, int camera_width, int camera_height);
        void gait(KinematicRobot* kinematic, double *data = NULL);
        void tracking(KinematicRobot* kinematic);
        void head(const double& x, const double& y, double& prev_x, double& prev_y);
        void InitFuzzyWalking();
        bool fuzzy_flag;
        void InitFuzzyTracking();

        const char* motor_name[20] = {
            "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
            "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
            "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
            "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/
        };

    private:
        int timeStep;
        webots::Robot* myRobot;
        webots::Motor* motors[20];
        webots::PositionSensor* position_sensors[20];
        webots::Accelerometer* accel;
        webots::Gyro* gyro;
        double width;
        double height;

        // walking
        fl::Engine* engine_walk;
        fl::InputVariable* accel_fuzzy;
        fl::InputVariable* gyro_fuzzy;
        fl::OutputVariable* angle;

        // navigation
        fl::Engine* engine_nav;
        fl::InputVariable* pan_fuzzy;
        fl::InputVariable* tilt_fuzzy;
        fl::OutputVariable* a_move;
        fl::OutputVariable* x_move;
};

#endif