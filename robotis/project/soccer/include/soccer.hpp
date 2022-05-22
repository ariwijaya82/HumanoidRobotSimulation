// #ifndef SOCCER_HPP
// #define SOCCER_HPP

// #define NMOTORS

// #include <webots/Robot.hpp>

// class Soccer : public webots::Robot {
//     public:
//         Soccer();
//         virtual ~Soccer();
//         void run();

//     private:
//         int mTimeStep;

//         void myStep();
//         void wait(int ms);
        
//         webots::Motor* mMotors[NMOTORS];
//         webots::PositionSensor* mPositionSensors[NMOTORS];
//         webots::Camera* mCamera;
//         webots::Accelerometer* mAccelerometer;
//         webots::Gyro* mGyro;

//         managers::RobotisOp2MotionManager* mMotionManager;
//         managers::RobotisOp2GaitManager* mGaitManager;
// };

// #endif