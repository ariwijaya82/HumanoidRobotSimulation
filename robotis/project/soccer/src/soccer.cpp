// #include "soccer.hpp"
// #include "RobotisOp2MotionManager.hpp"
// #include "RobotisOp2GaitManager.hpp"

// #include <iostream>

// static const char* motorNames[NMOTORS] = {
//   "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
//   "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
//   "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
//   "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/
// };

// Soccer::Soccer() : Robot() {
//     mTimeStep = getBasicTimeStep();
//     mCamera = getCamera("Camera");
//     mAccelerometer = getAccelerometer("Accelerometer");
//     mAccelerometer->enable(mTimeStep);
//     mGyro = getGyro("Gyro");
//     mGyro->enable(mTimeStep);

//     for (int i = 0; i < NMOTORS; i++) {
//         mMotors[i] = getMotor(motorNames[i]);
//         string sensorName = motorNames[i];
//         sensorName.push_back('S');
//         mPositionSensors[i] = getPositionSensor(sensorName);
//     }

//     mMotionManager = new RobotisOp2MotionManager(this);
//     mGaitManager = new RobotisOp2GaitManager(this, "data/config.ini");
// }

// Soccer::~Soccer() {}

// void Soccer::myStep() {
//     int ret = step(mTimeStep);
//     if (ret == -1) exit(EXIT_SUCCESS);
// }

// void Soccer::wait(int ms) {
//     double start_time = getTime();
//     double s = (double)ms / 1000.0;
//     while (s + start_time >= getTime())
//         myStep();
// }