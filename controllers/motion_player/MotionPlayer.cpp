// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "MotionPlayer.hpp"

#include <webots/utils/Motion.hpp>
#include <webots/Motor.hpp>
#include <webots/Gyro.hpp>

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>

using namespace std;
using namespace webots;

// Constructor
MotionPlayer::MotionPlayer() : Robot() {
  // Get time step
  mTimeStep = getBasicTimeStep();
}

// Destructor
MotionPlayer::~MotionPlayer() {
}

// Step function
void MotionPlayer::myStep() {
  int ret = step(mTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

// Wait function
void MotionPlayer::wait(int ms) {
  double startTime = getTime();
  double s = (double)ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

// function containing the main feedback loop
void MotionPlayer::run() {
  cout << "-------MotionPlayer example of ROBOTIS OP2-------" << endl;
  cout << "This example plays a Webots motion file" << endl;

  vector<string> name = {
    "neck_yaw", "neck_pitch", "left_shoulder_pitch [shoulder]", "left_shoulder_roll",
    "left_elbow", "right_shoulder_pitch [shoulder]", "right_shoulder_roll", "right_elbow",
    "left_hip_yaw [hip]", "left_hip_roll", "left_hip_pitch", "left_knee",
    "left_ankle_roll", "left_ankle_pitch", "right_hip_yaw [hip]", "right_hip_roll",
    "right_hip_pitch", "right_knee", "right_ankle_roll", "right_ankle_pitch"
  };

  ofstream file("servo_value.txt");

  Motor *servo[name.size()];
  for (size_t i = 0; i < name.size(); i++){
    servo[i] = Robot::internalGetInstance()->getMotor(name[i].c_str());
    file << name[i].c_str() << ", ";
  }
  Gyro *gyro = Robot::internalGetInstance()->getGyro("Gyro");
  gyro->enable(mTimeStep);
  file << "gyro_x, gyro_y, gyro_z\n";

  // step
  myStep();

  Motion motion("hand_high.motion");
  // motion.setLoop(true);
  motion.play();
  while (true){
    myStep();
    
    if (!motion.isOver()){
      for (size_t i = 0; i < name.size(); i++){
        file << to_string(servo[i]->getTargetPosition()) << ", ";
      }
      file << gyro->getValues()[0] << ", "
           << gyro->getValues()[1] << ", "
           << gyro->getValues()[2] << "\n";
    } else {
      file.close();
      break;
    }
  }
}
