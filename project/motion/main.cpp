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

// Description:   Manage the entree point function

#include "MotionPlayer.hpp"

#include <cstdlib>
#include <iostream>

using namespace webots;
using namespace std;

int main(int argc, char **argv) {
  cout << "costum motion" << endl;
  MotionPlayer *controller = new MotionPlayer();
  controller->run();
  delete controller;
  return EXIT_FAILURE;
}