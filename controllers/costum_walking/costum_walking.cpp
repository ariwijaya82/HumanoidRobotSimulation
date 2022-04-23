#include "walk.hpp"

#include <cstdlib>
#include <iostream>

using namespace webots;

int main(int argc, char** argv){
  std::cout << "test" << std::endl;
  Walk *walking = new Walk();
  walking->run();
  delete walking;
  return EXIT_FAILURE;
}