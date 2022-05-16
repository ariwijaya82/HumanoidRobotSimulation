#include "walk.hpp"
#include "Soccer.hpp"

#include <cstdlib>
#include <iostream>

using namespace webots;

int main(int argc, char** argv){
  std::cout << "its work" << std::endl;
  Soccer *walking = new Soccer();
  walking->run();
  delete walking;
  return EXIT_FAILURE;
}