#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <string>

using namespace webots;
using namespace std;

int main() {
    Robot* myRobot = new Robot();
    Motor* motor[4];
    int timeStep = myRobot->getBasicTimeStep();

    const string name[4] = {"wheel1", "wheel2", "wheel3", "wheel4"};
    for (int i = 0; i < 4; i++) {
        motor[i] = myRobot->getMotor(name[i]);
        motor[i]->setPosition(INFINITY);
        motor[i]->setVelocity(1.0);
    }

    while(true){
        int ret = myRobot->step(64);
        if (ret == -1) exit(EXIT_SUCCESS);
    }
}