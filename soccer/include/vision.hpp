#ifndef VISION_ROBOT_HPP
#define VISION_ROBOT_HPP

#include <Image.h>
#include <ColorFinder.h>
#include <ImgProcess.h>
#include <Point.h>

#include <string>
#include <fstream>

class VisionRobot{
    public:
        VisionRobot(int width, int height);
        ~VisionRobot();

        bool getBallCenter(double& x, double& y, const unsigned char* image);

    private:
        Robot::FrameBuffer *mBuffer;
        Robot::ColorFinder *mFinder;
};

#endif