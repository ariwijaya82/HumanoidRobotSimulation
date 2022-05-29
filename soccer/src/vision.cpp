#include "vision.hpp"
#include <nlohmann/json.hpp>

VisionRobot::VisionRobot(int width, int height){
    mFinder = new Robot::ColorFinder(28, 20, 50, 45, 0, 30);
    mBuffer = new Robot::FrameBuffer(width, height);
}

VisionRobot::~VisionRobot() {
    delete mFinder;
    delete mBuffer;
}

bool VisionRobot::getBallCenter(double& x, double& y, const unsigned char* image) {
    Robot::Point2D pos;

    mBuffer->m_BGRAFrame->m_ImageData = (unsigned char*) image;
    Robot::ImgProcess::BGRAtoHSV(mBuffer);
    pos = mFinder->GetPosition(mBuffer->m_HSVFrame);

    if (pos.X == -1 && pos.Y == -1) {
        x = 0.0;
        y = 0.0;
        return false;
    } else {
        x = pos.X;
        y = pos.Y;
        return true;
    }
}