#ifndef ALGEBRA_HPP
#define ALGEBRA_HPP

namespace alg {
    float piValue();
    float rad2Deg();
    float deg2Rad();
    float minValue(float a, float b);
    float maxValue(float a, float b);
    float clampValue(float value, float min, float max);
    float smoothValue(float a, float b, float percent);
}

#endif