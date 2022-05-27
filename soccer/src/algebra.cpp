#include "algebra.hpp"

float alg::piValue() {
    return 3.14159265359;
}

float alg::rad2Deg()
{
    return 180.0 / alg::piValue();
}

float alg::deg2Rad()
{
    return alg::piValue() / 180.0;
}

float alg::minValue(float a, float b){
    return a < b ? a : b;
}

float alg::maxValue(float a, float b){
    return a > b ? a : b;
}

float alg::clampValue(float value, float min, float max){
    return alg::maxValue(min, minValue(value, max));
}

float alg::smoothValue(float a, float b, float percent)
{
    return ((1.0 - percent) * a) + (percent * b);
}