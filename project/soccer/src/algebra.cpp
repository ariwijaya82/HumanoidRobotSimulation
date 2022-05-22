#include "algebra.hpp"

float alg::minValue(float a, float b){
    return a < b ? a : b;
}

float alg::maxValue(float a, float b){
    return a > b ? a : b;
}

float alg::clampValue(float value, float min, float max){
    return alg::maxValue(min, minValue(value, max));
}