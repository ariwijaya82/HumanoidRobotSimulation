#ifndef VISION_HPP
#define VISION_HPP

#include "algebra.hpp"

#include <opencv2/opencv.hpp>
#include <string>
#include <map>
#include <vector>

class ColorClassifier{
    public:
        enum {
            CLASSIFIER_TYPE_BALL,
            CLASSIFIER_TYPE_FIELD
        };

        ColorClassifier(int classifier_type);
        ~ColorClassifier();

        static ColorClassifier *getInstance(int classifier_type);
        static ColorClassifier *getInstance(std::string name);

        std::string getConfigName();
        bool loadConfiguration();
        bool saveConfiguration();
        bool syncConfiguration();

        cv::Mat classify(cv::Mat input);

        int getHue() {return hue_;}
        int getHueTolerance() {return hue_tolerance_;}
        int getMinSaturation() {return min_saturation_;}
        int getMaxSaturation() {return max_saturation_;}
        int getMinValue() {return min_value_;}
        int getMaxValue() {return max_value_;}

        void setHue(int value){ hue_ = alg::clampValue(value, 0, 360); }
        void setHueTolerance(int value){ hue_tolerance_ = alg::clampValue(value, 0, 179); }
        void setMinSaturation(int value){ min_saturation_ = alg::clampValue(value, 0, 100); }
        void setMaxSaturation(int value){ max_saturation_ = alg::clampValue(value, 0, 100); }
        void setMinValue(int value){ min_value_ = alg::clampValue(value, 0, 100); }
        void setMaxValue(int value){ max_value_ = alg::clampValue(value, 0, 100); }

    private:
        static std::map<int, ColorClassifier*> unique_instances_;

        int classifier_type_;
        std::string name_;

        int hue_;
        int hue_tolerance_;
        int min_saturation_;
        int max_saturation_;
        int min_value_;
        int max_value_;
};

#endif