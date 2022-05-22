#include "vision.hpp"
#include "algebra.hpp"

#include <yaml-cpp/yaml.h>

std::map<int, ColorClassifier*> ColorClassifier::unique_instances_;

ColorClassifier::ColorClassifier(int classifier_type) {
    classifier_type_= classifier_type;
    unique_instances_.insert(std::pair<int, ColorClassifier*>(classifier_type_, this));

    switch (classifier_type_) {
        case CLASSIFIER_TYPE_BALL: name_ = "ball"; break;
        case CLASSIFIER_TYPE_FIELD: name_ = "field"; break;
    }

    syncConfiguration();
}

ColorClassifier::~ColorClassifier(){
    unique_instances_.erase(classifier_type_);
}

ColorClassifier* ColorClassifier::getInstance(int classifier_type){
    if (unique_instances_.find(classifier_type) != unique_instances_.end()) 
        return unique_instances_[classifier_type];
    
    return (new ColorClassifier(classifier_type));
}

ColorClassifier* ColorClassifier::getInstance(std::string name) {
    for (const std::pair<int, ColorClassifier*> &keyval : unique_instances_)
        if (keyval.second->name_ == name)
            return keyval.second;

    return nullptr;
}

bool ColorClassifier::loadConfiguration() {
    std::string ss = "data/" + getConfigName();
    
    std::ifstream input(ss, std::ifstream::in);
    if (input.is_open() == false)
        return false;

    YAML::Node config = YAML::LoadFile(ss);
    YAML::Node color_section = config["Color"];
    hue_ = color_section["hue"].as<int>();
    hue_tolerance_ = color_section["hue_tolerance"].as<int>();
    min_saturation_ = color_section["min_saturation"].as<int>();
    max_saturation_ = color_section["max_saturation"].as<int>();
    min_value_ = color_section["min_value"].as<int>();
    max_value_ = color_section["max_value"].as<int>();
 
    return true;
}

bool ColorClassifier::saveConfiguration() {
    std::string ss = "data/" + getConfigName();

    std::ofstream output(ss, std::ofstream::out);
    if (output.is_open() == false) 
        return false;

    YAML::Node config;
    YAML::Node color_section = config["Color"];
    color_section["hue"] = hue_;
    color_section["hue_tolerance"] = hue_tolerance_;
    color_section["min_saturation"] = min_saturation_;
    color_section["max_saturation"] = max_saturation_;
    color_section["min_value"] = min_value_;
    color_section["max_value"] = max_value_;

    output << config;
    output.close();

    return true;
}

bool ColorClassifier::syncConfiguration() {
    if (!loadConfiguration()) return false;
    if (!saveConfiguration()) return false;
    return true;
}

std::string ColorClassifier::getConfigName() {
    std::stringstream config_name;
    config_name << name_ << ".yaml";
    return config_name.str();
}

cv::Mat ColorClassifier::classify(cv::Mat input) {
    int h_min = ((hue_ - hue_tolerance_) * 255) / 360;
    int h_max = ((hue_ + hue_tolerance_) * 255) / 360;

    int s_min = (min_saturation_ * 255) / 100;
    int s_max = (max_saturation_ * 255) / 100;

    int v_min = (min_value_ * 255) / 100;
    int v_max = (max_value_ * 255) / 100;

    cv::Scalar hsv_min = cv::Scalar(h_min, s_min, v_min);
    cv::Scalar hsv_max = cv::Scalar(h_max, s_max, v_max);

    cv::Mat output = input.clone();
    cv::inRange(input, hsv_min, hsv_max, output);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3), cv::Point(1,1));
    cv::morphologyEx(output, input, cv::MORPH_CLOSE, element);

    return output;
}