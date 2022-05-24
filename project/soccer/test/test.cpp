#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>

int main() {
    std::ifstream file("../data/motion/walkready.json");
    nlohmann::json j = nlohmann::json::parse(file);
    std::cout << j["poses"][0]["speed"].get<float>() * 2 << std::endl;
    return 0;
}