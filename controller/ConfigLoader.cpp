#include "ConfigLoader.h"
#include <iostream>

ConfigLoader::ConfigLoader(const std::string& filename) {
    try {
        YAML::Node config = YAML::LoadFile(filename);

        cfg_.robot.init_angle = config["robot"]["init_angle"].as<double>();


        cfg_.controller.M_x = config["controller"]["M_x"].as<double>();
        cfg_.controller.B_x = config["controller"]["B_x"].as<double>();
        cfg_.controller.B = config["controller"]["B"].as<double>();
        cfg_.controller.K = config["controller"]["K"].as<double>();
        cfg_.controller.L = config["controller"]["L"].as<double>();
        cfg_.controller.M = config["controller"]["M"].as<double>();
        cfg_.controller.F_max = config["controller"]["F_max"].as<double>();
        cfg_.controller.f_d_tem = config["controller"]["f_d_tem"].as<double>();
        cfg_.controller.Q_max = config["controller"]["Q_max"].as<double>();
        cfg_.controller.TimeUnit = config["controller"]["TimeUnit"].as<double>();
        cfg_.controller.control_mode = config["controller"]["control_mode"].as<std::string>();

    } catch (const std::exception& e) {
        std::cerr << "Failed to load config: " << e.what() << std::endl;
    }
}

Config ConfigLoader::getConfig() const {
    return cfg_;
}
