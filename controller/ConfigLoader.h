#pragma once
#include <yaml-cpp/yaml.h>
#include <string>

struct RobotConfig {
    double init_angle;
};

struct ControllerConfig {
     double M_x;
     double B_x;

     double K;
     double B;
     double L;

     double M;

     double F_max;
     double f_d_tem;
     double Q_max;
     double TimeUnit;

     std::string control_mode;
};

struct Config {
    RobotConfig robot;
    ControllerConfig controller;
};

class ConfigLoader {
public:
    explicit ConfigLoader(const std::string& filename);
    Config getConfig() const;

private:
    Config cfg_;
};
