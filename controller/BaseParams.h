#pragma once
#include <Eigen/Dense>
#include <iostream>
#include "ConfigLoader.h"
#include <variant>
#include <vector>


class BaseParams {
public:
    Eigen::Matrix2d M_x;
    Eigen::Matrix2d B_x;
    Eigen::Matrix2d K_x;
    Eigen::Matrix2d K;
    Eigen::Matrix2d B;
    Eigen::Matrix2d L;
    
    Eigen::Matrix2d M;
    Eigen::Vector2d F_max;
    Eigen::Vector2d f_d_0;
    Eigen::Vector2d f_d_1;
    Eigen::Vector2d f_d_2;
    Eigen::Vector2d Q_max;
    double dt;
    double SimTime;
    std::string controller_name;
    std::string control_target;
    Eigen::Vector2d init_angle;
    Eigen::Vector2d init_pos;
    double omega;
    double sin_T;
    double amplitude;
    double x_amplitude;
    
    
    BaseParams(const ConfigLoader& loader);
   
};
