#include "ConfigLoader.h"
#include <kinematics_model.h>

int main()
{
    ConfigLoader loader("../controller/controller.yaml");
    std::string type = loader.getNode("controller")["controller_name"].as<std::string>();
    YAML::Node ctrl = loader.getNode("controller");
    YAML::Node robot = loader.getNode("robot");

    Eigen::Matrix2d M_x;
    Eigen::Matrix2d B_x;
    Eigen::Matrix2d K_x;
    Eigen::Matrix2d K;
    Eigen::Matrix2d B;
    Eigen::Matrix2d L;
    
    Eigen::Matrix2d M;
    Eigen::Vector2d F_max;
    Eigen::Vector2d f_d_tem;
    Eigen::Vector2d Q_max;
    double dt;
    double SimTime;
    std::string control_mode;
    std::string control_target;
    // Eigen::Vector2d init_angle;
    Eigen::Vector2d init_pos;
    double omiga;
    double amplitude;
    double x_amplitude;

    M_x = loader.getMatrix2d(ctrl["M_x"]);
    B_x = loader.getMatrix2d(ctrl["B_x"]);
    K_x = loader.getMatrix2d(ctrl["K_x"]);
    K = loader.getMatrix2d(ctrl["K"]);
    B = loader.getMatrix2d(ctrl["B"]);
    L = loader.getMatrix2d(ctrl["L"]);
    M = loader.getMatrix2d(ctrl["M"]);
    F_max = loader.getVector2d(ctrl["F_max"]);
    Q_max = loader.getVector2d(ctrl["Q_max"]);
    f_d_tem = loader.getVector2d(ctrl["f_d_tem"]);
    dt = ctrl["dt"].as<double>();
    SimTime = ctrl["SimTime"].as<double>();
    control_mode = ctrl["control_mode"].as<std::string>();
    // control_target = ctrl["control_target"].as<std::string>();

    Eigen::Vector2d init_angle = loader.getVector2d(robot["init_angle"]);
    std:: cout << M_x << std::endl;
    std::cout << init_angle << std::endl;

}