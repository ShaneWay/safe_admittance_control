#include <BaseParams.h>

BaseParams::BaseParams(const ConfigLoader& loader)
{
    YAML::Node ctrl = loader.getNode("controller");
    YAML::Node robot = loader.getNode("robot");

    M_x = loader.getMatrix2d(ctrl["M_x"]);
    B_x = loader.getMatrix2d(ctrl["B_x"]);
    K_x = loader.getMatrix2d(ctrl["K_x"]);
    K = loader.getMatrix2d(ctrl["K"]);
    B = loader.getMatrix2d(ctrl["B"]);
    L = loader.getMatrix2d(ctrl["L"]);
    M = loader.getMatrix2d(ctrl["M"]);
    F_max = loader.getVector2d(ctrl["F_max"]);
    Q_max = loader.getVector2d(ctrl["Q_max"]);
    f_d_0 = loader.getVector2d(ctrl["f_d_0"]);
    f_d_1 = loader.getVector2d(ctrl["f_d_1"]);
    f_d_2 = loader.getVector2d(ctrl["f_d_2"]);
    dt = ctrl["dt"].as<double>();
    SimTime = ctrl["SimTime"].as<double>();
    controller_name = ctrl["controller_name"].as<std::string>();
    control_target = ctrl["control_target"].as<std::string>();

    init_angle = loader.getVector2d(robot["init_angle"]);
    init_pos = loader.getVector2d(robot["init_pos"]);
    sin_T = robot["sin_T"].as<double>();
    amplitude = robot["amplitude"].as<double>();
    x_amplitude = robot["x_amplitude"].as<double>();

    init_angle[0] = init_angle[0] / 180. * M_PI;
    init_angle[1] = init_angle[1] / 180. * M_PI;
    omega = 2 * M_PI / sin_T;
}