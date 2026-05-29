#pragma once

#include <Eigen/Dense>
#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "matplotlibcpp.h"

namespace fs = std::experimental::filesystem;
namespace plt = matplotlibcpp;

struct ControlLogSample
{
    double time = 0.0;
    int frame = 0;

    Eigen::Vector2d f_ext = Eigen::Vector2d::Zero();

    Eigen::Vector2d q0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d q_s = Eigen::Vector2d::Zero();
    Eigen::Vector2d q_s_hat = Eigen::Vector2d::Zero();

    Eigen::Vector2d q_x = Eigen::Vector2d::Zero();
    Eigen::Vector2d q_x_hat = Eigen::Vector2d::Zero();

    Eigen::Vector2d tau = Eigen::Vector2d::Zero();
    Eigen::Vector2d tau_star = Eigen::Vector2d::Zero();
    Eigen::Vector2d tau_ext = Eigen::Vector2d::Zero();

    Eigen::Vector2d X0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d X = Eigen::Vector2d::Zero();
    Eigen::Vector2d X_d = Eigen::Vector2d::Zero();
};

class DataLogger
{
  public:
    DataLogger()
    {
        plt::backend("Agg");
    }

    void reserve(size_t n)
    {
        data_.reserve(n);
    }

    void log(const ControlLogSample& sample)
    {
        data_.push_back(sample);
    }

    void clear()
    {
        data_.clear();
    }

    bool empty() const
    {
        return data_.empty();
    }

    size_t size() const
    {
        return data_.size();
    }

    bool saveCsv(const std::string& filename) const
    {
        fs::path path(filename);
        if (!path.parent_path().empty()) {
            fs::create_directories(path.parent_path());
        }

        std::ofstream file(filename, std::ios::out);
        if (!file.is_open()) {
            std::cout << "[DataLogger] failed to open " << filename << std::endl;
            return false;
        }

        file << "time,frame,"
             << "f1,f2,"
             << "q01,q02,q_s1,q_s2,q_s_hat1,q_s_hat2,"
             << "q_x1,q_x2,q_x_hat1,q_x_hat2,"
             << "tau1,tau2,tau_star1,tau_star2,tau_ext1,tau_ext2,"
             << "X0_x,X0_y,X_x,X_y,Xd_x,Xd_y\n";

        for (const auto& s : data_) {
            file << s.time << "," << s.frame << "," << s.f_ext[0] << "," << s.f_ext[1] << "," << s.q0[0] << "," << s.q0[1] << ","
                 << s.q_s[0] << "," << s.q_s[1] << "," << s.q_s_hat[0] << "," << s.q_s_hat[1] << "," << s.q_x[0] << "," << s.q_x[1] << ","
                 << s.q_x_hat[0] << "," << s.q_x_hat[1] << "," << s.tau[0] << "," << s.tau[1] << "," << s.tau_star[0] << ","
                 << s.tau_star[1] << "," << s.tau_ext[0] << "," << s.tau_ext[1] << "," << s.X0[0] << "," << s.X0[1] << "," << s.X[0] << ","
                 << s.X[1] << "," << s.X_d[0] << "," << s.X_d[1] << "\n";
        }

        file.close();

        std::cout << "[DataLogger] saved " << data_.size() << " samples to " << filename << std::endl;

        return true;
    }

    void plotAll(const std::string& save_dir = "data/plots") const
    {
        if (data_.empty()) {
            std::cout << "[DataLogger] empty, skip plot." << std::endl;
            return;
        }

        fs::create_directories(save_dir);

        plot_real_tau(save_dir + "/real_tao.pdf");
        plot_tau(save_dir + "/result_tao.pdf");
        plotCartesianPosition(save_dir + "/CartesianPosition_XY.pdf");
        plotJointAngle(save_dir + "/JointAngles.pdf");
        plotJointSpeed(save_dir + "/JointSpeed.pdf");
        plotExternalForce(save_dir + "/ExternalForce.pdf");
    }

    void plot_real_tau(const std::string& filename = "real_tao.pdf") const
    {
        std::vector<double> t, tau_star1, tau_star2;
        extractTime(t);

        tau_star1.reserve(data_.size());
        tau_star2.reserve(data_.size());

        for (const auto& s : data_) {
            tau_star1.push_back(s.tau_star[0]);
            tau_star2.push_back(s.tau_star[1]);
        }

        beginFigure();
        plt::named_plot("tau_star_1", t, tau_star1, "b-");
        plt::named_plot("tau_star_2", t, tau_star2, "r-");
        plt::xlabel("time (s)");
        plt::ylabel("torque (Nm)");
        plt::title("Real Joint Torque");
        finishFigure(filename);

        std::cout << "[DataLogger] " << filename << " saved." << std::endl;
    }

    void plot_tau(const std::string& filename = "result_tao.pdf") const
    {
        std::vector<double> t, tau1, tau2;
        extractTime(t);

        tau1.reserve(data_.size());
        tau2.reserve(data_.size());

        for (const auto& s : data_) {
            tau1.push_back(s.tau[0]);
            tau2.push_back(s.tau[1]);
        }

        beginFigure();
        plt::named_plot("tau_1", t, tau1, "b-");
        plt::named_plot("tau_2", t, tau2, "r-");
        plt::xlabel("time (s)");
        plt::ylabel("torque (Nm)");
        plt::title("Joint Torque");
        finishFigure(filename);

        std::cout << "[DataLogger] " << filename << " saved." << std::endl;
    }

    void plotCartesianPosition(const std::string& filename = "CartesianPosition_XY.pdf") const
    {
        std::vector<double> x0, y0, x, y, xd, yd;

        x0.reserve(data_.size());
        y0.reserve(data_.size());
        x.reserve(data_.size());
        y.reserve(data_.size());
        xd.reserve(data_.size());
        yd.reserve(data_.size());

        for (const auto& s : data_) {
            x0.push_back(s.X0[0]);
            y0.push_back(s.X0[1]);

            x.push_back(s.X[0]);
            y.push_back(s.X[1]);

            xd.push_back(s.X_d[0]);
            yd.push_back(s.X_d[1]);
        }

        beginFigure();

        plt::named_plot("X0 reference", x0, y0, "b--");
        plt::named_plot("X actual", x, y, "r-");
        plt::named_plot("X_d compliant", xd, yd, "g-");

        const double start_x = x0.front();
        const double start_y = y0.front();

        const double end_x = x0.back();
        const double end_y = y0.back();

        if (is_impact_experiment_ && has_impact_waypoints_) {
            std::vector<double> wp_x = {start_x, impact_mid_pos_[0], impact_end_pos_[0]};

            std::vector<double> wp_y = {start_y, impact_mid_pos_[1], impact_end_pos_[1]};

            plt::named_plot("waypoints", wp_x, wp_y, "ko");

            plt::annotate("start", start_x, start_y);
            plt::annotate("mid", impact_mid_pos_[0], impact_mid_pos_[1]);
            plt::annotate("end", impact_end_pos_[0], impact_end_pos_[1]);
        } else {
            std::vector<double> wp_x = {start_x, end_x};
            std::vector<double> wp_y = {start_y, end_y};

            plt::named_plot("trajectory endpoints", wp_x, wp_y, "ko");

            plt::annotate("start", start_x, start_y);
            plt::annotate("end", end_x, end_y);
        }

        plt::xlabel("x (m)");
        plt::ylabel("y (m)");
        plt::title("Cartesian Position");
        plt::grid(true);
        plt::axis("equal");

        finishFigure(filename);

        std::cout << "[DataLogger] " << filename << " saved." << std::endl;
    }

    void plotJointAngle(const std::string& filename = "JointAngles.pdf") const
    {
        std::vector<double> t, q01, q02, q_s1, q_s2, q_x1, q_x2;
        extractTime(t);

        q01.reserve(data_.size());
        q02.reserve(data_.size());
        q_s1.reserve(data_.size());
        q_s2.reserve(data_.size());
        q_x1.reserve(data_.size());
        q_x2.reserve(data_.size());

        for (const auto& s : data_) {
            q01.push_back(s.q0[0]);
            q02.push_back(s.q0[1]);

            q_s1.push_back(s.q_s[0]);
            q_s2.push_back(s.q_s[1]);

            q_x1.push_back(s.q_x[0]);
            q_x2.push_back(s.q_x[1]);
        }

        beginFigure();
        plt::named_plot("q0_1", t, q01, "b--");
        plt::named_plot("q0_2", t, q02, "r--");
        plt::named_plot("q_s_1", t, q_s1, "b-");
        plt::named_plot("q_s_2", t, q_s2, "r-");
        plt::named_plot("q_x_1", t, q_x1, "g-");
        plt::named_plot("q_x_2", t, q_x2, "k-");

        plt::xlabel("time (s)");
        plt::ylabel("joint angle (rad)");
        plt::title("Joint Angles");
        finishFigure(filename);

        std::cout << "[DataLogger] " << filename << " saved." << std::endl;
    }

    void plotJointSpeed(const std::string& filename = "JointSpeed.pdf") const
    {
        std::vector<double> t, q_s_hat1, q_s_hat2, q_x_hat1, q_x_hat2;
        extractTime(t);

        q_s_hat1.reserve(data_.size());
        q_s_hat2.reserve(data_.size());
        q_x_hat1.reserve(data_.size());
        q_x_hat2.reserve(data_.size());

        for (const auto& s : data_) {
            q_s_hat1.push_back(s.q_s_hat[0]);
            q_s_hat2.push_back(s.q_s_hat[1]);

            q_x_hat1.push_back(s.q_x_hat[0]);
            q_x_hat2.push_back(s.q_x_hat[1]);
        }

        beginFigure();
        plt::named_plot("q_s_hat_1", t, q_s_hat1, "b-");
        plt::named_plot("q_s_hat_2", t, q_s_hat2, "r-");
        plt::named_plot("q_x_hat_1", t, q_x_hat1, "b--");
        plt::named_plot("q_x_hat_2", t, q_x_hat2, "r--");

        plt::xlabel("time (s)");
        plt::ylabel("joint speed (rad/s)");
        plt::title("Joint Speed");
        finishFigure(filename);

        std::cout << "[DataLogger] " << filename << " saved." << std::endl;
    }

    void plotExternalForce(const std::string& filename = "ExternalForce.pdf") const
    {
        std::vector<double> t, f1, f2;
        extractTime(t);

        f1.reserve(data_.size());
        f2.reserve(data_.size());

        for (const auto& s : data_) {
            f1.push_back(s.f_ext[0]);
            f2.push_back(s.f_ext[1]);
        }

        beginFigure();
        plt::named_plot("f_ext_1", t, f1, "b-");
        plt::named_plot("f_ext_2", t, f2, "r-");

        plt::xlabel("time (s)");
        plt::ylabel("force (N)");
        plt::title("External Force");
        finishFigure(filename);

        std::cout << "[DataLogger] " << filename << " saved." << std::endl;
    }

    bool loadWaypointsFromYaml(const std::string& yaml_file)
    {
        try {
            YAML::Node config = YAML::LoadFile(yaml_file);
            YAML::Node controller = config["controller"];
            YAML::Node robot = config["robot"];

            if (!controller) {
                std::cout << "[DataLogger] controller node not found in " << yaml_file << std::endl;
                return false;
            }

            if (!robot) {
                std::cout << "[DataLogger] robot node not found in " << yaml_file << std::endl;
                return false;
            }

            std::string control_target = "position";

            if (controller["control_target"]) {
                control_target = controller["control_target"].as<std::string>();
            }

            is_impact_experiment_ = (control_target == "impact");

            if (robot["impact_mid_pos"]) {
                impact_mid_pos_[0] = robot["impact_mid_pos"][0].as<double>();
                impact_mid_pos_[1] = robot["impact_mid_pos"][1].as<double>();
            } else {
                impact_mid_pos_ << -0.107, 0.442;
            }

            if (robot["impact_end_pos"]) {
                impact_end_pos_[0] = robot["impact_end_pos"][0].as<double>();
                impact_end_pos_[1] = robot["impact_end_pos"][1].as<double>();
            } else {
                impact_end_pos_ << 0.034, 0.338;
            }

            has_impact_waypoints_ = true;

            std::cout << "[DataLogger] control_target = " << control_target << std::endl;
            std::cout << "[DataLogger] is_impact_experiment = " << is_impact_experiment_ << std::endl;
            std::cout << "[DataLogger] impact_mid_pos = " << impact_mid_pos_.transpose() << std::endl;
            std::cout << "[DataLogger] impact_end_pos = " << impact_end_pos_.transpose() << std::endl;

            return true;
        } catch (const std::exception& e) {
            std::cout << "[DataLogger] failed to load yaml: " << e.what() << std::endl;
            return false;
        }
    }

  private:
    std::vector<ControlLogSample> data_;

    Eigen::Vector2d impact_mid_pos_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d impact_end_pos_ = Eigen::Vector2d::Zero();

    bool is_impact_experiment_ = false;
    bool has_impact_waypoints_ = false;

    void extractTime(std::vector<double>& t) const
    {
        t.reserve(data_.size());

        for (const auto& s : data_) {
            t.push_back(s.time);
        }
    }

    void beginFigure() const
    {
        plt::clf();
        plt::figure_size(1200, 780);
    }

    void finishFigure(const std::string& filename) const
    {
        fs::path path(filename);
        if (!path.parent_path().empty()) {
            fs::create_directories(path.parent_path());
        }

        plt::legend();
        plt::save(filename);
        plt::clf();
        plt::close();
    }
};