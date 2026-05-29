#include <iostream>
#include <iomanip>
#include <cmath>
#include <Eigen/Dense>

#include "kinematics_model.h"

static constexpr double DEG2RAD = M_PI / 180.0;
static constexpr double RAD2DEG = 180.0 / M_PI;

static void printOneCase(KortexKinematics& model, double q1_deg, double q4_deg, const std::string& name)
{
    Eigen::Vector2d q;
    q << q1_deg * DEG2RAD, q4_deg * DEG2RAD;

    Eigen::Vector2d pos;
    model.getFowardKinematicsTwoDOF(q, pos);

    Eigen::Matrix2d J;
    model.getJacobianMatrixTwoDOF(q, J);

    std::cout << "\n========== " << name << " ==========" << std::endl;
    std::cout << "q1 = " << q1_deg << " deg, q4 = " << q4_deg << " deg" << std::endl;
    std::cout << "q  = [" << q.transpose() << "] rad" << std::endl;
    std::cout << "X  = [" << pos.transpose() << "] m" << std::endl;
    std::cout << "J  = \n" << J << std::endl;
}

static void checkJacobianByFiniteDifference(KortexKinematics& model, double q1_deg, double q4_deg)
{
    Eigen::Vector2d q;
    q << q1_deg * DEG2RAD, q4_deg * DEG2RAD;

    Eigen::Vector2d x0;
    model.getFowardKinematicsTwoDOF(q, x0);

    Eigen::Matrix2d J_model;
    model.getJacobianMatrixTwoDOF(q, J_model);

    const double eps = 1e-6;
    Eigen::Matrix2d J_num;

    for (int i = 0; i < 2; ++i) {
        Eigen::Vector2d q_plus = q;
        Eigen::Vector2d q_minus = q;
        q_plus[i] += eps;
        q_minus[i] -= eps;

        Eigen::Vector2d x_plus;
        Eigen::Vector2d x_minus;
        model.getFowardKinematicsTwoDOF(q_plus, x_plus);
        model.getFowardKinematicsTwoDOF(q_minus, x_minus);

        J_num.col(i) = (x_plus - x_minus) / (2.0 * eps);
    }

    std::cout << "\n========== Jacobian finite-difference check ==========" << std::endl;
    std::cout << "Test configuration: q1 = " << q1_deg << " deg, q4 = " << q4_deg << " deg" << std::endl;
    std::cout << "J from model:\n" << J_model << std::endl;
    std::cout << "J by finite difference:\n" << J_num << std::endl;
    std::cout << "J error:\n" << (J_model - J_num) << std::endl;
    std::cout << "max abs error = " << (J_model - J_num).cwiseAbs().maxCoeff() << std::endl;
}

int main()
{
    std::cout << std::fixed << std::setprecision(6);

    KortexKinematics model;

    // 你的关节空间轨迹起点与终点：
    // 7 维角度分别为：
    // start = 167.0, 90.0, 90.0, 122.0, 0.0, 0.0, 0.0 deg
    // end   = 236.0, 90.0, 90.0, 122.0, 0.0, 0.0, 0.0 deg
    // getFowardKinematicsTwoDOF() 内部固定 q2=90°, q3=90°, q5=q6=q7=0°，
    // 因此这里输入的 Eigen::Vector2d 只包含 q1 和 q4。

    printOneCase(model, 184.782, 121.953, "trajectory start");

    // 再测试几个中间点，方便观察末端位置是否连续变化。
    printOneCase(model, 184.25, 122.0, "25 percent");
    printOneCase(model, 201.50, 122.0, "50 percent");
    printOneCase(model, 218.75, 122.0, "75 percent");

    // 用数值差分检查 getFowardKinematicsTwoDOF() 与 getJacobianMatrixTwoDOF() 是否一致。
    // 如果 max abs error 很小，例如 1e-6 ~ 1e-4 量级，说明正运动学和雅可比基本一致。
    checkJacobianByFiniteDifference(model, 184.0, 122.0);

    return 0;
}
