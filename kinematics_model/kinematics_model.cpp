#include "kinematics_model.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace {
constexpr double DEG2RAD = M_PI / 180.0;

Eigen::Vector2d dampedLeastSquaresStep(const Eigen::Matrix2d& J, const Eigen::Vector2d& error, double lambda)
{
    Eigen::Matrix2d A = J * J.transpose() + lambda * lambda * Eigen::Matrix2d::Identity();

    return J.transpose() * A.ldlt().solve(error);
}
} // namespace

KortexKinematics::KortexKinematics()
{
    // 注意：这里的 frame 名称必须和 URDF 中完全一致
    loadModel("../urdf/kortex.urdf", "EndEffector_Link");
}

KortexKinematics::KortexKinematics(const std::string& urdf_path, const std::string& end_effector_frame_name)
{
    loadModel(urdf_path, end_effector_frame_name);
}

void KortexKinematics::loadModel(const std::string& urdf_path, const std::string& end_effector_frame_name)
{
    end_effector_frame_name_ = end_effector_frame_name;

    pinocchio::urdf::buildModel(urdf_path, model_);

    if (model_.nq < 7 || model_.nv < 7) {
        throw std::runtime_error("Pinocchio model nq/nv < 7. Please check the URDF model.");
    }

    if (!model_.existFrame(end_effector_frame_name_)) {
        std::cout << "Available frames:" << std::endl;
        for (pinocchio::FrameIndex i = 0; i < model_.frames.size(); ++i) {
            std::cout << i << " : " << model_.frames[i].name << std::endl;
        }

        throw std::runtime_error("End-effector frame not found in URDF: " + end_effector_frame_name_);
    }

    end_effector_frame_id_ = model_.getFrameId(end_effector_frame_name_);

    data_ = std::make_unique<pinocchio::Data>(model_);

    std::cout << "Pinocchio model loaded from: " << urdf_path << std::endl;

    std::cout << "End-effector frame: " << end_effector_frame_name_ << std::endl;
}

Eigen::VectorXd KortexKinematics::makeFullConfiguration(const Eigen::Vector2d& Q) const
{
    Eigen::VectorXd q_delta = Eigen::VectorXd::Zero(model_.nv);

    auto setJointAngle = [&](const std::string& joint_name, double angle) {
        if (!model_.existJointName(joint_name)) {
            throw std::runtime_error("Joint not found: " + joint_name);
        }

        pinocchio::JointIndex jid = model_.getJointId(joint_name);
        int idx_v = model_.joints[jid].idx_v();

        q_delta[idx_v] = angle;
    };

    setJointAngle("Actuator1", Q[0]);
    setJointAngle("Actuator2", 90.0 * DEG2RAD);
    setJointAngle("Actuator3", 90.0 * DEG2RAD);
    setJointAngle("Actuator4", Q[1]);
    setJointAngle("Actuator5", 0.0);
    setJointAngle("Actuator6", 0.0);
    setJointAngle("Actuator7", 0.0);

    Eigen::VectorXd q_full = pinocchio::integrate(model_, pinocchio::neutral(model_), q_delta);

    return q_full;
}

void KortexKinematics::getFowardKinematicsTwoDOF(const Eigen::Vector2d& Q, Eigen::Vector2d& pos)
{
    if (!data_) {
        throw std::runtime_error("Pinocchio data is not initialized.");
    }

    const Eigen::VectorXd q_full = makeFullConfiguration(Q);

    pinocchio::forwardKinematics(model_, *data_, q_full);
    pinocchio::updateFramePlacements(model_, *data_);

    // 直接计算 EndEffector_Link 在基坐标系下的位置
    const Eigen::Vector3d p = data_->oMf[end_effector_frame_id_].translation();

    pos[0] = p[0];
    pos[1] = p[1];
}

bool KortexKinematics::getJointAngelsTwoDOF(const Eigen::Vector2d& Q_init, const Eigen::Vector2d& target_position, Eigen::Vector2d& Q_res)
{
    if (!data_) {
        throw std::runtime_error("Pinocchio data is not initialized.");
    }

    Eigen::Vector2d q = Q_init;

    const int max_iter = 100;
    const double pos_eps = 1e-5;
    const double lambda = 1e-3;
    const double alpha = 0.6;
    const double max_step = 5.0 * DEG2RAD;

    for (int iter = 0; iter < max_iter; ++iter) {
        Eigen::Vector2d current_xy;
        getFowardKinematicsTwoDOF(q, current_xy);

        Eigen::Vector2d error = target_position - current_xy;

        if (error.norm() < pos_eps) {
            Q_res = q;
            return true;
        }

        Eigen::Matrix2d J;
        getJacobianMatrixTwoDOF(q, J);

        Eigen::Vector2d dq = dampedLeastSquaresStep(J, error, lambda);

        double dq_norm = dq.norm();
        if (dq_norm > max_step) {
            dq = dq / dq_norm * max_step;
        }

        q += alpha * dq;
    }

    Q_res = q;

    Eigen::Vector2d final_xy;
    getFowardKinematicsTwoDOF(Q_res, final_xy);

    double final_error = (target_position - final_xy).norm();

    if (final_error < 1e-3) {
        return true;
    }

    std::cout << "[IK WARNING] Can't get inverse solution. "
              << "target xy = " << target_position.transpose() << ", final xy = " << final_xy.transpose() << ", error = " << final_error
              << " m" << std::endl;

    return false;
}

void KortexKinematics::getJacobianMatrixTwoDOF(const Eigen::Vector2d& Q, Eigen::Matrix2d& J)
{
    if (!data_) {
        throw std::runtime_error("Pinocchio data is not initialized.");
    }

    const Eigen::VectorXd q_full = makeFullConfiguration(Q);

    pinocchio::Data::Matrix6x J6(6, model_.nv);
    J6.setZero();

    pinocchio::computeFrameJacobian(model_, *data_, q_full, end_effector_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J6);

    const int idx_v1 = model_.joints[model_.getJointId("Actuator1")].idx_v();

    const int idx_v4 = model_.joints[model_.getJointId("Actuator4")].idx_v();

    // Pinocchio 的前 3 行是线速度部分：vx, vy, vz
    // 当前二维控制只取末端 x、y 方向，并只取第 1、4 关节列
    J(0, 0) = J6(0, idx_v1);
    J(0, 1) = J6(0, idx_v4);
    J(1, 0) = J6(1, idx_v1);
    J(1, 1) = J6(1, idx_v4);
}