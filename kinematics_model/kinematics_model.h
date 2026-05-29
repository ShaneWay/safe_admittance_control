#ifndef KINEMATICS_MODEL_H
#define KINEMATICS_MODEL_H

#include <Eigen/Dense>
#include <memory>
#include <string>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

class KortexKinematics
{
  public:
    KortexKinematics();

    KortexKinematics(const std::string& urdf_path, const std::string& end_effector_frame_name);

    bool getJointAngelsTwoDOF(const Eigen::Vector2d& Q_init, const Eigen::Vector2d& target_position, Eigen::Vector2d& Q_res);

    void getJacobianMatrixTwoDOF(const Eigen::Vector2d& Q, Eigen::Matrix2d& J);

    void getFowardKinematicsTwoDOF(const Eigen::Vector2d& Q, Eigen::Vector2d& pos);

  private:
    pinocchio::Model model_;
    std::unique_ptr<pinocchio::Data> data_;

    pinocchio::FrameIndex end_effector_frame_id_;
    std::string end_effector_frame_name_;

    void loadModel(const std::string& urdf_path, const std::string& end_effector_frame_name);

    Eigen::VectorXd makeFullConfiguration(const Eigen::Vector2d& Q) const;
};

#endif