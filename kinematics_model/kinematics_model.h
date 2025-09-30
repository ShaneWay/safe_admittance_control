#ifndef KINEMATICS_MODEL_H
#define KINEMATICS_MODEL_H

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>

#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/urdfreader/urdfreader.h>
#include <vector>

using namespace std;


using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;


class KortexKinematics
{
    public:
        KortexKinematics();

        KortexKinematics(Model* model);

        Model* model_;
        bool getJointAngelsTwoDOF(const Eigen::Vector2d& Q_init, const Eigen::Vector3d& target_pose, Eigen::Vector2d& Q_req);
        void getJacobianMatrixTwoDOF(const Eigen::Vector2d& Q, Eigen::Matrix2d& J);
        void getFowardKinematicsTwoDOF(const Eigen::Vector2d& Q,  Eigen::Vector2d& pos);

        ~KortexKinematics();

};

#endif