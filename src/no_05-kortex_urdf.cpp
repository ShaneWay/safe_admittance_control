/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2016 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>

#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/urdfreader/urdfreader.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main (int argc, char* argv[]) {
    rbdl_check_api_version (RBDL_API_VERSION);

    Model* model = new Model();


    Addons::URDFReadFromFile ("/home/yxw/Project/series_admittance-impedance_controller/controller/GEN3_URDF_V12.urdf", model, false);
    


    std::cout << "Degree of freedom overview:" << std::endl;
    std::cout << Utils::GetModelDOFOverview(*model);

    std::cout << "Model Hierarchy:" << std::endl;
    std::cout << Utils::GetModelHierarchy(*model);
    model->gravity = Vector3d (0. ,0. , -9.81);
    VectorNd Q = VectorNd::Zero (model->q_size);
    VectorNd QDot = VectorNd::Zero (model->qdot_size);
    VectorNd Tau = VectorNd::Zero (model->qdot_size);
    VectorNd QDDot = VectorNd::Zero (model->qdot_size);
    // Tau << 0.5 , 0.5;

    // Q[0] = 44.173 / 180 * M_PI;
    // Q[1] = 7.676 / 180 * M_PI;
    // Q[2] = 187.008 / 180 * M_PI;
    // Q[3] = 271.121 / 180 * M_PI;
    // Q[4] = 106.686 / 180 * M_PI;
    // Q[5] = 56.771 / 180 * M_PI;
    // Q[6] = 337.161 / 180 * M_PI;
  
    Q[0] = 359.998 / 180 * M_PI;
    Q[1] = 15.004 / 180 * M_PI;
    Q[2] = 180.008 / 180 * M_PI;
    Q[3] = 229.999 / 180 * M_PI;
    Q[4] = 359.999 / 180 * M_PI;
    Q[5] = 55.0 / 180 * M_PI;
    Q[6] = 90.002 / 180 * M_PI;

    std::vector<unsigned int> id;
    id.push_back(7);
    std::vector<Vector3d> point;
    point.push_back(Vector3d(0, 0, -0.0615));

    std::vector<Vector3d> pose;
    pose.push_back(Vector3d (0.456648,0.00130385,0.433686));

    VectorNd Qres = VectorNd::Zero (model->q_size);

    auto posistion = CalcBodyToBaseCoordinates(*model, Q, 7 , Vector3d(0, 0, -0.0615));

    bool get = InverseKinematics(*model, Q, id, point, pose, Qres);
    std::cout << Qres.transpose() << std::endl;

    std::cout << posistion.transpose() << std::endl;//0.46, 0.008, 0.438

    // VectorNd QDot = VectorNd::Zero(model->dof_count);
	// VectorNd Tau = VectorNd::Zero(model->dof_count);
	// // Tau << 0.5 , 0.5;
	// VectorNd QDDot = VectorNd::Zero(model->dof_count);

 	InverseDynamics(*model, Q, QDot, QDDot, Tau);

    std::cout << Tau.transpose() << std::endl;

    // std::cout << "Forward Dynamics with q, qdot, tau set to zero:" << std::endl;
    // ForwardDynamics (*model, Q, QDot, Tau, QDDot);

    // std::cout << QDDot.transpose() << std::endl;

    delete model;

    return 0;
}

