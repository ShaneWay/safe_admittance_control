#include <iostream>

#include <rbdl/rbdl.h>
#include <kinematics_model.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;


int main (int argc, char* argv[]) {
	rbdl_check_api_version (RBDL_API_VERSION);

	Model* model = NULL;

	unsigned int base_link, body_link1_id, body_link2_id,
                 body_link3_id, body_link4_id, body_link5_id, 
                 body_link6_id, body_link7_id;

	Body  body_1, body_2, body_3, body_4, body_5, body_6, body_7;
	Joint joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7;

    model = new Model();

    model->gravity = Vector3d (0., 0., -9.81);


    // base link profile

    // Matrix3d inertia_base;
    // Vector3d mass_center_base(-0.000648, -0.000166, 0.084487);
    // double mass_base = 1.697;
	// inertia_base << 0.004622, -0.000009, -0.000060, 
	// 			   -0.000009,  0.004495, -0.000009,
	// 			   -0.000060, -0.000009,  0.002079;

    // joint_base = Joint(JointTypeFixed, Vector3d (0., 0., 1.));
    // base = Body (mass_base, mass_center_base, inertia_base);

    // body_base_id = model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_base, base);

    // link1 profile

    Matrix3d inertia_link1;
    Vector3d mass_center_link1(-0.000023, -0.010364, -0.073360);
    double mass_link1 = 1.697;
	inertia_link1 << 0.004570,  -0.000001, -0.000002, 
				    -0.000001,   0.004831, -0.000448,
				    -0.000002,  -0.000448,  0.001409;

    joint_1 = Joint(JointTypeRevoluteZ);
    body_1 = Body (mass_link1, mass_center_link1, inertia_link1);

    Matrix3d trans_rotate1 = rotx(M_PI);     
    Vector3d trans_trans1(0., 0., 0.15643);

    body_link1_id = model->AddBody(0, SpatialTransform(trans_rotate1, trans_trans1), joint_1, body_1, "link1");

    // link2 profile

    Matrix3d inertia_link2;
    Vector3d mass_center_link2(-0.000044, -0.099580, -0.013278);
    double mass_link2 = 1.1636;
	inertia_link2 << 0.011088, -0.000005, -0.000000, 
				    -0.000005,  0.001072,  0.000691,
				    -0.000000,  0.000691,  0.011255;

    joint_2 = Joint(JointTypeRevoluteZ);
    body_2 = Body (mass_link2, mass_center_link2, inertia_link2);

    Matrix3d trans_rotate2 = rotx(M_PI /2);
    Vector3d trans_trans2(0., 0.005375, -0.12838);

    body_link2_id = model->AddBody(body_link1_id, SpatialTransform(trans_rotate2, trans_trans2), joint_2, body_2, "link2");

    // link3 profile

    Matrix3d inertia_link3;
    Vector3d mass_center_link3(-0.000044, -0.006641, -0.117892);
    double mass_link3 = 1.1636;
	inertia_link3 << 0.010932, 0.000000, 0.000007, 
				     0.000000, 0.011127, -0.000606,
				     0.000007, -0.000606, 0.001043;

    joint_3 = Joint(JointTypeRevoluteZ);
    body_3 = Body (mass_link3, mass_center_link3, inertia_link3);

    Matrix3d trans_rotate3 = rotx(-M_PI /2);
    // Vector3d trans_trans3(0., -0.21038, -0.006375);
    Vector3d trans_trans3(0., -0.20338, -0.006375);

    // std::cout <<trans_rotate3 << std::endl;

    body_link3_id = model->AddBody(body_link2_id, SpatialTransform(trans_rotate3, trans_trans3), joint_3, body_3, "link3");

    // link4 profile

    Matrix3d inertia_link4;
    Vector3d mass_center_link4(-0.000018, -0.075478, -0.015006);
    double mass_link4 = 0.930;
	inertia_link4 << 0.008147, 0.000001, 0.000000, 
				     0.000001, 0.000631, 0.000500,
				     0.000000, 0.000500, 0.008316;

    joint_4 = Joint(JointTypeRevoluteZ);
    body_4 = Body (mass_link4, mass_center_link4, inertia_link4);

    Matrix3d trans_rotate4 = rotx(M_PI /2);
    Vector3d trans_trans4(0., 0.006375, -0.21038);

    body_link4_id = model->AddBody(body_link3_id, SpatialTransform(trans_rotate4, trans_trans4), joint_4, body_4, "link4");

    // link5 profile

    Matrix3d inertia_link5;
    Vector3d mass_center_link5(0.000001, -0.009432, -0.063883);
    double mass_link5 = 0.678;
	inertia_link5 << 0.001596, 0.000000, 0.000000, 
				     0.000000, 0.001607, -0.000256,
				     0.000000, -0.000256, 0.000399;

    joint_5 = Joint(JointTypeRevoluteZ);
    body_5 = Body (mass_link5, mass_center_link5, inertia_link5);

    Matrix3d trans_rotate5 = rotx(-M_PI /2);;
    Vector3d trans_trans5(0., -0.20843, -0.006375);

    body_link5_id = model->AddBody(body_link4_id, SpatialTransform(trans_rotate5, trans_trans5), joint_5, body_5, "link5");

    // link6 profile
    
    Matrix3d inertia_link6;
    Vector3d mass_center_link6(0.000001, -0.045483, -0.009650);
    double mass_link6 = 0.678;
	inertia_link6 << 0.001641, 0.000000, 0.000000, 
				     0.000000, 0.000410, 0.000278,
				     0.000000, 0.000278, 0.001641;

    joint_6 = Joint(JointTypeRevoluteZ);
    body_6 = Body (mass_link6, mass_center_link6, inertia_link6);

    Matrix3d trans_rotate6 = rotx(M_PI /2);
    Vector3d trans_trans6(0., 0.00017505, -0.10593);

    body_link6_id = model->AddBody(body_link5_id, SpatialTransform(trans_rotate6, trans_trans6), joint_6, body_6, "link6");

    // link7 profile

    Matrix3d inertia_link7;
    Vector3d mass_center_link7(-0.000093, 0.000132, -0.022905);
    double mass_link7 = 0.364;
	inertia_link7 << 0.000214, 0.000000, -0.000001, 
				     0.000000, 0.000223, 0.000002,
				     -0.000000, 0.000002, 0.000240;

    joint_7 = Joint(JointTypeRevoluteZ);
    body_7 = Body (mass_link7, mass_center_link7, inertia_link7);

    Matrix3d trans_rotate7 = rotx(-M_PI /2);
    Vector3d trans_trans7(0., -0.10593, -0.00017505);

    body_link7_id = model->AddBody(body_link6_id, SpatialTransform(trans_rotate7, trans_trans7), joint_7, body_7, "link7");

    std::cout << Utils::GetModelHierarchy(*model);

    VectorNd Q = VectorNd::Zero(model->dof_count);
    // Q[0] = 359.998 / 180 * M_PI;
    // Q[1] = 15.004 / 180 * M_PI;
    // Q[2] = 180.008 / 180 * M_PI;
    // Q[3] = 229.999 / 180 * M_PI;
    // Q[4] = 359.999 / 180 * M_PI;
    // Q[5] = 55.0 / 180 * M_PI;
    // Q[6] = 90.002 / 180 * M_PI;

    // Q[0] = 44.173 / 180 * M_PI;
    // Q[1] = 7.676 / 180 * M_PI;
    // Q[2] = 187.008 / 180 * M_PI;
    // Q[3] = 271.121 / 180 * M_PI;
    // Q[4] = 106.686 / 180 * M_PI;
    // Q[5] = 56.771 / 180 * M_PI;
    // Q[6] = 337.161 / 180 * M_PI;

    VectorNd Q_status = VectorNd::Zero(7);
    Q_status[0] = -53. / 180 * M_PI;
    Q_status[1] = 90. / 180 * M_PI;
    Q_status[2] = 90. / 180 * M_PI;
    Q_status[3] = 122. / 180 * M_PI;
    Q_status[4] = 0. / 180 * M_PI;
    Q_status[5] = 0. / 180 * M_PI;
    Q_status[6] = 0.0 / 180 * M_PI;

    std::cout << "body_link7_id: " << body_link7_id << std::endl; 
    std::cout << Q_status.transpose() << std::endl;
    auto posistion = CalcBodyToBaseCoordinates(*model, Q_status, body_link4_id, Vector3d(0., -0.4817, -0.0064));

    std::cout << posistion.transpose() << std::endl;//0.46, 0.008, 0.438


    // VectorNd QDot = VectorNd::Zero(model->dof_count);
	// VectorNd Tau = VectorNd::Zero(model->dof_count);
	// // Tau << 0.5 , 0.5;
	// VectorNd QDDot = VectorNd::Zero(model->dof_count);

 	// InverseDynamics(*model, Q, QDot, QDDot, Tau);

    // std::cout << Tau.transpose() << std::endl;
    Eigen::Vector2d qres;
    Eigen::Vector2d qinit = Q;
    Eigen::Vector3d target_pose(0.309635, -0.011725,  0.336539);
    KortexKinematics korkin;
    korkin.getJointAngelsTwoDOF(qinit, target_pose, qres);
    cout << "inverse solution: " << qres.transpose() << endl;
    Eigen::Matrix2d jacobian;
    korkin.getJacobianMatrixTwoDOF(qinit, jacobian);
    cout << "jacobian matrix: \n" << jacobian << endl;


	delete model;
}