#include <kinematics_model.h>

KortexKinematics::KortexKinematics()
{
    model_ = new Model();

    unsigned int base_link, body_link1_id, body_link2_id,
                 body_link3_id, body_link4_id, body_link5_id, 
                 body_link6_id, body_link7_id;

	Body  body_1, body_2, body_3, body_4, body_5, body_6, body_7;
	Joint joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7;


    // base link profile

    // Matrix3d inertia_base;
    // Vector3d mass_center_base(-0.000648, -0.000166, 0.084487);
    // double mass_base = 1.697;
	// inertia_base << 0.004622, -0.000009, -0.000060, 
	// 			   -0.000009,  0.004495, -0.000009,
	// 			   -0.000060, -0.000009,  0.002079;

    // joint_base = Joint(JointTypeFixed, Vector3d (0., 0., 1.));
    // base = Body (mass_base, mass_center_base, inertia_base);

    // body_base_id = model_->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_base, base);

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

    body_link1_id = model_->AddBody(0, SpatialTransform(trans_rotate1, trans_trans1), joint_1, body_1, "link1");

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

    body_link2_id = model_->AddBody(body_link1_id, SpatialTransform(trans_rotate2, trans_trans2), joint_2, body_2, "link2");

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
    Vector3d trans_trans3(0., -0.21038, -0.006375);

    // std::cout <<trans_rotate3 << std::endl;

    body_link3_id = model_->AddBody(body_link2_id, SpatialTransform(trans_rotate3, trans_trans3), joint_3, body_3, "link3");

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

    body_link4_id = model_->AddBody(body_link3_id, SpatialTransform(trans_rotate4, trans_trans4), joint_4, body_4, "link4");

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

    body_link5_id = model_->AddBody(body_link4_id, SpatialTransform(trans_rotate5, trans_trans5), joint_5, body_5, "link5");

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

    body_link6_id = model_->AddBody(body_link5_id, SpatialTransform(trans_rotate6, trans_trans6), joint_6, body_6, "link6");

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

    body_link7_id = model_->AddBody(body_link6_id, SpatialTransform(trans_rotate7, trans_trans7), joint_7, body_7, "link7");

    // std::cout << Utils::GetModelHierarchy(*model_) << std::endl;
    
}

KortexKinematics::KortexKinematics(Model* model)
{
    model_ = model;
    std::cout << Utils::GetModelHierarchy(*model_) << std::endl;
}

bool KortexKinematics::getJointAngelsTwoDOF(const Eigen::Vector2d& Q_init, const Eigen::Vector3d& target_posistion, Eigen::Vector2d& Q_res)
{
    bool isGetInverseSolution;
    VectorNd Qinit = Q_init;
    VectorNd Qres = Q_res;

    vector<Vector3d> target_pos;
    target_pos.push_back(target_posistion);

    vector<Vector3d> body_point;
    body_point.push_back(Vector3d(0., -0.4817, -0.0064));

    vector<unsigned int> body_id;
    body_id.push_back(2);
    
    isGetInverseSolution = InverseKinematics(*model_, Qinit, body_id, body_point, target_pos, Qres);

    if(isGetInverseSolution)
    {
        Q_res[0] = Qres[0];
        Q_res[1] = Qres[1];
    }else{
        std::cout << "Can't get the inverse solutions with this configuration." << std::endl;
    }
    

    return isGetInverseSolution;
}


void KortexKinematics::getJacobianMatrixTwoDOF(const Eigen::Vector2d& Q, Eigen::Matrix2d& J)
{
    VectorNd Q_status = VectorNd::Zero(7);
    Q_status[0] = Q[0];
    Q_status[1] = 90. / 180 * M_PI;
    Q_status[2] = 90. / 180 * M_PI;
    Q_status[3] = Q[1];
    Q_status[4] = 0. / 180 * M_PI;
    Q_status[5] = 0. / 180 * M_PI;
    Q_status[6] = 0.0 / 180 * M_PI;
    
    MatrixNd jacobian = MatrixNd::Zero(7,7);
    unsigned int body_id = 4;
    Vector3d pody_point(0., -0.4817, -0.0064);

    CalcPointJacobian(*model_, Q_status, body_id, pody_point, jacobian);
    // std:: cout << jacobian << endl;
    // J(0,0) = jacobian(0,0);
    // J(0,1) = jacobian(0,1);
    // J(1,0) = jacobian(1,0);
    // J(1,1) = jacobian(1,1);
    J(0,0) = jacobian(0,0);
    J(0,1) = jacobian(0,3);
    J(1,0) = jacobian(1,0);
    J(1,1) = jacobian(1,3);
    // cout << jacobian << endl;
    // cout << "================================" << endl;
    
}


void KortexKinematics::getFowardKinematicsTwoDOF(const Eigen::Vector2d& Q, Eigen::Vector2d& pos)
{
    VectorNd Q_status = VectorNd::Zero(7);
    Q_status[0] = Q[0];
    Q_status[1] = 90. / 180 * M_PI;
    Q_status[2] = 90. / 180 * M_PI;
    Q_status[3] = Q[1];
    Q_status[4] = 0. / 180 * M_PI;
    Q_status[5] = 0. / 180 * M_PI;
    Q_status[6] = 0.0 / 180 * M_PI;
    unsigned int body_id = 4;
    Vector3d pody_point(0., -0.4817, -0.0064);

    auto posistion = CalcBodyToBaseCoordinates(*model_, Q_status, body_id, Vector3d(0., -0.4817, -0.0064));
    // std:: cout << jacobian << endl;
    // J(0,0) = jacobian(0,0);
    // J(0,1) = jacobian(0,1);
    // J(1,0) = jacobian(1,0);
    // J(1,1) = jacobian(1,1);
    pos[0] = posistion[0];
    pos[1] = posistion[1];
    
    
}


KortexKinematics::~KortexKinematics()
{
    delete model_;
}