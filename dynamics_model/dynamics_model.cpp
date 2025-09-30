#include <dynamics_model.h>

KortexDynamics::KortexDynamics(){
    model_ = new Model();
    Addons::URDFReadFromFile ("/home/kortex/yxw/kortex_admittance_control/controller/GEN3_URDF_V12.urdf", model_, false);
    model_->gravity = Vector3d (0. ,0. , -9.81);

    // std::cout << "Degree of freedom overview:" << std::endl;
    // std::cout << Utils::GetModelDOFOverview(*model_);

    // std::cout << "Model Hierarchy:" << std::endl;
    // std::cout << Utils::GetModelHierarchy(*model_);
}

KortexDynamics::KortexDynamics(Model* model)
{
    model_ = model;
    model_->gravity = Vector3d (0. ,0. , -9.81);

    // std::cout << "Degree of freedom overview:" << std::endl;
    // std::cout << Utils::GetModelDOFOverview(*model_);

    // std::cout << "Model Hierarchy:" << std::endl;
    // std::cout << Utils::GetModelHierarchy(*model_);
}

void KortexDynamics::getGravity(vector<double>& q, vector<double>& g)
{
    VectorNd Q = VectorNd::Zero (model_->q_size);
    for(int i = 0; i < model_->q_size; i++)
    {
        Q[i] = q[i];
    }
    VectorNd QDot = VectorNd::Zero (model_->qdot_size);
    VectorNd Tau = VectorNd::Zero (model_->qdot_size);
    VectorNd QDDot = VectorNd::Zero (model_->qdot_size);
    InverseDynamics(*model_, Q, QDot, QDDot, Tau);

    for(int i = 0; i < model_->q_size; i++)
    {
        g[i] = Tau[i];
    }

}

KortexDynamics::~KortexDynamics()
{
    delete model_;
}