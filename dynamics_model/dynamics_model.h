#ifndef DYNAMICS_MODEL_H
#define DYNAMICS_MODEL_H

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


class KortexDynamics
{
    public:
        KortexDynamics();

        KortexDynamics(Model* model);

        Model* model_;
        void getGravity(vector<double>& q, vector<double>& g);

        ~KortexDynamics();



};










#endif