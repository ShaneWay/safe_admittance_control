#pragma once
#include "BaseController.h"
#include <iostream>
#include <numeric>

class DismcController : public BaseController {
private:

public:
    DismcController(BaseParams& params, ControlState& state) : BaseController(params, state)
    {

    }
    Eigen::Vector2d getTorqueOnForceControl(Eigen::Vector2d & f_ext_from_sensor, Eigen::Vector2d& q_frome_sensor) override
    {
        
    }

    void refreshOnForceControl() override
    {
       
    }


    Eigen::Vector2d getTorqueOnPositionControl(Eigen::Vector2d & f_ext_from_sensor, Eigen::Vector2d& q_frome_sensor) override
    {
        f.push_back(f_ext_from_sensor);
        q_s.push_back(q_frome_sensor);
        q_s_hat.push_back((q_s[frame] - q_s[frame - 1])/dt);
        cout << "f[frame " << frame << "] = " << f[frame].transpose() << endl;
        cout << "q[frame " << frame << "] = " << q_s[frame].transpose() << endl;
        model.getJacobianMatrixTwoDOF(q_frome_sensor, jacobian_now);
        //update q and f_ext , get them from sensor
        tau_ext.push_back(jacobian_now.transpose() * f_ext_from_sensor);
                    
            

    }

    void refreshOnPositionControl() override
    {
       
    }
};
