#ifndef CONTROLLER_2_H
#define CONTROLLER_2_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <kinematics_model.h>

using namespace std;

typedef vector<Eigen::Vector2d> vv2d;

#define TimeUnit 0.002

const Eigen::Vector2d init_pos(0.43, -0.126);
const Eigen::Vector2d init_angle(-53. / 180. * M_PI, 122. / 180. * M_PI);

constexpr int count_total = 15000; // total number of control frame
constexpr double omiga = 2. * M_PI / 12.;   // omiga of sine curve
constexpr double amplitude = 0.15;           // amplitude of Sine curve
constexpr double x_amplitude = 0.005; 
// constexpr double amplitude = 0.;           // amplitude of Sine curve
// constexpr double x_amplitude = 0.00; 

enum class Mode
{
    NORMAL,
    DI,
    DISM,
    DIML
};

class controller_2
{
private:
    //------------------------------------------------------
    // control parameters
    //------------------------------------------------------

    Eigen::Vector2d F_max;
    Eigen::Matrix2d M_d_a;
    Eigen::Matrix2d D_d_a;
    Eigen::Matrix2d K_d_a;
    Eigen::Matrix2d M;
    Eigen::Matrix2d K;
    Eigen::Matrix2d B;
    Eigen::Matrix2d L;

    Eigen::Matrix2d M_s;
    Eigen::Matrix2d D_s;
    Eigen::Matrix2d M_s_hat;
    Eigen::Matrix2d D_s_hat;
    Eigen::Matrix2d Lamda_1;
    Eigen::Matrix2d Lamda_2;
    Eigen::Vector2d F0;

    Eigen::Matrix2d K_star;




    //------------------------------------------------------
    // control procedure parameters
    //------------------------------------------------------

    vv2d u_x_star;
    vv2d q_x_star;
    vv2d phi_b;
    vv2d phi_a;
    vv2d q_star;
    vv2d tau_star;
    vv2d tau;
    vv2d q_x;
    vv2d q_x_dot;
    vv2d u_x;
    vv2d a;

    vv2d e_r;
    vv2d e_q;
    vv2d v_x;

    //------------------------------------------------------
    // input parameters get from kinova roboe
    //------------------------------------------------------

    vv2d q;     // the joint position of the manipulator
    vv2d f_ext;     // the external force get from force sersor
    vv2d tau_ext; // the joint torque computed from Jacobi matrix

    //------------------------------------------------------
    // init parameters to generate the trajectory
    //------------------------------------------------------

    vv2d q0_ddot;
    vv2d q0_dot;
    vv2d q0;
    vv2d X;
    vv2d X_d;
    vv2d X0, X0_dot, X0_ddot;

    //------------------------------------------------------
    // useful parameters to get internal variable
    //------------------------------------------------------

    Eigen::Matrix2d K_hat;
    Eigen::Matrix2d jacobian_last;
    Eigen::Matrix2d jacobian_now;

    //------------------------------------------------------
    // control frame number
    //------------------------------------------------------

    int32_t frame;
    KortexKinematics model;


    // Meta learning



    vv2d e_v_ML;
    vv2d f_e_ML;
  

    Eigen::Matrix<double,32,4> omiga1_ML;
    Eigen::Matrix<double,32,32> omiga2_ML;
    Eigen::Matrix<double,32,1> b1_ML;
    Eigen::Matrix<double,32,1> b2_ML;

    Eigen::Matrix<double,2,32> A_ML;
    Eigen::Matrix<double,2,32> A_last_ML;
    Eigen::Matrix<double,2,32> A_dot_ML;

    Eigen::Matrix2d Tau_ML;

    

public:
    controller_2(/* args */);

    /**
     * @brief                       get joint torque according to control algorithm which was seleted by the _mode enum
     * @param T                     unit time of discrete system
     * @param f_ext_from_sensor     force exerted on the end of robot at this frame
     * @param q_frame_sensor        joint angle of robot at this frame
     * @return                      get joint torque to control the robot
     * @retval                      Eigen::Vector2d joint torque to control the robot
     * @enum                        controller_2::Mode
     */
    Eigen::Vector2d getTorque(const double &T, Eigen::Vector2d &f_ext_from_sensor, Eigen::Vector2d &q_frome_sensor);

    /**
     * @brief                       get joint torque according to DI admittance control algorithm
     * @param T                     unit time of discrete system
     * @param f_ext_from_sensor     force exerted on the end of robot at this frame
     * @param q_frame_sensor        joint angle of robot at this frame
     * @return                      get joint torque to control the robot
     * @retval                      Eigen::Vector2d joint torque to control the robot
     * @enum                        controller_2::Mode
     */
    Eigen::Vector2d getTorqueDI(const double &T, Eigen::Vector2d &f_ext_from_sensor, Eigen::Vector2d &q_frome_sensor);

    /**
     * @brief                       get joint torque according to normal admittance control algorithm with the first discrete method
     * @param T                     unit time of discrete system
     * @param f_ext_from_sensor     force exerted on the end of robot at this frame
     * @param q_frame_sensor        joint angle of robot at this frame
     * @return                      get joint torque to control the robot
     * @retval                      Eigen::Vector2d joint torque to control the robot
     * @enum                        controller_2::Mode
     */
    Eigen::Vector2d getTorqueNormal(const double &T, Eigen::Vector2d &f_ext_from_sensor, Eigen::Vector2d &q_frome_sensor);

    /**
     * @brief                       get joint torque according to normal admittance control algorithm with the second discrete method
     * @param T                     unit time of discrete system
     * @param f_ext_from_sensor     force exerted on the end of robot at this frame
     * @param q_frame_sensor        joint angle of robot at this frame
     * @return                      get joint torque to control the robot
     * @retval                      Eigen::Vector2d joint torque to control the robot
     * @enum                        controller_2::Mode
     */
    Eigen::Vector2d getTorqueNormal_2(const double &T, Eigen::Vector2d &f_ext_from_sensor, Eigen::Vector2d &q_frome_sensor);

    /**
     * @brief                       get joint torque according to DISM admittance control algorithm
     * @param T                     unit time of discrete system
     * @param f_ext_from_sensor     force exerted on the end of robot at this frame
     * @param q_frame_sensor        joint angle of robot at this frame
     * @return                      get joint torque to control the robot
     * @retval                      Eigen::Vector2d joint torque to control the robot
     * @enum                        controller_2::Mode
     */
    Eigen::Vector2d getTorqueDISM(const double &T, Eigen::Vector2d &f_ext_from_sensor, Eigen::Vector2d &q_frome_sensor);

    Eigen::Vector2d getTorqueDIML(const double &T, Eigen::Vector2d &f_ext_from_sensor, Eigen::Vector2d &q_frome_sensor);
    void refreshDIML(const double & T);

    /**
     * @brief                       refresh the control frame once
     * @param T                     unit time of discrete system
     */
    void refresh(const double &T);

    /**
     * @brief                       refresh the control frame once
     * @param T                     unit time of discrete system
     */
    void refreshDI(const double & T);

    /**
     * @brief                       refresh the control frame once
     * @param T                     unit time of discrete system
     */
    void refreshNormal(const double &T);

    /**
     * @brief                       refresh the control frame once
     * @param T                     unit time of discrete system
     */
    void refreshDISM(const double &T);
    void getfe(vv2d &q, const double &T);
    void plotJointTorque();
    void plotExternalForce();
    void plotExternalJointForce();
    void plotCartesianPosition();
    void plotJointAngles();


    void plotGeneratedTrajectory();

    void plot_q_star();
    void plot_q_x_star();
    void plot_u_x_star();

    void saveData();

    void setMode(Mode mode);

private:
    Mode _mode;

    // Project the computed torque to saturated torque
    Eigen::Vector2d proj(Eigen::Vector2d &tau_star_input);
    Eigen::Vector2d project(Eigen::Matrix2d S, Eigen::Vector2d q);

    void generateJointTrajectory();
    void generateJointTrajectory_line();

public:
    virtual ~controller_2();
};

#endif