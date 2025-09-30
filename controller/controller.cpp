#include <controller.h>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

controller::controller()
{
    tao.push_back(0);
    tao_star.push_back(0);

    u_x.push_back(0);
    u_x_star.push_back(0);

    q_x.push_back(init_angle);
    q_x_star.push_back(init_angle);
    q_x_hat.pop_back(0);

    phi_b.push_back(0);
    phi_a.push_back(0);

    q_s_star.push_back(init_angle);
    q_s.push_back(init_angle);

    a.push_back(0);

    f.push_back(0);
    f_d.push_back(0);

    frame = 1;
}

double controller::get_tau(const double & T, double & f_ext_from_sensor, double & q_frome_sensor)
{ 

    f.push_back(f_ext_from_sensor);
    q_x.push_back(q_frome_sensor);
    cout << "f[frame " << frame << "] = " << f[frame] << endl;
    cout << "q[frame " << frame << "] = " << q[frame] << endl;


    double K_hat = K + B / T + L * T;

    double u_x_star_tem = ((M_x * q_x_hat[frame - 1]) + T * (f[frame] + f_d[frame]))
    / (M_x + B_x * T);
    u_x_star.push_back(u_x_star_tem);

    double q_x_star_hat = proj_Q(u_x_star_tem)

    double lamda = f[frame] + f_d[frame] - M_x * (q_x_star_hat - q_x_hat[frame - 1])/ T - B_x * q_x_star_hat;

    double q_x_star_tem = q_x[frame - 1] + T * u_x_star[frame];
    q_x_star.push_back(q_x_star_tem);

    double phi_b_tem = (B * (q_x[frame - 1] - q_s[frame - 1])) / T - L * a[frame - 1];
    phi_b.push_back(phi_b_tem);

    double phi_a_tem = M * (q_s[frame] - q_x[ - 1] - T * u_x[frame - 1]) / (T * T);
    phi_a.push_back(phi_a_tem);

    double q_s_star_tem = q_s[frame] + (phi_b[frame] - phi_a[frame]) / (K_hat + M / (T * T));
    q_s_star.push_back(q_s_star_tem);

    double Mat = K_hat + M / (T * T);

    double tao_star_tem = Mat * (q_x_star - q_s_star - T * T * lamda / (M_x + T * B_x));
    
    tao_star.push_back(tao_star_tem);

    std::cout << "tao_star: " << tao_star_tem << endl;
    double tao_tem = proj(tao_star[frame]);
    tao.push_back(tao_tem);

    return tao[frame];
}

double controller::get_tau_normal(const double & T, double & f_input, double & q_input)
{ 
    f.push_back(0.);
    q.push_back(q_input);
    cout << "f[frame " << frame << "] = " << f[frame] << endl;
    cout << "q_s[frame " << frame << "] = " << q[frame] << endl;

    double omiga = 2. * M_PI / 10.;

    q0.push_back(0.2 * sin(omiga * frame * TimeUnit) + init_angle);
    q0_dot.push_back(0.2 * omiga * cos(omiga * frame * TimeUnit));
    q0_ddot.push_back(-0.2 * omiga * omiga * sin(omiga * frame * TimeUnit));

    // q0.push_back( -0.5 + init_angle);
    // q0_dot.push_back(0.);
    // q0_ddot.push_back(0.);

    double K_hat = K + B / T + L * T;

    double q_x_tem;

    if (frame == 0)
    {
        q_x_tem = (M_d_a / (T * T) * (2 * q_x[frame])
        + D_d_a / T * q_x[frame] + (M_d_a * q0_ddot[frame + 1] + D_d_a * q0_dot[frame + 1] + K_d_a * q0[frame + 1] + f[frame + 1]))
        / (M_d_a / (T * T) + D_d_a / T + K_d_a);
    }else{
        q_x_tem = (M_d_a / (T * T) * (2 * q_x[frame] - q_x[frame - 1])
        + D_d_a / T * q_x[frame] + (M_d_a * q0_ddot[frame + 1] + D_d_a * q0_dot[frame + 1] + K_d_a * q0[frame + 1] + f[frame + 1]))
        / (M_d_a / (T * T) + D_d_a / T + K_d_a);
    }
    q_x.push_back(q_x_tem);

    double u_x_tem;
    u_x_tem = (q_x[frame + 1] - q_x[frame]) / T;

    double a_tem = a[frame]  + T * (q_x[frame + 1] - q[frame + 1]);
    a.push_back(a_tem);

    double tau_tem;
    
    if (frame == 0)
    {
        tau_tem = M * (q_x[frame + 1] - 2 * q_x[frame]) / (T * T)
        + B * (T * u_x[frame + 1] - (q_x[frame + 1] - q_x[frame])) / T
        + K * (q_x[frame + 1] - q[frame + 1]) + L * a[frame + 1];
    }else 
    {
        tau_tem = M * (q_x[frame + 1] - 2 * q_x[frame] + q_x[frame -1]) / (T * T)
        + B * (T * u_x[frame + 1] - (q_x[frame + 1] - q_x[frame])) / T
        + K * (q_x[frame + 1] - q[frame + 1]) + L * a[frame + 1];
    }


    double tao_pro = proj(tau_tem);
    tao.push_back(tao_pro);

    return tao[frame + 1];
}

void controller::refresh(const double & T)
{
    double K_hat = K + B / T + L * T;
    double q_x_tem = q_s_star[frame] + (tao[frame + 1] / (K_hat + M / (T * T)));
    q_x.push_back(q_x_tem);

    double q_x_hat = (q_x[frame] - q_x[frame - 1]) / T;
    u_x.push_back(u_x_tem);
    
    double a_tem = a[frame - 1]  + T * (q_x[frame] - q_s[frame]);
    a.push_back(a_tem);

    f_d.push_back(0.1);

    frame++;
}

void controller::refresh_normal(const double & T)
{
    double K_hat = K + B / T + L * T;
    double q_x_tem = q_star[frame + 1] + (tao[frame + 1] / (K_hat + M / (T * T)));
    q_x.push_back(q_x_tem);

    double u_x_tem = (q_x[frame + 1] - q_x[frame]) / T;
    u_x.push_back(u_x_tem);
    
    double a_tem = a[frame]  + T * (q_x[frame + 1] - q[frame + 1]);
    a.push_back(a_tem);

    frame++;
}

double controller::proj(double tao_star_input)
{
    if (tao_star_input < -F_max)
    {
        return -F_max;
    }
    else if (tao_star_input > F_max)
    {
        return F_max;
    }
    else{
        return tao_star_input;
    }
}

double controller::proj_Q(double u_x_star)
{
    if (u_x_star < -F_max)
    {
        return -F_max;
    }
    else if (u_x_star > F_max)
    {
        return F_max;
    }
    else{
        return u_x_star;
    }
}


void controller::plot_tao()
{
    plt::figure_size(1200, 780);
    plt::named_plot("real_tao", tao);
    plt::named_plot("input_f", f);
    plt::xlabel("time (us)");
    plt::ylabel("torque (Nm)");
    plt::legend();
    plt::save("result_tao.pdf");
}

void controller::plot_tao_realtime(vector<double>& f)
{
    plt::figure(1);
    // Clear previous plot
	plt::clf();
    // plt::figure_size(1200, 780);
	// Plot line from given x and y data. Color is selected automatically.
	plt::named_plot("computed_tao", tao_star, "r--");
	// Plot a line whose name will show up as "log(x)" in the legend.
	plt::named_plot("real_tao", tao);
	// Set x-axis to interval [0,1000000]
	plt::named_plot("input_f", f);
	// Add graph title
	plt::title("torque");
    // plt::xlim(0, 100);
	// Enable legend.
	plt::legend();
	// Display plot continuously
	plt::pause(0.01);
}

void controller::plot_q()
{
    plt::figure_size(1200, 780);
    plt::named_plot("computed_qx", q_x,  "r--");
    plt::named_plot("input_q", q);
    plt::legend();
    plt::title("q");
    // plt::xlim(0, 100);
    plt::save("result_qs.pdf");
}

void controller::plot_q0()
{
    plt::figure_size(1200, 780);
    plt::named_plot("q0", q0,  "r--");
    plt::legend();
    plt::title("q0");
    // plt::xlim(0, 100);
    plt::save("result_q0.pdf");
}

void controller::plot_q_realtime(vector<double>& qs)
{
    // Clear previous plot
    plt::figure(2);
	plt::clf();
    plt::figure_size(1200, 780);
	// Plot line from given x and y data. Color is selected automatically.
	plt::named_plot("computed_qx", q_x_star , "r--");
	plt::named_plot("input_qs", qs);
	// Add graph title
	plt::title("q");
    // plt::xlim(0, 100);
	// Enable legend.
	plt::legend();
	// Display plot continuously
	plt::pause(0.01);
}


controller::~controller()
{

}
