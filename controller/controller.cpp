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
    q_x_hat.push_back(0);

    phi_b.push_back(0);
    phi_a.push_back(0);

    q_s_star.push_back(0);
    q_s.push_back(init_angle);
    q_s_hat.push_back(0);

    a.push_back(0);

    f.push_back(0);
    f_d.push_back(f_d_tem);

    frame = 1;
    for(int i=0; i< 5001; i++)
    {
        if(i >1000 && i < 1500)
        {
            f_d.push_back(10);
        }
        else{
            f_d.push_back(f_d_tem);
        }
    }
}

double controller::get_tau(const double & T, double & f_ext_from_sensor, double & q_frome_sensor)
{ 

    f.push_back(f_ext_from_sensor);
    q_s.push_back(q_frome_sensor);
    q_s_hat.push_back((q_s[frame] - q_s[frame - 1])/T);
    cout << "f[frame " << frame << "] = " << f[frame] << endl;
    cout << "q[frame " << frame << "] = " << q_s[frame] << endl;

    // f_d.push_back(f_d_tem);

    double K_hat = K + B / T + L * T;

    double u_x_star_tem = ((M_x * q_x_hat[frame - 1]) + T * (f[frame] + f_d[frame - 1]))
    / (M_x + B_x * T);
    u_x_star.push_back(u_x_star_tem);
    // std::cout << "f_d: " << f_d[frame] << std::endl;

    double q_x_star_hat = proj_Q(u_x_star_tem);
    std::cout << "q_x_star_hat: " << q_x_star_hat << std::endl;

    double lamda = f[frame] + f_d[frame] - M_x * (q_x_star_hat - q_x_hat[frame - 1])/ T - B_x * q_x_star_hat;
    // std::cout << "lamda: " << lamda << std::endl;

    double q_x_star_tem = q_x[frame - 1] + T * u_x_star[frame];
    // std::cout << "q_x_star: " << q_x_star_tem << std::endl;
    q_x_star.push_back(q_x_star_tem);

    double phi_b_tem = (B * (q_x[frame - 1] - q_s[frame - 1])) / T - L * a[frame - 1];
    phi_b.push_back(phi_b_tem);
    // std::cout << "phi_b_tem: " << phi_b_tem << std::endl;


    double phi_a_tem = M * (q_s[frame] - q_x[frame - 1] - T * q_x_hat[frame - 1]) / (T * T);
    phi_a.push_back(phi_a_tem);
    // std::cout << "phi_a_tem: " << phi_a_tem << std::endl;


    double q_s_star_tem = q_s[frame] + (phi_b[frame] - phi_a[frame]) / (K_hat + M / (T * T));
    q_s_star.push_back(q_s_star_tem);
    // std::cout << "q_s_star: " << q_s_star_tem << std::endl;


    double Mat = K_hat + M / (T * T);

    double tao_star_tem = Mat * (q_x_star[frame] - q_s_star[frame]) - T * T * lamda / (M_x + T * B_x);
    
    tao_star.push_back(tao_star_tem);

    std::cout << "tao_star: " << tao_star_tem << endl;
    double tao_tem = proj(tao_star[frame]);
    tao.push_back(tao_tem);

    return tao[frame];
}



void controller::refresh(const double & T)
{
    double K_hat = K + B / T + L * T;
    double q_x_tem = q_s_star[frame] + tao[frame] / (K_hat + M / (T * T));
    q_x.push_back(q_x_tem);

    double q_x_hat_tem = (q_x[frame] - q_x[frame - 1]) / T;
    q_x_hat.push_back(q_x_hat_tem);
    std::cout << "q_x_hat_tem: " << q_x_hat_tem << std::endl;
    
    double a_tem = a[frame - 1]  + T * (q_x[frame] - q_s[frame]);
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
    if (u_x_star < -Q_max)
    {
        return -Q_max;
    }
    else if (u_x_star > Q_max)
    {
        return Q_max;
    }
    else{
        return u_x_star;
    }
}


void controller::plot_tao()
{
    plt::figure_size(1200, 780);
    plt::named_plot("real_tao", tao_star);
    plt::named_plot("projected_tao", tao);
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
    plt::named_plot("q_s", q_s);
    plt::legend();
    plt::title("q");
    // plt::xlim(0, 100);
    plt::save("result_qs.pdf");
}

void controller::plot_q_hat()
{
    plt::figure_size(1200, 780);
    plt::named_plot("q_x_hat", q_x_hat,  "r--");
    // plt::named_plot("q_s_hat", q_s_hat);
    plt::legend();
    plt::title("q_hat");
    // plt::xlim(0, 100);
    plt::save("q_hat.pdf");
}


void controller::plot_q_realtime(vector<double>& qs)
{
    // Clear previous plot
    plt::figure(2);
	plt::clf();
    plt::figure_size(1200, 780);
	// Plot line from given x and y data. Color is selected automatically.
	plt::named_plot("computed_qx", q_x_star , "r--");
	plt::named_plot("input_qs", q_s);
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
