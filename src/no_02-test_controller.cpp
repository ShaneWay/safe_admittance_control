#include <controller_2.h>
#include <vector>
#include <iostream>
#include <time.h>
#include <math.h>
#include <Eigen/Dense>

using namespace std;



int64_t GetTickUs()
{
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
}

float TIME_DURATION = 10.0f;

int main()
{

    Eigen::Vector2d f;
    f << 0.0, 0.0;
    cout << "input f :" << f.transpose() << endl;
    Eigen::Vector2d q;
    q << 1.0,1.0;
    cout << "input qs : " << q.transpose() << endl;

    int64_t now = 0;
    int64_t last = 0;

    int timer_count = 0;
    double T = 0.001;

    controller_2 cal_test;

    Eigen::VectorXd tau(7);
    cout << "this is a test"  << endl;
    while (timer_count < (TIME_DURATION * 10))
    {
        now = GetTickUs();
        if (now - last > 100000)
        {
            double f_in = 0;
            if(timer_count < 50)
            {
                f_in = sin(timer_count / M_PI * 180.);
            };
            tau = cal_test.getTorqueDI(T, f, q);
            cout << "tau[" << timer_count +1 << "]: " << tau.transpose() << endl;

            cout << "***************************" << endl;

            cal_test.refresh(T);
            timer_count++;
            last = GetTickUs();
        }
        
    }
    cal_test.plotJointTorque();
    cal_test.plotCartesianPosition();
    // cal_test.plot_tau_realtime(f);


}