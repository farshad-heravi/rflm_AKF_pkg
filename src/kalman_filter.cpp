#include <iostream>
#include <KalmanFilter.hpp>
#include <chrono>
#include <vector>
#include <math.h>

int main()
{
    int state_count = 4;    //  system state: [y, yd, w, wd]
    std::vector<double> x_init = {0, 0, 0, 0};
    std::vector<double> Q = {   pow(10, -20), 0, 0, 0,                                      //  imu, cam
                                0, pow(10, -20), 0, 0,                                      // cam
                                0, 0, pow(10, -20), 0,                                      // ur5e_pos
                                0, 0, 0, pow(10, -20) };                                    // ur5e_vel
    std::vector<double> R = {pow(10, -20), pow(10, -20), pow(10, -20), pow(10, -20)};       //  imu, cam, ur5e_pos, ur5e_vel
    // Hs ?
    std::time_t current_time = std::chrono::system_clock::to_time_t( std::chrono::system_clock::now() );

    KalmanFilter kf = KalmanFilter(state_count, x_init, Q, R, 2, current_time);

    // new measurements
    std::vector<double> measurements = {2.0, .001, 0, 0};
    std::vector<std::string> measurement_types = {kf.IMU_DATA, kf.CAM_DATA};
    current_time = std::chrono::system_clock::to_time_t( std::chrono::system_clock::now() );
    double u = 0.1;
    std::vector<double> new_state = kf.update(u, measurements, measurement_types, current_time);
    std::cout << "new state: [ " << new_state[0] << ", " << new_state[1] << ", " << new_state[2] << ", " << new_state[3] << " ]" << std::endl;

    return 0;
}