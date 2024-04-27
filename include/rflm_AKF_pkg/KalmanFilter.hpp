#ifndef KALMANFILTER_HPP
#define KALMANFILTER_HPP

#include <iostream>
#include <chrono>
#include <vector>
#include <eigen3/Eigen/Dense>


class KalmanFilter {
    public:
        KalmanFilter(int state_count, std::vector<double> x_init, std::vector<double> Q, std::vector<double> R, int camera_role, std::time_t current_time);
        std::vector<double> update(double u, std::vector<double> measurements, std::vector<std::string> measurement_types, std::time_t current_time);   // measurement vector should consisted of data in this order: [imu, cam, ur5e_pos, ur5e_vel]; put 0 if measurement is not available; fill measuremen_type vector with available sensor data at this time step
        void set_A(std::vector<double> A);
        void set_B(std::vector<double> B);
        void set_H(std::vector<double> H, std::string H_name);
        void set_R(std::vector<double> R, std::string R_name);

        const std::string IMU_DATA = "imu_data";
        const std::string CAM_DATA = "cam_data";
        const std::string UR5E_DATA = "ur5e_data";

    private:
        int state_count_;       // number of state variable of the system

        Eigen::VectorXd x_p_;   // predicted state of the system in AKF
        Eigen::VectorXd x_k_;   // estimated state of the system in AKF
        Eigen::Matrix<double, 1, Eigen::Dynamic> H_imu_;            // H matrix for only imu measurements
        Eigen::Matrix<double, 1, Eigen::Dynamic> H_cam_;            // H matrix for only camera measurements
        Eigen::Matrix<double, 2, Eigen::Dynamic> H_ur5e_;           // H matrix for only UR5e arm measurements
        Eigen::Matrix<double, 2, Eigen::Dynamic> H_imu_cam_;        // H matrix for imu and camera measurements
        Eigen::Matrix<double, 3, Eigen::Dynamic> H_imu_ur5e_;       // H matrix for imu and UR5e arm measurements
        Eigen::Matrix<double, 3, Eigen::Dynamic> H_cam_ur5e_;       // H matrix for camera and UR5e measurements
        Eigen::Matrix<double, 4, Eigen::Dynamic> H_all_sensors_;    // H matrix for all sensrors' measurements

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A_;   // System matrix, A
        Eigen::Matrix<double, Eigen::Dynamic, 1> B_;                // System matrix, B
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A_kf_;
        Eigen::Matrix<double, Eigen::Dynamic, 1> B_kf_;

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Pp_;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P_k_;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Q_;
        Eigen::Matrix<double, Eigen::Dynamic, 1> R_values_;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Kk_;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> eye_;

        double R_imu_, R_cam_, R_ur5e_pos_, R_ur5e_vel_;
        double latest_imu_data_, latest_cam_data_, latest_ur5e_pos_data_, latest_ur5e_vel_data_;

        double prev_time_;  // to store previous computation time
        double dt_;         // to obtain and store the deltaT between runs
        int camera_role_;   // 0 for not use of camera; 1 for use camera within AKF; 2 for using camera for correction

        const std::string HIMU_NAME = "H_imu";
        const std::string HCAM_NAME = "H_cam";
        const std::string HUR5E_NAME = "H_ur5e";

        bool has_new_imu_data_ = false, has_new_cam_data_ = false, has_new_ur5e_data_ = false;

        void predict(double u);
        void estimate(std::vector<double> measurements);
        void do_correction();
};

#endif