#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter(int state_count, std::vector<double> x_init, std::vector<double> Q, std::vector<double> R, int camera_role, std::time_t current_time) : state_count_(state_count),
    prev_time_(current_time),
    camera_role_(camera_role)
{
    // initiate x_k_
    x_k_.resize(state_count, 1);
    Eigen::Map<Eigen::VectorXd>(x_k_.data(), state_count, 1) = Eigen::Map<Eigen::VectorXd>(x_init.data(), state_count, 1);

    // resize the matrices accoring to the state_count_
    A_.resize(state_count_, state_count_);
    B_.resize(state_count_, 1);
    A_kf_.resize(state_count_, state_count_);
    B_kf_.resize(state_count_, 1);
    
    H_imu_.resize(1, state_count_);
    H_cam_.resize(1, state_count_);
    H_ur5e_.resize(2, state_count_);
    H_imu_cam_.resize(2, state_count_);
    H_imu_ur5e_.resize(3, state_count_);
    H_cam_ur5e_.resize(3, state_count_);
    H_all_sensors_.resize(4, state_count_);

    Pp_.resize(state_count_, state_count_);
    P_k_.resize(state_count_, state_count_);
    Q_.resize(state_count_, state_count_);
    Kk_.resize(state_count_, state_count_);
    eye_.resize(state_count_, state_count_);
    R_values_.resize(state_count_, 1);

    // assigne the Q and R matrices
    for (size_t i = 0; i < state_count_; i++)
        Q_(i, i) = Q[i + i*state_count_];
    for (size_t i = 0; i < state_count_; i++)
        R_values_(i) = R[i];

}

std::vector<double> KalmanFilter::update(double u, std::vector<double> measurements, std::vector<std::string> measurement_types, std::time_t current_time)
{
    /*
        args: 
            u: (double) system input (e.g. force)
            measurements: (vector<double>) should be in this order: [imu, cam, ur5e_pos, ur5e_vel]; put 0 if measurement is not available;
            measurement_types: (vector<string>) fill measuremen_type vector with available sensor data at this time step
            curren_time: (double) time in seconds
        output:
            x_k_: (vector<double>)  latest estimated system state in this time step
    */

    dt_ =  static_cast<double>( current_time - prev_time_ );    // calculate dt_
    // check measurement types
    for(size_t i; i<measurement_types.size(); i++) {
        if (measurement_types[i] == IMU_DATA) {
            has_new_imu_data_ = true;
            latest_imu_data_ = measurements[0];
        } else if (measurement_types[i] == CAM_DATA) {
            has_new_cam_data_ = true;
            latest_cam_data_ = measurements[1];
        }  else if (measurement_types[i] == UR5E_DATA) {
            has_new_ur5e_data_ = true;
            latest_ur5e_pos_data_ = measurements[2];
            latest_ur5e_vel_data_ = measurements[3];
        }
    }

    predict(u);
    estimate(measurements);

    if( camera_role_ == 2)            // use camera for correction
        do_correction();

    prev_time_ = current_time;          // update prev_time
    
    std::vector<double> x_k(x_k_.data(), x_k_.data() + x_k_.size());    // convert x_k_ from Eigen::vector to std::vector
    return x_k;
}

void KalmanFilter::set_A(const std::vector<double> A)
{   
    Eigen::Map<Eigen::MatrixXd>(A_.data(), state_count_, state_count_) = Eigen::Map<const Eigen::MatrixXd>(A.data(), state_count_, state_count_);
}

void KalmanFilter::set_B(const std::vector<double> B)
{
    Eigen::Map<Eigen::MatrixXd>(B_.data(), state_count_, state_count_) = Eigen::Map<const Eigen::MatrixXd>(B.data(), state_count_, state_count_);
}

void KalmanFilter::set_H(const std::vector<double> H, const std::string H_name)
{
    if( H_name == HIMU_NAME ) {
        Eigen::Map<Eigen::MatrixXd>(H_imu_.data(), 1, state_count_) = Eigen::Map<const Eigen::MatrixXd>(H.data(), 1, state_count_);
    } else if ( H_name == HCAM_NAME ) {
        Eigen::Map<Eigen::MatrixXd>(H_cam_.data(), 1, state_count_) = Eigen::Map<const Eigen::MatrixXd>(H.data(), 1, state_count_);
    } else if ( H_name == HUR5E_NAME ) {
        Eigen::Map<Eigen::MatrixXd>(H_ur5e_.data(), 2, state_count_) = Eigen::Map<const Eigen::MatrixXd>(H.data(), 2, state_count_);
    }

    // update the dependent matrices
    H_imu_cam_ << H_imu_, H_cam_;
    H_imu_ur5e_ << H_imu_, H_ur5e_;
    H_cam_ur5e_ << H_cam_, H_ur5e_;
    H_all_sensors_ << H_imu_, H_cam_, H_ur5e_;
}

void KalmanFilter::set_R(const std::vector<double> R, const std::string R_name)
{
    if( R_name == HIMU_NAME )
        R_imu_ = R[0];
    else if ( R_name == HCAM_NAME )
        R_cam_ = R[0];
    else if ( R_name == HUR5E_NAME ) {
        R_ur5e_pos_ = R[0];
        R_ur5e_vel_ = R[1];
    }
}

void KalmanFilter::predict(double u)
{
    A_kf_ = eye_ + dt_ * A_;
    B_kf_ = dt_ * B_;

    x_p_ = A_kf_ * x_k_ + B_kf_ * u;                // predict xp
    Pp_ = A_kf_ * P_k_ * A_kf_.transpose() + Q_;    // predict Pp
}

void KalmanFilter::estimate(std::vector<double> measurements)
{
    Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> H, R, measures;

    if( has_new_imu_data_ && has_new_cam_data_ && has_new_ur5e_data_ && camera_role_ == 1 ) {    // IMU && Cam && UR5e measurements
        int temp_size = 4;
        measures.resize(temp_size,1);
        H.resize(temp_size, state_count_);
        R.resize(temp_size, state_count_);
        measures << latest_imu_data_, latest_cam_data_, latest_ur5e_pos_data_, latest_ur5e_vel_data_;
        H = H_all_sensors_;
        R << R_imu_, R_cam_, R_ur5e_pos_, R_ur5e_vel_;
        has_new_imu_data_ = !has_new_imu_data_;
        has_new_cam_data_ = !has_new_cam_data_;
        has_new_ur5e_data_ = !has_new_ur5e_data_;
    } else if( has_new_cam_data_ && has_new_ur5e_data_ && camera_role_ == 1 ) {                  // Cam && UR5e measurements
        int temp_size = 3;
        measures.resize(temp_size,1);
        H.resize(temp_size, state_count_);
        R.resize(temp_size, state_count_);
        measures << latest_cam_data_, latest_ur5e_pos_data_, latest_ur5e_vel_data_;
        H = H_cam_ur5e_;
        R << R_cam_, R_ur5e_pos_, R_ur5e_vel_;
        has_new_cam_data_ = !has_new_cam_data_;
        has_new_ur5e_data_ = !has_new_ur5e_data_;
    } else if( has_new_imu_data_ && has_new_ur5e_data_ ) {                  // IMU && UR5e measurements
        int temp_size = 3;
        measures.resize(temp_size,1);
        H.resize(temp_size, state_count_);
        R.resize(temp_size, state_count_);
        measures << latest_imu_data_, latest_ur5e_pos_data_, latest_ur5e_vel_data_;
        H = H_imu_ur5e_;
        R << R_imu_, R_ur5e_pos_, R_ur5e_vel_;
        has_new_imu_data_ = !has_new_imu_data_;
        has_new_ur5e_data_ = !has_new_ur5e_data_;
    } else if( has_new_cam_data_ && has_new_imu_data_ && camera_role_ == 1 ) {                   // Cam && IMU measurements
        int temp_size = 2;
        measures.resize(temp_size,1);
        H.resize(temp_size, state_count_);
        R.resize(temp_size, state_count_);
        measures << latest_imu_data_, latest_cam_data_;
        H = H_imu_cam_;
        R << R_imu_, R_cam_;
        has_new_imu_data_ = !has_new_imu_data_;
        has_new_cam_data_ = !has_new_cam_data_;
    } else if( has_new_cam_data_ && camera_role_ == 1 ) {                   // Cam measurements
        int temp_size = 1;
        measures.resize(temp_size,1);
        H.resize(temp_size, state_count_);
        R.resize(temp_size, state_count_);
        measures << latest_cam_data_;
        H = H_cam_;
        R << R_cam_;
        has_new_cam_data_ = !has_new_cam_data_;
    } else if( has_new_ur5e_data_ ) {                   // UR5e measurements
        int temp_size = 2;
        measures.resize(temp_size,1);
        H.resize(temp_size, state_count_);
        R.resize(temp_size, state_count_);
        measures << latest_ur5e_pos_data_, latest_ur5e_vel_data_;
        H = H_ur5e_;
        R << R_ur5e_pos_, R_ur5e_vel_;
        has_new_ur5e_data_ = !has_new_ur5e_data_;
    } else if( has_new_imu_data_ ) {                   // IMU measurements
        int temp_size = 1;
        measures.resize(temp_size,1);
        H.resize(temp_size, state_count_);
        R.resize(temp_size, state_count_);
        measures << latest_imu_data_;
        H = H_imu_;
        R << R_imu_;
        has_new_imu_data_ = !has_new_imu_data_;
    } else {                                           // if no measurements are available
        x_k_ = x_p_;
        P_k_ = Pp_;
        return;
    }

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> temp(H.rows(), H.rows());
    temp = H * P_k_ * H.transpose() + R;
    temp = temp.inverse();
    Kk_ = P_k_ * H.transpose() * temp;
    x_k_ = x_k_ + Kk_ * (measures - H * x_p_);                   // update x_k_
    temp = eye_ - Kk_ * H;
    P_k_ = temp * Pp_;                                          // update P_k_
}

void KalmanFilter::do_correction()
{
    if (has_new_cam_data_){
        x_k_(1) = latest_cam_data_;
        has_new_cam_data_ = !has_new_cam_data_;
    }
}
