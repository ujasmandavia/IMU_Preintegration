#include "IMU_Data.h"

//Initialize those static member variables
//Set the IMU parameters, cotinious parameters(defaul set for Euroc dataset)
double IMUData::sigma_gyroscope_ = 1.6968e-4;
double IMUData::sigma_accelerometer_ = 2.0e-3;

double IMUData::sigma_gyroscope_walk_ = 1.9393E-5;
double IMUData::sigma_accelerometer_walk_ = 3.0e-3;

double IMUData::delta_timestamp_ = 0.005;

//Calculate discrete IMU sensor parameters
double IMUData::gyroscope_noise_rw2_ = IMUData::sigma_gyroscope_ * IMUData::sigma_gyroscope_/IMUData::delta_timestamp_;
double IMUData::accelerometer_noise_rw2_ = IMUData::sigma_accelerometer_ *  IMUData::sigma_accelerometer_/IMUData::delta_timestamp_;

Eigen::Matrix3d IMUData::gyroscope_meas_covariance_ = Eigen::Matrix3d::Identity() *  IMUData::gyroscope_noise_rw2_;
Eigen::Matrix3d IMUData::accelerometer_meas_covariance_ =  Eigen::Matrix3d::Identity() * IMUData::accelerometer_noise_rw2_;

double IMUData::gyroscope_bias_rw2_ = IMUData::sigma_gyroscope_walk_ * IMUData::sigma_gyroscope_walk_ * IMUData::delta_timestamp_;
double IMUData::accelerometer_bias_rw2_ = IMUData::sigma_accelerometer_walk_ * IMUData::sigma_accelerometer_walk_ * IMUData::delta_timestamp_;

Eigen::Matrix3d IMUData::gyroscope_bias_rw_covariance_ = Eigen::Matrix3d::Identity() * IMUData::gyroscope_bias_rw2_;
Eigen::Matrix3d IMUData::accelerometer_bias_rw_covariance_ = Eigen::Matrix3d::Identity() * IMUData::accelerometer_bias_rw2_;

IMUData::IMUData(const double &gx, const double &gy, const double &gz,
                 const double &ax, const double &ay, const double &az,
                const double &t) : gyroscope_(gx,gy,gz), accelerometer_(ax,ay,az), timestamp_(t) {}
