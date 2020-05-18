#ifndef IMU_PREINTEGRATION_IMUDATA_H
#define IMU_PREINTEGRATION_IMUDATA_H

#include <Eigen/Core>

/**
* IMU model https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
* the web IMU Parameter[sigma_g, sigma_gw, sigma_a, sigma_aw]
* For EuRoc dataset, according to V1_01_easy/imu0/sensor.yaml
* The params:
* sigma_g: 1.6968e-4       rad / s / sqrt(Hz)    n_g(gyroscope_noise_density)           // continuous white noise for gyroscope
* sigma_gw: 1.9393e-5      rad / s^2 / sqrt(Hz)  n_bg(gyroscope_random_walk)            // continuous random walk for gyroscope
* sigma_a: 2.0e-3          m / s^2 / sqrt(Hz)    na(accelerometer_noise_density)        // continuous white noise for accelerometer
* sigma_aw: 3.0e-3         m / s^3 / sqrt(Hz)    n_ba(accelerometer_random_walk)        // continuous random walk for accelerometer
*/

//Save IMU sensor parameters and raw data
class IMUData{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  IMUData() = default;

  IMUData(const double &gx, const double &gy, const double &gz,
          const double &ax, const double &ay, const double &az,
          const double &t);

  //IMU raw data
  Eigen::Vector3d gyroscope_;
  Eigen::Vector3d accelerometer_;

  //Data timestamp
  double timestamp_;

  //IMU sensor parameters, no need to set multiple times
  //IMU sensor continuous Gaussian white noise
  static double sigma_gyroscope_;
  static double sigma_accelerometer_;

  //IMU sensor continous random walk
  static double sigma_gyroscope_walk_;
  static double sigma_accelerometer_walk_;

  //IMU sensor data collection interval
  static double delta_timestamp_;

  //For the correspondences between discrete and continuous parameters, see the introduction of IMU model
  //Square of discrete random walk of IMU sensor
  static double gyroscope_noise_rw2_;
  static double accelerometer_noise_rw2_;

  //Square of discrete noise of IMU data
  static double gyroscope_bias_rw2_;
  static double accelerometer_bias_rw2_;

  //Noise matrix
  static Eigen::Matrix3d gyroscope_meas_covariance_;
  static Eigen::Matrix3d accelerometer_meas_covariance_;

  //random walk matrix
  static Eigen::Matrix3d gyroscope_bias_rw_covariance_;
  static Eigen::Matrix3d accelerometer_bias_rw_covariance_;

}; //class IMUData

#endif  //IMU_PREINTEGRATION_IMUDATA_H
