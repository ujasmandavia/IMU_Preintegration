#ifndef IMU_PREINTEGRATION_IMUPREINTEGRATION_H
#define IMU_PREINTEGRATION_IMUPREINTEGRATION_H

#include "so3.h"
#include "IMU_Data.h"

typedef Eigen::Matrix<double,9,9> Matrix9d;

class IMUPreintegrator{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //Default constructor
  IMUPreintegrator();

  //Constructor
  IMUPreintegrator(const IMUPreintegrator &preintegrator);

  /*
  @brief   Incremental update of IMU pre-integrated values
  @param   omega   GYroscope data after subtracting bias
  @param acc accelerometer data after subtracting bias
  @param delta_time IMU data cycle
  @return
  */
  void Update(const Eigen::Vector3d &omega, const Eigen::Vector3d &acc, const double &delta_timestamp_);

  /*
  Reset all state quantities for the next pre-integration
  */
  void reset();

  /*
  Obtain IMU pre-integrated positian value
  */
  inline Eigen::Vector3d GetDeltaP() const {
    return delta_p_;
  }

  /*
  Obtain IMU pre-intgrated speed values
  */
  inline Eigen::Vector3d GetDeltaV() const{
    return delta_v_;
  }

  /*
  OBtain IMU preintegrated rotation values
  */
  inline Eigen::Matrix3d GetDeltaRot() const{
    return delta_rot_;
  }

  /*
  Obtain the uncertainity of noise error of IMU pre-integrated values
  */
  inline Matrix9d GetCovPVRot() const {
    return cov_p_v_rot_;
  }

  /*
  Jacobian to obtain imu pre-integrated position measurement value relative to gyroscope error
  */
  inline Eigen::Matrix3d GetPJacoBiasgur() const {
    return jacobian_p_bias_gyroscope_;
  }

  /*
  Jacobian to obtain imu pre-integrated position measurement value relative to accelerometer bias
  */
  inline Eigen::Matrix3d GetPJacoBiasacc() const {
    return jacobian_p_bias_accelerometer_;
  }

  /*
  Jacobian to obtain IMY pre-integradted velocity measurement value relative to gyroscope bias
  */
  inline Eigen::Matrix3d GetVJacobBiasgyro() const {
    return jacobian_v_bias_gyroscope_;
  }

  /*
  Jacobian to obtain IMU pre-integrated velocity measurement value relative to accelerometer bias
  */
  inline Eigen::Matrix3d GetVJacobBiasacc() const {
    return jacobian_v_bias_accelerometer_;
  }

  /*
  Jacobian to obtain IMU pre-integrayted attitude measurement value relative to gyroscope measurement
  */
  inline Eigen::Matrix3d GetRotJacobBiasgyr() const{
    return jacobian_rot_bias_gyroscope_;
  }

  /*
  @brief Normalized quaternion
  @param q Enter quaternion
  @return Normalized quaternion
  */
  inline Eigen::Quaterniond NormalizeRotationQ(const Eigen::Quaterniond &q){
    Eigen::Quaterniond quat(q);
    //Two quaternion that are opposite to each other indicate the same rotation, unified direction
    if(quat.w() < 0)
      quat.coeffs() *= -1;

    return quat.normalized();
  }

  /*
  @brief Normalize rotation matrix
  @param R rotation matrix
  @return normalized rotation matrix
  */
  inline Eigen::Matrix3d NormalizedRotationM(const Eigen::Matrix3d &R){
    Eigen::Quaterniond q(R);

    return NormalizeRotationQ(q).toRotationMatrix();
  }

private:
  //Pre-integrated measurements of IMU position, velocity and rotation between key frames
  Eigen::Vector3d delta_p_;
  Eigen::Vector3d delta_v_;
  Eigen::Matrix3d delta_rot_;

  Eigen::Matrix3d jacobian_p_bias_gyroscope_;
  Eigen::Matrix3d jacobian_p_bias_accelerometer_;
  Eigen::Matrix3d jacobian_v_bias_gyroscope_;
  Eigen::Matrix3d jacobian_v_bias_accelerometer_;
  Eigen::Matrix3d jacobian_rot_bias_gyroscope_;

  //Unvertainity variance of noise error of IMU pre-integration observation equation
  Matrix9d cov_p_v_rot_;

  //Duration of pre-integrated IMU data
  double delta_time_;

};  //class IMUPreintegrator

#endif
