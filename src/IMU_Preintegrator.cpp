#include <IMU_Preintegrator.h>
#include <fstream>

using namespace std;

IMUPreintegrator::IMUPreintegrator(){
  delta_p_.setZero();
  delta_v_.setZero();
  delta_rot_.setIdentity();

  jacobian_v_bias_gyroscope_.setZero();
  jacobian_v_bias_accelerometer_.setZero();

  jacobian_p_bias_gyroscope_.setZero();
  jacobian_p_bias_accelerometer_.setZero();

  jacobian_rot_bias_gyroscope_.setZero();

  cov_p_v_rot_.setZero();

  delta_time_ = 0.0;
}

IMUPreintegrator::IMUPreintegrator(const IMUPreintegrator &preintegrator) :
        delta_p_(preintegrator.delta_p_), delta_v_(preintegrator.delta_v_), delta_rot_(preintegrator.delta_rot_),
        jacobian_p_bias_gyroscope_(),
        jacobian_p_bias_accelerometer_(),
        jacobian_v_bias_gyroscope_(),
        jacobian_v_bias_accelerometer_(),
        jacobian_rot_bias_gyroscope_(),
        cov_p_v_rot_(), delta_time_(preintegrator.delta_time_) {}

void IMUPreintegrator::Update(const Eigen::Vector3d &omega, const Eigen::Vector3d &acc,const double &delta_timestamp_){
  double dt2 = delta_timestamp_*delta_timestamp_;

  //Step-1: Calculate IMU pre-integral rotation increment and Jacobian
  Eigen::Matrix3d dR = Sophus::SO3::exp(omega*delta_timestamp_).matrix();
  Eigen::Matrix3d Jr_R = Sophus::SO3::JacobianR(omega*delta_timestamp_);

  //Step 2: Calculate the uncertainity variance of the noise error of the IMU pre-integrator observation equation
  Eigen::Matrix3d I3x3;
  I3x3.setIdentity();

  Matrix9d A;
  A.setIdentity();
  //  The block size is 3x3, the starting position (6,6)
  A.block<3,3>(6,6) = dR.transpose();
  A.block<3,3>(3,6) = -delta_rot_ * Sophus::SO3::hat(acc) * delta_timestamp_;
  A.block<3,3>(0,6) = -0.5* delta_rot_ *Sophus::SO3::hat(acc) * dt2;
  A.block<3,3>(0,3) = I3x3 *delta_timestamp_;

  //Split the B in the paper into two parts
  Eigen::Matrix<double,9,3> Bg;
  Bg.setZero();
  Bg.block<3,3>(6,0) = Jr_R * delta_timestamp_;

  Eigen::Matrix<double,9,3> Ca;
  Ca.setZero();
  Ca.block<3,3>(3,0) = delta_rot_ * delta_timestamp_;
  Ca.block<3,3>(0,0) = 0.5 * delta_rot_ * dt2;

  cov_p_v_rot_ = A * cov_p_v_rot_ * A.transpose() +
                 Bg * IMUData::gyroscope_meas_covariance_ * Bg.transpose() +
                 Ca * IMUData::accelerometer_meas_covariance_ * Ca.transpose();

  //Step-3: Calculate Jacobian, used to correct the biased IMU pre-integrated measurement value

  //Recursion based on supplementary material formula
  jacobian_p_bias_accelerometer_ += jacobian_v_bias_accelerometer_ * delta_timestamp_ -0.5 * delta_rot_ * dt2;
  jacobian_p_bias_gyroscope_ += jacobian_v_bias_gyroscope_ * delta_timestamp_ - 0.5 * delta_rot_ * Sophus::SO3::hat(acc) * jacobian_rot_bias_gyroscope_ * dt2;

  jacobian_v_bias_accelerometer_ += -delta_rot_ * delta_timestamp_;
  jacobian_v_bias_gyroscope_ += -delta_rot_ * Sophus::SO3::hat(acc) * jacobian_rot_bias_gyroscope_ * delta_timestamp_;

  jacobian_rot_bias_gyroscope_ = dR.transpose() * jacobian_rot_bias_gyroscope_ - Jr_R * delta_timestamp_;

  //Step-4: Calculate IMU pre-integrated measurement value
  delta_p_ += delta_v_*delta_timestamp_ + 0.5 * delta_rot_ * acc * dt2;
  delta_v_ += delta_rot_ * acc * delta_timestamp_;
  delta_rot_ += NormalizedRotationM(delta_rot_*dR);
  delta_time_ += delta_timestamp_;
}

void IMUPreintegrator::reset(){
  delta_p_.setZero();
  delta_v_.setZero();
  delta_rot_.setIdentity();

  jacobian_v_bias_gyroscope_.setZero();
  jacobian_v_bias_accelerometer_.setZero();

  jacobian_p_bias_gyroscope_.setZero();
  jacobian_p_bias_accelerometer_.setZero();

  jacobian_rot_bias_gyroscope_.setZero();

  cov_p_v_rot_.setZero();

  delta_time_ = 0.0;
}
