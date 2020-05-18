// This file is part of Sophus.
//
// Copyright 2011 Hauke Strasdat (Imperial College London)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights  to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.


#include <iostream>
#include <so3.h>

namespace Sophus
{

    // se3 => SE3指数映射位移的线性变换Jacobian
    // Right Jacobian for Log map in SO(3) - equation (10.86) in
    // G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups", Volume 2, 2008.
    Matrix3d SO3::JacobianR(const Vector3d &w)
    {
        Matrix3d Jr = Matrix3d::Identity();
        double theta = w.norm();        // 旋转角
        if (theta < 0.00001)            // 旋转角很小
        {
            return Jr;                // 返回单位阵
        }
        else
        {
            Vector3d k = w.normalized();    // 旋转向量的单位向量.
            Matrix3d K = SO3::hat(k);        // 反对称

            Jr = Matrix3d::Identity() - (1 - cos(theta)) / theta * K + (1 - sin(theta) / theta) * K * K;
        }

        return Jr;
    }


    Matrix3d SO3::JacobianRInv(const Vector3d &w)
    {

        Matrix3d Jrinv = Matrix3d::Identity();
        double theta = w.norm();
        if (theta < 0.00001)
        {
            return Jrinv;
        }

        else
        {
            Vector3d k = w.normalized();
            Matrix3d K = SO3::hat(k);
            Jrinv = Matrix3d::Identity() + 0.5 * SO3::hat(w) +
                    (1.0 - (1.0 + cos(theta)) * theta / (2.0 * sin(theta))) * K * K;
        }

        return Jrinv;
    }


    Matrix3d SO3::JacobianL(const Vector3d &w)
    {
        return JacobianR(-w);
    }


    Matrix3d SO3::JacobianLInv(const Vector3d &w)
    {
        return JacobianRInv(-w);
    }


    SO3::SO3()
    {
        unit_quaternion_.setIdentity();
    }


    SO3::SO3(const SO3 &other) : unit_quaternion_(other.unit_quaternion_)
    {
        unit_quaternion_.normalize();
    }


    SO3::SO3(const Matrix3d &R) : unit_quaternion_(R)
    {
        unit_quaternion_.normalize();
    }


    SO3::SO3(const Quaterniond &quat) : unit_quaternion_(quat)
    {
        assert(unit_quaternion_.squaredNorm() > SMALL_EPS);
        unit_quaternion_.normalize();
    }


    // 绕三轴的旋转
    SO3::SO3(double rot_x, double rot_y, double rot_z)
    {
        unit_quaternion_ = (SO3::exp(Vector3d(rot_x, 0.f, 0.f))
                            * SO3::exp(Vector3d(0.f, rot_y, 0.f))
                            * SO3::exp(Vector3d(0.f, 0.f, rot_z))).unit_quaternion_;
    }


    // 符号重载
    void SO3::operator=(const SO3 &other)
    {
        this->unit_quaternion_ = other.unit_quaternion_;
    }

    SO3 SO3::operator*(const SO3 &other) const
    {
        SO3 result(*this);
        result.unit_quaternion_ *= other.unit_quaternion_;
        result.unit_quaternion_.normalize();
        return result;
    }

    void SO3::operator*=(const SO3 &other)
    {
        unit_quaternion_ *= other.unit_quaternion_;
        unit_quaternion_.normalize();
    }

    // 用于和向量相乘
    Vector3d SO3::operator*(const Vector3d &xyz) const
    {
        return unit_quaternion_._transformVector(xyz);
    }


    // SO3的逆
    SO3 SO3::inverse() const
    {
        return SO3(unit_quaternion_.conjugate());    // 返回共轭四元数
    }

    // 转换为旋转矩阵
    Matrix3d SO3::matrix() const
    {
        return unit_quaternion_.toRotationMatrix();
    }

    // 返回旋转矩阵的伴随矩阵
    Matrix3d SO3::Adj() const
    {
        return matrix();
    }

    Matrix3d SO3::generator(int i)
    {
        // 断言函数,参数条件为假则终止
        assert(i >= 0 && i < 3);
        Vector3d e;
        e.setZero();
        e[i] = 1.f;
        return hat(e);
    }


    // SO3的映射
    Vector3d SO3::log() const
    {
        return SO3::log(*this);
    }

    Vector3d SO3::log(const SO3 &other)
    {
        double theta;
        return logAndTheta(other, &theta);
    }

    // 映射为旋转向量和旋转角
    Vector3d SO3::logAndTheta(const SO3 &other, double *theta)
    {
        double n = other.unit_quaternion_.vec().norm();        // 四元数虚部的模长
        double w = other.unit_quaternion_.w();            // 四元数实部
        double squared_w = w * w;

        double two_atan_nbyw_by_n;
        // 变换原理
        // C. Hertzberg et al.:
        // "Integrating Generic Sensor Fusion Algorithms with Sound State
        // Representation through Encapsulation of Manifolds"
        // Information Fusion, 2011
        if (n < SMALL_EPS)
        {
            // 四元数归一化后,n=1,w=1.
            assert(fabs(w) > SMALL_EPS);

            two_atan_nbyw_by_n = 2. / w - 2. * (n * n) / (w * squared_w);
        }
        else
        {
            if (fabs(w) < SMALL_EPS)
            {
                if (w > 0)
                {
                    two_atan_nbyw_by_n = M_PI / n;
                }
                else
                {
                    two_atan_nbyw_by_n = -M_PI / n;
                }
            }
            two_atan_nbyw_by_n = 2 * atan(n / w) / n;
        }

        *theta = two_atan_nbyw_by_n * n;
        return two_atan_nbyw_by_n * other.unit_quaternion_.vec();

    }

    SO3 SO3::exp(const Vector3d &omega)
    {
        double theta;
        return expAndTheta(omega, &theta);
    }

    // 旋转向量转换成四元数
    SO3 SO3::expAndTheta(const Vector3d &omega, double *theta)
    {
        *theta = omega.norm();
        double half_theta = 0.5 * (*theta);

        double imag_factor;                // 虚部系数
        double real_factor = cos(half_theta);        //
        if ((*theta) < SMALL_EPS)
        {
            double theta_sq = (*theta) * (*theta);
            double theta_po4 = theta_sq * theta_sq;
            imag_factor = 0.5 - 0.0208333 * theta_sq + 0.000260417 * theta_po4;
        }
        else
        {
            double sin_half_theta = sin(half_theta);
            imag_factor = sin_half_theta / (*theta);
        }

        return SO3(Quaterniond(real_factor,
                               imag_factor * omega.x(),
                               imag_factor * omega.y(),
                               imag_factor * omega.z()));

    }

    Matrix3d SO3::hat(const Vector3d &v)
    {
        Matrix3d Omega;
        Omega << 0, -v(2), v(1),
                v(2), 0, -v(0),
                -v(1), v(0), 0;
        return Omega;

    }

    // 反对称矩阵到向量
    Vector3d SO3::vee(const Matrix3d &Omega)
    {
        assert(fabs(Omega(2, 1) + Omega(1, 2)) < SMALL_EPS);
        assert(fabs(Omega(0, 2) + Omega(2, 0)) < SMALL_EPS);
        assert(fabs(Omega(1, 0) + Omega(0, 1)) < SMALL_EPS);
        return Vector3d(Omega(2, 1), Omega(0, 2), Omega(1, 0));
    }

    // 李括号操作，向量叉乘
    Vector3d SO3::lieBracket(const Vector3d &omega1, const Vector3d &omega2)
    {
        return omega1.cross(omega2);
    }

    // 是把一个角速度转化为角速度矩阵的函数
    Matrix3d SO3::d_lieBracketab_by_d_a(const Vector3d &b)
    {
        return -hat(b);
    }

    void SO3::setQuaternion(const Quaterniond &quaternion)
    {
        assert(quaternion.norm() != 0);
        unit_quaternion_ = quaternion;
        unit_quaternion_.normalize();
    }


}
