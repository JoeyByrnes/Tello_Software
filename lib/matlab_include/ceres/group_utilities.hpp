// Copyright 2021-2022 The MathWorks, Inc.
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>
#include "ceres/ceres.h"

#ifdef BUILDING_LIBMWCERESCODEGEN
    #include "cerescodegen/cerescodegen_spec.hpp"
#else
    /* To deal with the fact that PackNGo has no include file hierarchy */
    #include "cerescodegen_spec.hpp"
#endif

#ifndef GROUP_UTILITIES_HPP
#define GROUP_UTILITIES_HPP

namespace mw_ceres {
    namespace SO3 {
        /** SO3 hat operator. From compact vector representation to so(3) */
        template <typename T>
        inline Eigen::Matrix<T, 3, 3> hat(const Eigen::Matrix<T, 3, 1>& phi) {
            Eigen::Matrix<T, 3, 3> S;
            S << static_cast<T>(0), -phi[2], phi[1],
                 phi[2], static_cast<T>(0), -phi[0], 
                -phi[1], phi[0], static_cast<T>(0);
            return S;
        }

        /** SO3 vee operator. Inverse of hat operator */
        template <typename T>
        inline void vee(const Eigen::Matrix<T, 3, 3>& S, Eigen::Matrix<T, 3, 1>& phi) {
            phi[0] = S(2, 1);
            phi[1] = S(0, 2);
            phi[2] = S(1, 0);
        }

        /** Normalize rotation */
        template <typename T>
        inline Eigen::Matrix<T, 3, 3> NormalizeRotation(const Eigen::Matrix<T, 3, 3> &R){
            Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
            return svd.matrixU() * svd.matrixV().transpose();
        }

        /** SO3 Exponential map */
        template <typename T>
        inline Eigen::Matrix<T, 3, 3> expm(const Eigen::Matrix<T, 3, 1>& phi) {
            T thetaSq = phi.dot(phi);
            T theta = ceres::sqrt(thetaSq);
                      
            if (thetaSq > std::numeric_limits<T>::epsilon()) {
                T sinTheta = ceres::sin(theta);
                T sinHalfTheta = ceres::sin(theta / static_cast<T>(2.0));
                T oneMinusCosTheta = static_cast<T>(2.0) * sinHalfTheta * sinHalfTheta;

                Eigen::Matrix<T, 3, 3> phiHat = hat<T>(phi) / theta;
                return Eigen::Matrix<T, 3, 3>::Identity() + sinTheta * phiHat +
                       oneMinusCosTheta * phiHat * phiHat;
            }
            else {
                return Eigen::Matrix<T, 3, 3>::Identity() + hat<T>(phi);
            }
        }

        /** Compute the right Jacobian of SO3 */
        /// i.e. Jr as in (Exp(phi + delta_phi)) /approx Exp(phi) * Exp(Jr(phi)*delta_phi)
        inline Eigen::Matrix3d rightJacobianExpm(const Eigen::Vector3d& phi) {
            double thetaSq = phi.dot(phi);
            Eigen::Matrix3d phiHat = hat<double>(phi);
            if (thetaSq < std::numeric_limits<double>::epsilon()) {
                return Eigen::Matrix3d::Identity() - 0.5 * phiHat;
            }
            else {
                double theta = ceres::sqrt(thetaSq);
                Eigen::Matrix3d K = phiHat / theta;
                double sinHalfTheta = ceres::sin(0.5*theta);
                double oneMinusCosTheta = 2 * sinHalfTheta * sinHalfTheta;
                return Eigen::Matrix3d::Identity() - (oneMinusCosTheta / theta) * K + (1 - ceres::sin(theta) / theta) * K * K;
            }
        }

        /** SO3 Logarithmic map */
        template <typename T>
        inline Eigen::Matrix<T, 3, 1> logm(const Eigen::Matrix<T, 3, 3>& R) {
            T tr = R.trace();
            T theta = ceres::acos((tr - static_cast<T>(1.0)) / static_cast<T>(2.0));

            T mag;
            if (tr < static_cast<T>(-1.0 + 1e-10)) { // when trace(R) is near -1, cos(theta) -> -1, i.e. theta = +/- pi
                // In such case we cannot extract the rotation axis using the regular formula
                // if n is the rotation axis (unit vector), and v is an arbitrary vector to be rotated by pi,
                // we have identity (R + I)v = 2*n*(n.dot(v)) = 2*(n*n')*v
                // i.e. 0.5*(R + I) = n*n'
                Eigen::Matrix<T, 3, 3> M = static_cast<T>(0.5) * (R + Eigen::Matrix<T, 3, 3>::Identity());
                T eps = static_cast<T>(1e-5);
                T pi = static_cast<T>(M_PI);
                if (ceres::abs(M(0, 0)) > eps) {
                    return pi * M.col(0) / M(0, 0);
                }
                else if ( ceres::abs(M(1, 1)) > eps) {
                    return pi * M.col(1) / M(1, 1);
                }
                else {
                    return pi * M.col(2) / M(2, 2);
                }
            }
            else {
                T traceMinusThree = tr - static_cast<T>(3.0);
                //assert(traceMinusThree <= static_cast<T>(0.0));
                if (traceMinusThree > -static_cast<T>(1e-7)) {
                    // when trace(R) is near 3, cos(theta) -> 1, i.e. theta -> 0
                    // x/sin(x) = 1 + 1/6*x^2 + 7/360*x^4 + ... (taylor expansion)
                    // when x -> 0, x/sin(x) can be approximated as 1 + 1/6*x^2
                    // also when x -> 0, trace(R)-3 = 2*(cos(x)-1) approx. 2*(-x^2/2) = -x^2
                    mag = static_cast<T>(0.5) -
                          static_cast<T>(1.0) / static_cast<T>(12.0) * traceMinusThree;
                } else {
                    mag = static_cast<T>(0.5) * theta / ceres::sin(theta);
                }

                Eigen::Matrix<T, 3, 1> vec;
                vec << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
                return mag * vec;
            }

        }

        /** Compute the right Jacobian of SO3 logarithm */
        /// i.e. Jr_inv as in Logm(Exp(phi)*Exp(delta_phi)) /approx phi + Jr_inv*delta_phi
        inline Eigen::Matrix3d rightJacobianLogm(const Eigen::Vector3d& phi) {
            Eigen::Matrix3d phiHat = mw_ceres::SO3::hat<double>(phi);
            double thetaSq = phi.dot(phi);
            if (thetaSq < std::numeric_limits<double>::epsilon()) {
                return Eigen::Matrix3d::Identity();
            }
            double theta = phi.norm();
            // closed form for Jr_inv
            return Eigen::Matrix3d::Identity() + 0.5 * phiHat + (1 / thetaSq - (1 + ceres::cos(theta)) / (2 * theta * ceres::sin(theta))) * phiHat * phiHat;
        }
    }

}

#endif // GROUP_UTILITIES_HPP
