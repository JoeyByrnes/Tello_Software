// Copyright 2021-2022 The MathWorks, Inc.
#include <vector>
#include "ceres/ceres.h"
#include <Eigen/Core>

#ifdef BUILDING_LIBMWCERESCODEGEN
    #include "cerescodegen/group_utilities.hpp"
#else
    /* To deal with the fact that PackNGo has no include file hierarchy */
    #include "group_utilities.hpp"
#endif


using namespace std;

#ifndef CUSTOM_LOCAL_PARAMETERIZATION_HPP
#define CUSTOM_LOCAL_PARAMETERIZATION_HPP
namespace mw_ceres {

    template <typename T>
    T wrapToPi(T theta) {
        T twoPi = T(2.0 * M_PI);
        return theta - ceres::floor((theta + T(M_PI)) / twoPi) * twoPi; // must use ceres::floor (cannot use %)
    }

    class PseudoSE3LocalParameterization : public ceres::LocalParameterization {
        /**
         * @brief Implements Oplus(x, delta_x) for SE(3) pose
         * @param[in] x An SE(3) pose: [x, y, z, qx, qy, qz, qw]
         * @param[in] delta Delta vector: [dx, dy, dz, dwx, dwy, dwz]
         * @param[out] x_oplus_delta The Oplus result
         * @return True if operation is successful
        */
        virtual bool Plus(const double* x, const double* delta, double* x_oplus_delta) const override;

        /**
         * @brief Computes Jacobian D(Oplus(x, delta_x))/D(delta_x), i.e.
         *        global-to-local Jacobian (from manifold variable to local tangent space variable)
         * @param[in] x Manifold variable
         * @param[out] jacobian Computed Jacobian
         * @return True if operation is successful
        */
        virtual bool ComputeJacobian(const double* x, double* jacobian) const override;

        virtual int GlobalSize() const {
            return 7;
        };

        virtual int LocalSize() const {
            return 6;
        };
    };


    class SO3LocalParameterization : public ceres::LocalParameterization {
        /**
         * @brief Implements Oplus(x, delta_x) for SO(3) pose
         * @param[in] x An SO(3) pose given as quaternion: [qx, qy, qz, qw]
         * @param[in] delta Delta vector: [dwx, dwy, dwz]
         * @param[out] x_oplus_delta The Oplus result
         * @return True if operation is successful
        */
        virtual bool Plus(const double* x, const double* delta, double* x_oplus_delta) const override;

        /**
         * @brief Computes Jacobian D(Oplus(x, delta_x))/D(delta_x)
         */
        virtual bool ComputeJacobian(const double* x, double* jacobian) const override;
        
        virtual int GlobalSize() const {
            return 4;
        };

        virtual int LocalSize() const {
            return 3;
        };
    };

    /// Functor to compute oplus(x, delta) for S^1 group (1-circle)
    /// To be used with ceres::AutoDiffLocalParameterization
    class AngleOPlus {
    public:
        template <typename T>
        bool operator()(const T* theta, const T* deltaTheta,
                        T* theta_plus_deltaTheta) const {
            *theta_plus_deltaTheta = mw_ceres::wrapToPi<T>(*theta + *deltaTheta);
            return true;
        }
    };

    /// Functor to compute oplus(x, delta) for SO(3) orientation (through matrix exponential multiplied on the right) 
    /// To be used with ceres::AutoDiffLocalParameterization
    class SO3_OPlus {
    public:
        template <typename T>
        bool operator()(const T* x, const T* delta, T* x_oplus_delta) const {
            // x is given as [qx, qy, qz, qw]
            Eigen::Map<const Eigen::Quaternion<T>> quat(x);
            const Eigen::Matrix<T, 3, 1> domega(delta);
            Eigen::Matrix<T, 3, 3> R = quat.normalized().toRotationMatrix() * mw_ceres::SO3::expm<T>(domega);
            Eigen::Quaternion<T> quatOut(R);
            quatOut.normalize();
            x_oplus_delta[0] = quatOut.x();
            x_oplus_delta[1] = quatOut.y();
            x_oplus_delta[2] = quatOut.z();
            x_oplus_delta[3] = quatOut.w();
            return true;
        }
    };
}

inline bool mw_ceres::PseudoSE3LocalParameterization::Plus(const double* x, const double* delta, double* x_oplus_delta) const {
    // x comes in as [x, y, z, qx, qy, qz, qw]
    // x_oplus_delta goes out as [x, y, z, qx, qy, qz, qw]
    Eigen::Map<const Eigen::Vector3d> pos(x);
    // Eigen quaternion constructor specifies the real w coefficient first,
    // while internally the coefficients are stored in the following order: [x, y, z, w]
    Eigen::Map<const Eigen::Quaterniond> quat(x+3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);
    const Eigen::Vector3d domega(delta+3);

    Eigen::Matrix3d R0 = quat.normalized().toRotationMatrix();
    Eigen::Matrix3d R = R0 * mw_ceres::SO3::expm(domega);
    
    Eigen::Map<Eigen::Vector3d> posOut(x_oplus_delta);

    posOut = pos + R0 * dp;
    Eigen::Quaterniond quatOut(R);
    quatOut.normalize();
    x_oplus_delta[3] = quatOut.x();
    x_oplus_delta[4] = quatOut.y();
    x_oplus_delta[5] = quatOut.z();
    x_oplus_delta[6] = quatOut.w();

    return true;
}

inline bool mw_ceres::PseudoSE3LocalParameterization::ComputeJacobian(const double* x, double* jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> jac(jacobian);
    jac.topRows<6>().setIdentity();
    jac.bottomRows<1>().setZero();

    return true;
}

inline bool mw_ceres::SO3LocalParameterization::Plus(const double* x, const double* delta, double* x_oplus_delta) const {
    // x comes in as [qx, qy, qz, qw]
    Eigen::Map<const Eigen::Quaterniond> quat(x);
    const Eigen::Vector3d domega(delta);
    Eigen::Matrix3d R = quat.normalized().toRotationMatrix()* mw_ceres::SO3::expm(domega);
    Eigen::Quaterniond quatOut(R);
    quatOut.normalize();
    x_oplus_delta[0] = quatOut.x();
    x_oplus_delta[1] = quatOut.y();
    x_oplus_delta[2] = quatOut.z();
    x_oplus_delta[3] = quatOut.w();

    return true;
}

inline bool mw_ceres::SO3LocalParameterization::ComputeJacobian(const double* x, double* jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> jac(jacobian);
    jac.topRows<3>().setIdentity();
    jac.bottomRows<1>().setZero();
    return true;
}

#endif // CUSTOM_LOCAL_PARAMETERIZATION_HPP
