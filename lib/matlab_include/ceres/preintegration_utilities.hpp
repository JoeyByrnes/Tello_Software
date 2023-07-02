// Copyright 2021-2022 The MathWorks, Inc.
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "ceres/ceres.h"

#ifdef BUILDING_LIBMWCERESCODEGEN
    #include "cerescodegen/cerescodegen_spec.hpp"
    #include "cerescodegen/group_utilities.hpp"
#else
    /* To deal with the fact that PackNGo has no include file hierarchy */
    #include "cerescodegen_spec.hpp"
    #include "group_utilities.hpp"
#endif

#ifndef PREINTEGRATION_UTILITIES_HPP
#define PREINTEGRATION_UTILITIES_HPP

namespace mw_ceres {

    class IMUPreIntegratorBase {
    public:
        IMUPreIntegratorBase(const Eigen::Vector3d& biasGyro, 
            const Eigen::Vector3d& biasAcc, 
            double dT)
            : m_BiasGyro(biasGyro),
            m_BiasAcc(biasAcc),
            m_DeltaT(dT) {
            m_GyroscopeNoise << Eigen::Matrix3d::Identity();
            m_AccelerometerNoise << Eigen::Matrix3d::Identity();

            m_Rot << Eigen::Matrix3d::Identity();
            m_Vel << Eigen::Vector3d::Zero();
            m_Pos << Eigen::Vector3d::Zero();
            m_dRot_dBiasG << Eigen::Matrix3d::Zero();
            m_dVel_dBiasG << Eigen::Matrix3d::Zero();
            m_dVel_dBiasA << Eigen::Matrix3d::Zero();
            m_dPos_dBiasG << Eigen::Matrix3d::Zero();
            m_dPos_dBiasA << Eigen::Matrix3d::Zero();
        }

        virtual ~IMUPreIntegratorBase() = default;

        virtual bool integrateMeasurement(const Eigen::Vector3d& omegaMeasurement,
            const Eigen::Vector3d& accMeasurement) = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    protected:
        /** Gyroscope sensor bias in body frame*/
        Eigen::Vector3d m_BiasGyro;

        /** Accelerometer sensor bias in body frame*/
        Eigen::Vector3d m_BiasAcc;

        /** Raw angular velocity measurement noise covariance matrix*/
        Eigen::Matrix3d m_GyroscopeNoise;

        /** Raw linear acceleration measurement noise covariance matrix*/
        Eigen::Matrix3d m_AccelerometerNoise;

        /** 9x9 IMU pre-integrated measurement noise covariance*/
        Eigen::Matrix<double, 9, 9> m_Covariance;

        /** Time interval to receive IMU reading updates*/
        double m_DeltaT;

        /** Pre-integrated rotation measurement*/
        Eigen::Matrix3d m_Rot;

        /** Pre-integrated velocity measurement*/
        Eigen::Vector3d m_Vel;

        /** Pre-integrated position measurement*/
        Eigen::Vector3d m_Pos;

        // Jacobians w.r.t. bias terms
        Eigen::Matrix3d m_dRot_dBiasG;
        Eigen::Matrix3d m_dVel_dBiasA;
        Eigen::Matrix3d m_dVel_dBiasG;
        Eigen::Matrix3d m_dPos_dBiasA;
        Eigen::Matrix3d m_dPos_dBiasG;

        // For covariance update
        Eigen::Matrix<double, 9, 9> m_A;
        Eigen::Matrix<double, 9, 6> m_B;
    };


    /* IMU Pre-Integration over the navState manifold */
    // Implementation is based on details in [1]
    // [1] C. Foster, L. Carlone, F. Dellaert and D. Scaramuzza, "On-Manifold Preintegration
    //     for Real-Time Visual-Inertial Odometry," IEEE Transactions on Robotics, Vol. 33, No. 1,
    //     pp. 1-21, Feb. 2017, doi: 10.1109/TRO.2016.2597321
    //
    // See more details at https://ieeexplore.ieee.org/document/7557075
    class OnManifoldPreIntegrator : public mw_ceres::IMUPreIntegratorBase{
    public:
        OnManifoldPreIntegrator(const Eigen::Vector3d& biasGyro,
            const Eigen::Vector3d& biasAcc, double dT) : IMUPreIntegratorBase(biasGyro, biasAcc, dT) {
            m_A = Eigen::Matrix<double, 9, 9>::Identity();
            m_B = Eigen::Matrix<double, 9, 6>::Zero();
            m_Covariance = Eigen::Matrix<double, 9, 9>::Zero();
        }

        /** Accumulate raw IMU readings */
        bool integrateMeasurement(const Eigen::Vector3d& omegaMeasurement, 
            const Eigen::Vector3d& accMeasurement) override{
            
            Eigen::Vector3d correctedOmegaMeasurement = (omegaMeasurement - m_BiasGyro);
            Eigen::Vector3d dTheta = correctedOmegaMeasurement * m_DeltaT;
            Eigen::Vector3d correctedAccMeasurement = accMeasurement - m_BiasAcc; // Note that accMeasurement here includes the gravity acceleration in local frame
            Eigen::Matrix3d dRot = mw_ceres::SO3::expm(dTheta);
            double m_DeltaT_Sq = m_DeltaT * m_DeltaT;

            // update Jacobian of pre-integrated measurements w.r.t. bias change
            Eigen::Matrix3d tmp1 = m_Rot * mw_ceres::SO3::hat<double>(correctedAccMeasurement);
            Eigen::Matrix3d tmp2 = tmp1 * m_dRot_dBiasG;

            m_dPos_dBiasA += m_dVel_dBiasA * m_DeltaT - 0.5 * m_Rot * m_DeltaT_Sq;
            m_dPos_dBiasG += m_dVel_dBiasG * m_DeltaT - 0.5 * tmp2 * m_DeltaT_Sq;

            m_dVel_dBiasA -= m_Rot * m_DeltaT;
            m_dVel_dBiasG -= tmp2 * m_DeltaT;

            Eigen::Matrix3d rightJac_dTheta = mw_ceres::SO3::rightJacobianExpm(dTheta);
            m_dRot_dBiasG = dRot.transpose() * m_dRot_dBiasG - rightJac_dTheta * m_DeltaT;


            // update covariance for pre-integrated measurements
            // noise propagation is determined by the linear system dx_(i+1) = A * dx_i + B * u
            // where dx = [ dPos, dphi, dvel]' and u = [ ita_gd, ita_ad]'

            //m_A.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
            m_A.block<3, 3>(0, 3) = -0.5 * tmp1 * m_DeltaT_Sq;
            m_A.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * m_DeltaT;
            m_A.block<3, 3>(3, 3) = dRot.transpose();
            m_A.block<3, 3>(6, 3) = -tmp1 * m_DeltaT;
            //m_A.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();

            m_B.block<3, 3>(0, 3) = 0.5 * m_Rot * m_DeltaT_Sq;
            m_B.block<3, 3>(3, 0) = rightJac_dTheta * m_DeltaT;
            m_B.block<3, 3>(6, 3) = m_Rot * m_DeltaT;

            // raw IMU measurement noise covariance matrix, combined from m_GyroscopeNoise & m_AccelerometerNoise
            Eigen::Matrix<double, 6, 6> imuMeasurementNoise = Eigen::Matrix<double, 6,6>::Identity();
            imuMeasurementNoise.block<3, 3>(0, 0) = m_GyroscopeNoise;
            imuMeasurementNoise.block<3, 3>(3, 3) = m_AccelerometerNoise;
            m_Covariance = m_A * m_Covariance * m_A.transpose() + m_B * imuMeasurementNoise * m_B.transpose();

            // we could further consider the error introduced by integrating positions from velocities
            // assuming integrationErrorCov represents the continuous-time integration uncertainty (typically 1e-8 on diagonal),
            // then m_Covariance can be updated as following

            // m_Covariance.block<3, 3>(0, 0) += integrationErrorCov * m_DeltaT;

            // update pre-integrated measurements
            m_Pos += m_Vel * m_DeltaT + 0.5 * m_Rot * correctedAccMeasurement * m_DeltaT_Sq;
            m_Vel += m_Rot * correctedAccMeasurement * m_DeltaT;
            m_Rot *= dRot;

            return true;
        }

        /** Extract accumulated delta quaternion*/
        Eigen::Quaterniond deltaQuat() const {
            Eigen::Quaterniond result(m_Rot);
            return result;
        }

        /** Extract accumulated delta position*/
        Eigen::Vector3d deltaPos() const {
            return m_Pos;
        }

        /** Extract accumulated delta velocity*/
        Eigen::Vector3d deltaVel() const {
            return m_Vel;
        }

        /** Extract accumulated delta navState*/
        Eigen::Matrix<double, 9, 1> deltaNavState() const{
            Eigen::Matrix<double, 9, 1> result;
            result << mw_ceres::SO3::logm<double>(m_Rot), m_Vel, m_Pos;
            return result;
        }

        Eigen::Matrix<double, 9, 9> deltaCovariance() const {
            return m_Covariance;
        }

        /** The name dR_dBiasGryo is consistent with the paper notation, but it's a bit misleading. 
         * As it does not reflect the linearized change of rotation w.r.t. gyro bias, but instead
         * it is used in this expression: Rot * expm( dR_dBiasGyro * delta_BiasGyro)*/
        Eigen::Matrix<double, 3, 3> Jacobian_dR_dBiasGyro() const {
            return m_dRot_dBiasG;
        }

        /** Extract Jacobian: change of pre-integrated velocity w.r.t. gyro bias*/
        Eigen::Matrix<double, 3, 3> Jacobian_dV_dBiasGyro() const {
            return m_dVel_dBiasG;
        }

        /** Extract Jacobian: change of pre-integrated velocity w.r.t. accelerometer bias*/
        Eigen::Matrix<double, 3, 3> Jacobian_dV_dBiasAcc() const {
            return m_dVel_dBiasA;
        }

        /** Extract Jacobian: change of pre-integrated position w.r.t. gyro bias*/
        Eigen::Matrix<double, 3, 3> Jacobian_dP_dBiasGyro() const {
            return m_dPos_dBiasG;
        }

        /** Extract Jacobian: change of pre-integrated position w.r.t. accelerometer bias*/
        Eigen::Matrix<double, 3, 3> Jacobian_dP_dBiasAcc() const {
            return m_dPos_dBiasA;
        }

        /** Setter for gyroscope noise covariance*/
        void setGyroscopeNoise(const Eigen::Matrix<double,3,3>& cov) {
            m_GyroscopeNoise = cov;
        }

        /** Setter for accelerometer noise covariance*/
        void setAccelerometerNoise(const Eigen::Matrix<double, 3, 3>& cov) {
            m_AccelerometerNoise = cov;
        }
    };
}

#endif //PREINTEGRATION_UTILITIES_HPP
