//
// Created by hs on 22. 10. 14.
//

#ifndef CANINE_ROBOTMATH_HPP
#define CANINE_ROBOTMATH_HPP

#include <math.h>
#include <iostream>

#include <Eigen/Dense>
#include "CAMEL_SDK/CAMEL_api.hpp"
#include "QuadParameters.h"

namespace RobotMath
{
    Eigen::Matrix3d GetBaseRotationMat(const Eigen::Vector4d& quat);
    Eigen::Matrix3d GetSlopeRotationMat(const Eigen::Vector3d& rpy);
    Eigen::Matrix3d GetYawRotationMat(const Eigen::Vector3d& rpy);
    Eigen::Matrix3d getRmatrix(const Eigen::Vector3d& w, const double& rTheta);
    Eigen::Matrix3d GetBaseRotationMatInverse(const Eigen::Vector4d& quat);
    Eigen::Vector3d OrientationError(Eigen::Matrix3d RDesired, Eigen::Matrix3d R);
    Eigen::Matrix4d GetGlobal2BodyTransMat(const Eigen::Vector4d& quat, const Eigen::Vector3d& pos);
    Eigen::Matrix4d GetBody2GlobalTransMat(const Eigen::Vector4d& quat, const Eigen::Vector3d& pos);
    void TransMatBody2Foot(Eigen::Matrix4d* Base2Foot, LEG_INDEX legIndex, const double& hip, const double& thi,
                           const double& cal);
    void TransMatBody2Knee(Eigen::Matrix4d* Base2Foot, LEG_INDEX legIndex, const double& hip, const double& thi,
                           const double& cal);
    void TransformQuat2Euler(const Eigen::Vector4d& quat, double* euler);
    void GetJacobian(Eigen::Matrix<double, 3, 3>& J, const Eigen::Matrix<double, 3, 1>& pos, int side);
    void GetJacobian2(Eigen::Matrix<double, 3, 3>& J, const Eigen::Matrix<double, 3, 1>& pos, int side);
    Eigen::Matrix<double, 3, 3> getSkewMatrix(Eigen::Vector3d r);
    int8_t NearZero(float a);
    int8_t NearOne(float a);
    Eigen::Matrix<double, 6, 1> getScrew(const Eigen::Vector3d& w, const Eigen::Vector3d& q);
    Eigen::Matrix4d getTmatrix(const Eigen::VectorXd& Screw, const double& rTheta);
    inline Eigen::Matrix<double, 6, 6> getAdjMatrix(const Eigen::Matrix4d& T);
    inline Eigen::Matrix4d getInvTMatrix(const Eigen::Matrix4d& T);
    void GetLegIK(Eigen::Vector3d& jointPos, Eigen::Vector3d footPos, const int& leg);

    Eigen::Vector3d rot2RPY(const Eigen::Matrix3d& mat);
    Eigen::Vector4d rpy2QUAT(const Eigen::Vector3d& rpy);
    Eigen::Vector2d computeDiagonalIntersection(
        const Eigen::Vector3d& FL, // 전면 좌측
        const Eigen::Vector3d& FR, // 전면 우측
        const Eigen::Vector3d& RR, // 후면 우측
        const Eigen::Vector3d& RL // 후면 좌측
    );

    Eigen::Matrix4d vec2T(Eigen::Vector4d quat, Eigen::Vector3d posLinear);
    Eigen::Vector3d RotLogarithm(const Eigen::Matrix3d& rot);
    Eigen::VectorXd TLogarithm(const Eigen::Matrix4d& T);
}

#endif //CANINE_ROBOTMATH_HPP
