//
// Created by ys on 24. 2. 26.
//

#ifndef RBCANINE_GLOBALPARAMETERS_HPP
#define RBCANINE_GLOBALPARAMETERS_HPP

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <Eigen/Dense>
#include "QuadParameters.h"

/// ------------------------------ rbdl model definition --------------------------

class FloatingBaseRobotModel{
public:
    static RigidBodyDynamics::Model& getModel();

    static void initModel();

    static bool isInitialized();

private:
    static std::unique_ptr<RigidBodyDynamics::Model> model;
    static bool initialized;

    FloatingBaseRobotModel() = default;
    ~FloatingBaseRobotModel() = default;
    static Eigen::Matrix3d shiftInertiaToNewCOM(const Eigen::Matrix3d& I_com, double mass, const Eigen::Vector3d& original_com, const Eigen::Vector3d& new_com);
};



#endif //RBCANINE_GLOBALPARAMETERS_HPP
