#ifndef DRAKE_CALCULATIONS_HPP_
#define DRAKE_CALCULATIONS_HPP_

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/math/rigid_transform.h"
// #include "drake/multibody/plant/multibody_plant.h"

namespace base_package{

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
// using multibody::RigidBody;
using drake::multibody::RigidBodyFrame;
using drake::multibody::Frame;
// using multibody::Context;
using drake::systems::Context;
using drake::math::RigidTransformd;

class DifferentialInverseKinematicsCalculator {

    public:
        DifferentialInverseKinematicsCalculator();

        void load_model(std::string urdf);

        // Eigen::VectorXd goalPosition, Eigen::VectorXd currentPose
        Eigen::VectorXd calculateOneStep(); // returns vector of goal joint velocities (or integrated into positions?).

        Eigen::MatrixXd calculate2DPseudoInverseJacobian(); // based on current robot pose

        Eigen::VectorXd calculateCartesianCoordinates(std::string joint_name);

        int getp();
        std::vector<std::string> getpn();

    private:

        MultibodyPlant<double> _plant;
        std::unique_ptr<Context<double>> _plantContextPointer; // initialization can be delayed since it is a pointer, but I might as well do it in the initializer list

};

}

#endif